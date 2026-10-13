// teensy_thrusters -- Teensy 4.1 pulse generator for Blue Robotics Basic ESCs
// and hobby servos
//
// Implements PROTOCOL.md (protocol 1.1) in this folder. Read that first; this
// file follows its section numbers in comments.
//
// Build: Arduino IDE + Teensyduino, Board "Teensy 4.1", USB Type "Serial",
// CPU Speed 600 MHz (the pulse math assumes the 150 MHz bus clock that speed
// gives). No third-party libraries.
//
// SAFETY SUMMARY (PROTOCOL.md sections 8 and 9):
//   * Every thruster channel outputs stop_us (1500 us) from the first lines of
//     setup(), before USB is started, and never stops pulsing while this
//     firmware runs. Because the pulses never stop, the ESC's own ~0.3 s
//     signal-loss stop never fires -- the command watchdog below is the only
//     thing that stops a thruster when the host dies.
//   * Watchdog: in RUN, no accepted SA/SP for watchdog_ms -> every channel to
//     stop_us, state TRIPPED, until the next accepted SA.
//   * A malformed or out-of-range frame is rejected whole. Nothing is ever
//     clamped into range: clamping would turn a host bug into motion.
//   * DTR falling (host closed the port) -> every channel to stop_us.
//   * NOT covered: a hang of this firmware. PWM hardware keeps outputting the
//     last duty value if loop() stops running. The i.MX RT1062 hardware
//     watchdog is not enabled in this version (listed follow-up).
//
// SERVOS (PROTOCOL.md 4.1 and 6.7-6.11): four servo channels on their own
// QuadTimer groups, so the servo frame rate never touches the thruster rate.
// Servos are off (pin LOW, no pulses) from boot until the host commands one.
// Servo commands never feed the thruster watchdog, and the thruster watchdog
// never moves a servo: a servo holds its last pulse until told otherwise. On
// DTR falling, servos hold or go limp per the SC host_loss setting.

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Identity (PROTOCOL.md 6.1 and 11)
// ---------------------------------------------------------------------------
static const char FIRMWARE_NAME[] = "teensy_thrusters";
static const char PROTOCOL_VERSION[] = "1.1";
static const char FIRMWARE_VERSION[] = "1.1.0";

// ---------------------------------------------------------------------------
// Channel and pin map (PROTOCOL.md 4)
// ---------------------------------------------------------------------------
static const uint8_t THRUSTER_COUNT = 8;
//                                               ch: 0  1  2  3  4  5   6   7
static const uint8_t THRUSTER_PINS[THRUSTER_COUNT] = {2, 3, 4, 5, 6, 9, 22, 23};
// ID <servo_channel>: index of the first servo channel when thruster and
// servo channels are counted together. Servo commands number servos 0-3.
static const uint8_t SERVO_CHANNEL = THRUSTER_COUNT;
static const uint8_t SERVO_COUNT = 4;
//                                         servo: 0   1   2   3
static const uint8_t SERVO_PINS[SERVO_COUNT] = {10, 11, 12, 14};  // QuadTimer1.0, 1.2, 1.1, 3.2
static const uint8_t LED_PIN = 13;

// ---------------------------------------------------------------------------
// Limits and boot defaults (PROTOCOL.md 5)
// ---------------------------------------------------------------------------
static const uint32_t PULSE_HARD_MIN_US = 800;
static const uint32_t PULSE_HARD_MAX_US = 2200;
static const uint32_t PERIOD_GUARD_US = 100;   // pulse at least this much shorter than its frame
static const uint32_t WATCHDOG_MIN_MS = 50;
static const uint32_t WATCHDOG_MAX_MS = 1000;
static const uint32_t ARM_HOLD_MIN_MS = 2000;
static const uint32_t ARM_HOLD_MAX_MS = 15000;
static const uint32_t ALLOWED_RATES_HZ[] = {50, 100, 200, 400};

static const uint32_t DEFAULT_RATE_HZ = 200;
static const uint32_t DEFAULT_WATCHDOG_MS = 200;
static const uint8_t DEFAULT_MASK = 0xFF;
static const uint32_t DEFAULT_MIN_US = 1100;
static const uint32_t DEFAULT_STOP_US = 1500;
static const uint32_t DEFAULT_MAX_US = 1900;

// Servo limits and boot defaults (PROTOCOL.md 5.1)
static const uint32_t SERVO_HARD_MIN_US = 500;
static const uint32_t SERVO_HARD_MAX_US = 2500;
static const uint32_t SERVO_ALLOWED_RATES_HZ[] = {50, 100, 200, 333};
static const uint32_t SERVO_DEFAULT_RATE_HZ = 50;
static const uint32_t SERVO_DEFAULT_MIN_US = 1000;
static const uint32_t SERVO_DEFAULT_MAX_US = 2000;

// ---------------------------------------------------------------------------
// Framing (PROTOCOL.md 3)
// ---------------------------------------------------------------------------
static const size_t MAX_FRAME_CHARS = 120;            // '$' through last checksum digit
static const size_t MAX_AFTER_DOLLAR = MAX_FRAME_CHARS - 1;
static const size_t MAX_FIELDS = 16;
static const size_t MAX_NUMBER_DIGITS = 10;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
enum State { STATE_IDLE, STATE_ARMING, STATE_RUN, STATE_TRIPPED };
static const char *STATE_NAMES[] = {"IDLE", "ARMING", "RUN", "TRIPPED"};

struct Config {
  uint32_t rate_hz;
  uint32_t watchdog_ms;
  uint8_t mask;
  uint32_t min_us;
  uint32_t stop_us;
  uint32_t max_us;
};

static Config cfg = {DEFAULT_RATE_HZ, DEFAULT_WATCHDOG_MS, DEFAULT_MASK,
                     DEFAULT_MIN_US, DEFAULT_STOP_US, DEFAULT_MAX_US};
static bool cfg_received = false;      // ST <cfg>: 0 after a reboot
static State state = STATE_IDLE;
static uint32_t last_feed_ms = 0;      // last accepted SA or SP
static uint32_t arm_start_ms = 0;
static uint32_t arm_hold_ms = 0;
static bool dtr_prev = false;
static uint32_t rx_errors = 0;

// Pulse width each thruster channel is outputting; 0 = inactive (held LOW).
static uint32_t out_us[THRUSTER_COUNT];

// Servo configuration (PROTOCOL.md 5.1). One frame rate for every servo
// group; per-servo pulse limits; what servos do when DTR falls.
static uint32_t servo_rate_hz = SERVO_DEFAULT_RATE_HZ;
static bool servo_hold_on_host_loss = true;
static uint32_t servo_min_us[SERVO_COUNT];
static uint32_t servo_max_us[SERVO_COUNT];
static bool servo_cfg_received = false;   // SV <scfg>: 0 after a reboot
// Pulse width each servo is outputting; 0 = off (held LOW, servo limp).
static uint32_t servo_out_us[SERVO_COUNT];

// Receive line buffer: the characters after '$'.
static char rx_buf[MAX_AFTER_DOLLAR + 1];
static size_t rx_len = 0;
static bool rx_in_frame = false;
static bool rx_overflow = false;
static bool rx_bad_char = false;

// ---------------------------------------------------------------------------
// Pulse output
// ---------------------------------------------------------------------------

// 16-bit duty for a pulse width at a frame rate (PROTOCOL.md 4).
static uint32_t dutyForPulseAt(uint32_t pulse_us, uint32_t rate_hz) {
  uint64_t duty = ((uint64_t)pulse_us * rate_hz * 65536ULL + 500000ULL) / 1000000ULL;
  if (duty > 65535ULL) {
    duty = 65535ULL;  // unreachable: CF/SC/SL enforce max_us <= period - guard
  }
  return (uint32_t)duty;
}

static uint32_t dutyForPulse(uint32_t pulse_us) {
  return dutyForPulseAt(pulse_us, cfg.rate_hz);
}

static bool channelActive(uint8_t ch) {
  return (cfg.mask >> ch) & 0x01;
}

// Drive one channel. pulse_us == 0 means no pulses: the pin is a GPIO held
// LOW, so an ESC on an undeclared channel never sees a pulse and never arms.
static void writeChannel(uint8_t ch, uint32_t pulse_us) {
  uint8_t pin = THRUSTER_PINS[ch];
  if (pulse_us == 0) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  } else {
    // analogWrite re-selects the PWM pin mux on every call, so this also
    // re-activates a channel that was held LOW.
    analogWrite(pin, dutyForPulse(pulse_us));
  }
  out_us[ch] = pulse_us;
}

// SAFETY: the one function every stop path goes through.
static void stopAllChannels() {
  for (uint8_t ch = 0; ch < THRUSTER_COUNT; ch++) {
    writeChannel(ch, channelActive(ch) ? cfg.stop_us : 0);
  }
}

// Program every thruster timer group to cfg.rate_hz. Calling it per pin sets
// that pin's whole group; pins 2/3 and 6/9 share groups by construction.
static void applyFrameRate() {
  analogWriteResolution(16);
  for (uint8_t ch = 0; ch < THRUSTER_COUNT; ch++) {
    analogWriteFrequency(THRUSTER_PINS[ch], (float)cfg.rate_hz);
  }
}

// Drive one servo. pulse_us == 0 means off: the pin is a GPIO held LOW and
// the servo goes limp.
static void writeServo(uint8_t n, uint32_t pulse_us) {
  uint8_t pin = SERVO_PINS[n];
  if (pulse_us == 0) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  } else {
    analogWrite(pin, dutyForPulseAt(pulse_us, servo_rate_hz));
  }
  servo_out_us[n] = pulse_us;
}

static void servosOff() {
  for (uint8_t n = 0; n < SERVO_COUNT; n++) {
    writeServo(n, 0);
  }
}

// Program every servo timer group to servo_rate_hz. Each servo pin is on its
// own QuadTimer channel, none shared with a thruster.
static void applyServoFrameRate() {
  for (uint8_t n = 0; n < SERVO_COUNT; n++) {
    analogWriteFrequency(SERVO_PINS[n], (float)servo_rate_hz);
  }
}

static bool servoRateAllowed(uint32_t rate_hz) {
  for (size_t i = 0; i < sizeof(SERVO_ALLOWED_RATES_HZ) / sizeof(SERVO_ALLOWED_RATES_HZ[0]); i++) {
    if (SERVO_ALLOWED_RATES_HZ[i] == rate_hz) {
      return true;
    }
  }
  return false;
}

// ---------------------------------------------------------------------------
// Replies (PROTOCOL.md 3 and 7)
// ---------------------------------------------------------------------------

static uint8_t checksumOf(const char *body, size_t len) {
  uint8_t sum = 0;
  for (size_t i = 0; i < len; i++) {
    sum ^= (uint8_t)body[i];
  }
  return sum;
}

// Reply writes go through the core's USB transmit path. If a host holds DTR
// but never reads, the core's transmit timeout bounds how long a write can
// block loop(), so the watchdog check is delayed, not lost.
static void sendFrame(const char *body) {
  char out[MAX_FRAME_CHARS + 8];
  snprintf(out, sizeof(out), "$%s*%02X\n", body, checksumOf(body, strlen(body)));
  Serial.write(out, strlen(out));
  Serial.send_now();
}

static void sendOk(const char *cmd) {
  char body[8];
  snprintf(body, sizeof(body), "OK,%s", cmd);
  sendFrame(body);
}

static void sendError(const char *code, const char *cmd) {
  char body[32];
  snprintf(body, sizeof(body), "ER,%s,%s", code, cmd);
  sendFrame(body);
  rx_errors++;
}

// ---------------------------------------------------------------------------
// Field parsing
// ---------------------------------------------------------------------------

static bool parseUint(const char *s, uint32_t *out) {
  size_t n = strlen(s);
  if (n == 0 || n > MAX_NUMBER_DIGITS) {
    return false;
  }
  uint64_t v = 0;
  for (size_t i = 0; i < n; i++) {
    if (s[i] < '0' || s[i] > '9') {
      return false;
    }
    v = v * 10 + (uint64_t)(s[i] - '0');
  }
  if (v > 0xFFFFFFFFULL) {
    return false;
  }
  *out = (uint32_t)v;
  return true;
}

static int hexDigit(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

static bool parseHexByte(const char *s, uint8_t *out) {
  if (strlen(s) != 2) {
    return false;
  }
  int hi = hexDigit(s[0]);
  int lo = hexDigit(s[1]);
  if (hi < 0 || lo < 0) {
    return false;
  }
  *out = (uint8_t)((hi << 4) | lo);
  return true;
}

static bool rateAllowed(uint32_t rate_hz) {
  for (size_t i = 0; i < sizeof(ALLOWED_RATES_HZ) / sizeof(ALLOWED_RATES_HZ[0]); i++) {
    if (ALLOWED_RATES_HZ[i] == rate_hz) {
      return true;
    }
  }
  return false;
}

// ---------------------------------------------------------------------------
// Commands (PROTOCOL.md 6). fields[0] is the command; fields[1..] its data.
// Every handler validates everything before it changes anything.
// ---------------------------------------------------------------------------

static void handleId(size_t nfields) {
  if (nfields != 1) {
    sendError("FIELDS", "ID");
    return;
  }
  // Same derivation as the Teensy core's USB serial-number string.
  uint32_t serial = HW_OCOTP_MAC0 & 0xFFFFFF;
  if (serial < 10000000) {
    serial = serial * 10;
  }
  // <servo_count> is appended last (protocol 1.1), so a 1.0 host that reads
  // only the first seven fields still parses this reply.
  char body[96];
  snprintf(body, sizeof(body), "ID,%s,%s,%s,%u,%u,%lu,%u", FIRMWARE_NAME,
           PROTOCOL_VERSION, FIRMWARE_VERSION, (unsigned)THRUSTER_COUNT,
           (unsigned)SERVO_CHANNEL, (unsigned long)serial, (unsigned)SERVO_COUNT);
  sendFrame(body);
}

static void handleSetAll(char **fields, size_t nfields) {
  if (nfields != 1 + THRUSTER_COUNT) {
    sendError("FIELDS", "SA");
    return;
  }
  uint32_t values[THRUSTER_COUNT];
  for (uint8_t ch = 0; ch < THRUSTER_COUNT; ch++) {
    if (!parseUint(fields[1 + ch], &values[ch])) {
      sendError("FORMAT", "SA");
      return;
    }
  }
  // SAFETY: reject the whole frame on any bad value; never clamp.
  for (uint8_t ch = 0; ch < THRUSTER_COUNT; ch++) {
    if (channelActive(ch)) {
      if (values[ch] < cfg.min_us || values[ch] > cfg.max_us) {
        sendError("RANGE", "SA");
        return;
      }
    } else if (values[ch] != cfg.stop_us) {
      sendError("INACTIVE", "SA");
      return;
    }
  }
  // SAFETY: no thrust without a host holding the port open.
  if (!Serial.dtr()) {
    sendError("NO_HOST", "SA");
    return;
  }
  // SAFETY: the ESCs may still be initializing; thrust waits for the hold.
  if (state == STATE_ARMING) {
    sendError("ARMING", "SA");
    return;
  }
  for (uint8_t ch = 0; ch < THRUSTER_COUNT; ch++) {
    if (channelActive(ch) && out_us[ch] != values[ch]) {
      writeChannel(ch, values[ch]);
    }
  }
  state = STATE_RUN;
  last_feed_ms = millis();
  sendOk("SA");
}

static void handleStop(size_t nfields) {
  if (nfields != 1) {
    sendError("FIELDS", "SP");
    return;
  }
  stopAllChannels();
  if (state != STATE_ARMING) {
    state = STATE_IDLE;
  }
  last_feed_ms = millis();
  sendOk("SP");
}

static void handleArm(char **fields, size_t nfields) {
  if (nfields != 2) {
    sendError("FIELDS", "AR");
    return;
  }
  uint32_t hold_ms;
  if (!parseUint(fields[1], &hold_ms)) {
    sendError("FORMAT", "AR");
    return;
  }
  if (hold_ms < ARM_HOLD_MIN_MS || hold_ms > ARM_HOLD_MAX_MS) {
    sendError("RANGE", "AR");
    return;
  }
  stopAllChannels();
  state = STATE_ARMING;
  arm_start_ms = millis();
  arm_hold_ms = hold_ms;
  sendOk("AR");
}

static void handleConfigure(char **fields, size_t nfields) {
  if (nfields != 7) {
    sendError("FIELDS", "CF");
    return;
  }
  Config next;
  if (!parseUint(fields[1], &next.rate_hz) || !parseUint(fields[2], &next.watchdog_ms) ||
      !parseHexByte(fields[3], &next.mask) || !parseUint(fields[4], &next.min_us) ||
      !parseUint(fields[5], &next.stop_us) || !parseUint(fields[6], &next.max_us)) {
    sendError("FORMAT", "CF");
    return;
  }
  bool ok = rateAllowed(next.rate_hz) &&
            next.watchdog_ms >= WATCHDOG_MIN_MS && next.watchdog_ms <= WATCHDOG_MAX_MS &&
            next.mask != 0 &&
            next.min_us >= PULSE_HARD_MIN_US && next.max_us <= PULSE_HARD_MAX_US &&
            next.min_us < next.stop_us && next.stop_us < next.max_us &&
            next.max_us + PERIOD_GUARD_US <= 1000000UL / next.rate_hz;
  if (!ok) {
    sendError("RANGE", "CF");
    return;
  }
  bool rate_changed = (next.rate_hz != cfg.rate_hz);
  cfg = next;
  if (rate_changed) {
    applyFrameRate();
  }
  // Rewrites every channel unconditionally, which also recomputes each duty
  // for the new rate.
  stopAllChannels();
  state = STATE_IDLE;
  cfg_received = true;
  sendOk("CF");
}

static void handleStatus(size_t nfields) {
  if (nfields != 1) {
    sendError("FIELDS", "ST");
    return;
  }
  uint32_t now = millis();
  uint32_t arm_left = 0;
  if (state == STATE_ARMING) {
    uint32_t elapsed = now - arm_start_ms;
    arm_left = (elapsed < arm_hold_ms) ? (arm_hold_ms - elapsed) : 0;
  }
  char body[MAX_FRAME_CHARS];
  int n = snprintf(body, sizeof(body), "ST,%s,%u,%u,%lu,%lu,%lu,%02X,%lu,%lu,%lu,%lu,%lu",
                   STATE_NAMES[state], cfg_received ? 1u : 0u, Serial.dtr() ? 1u : 0u,
                   (unsigned long)arm_left, (unsigned long)cfg.rate_hz,
                   (unsigned long)cfg.watchdog_ms, (unsigned)cfg.mask,
                   (unsigned long)cfg.min_us, (unsigned long)cfg.stop_us,
                   (unsigned long)cfg.max_us, (unsigned long)rx_errors,
                   (unsigned long)now);
  for (uint8_t ch = 0; ch < THRUSTER_COUNT && n > 0 && (size_t)n < sizeof(body); ch++) {
    n += snprintf(body + n, sizeof(body) - n, ",%lu", (unsigned long)out_us[ch]);
  }
  sendFrame(body);
}

// ---------------------------------------------------------------------------
// Servo commands (PROTOCOL.md 6.7-6.11). None of them feeds the thruster
// watchdog or changes the thruster state.
// ---------------------------------------------------------------------------

// Parse a servo index field. Replies FORMAT or RANGE and returns false if bad.
static bool parseServoIndex(const char *field, const char *cmd, uint8_t *out) {
  uint32_t n;
  if (!parseUint(field, &n)) {
    sendError("FORMAT", cmd);
    return false;
  }
  if (n >= SERVO_COUNT) {
    sendError("RANGE", cmd);
    return false;
  }
  *out = (uint8_t)n;
  return true;
}

// SC,<rate_hz>,<host_loss>: servo frame rate and DTR-loss behavior.
static void handleServoConfigure(char **fields, size_t nfields) {
  if (nfields != 3) {
    sendError("FIELDS", "SC");
    return;
  }
  uint32_t rate_hz;
  if (!parseUint(fields[1], &rate_hz)) {
    sendError("FORMAT", "SC");
    return;
  }
  bool hold;
  if (strcmp(fields[2], "HOLD") == 0) {
    hold = true;
  } else if (strcmp(fields[2], "LIMP") == 0) {
    hold = false;
  } else {
    sendError("FORMAT", "SC");
    return;
  }
  if (!servoRateAllowed(rate_hz)) {
    sendError("RANGE", "SC");
    return;
  }
  // Every servo's configured max must still fit the new frame.
  for (uint8_t n = 0; n < SERVO_COUNT; n++) {
    if (servo_max_us[n] + PERIOD_GUARD_US > 1000000UL / rate_hz) {
      sendError("RANGE", "SC");
      return;
    }
  }
  bool rate_changed = (rate_hz != servo_rate_hz);
  servo_rate_hz = rate_hz;
  servo_hold_on_host_loss = hold;
  if (rate_changed) {
    applyServoFrameRate();
    // Recompute each running servo's duty for the new rate; positions do not move.
    for (uint8_t n = 0; n < SERVO_COUNT; n++) {
      if (servo_out_us[n] != 0) {
        writeServo(n, servo_out_us[n]);
      }
    }
  }
  servo_cfg_received = true;
  sendOk("SC");
}

// SL,<n>,<min_us>,<max_us>: servo n's allowed pulse range.
static void handleServoLimits(char **fields, size_t nfields) {
  if (nfields != 4) {
    sendError("FIELDS", "SL");
    return;
  }
  uint8_t n;
  if (!parseServoIndex(fields[1], "SL", &n)) {
    return;
  }
  uint32_t min_us;
  uint32_t max_us;
  if (!parseUint(fields[2], &min_us) || !parseUint(fields[3], &max_us)) {
    sendError("FORMAT", "SL");
    return;
  }
  if (min_us < SERVO_HARD_MIN_US || max_us > SERVO_HARD_MAX_US || min_us >= max_us ||
      max_us + PERIOD_GUARD_US > 1000000UL / servo_rate_hz) {
    sendError("RANGE", "SL");
    return;
  }
  servo_min_us[n] = min_us;
  servo_max_us[n] = max_us;
  // SAFETY: a servo now outside its new range goes limp rather than being
  // moved to the nearest end -- moving it would be motion nobody commanded.
  if (servo_out_us[n] != 0 && (servo_out_us[n] < min_us || servo_out_us[n] > max_us)) {
    writeServo(n, 0);
  }
  sendOk("SL");
}

// SS,<n>,<pulse_us>: drive servo n.
static void handleServoSet(char **fields, size_t nfields) {
  if (nfields != 3) {
    sendError("FIELDS", "SS");
    return;
  }
  uint8_t n;
  if (!parseServoIndex(fields[1], "SS", &n)) {
    return;
  }
  uint32_t pulse_us;
  if (!parseUint(fields[2], &pulse_us)) {
    sendError("FORMAT", "SS");
    return;
  }
  // SAFETY: rejected, never clamped.
  if (pulse_us < servo_min_us[n] || pulse_us > servo_max_us[n]) {
    sendError("RANGE", "SS");
    return;
  }
  if (!Serial.dtr()) {
    sendError("NO_HOST", "SS");
    return;
  }
  if (servo_out_us[n] != pulse_us) {
    writeServo(n, pulse_us);
  }
  sendOk("SS");
}

// SO,<n>: servo n off (limp). Accepted in every state.
static void handleServoOff(char **fields, size_t nfields) {
  if (nfields != 2) {
    sendError("FIELDS", "SO");
    return;
  }
  uint8_t n;
  if (!parseServoIndex(fields[1], "SO", &n)) {
    return;
  }
  writeServo(n, 0);
  sendOk("SO");
}

// SV: servo status.
static void handleServoStatus(size_t nfields) {
  if (nfields != 1) {
    sendError("FIELDS", "SV");
    return;
  }
  char body[MAX_FRAME_CHARS];
  int n = snprintf(body, sizeof(body), "SV,%u,%lu,%s", servo_cfg_received ? 1u : 0u,
                   (unsigned long)servo_rate_hz, servo_hold_on_host_loss ? "HOLD" : "LIMP");
  for (uint8_t s = 0; s < SERVO_COUNT && n > 0 && (size_t)n < sizeof(body); s++) {
    n += snprintf(body + n, sizeof(body) - n, ",%lu,%lu,%lu", (unsigned long)servo_out_us[s],
                  (unsigned long)servo_min_us[s], (unsigned long)servo_max_us[s]);
  }
  sendFrame(body);
}

// ---------------------------------------------------------------------------
// Frame handling (PROTOCOL.md 3 and 7)
// ---------------------------------------------------------------------------

// Best-effort command name for an error reply: the first two characters when
// they are uppercase letters followed by ',' or '*', else "--".
static void commandName(const char *s, size_t len, char *out) {
  if (len >= 3 && s[0] >= 'A' && s[0] <= 'Z' && s[1] >= 'A' && s[1] <= 'Z' &&
      (s[2] == ',' || s[2] == '*')) {
    out[0] = s[0];
    out[1] = s[1];
    out[2] = '\0';
  } else {
    strcpy(out, "--");
  }
}

// s holds everything after '$' (body, '*', checksum), NUL-terminated.
static void handleFrame(char *s, size_t len) {
  char cmd[3];
  commandName(s, len, cmd);

  char *star = strrchr(s, '*');
  if (star == NULL || strlen(star + 1) != 2) {
    sendError("CHECKSUM", cmd);
    return;
  }
  uint8_t sent_sum;
  if (!parseHexByte(star + 1, &sent_sum) || sent_sum != checksumOf(s, star - s)) {
    sendError("CHECKSUM", cmd);
    return;
  }
  if (rx_bad_char) {
    sendError("FORMAT", cmd);
    return;
  }
  if (strcmp(cmd, "--") == 0) {
    sendError("FORMAT", "--");   // command is not two uppercase letters
    return;
  }
  *star = '\0';  // s is now the body alone

  // Split on commas. An empty field is a FORMAT error.
  char *fields[MAX_FIELDS];
  size_t nfields = 0;
  char *p = s;
  while (true) {
    if (nfields >= MAX_FIELDS) {
      sendError("FIELDS", cmd);
      return;
    }
    fields[nfields++] = p;
    char *comma = strchr(p, ',');
    if (comma == NULL) {
      break;
    }
    *comma = '\0';
    p = comma + 1;
  }
  for (size_t i = 0; i < nfields; i++) {
    if (fields[i][0] == '\0') {
      sendError("FORMAT", cmd);
      return;
    }
  }

  if (strcmp(cmd, "ID") == 0) {
    handleId(nfields);
  } else if (strcmp(cmd, "SA") == 0) {
    handleSetAll(fields, nfields);
  } else if (strcmp(cmd, "SP") == 0) {
    handleStop(nfields);
  } else if (strcmp(cmd, "AR") == 0) {
    handleArm(fields, nfields);
  } else if (strcmp(cmd, "CF") == 0) {
    handleConfigure(fields, nfields);
  } else if (strcmp(cmd, "ST") == 0) {
    handleStatus(nfields);
  } else if (strcmp(cmd, "SC") == 0) {
    handleServoConfigure(fields, nfields);
  } else if (strcmp(cmd, "SL") == 0) {
    handleServoLimits(fields, nfields);
  } else if (strcmp(cmd, "SS") == 0) {
    handleServoSet(fields, nfields);
  } else if (strcmp(cmd, "SO") == 0) {
    handleServoOff(fields, nfields);
  } else if (strcmp(cmd, "SV") == 0) {
    handleServoStatus(nfields);
  } else {
    sendError("UNKNOWN", cmd);
  }
}

static void resetReceiver() {
  rx_len = 0;
  rx_in_frame = false;
  rx_overflow = false;
  rx_bad_char = false;
}

// Assemble frames byte by byte. A frame split across USB packets, or several
// frames in one packet, both work: nothing happens until a '\n' arrives.
static void readSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      if (rx_overflow) {
        sendError("OVERFLOW", "--");
      } else if (rx_in_frame) {
        rx_buf[rx_len] = '\0';
        handleFrame(rx_buf, rx_len);
      }
      // A line with no '$' is ignored without a reply.
      resetReceiver();
    } else if (c == '\r' || rx_overflow) {
      // '\r' before '\n' is ignored; an overflowed line is discarded to '\n'.
    } else if (c == '$') {
      // Resynchronize: anything before the latest '$' is dropped.
      rx_len = 0;
      rx_in_frame = true;
      rx_bad_char = false;
    } else if (rx_in_frame) {
      if (rx_len >= MAX_AFTER_DOLLAR) {
        rx_overflow = true;
      } else {
        if (c <= ' ' || c > '~') {
          rx_bad_char = true;  // spaces and control characters are not allowed
        }
        rx_buf[rx_len++] = c;
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Timed safety checks (PROTOCOL.md 8 and 9)
// ---------------------------------------------------------------------------

static void checkHost() {
  bool dtr = Serial.dtr();
  if (dtr_prev && !dtr) {
    // SAFETY: host closed the port or died -> stop everything at once rather
    // than waiting out the watchdog. DTR falls only when the LAST host
    // process closes the port (several NEPI nodes share it).
    stopAllChannels();
    state = STATE_IDLE;
    // Servos hold by default (a gripper keeps its grip); LIMP releases them.
    if (!servo_hold_on_host_loss) {
      servosOff();
    }
  }
  dtr_prev = dtr;
}

static void checkWatchdog(uint32_t now) {
  // SAFETY: the only stop when the Pi, the node or the link dies. Unsigned
  // subtraction is correct across the millis() wrap.
  if (state == STATE_RUN && (now - last_feed_ms) >= cfg.watchdog_ms) {
    stopAllChannels();
    state = STATE_TRIPPED;
  }
}

static void checkArming(uint32_t now) {
  if (state == STATE_ARMING && (now - arm_start_ms) >= arm_hold_ms) {
    // Outputs are already at stop; they stay there until the next SA.
    state = STATE_IDLE;
    last_feed_ms = now;
  }
}

static void updateLed(uint32_t now) {
  bool on;
  if (!Serial.dtr()) {
    on = false;
  } else if (state == STATE_ARMING) {
    on = (now / 100) % 2;   // fast blink
  } else if (state == STATE_TRIPPED) {
    on = (now / 500) % 2;   // slow blink
  } else {
    on = true;
  }
  digitalWrite(LED_PIN, on ? HIGH : LOW);
}

// ---------------------------------------------------------------------------

void setup() {
  // SAFETY: these are the first instructions. Until here every pin is an
  // input (no pulses); from here every thruster channel pulses stop_us at
  // the default rate, with or without a host. Nothing below waits for USB.
  applyFrameRate();
  for (uint8_t ch = 0; ch < THRUSTER_COUNT; ch++) {
    out_us[ch] = 0;
  }
  stopAllChannels();

  // Servos: off (LOW, no pulses) until the host commands one (PROTOCOL.md 4.1).
  for (uint8_t n = 0; n < SERVO_COUNT; n++) {
    servo_min_us[n] = SERVO_DEFAULT_MIN_US;
    servo_max_us[n] = SERVO_DEFAULT_MAX_US;
  }
  applyServoFrameRate();
  servosOff();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // The baud rate is ignored for USB serial. Never `while (!Serial)`: the
  // firmware must run, pulsing stop, with no host attached.
  Serial.begin(115200);
  resetReceiver();
  dtr_prev = Serial.dtr();
  last_feed_ms = millis();
}

void loop() {
  readSerial();
  checkHost();
  uint32_t now = millis();
  checkArming(now);
  checkWatchdog(now);
  updateLed(now);
}
