#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_drivers) repo
# (see https://https://github.com/nepi-engine/nepi_drivers)
#
# License: nepi applications are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment bstab.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com

##############################################################################
# NPX (Navigation / NavPose) DRIVER TEMPLATE -- NODE
# ---------------------------------------------------------------------------
# Registers an NPXDeviceIF. NPX is the simplest interface: the node's ONE job
# is to keep a NavPose dict up to date and hand it back through getNavPoseCb.
# NPXDeviceIF polls that callback at max_navpose_update_rate and publishes the
# standard NEPI navpose topics.
#
# The NavPose dict (nepi_nav.BLANK_NAVPOSE_DICT) carries independent, optional
# sub-reports, each gated by a has_* flag:
#   has_location    -> latitude, longitude           (WGS84 degrees)
#   has_heading     -> heading_deg                    (degrees true north)
#   has_altitude    -> altitude_m
#   has_orientation -> roll/pitch/yaw_deg             (ENU degrees)
#   has_position    -> x/y/z_m                        (local 3D frame)
#   has_depth       -> depth_m
#   has_pan_tilt    -> pan_deg, tilt_deg (+ pan_tilt_* pose)
# Each sub-report also carries a 'time_<name>' stamp and matching rate fields
# ('*_m_per_sec' / '*_deg_per_sec'). Set has_* True ONLY for what the sensor
# actually provides; leave the rest. Never enumerate the has_* flags by hand
# when clearing them -- iterate keys starting with 'has_', so a flag added to
# the dict later is not silently left stale (see _clear_navpose below).
#
# Always copy.deepcopy(BLANK_NAVPOSE_DICT). It is a module-level dict; mutating
# it in place corrupts it for every other consumer in the process.
#
# A navpose sensor with runtime-tunable settings also passes getSettingsFunction
# and setSettingFunction here -- NPXDeviceIF accepts both, and builds a
# SettingsIF only when both are non-None. Most navpose sensors have no settings
# and pass neither; that is the correct state, not an omission. See
# lsx_template_node.py for the full settings pattern.
#
# This template reads an NMEA-like stream over TCP in a background client
# thread. Swap the transport/parser for your device.
##############################################################################

import copy
import socket
import threading

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_nav          # BLANK_NAVPOSE_DICT

from nepi_api.messages_if import MsgIF
from nepi_api.device_if_npx import NPXDeviceIF

PKG_NAME = 'NPX_TEMPLATE'
FILE_TYPE = 'NODE'


class NpxTemplateNode:

    device_info_dict = dict(device_name="", path="", serial_number="",
                            hw_version="", sw_version="")

    navpose_update_rate = 20
    npx_if = None
    DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"

    ##########################################################################
    def __init__(self):
        nepi_sdk.init_node(name=self.DEFAULT_NODE_NAME)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        self.msg_if = MsgIF(log_name=self.class_name)
        self.msg_if.pub_info("Starting Node Initialization Processes")

        try:
            self.drv_dict = nepi_sdk.get_param('~drv_dict', dict())
            self.device_name = self.drv_dict['DEVICE_DICT']['device_name']
            self.device_path = self.drv_dict['DEVICE_DICT']['device_path']
            self.host = self.drv_dict['DEVICE_DICT']['tcp_host']
            self.port = int(self.drv_dict['DEVICE_DICT']['tcp_port'])
        except Exception as e:
            self.msg_if.pub_warn("Failed to load Device Dict " + str(e))
            nepi_sdk.signal_shutdown(self.node_name + ": no valid Device Dict")
            return

        # ---- Shared navpose state, guarded by a lock because the reader thread
        #      writes it and NPXDeviceIF reads it from a different thread.
        self.navpose_lock = threading.Lock()
        self.driver_navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        # Pre-declare what this sensor CAN report, before the reader connects,
        # so NPXDeviceIF creates the right publishers even when the device is
        # unreachable at startup. TODO: set only the flags your sensor provides.
        self.driver_navpose_dict['has_location'] = True
        self.driver_navpose_dict['has_heading'] = True
        self.driver_navpose_dict['has_altitude'] = True

        self.device_info_dict["device_name"] = self.device_name
        self.device_info_dict["path"] = self.device_path
        self.device_info_dict["serial_number"] = ""
        self.device_info_dict["hw_version"] = ""
        self.device_info_dict["sw_version"] = ""

        # ---- Register with NEPI. NPXDeviceIF will call getNavPoseCb on a timer.
        #      capSettings/settings args are optional -- omit if the sensor has
        #      no runtime-tunable settings (see lsx_template for that pattern).
        # The frame declarations (navpose_frame='sensor_frame', frame_nav='ENU',
        # frame_altitude='WGS84', frame_depth) all have defaults on the IF; pass
        # one only when your sensor genuinely reports in a different frame.
        # getSettingsFunction/setSettingFunction are also accepted here -- omit
        # both for a sensor with no runtime-tunable settings.
        self.npx_if = NPXDeviceIF(
            device_info=self.device_info_dict,
            data_source_description="navpose_sensor",
            data_ref_description="sensor_center",
            getNavPoseCb=self.getNavPoseCb,
            max_navpose_update_rate=self.navpose_update_rate,
            msg_if=self.msg_if,
        )

        # ---- Start the hardware reader AFTER the IF, so the IF's init-time
        #      getNavPoseCb snapshot always sees the pre-declared capabilities
        #      rather than a half-filled dict. (A plain client that connects OUT
        #      to the configured endpoint and keeps driver_navpose_dict fresh.)
        self._stop_evt = threading.Event()
        self._reader = threading.Thread(target=self._tcp_loop, daemon=True)
        self._reader.start()

        nepi_sdk.on_shutdown(self.cleanup_actions)
        self.msg_if.pub_info("Initialization complete")
        nepi_sdk.spin()

    ##########################################################################
    # THE NPX CALLBACK: hand NPXDeviceIF a snapshot of the current NavPose.
    ##########################################################################
    def getNavPoseCb(self):
        with self.navpose_lock:
            return copy.deepcopy(self.driver_navpose_dict)

    ##########################################################################
    # HARDWARE READER  (TODO: replace transport + parser for your device)
    ##########################################################################
    def _tcp_loop(self):
        while not self._stop_evt.is_set() and not nepi_sdk.is_shutdown():
            try:
                sock = socket.create_connection((self.host, self.port), timeout=3.0)
                stream = sock.makefile('r')
                self.msg_if.pub_info("Connected to %s:%d" % (self.host, self.port))
                for line in stream:
                    if self._stop_evt.is_set():
                        break
                    line = line.strip()
                    if line.startswith('$'):
                        self._handle_sentence(line)
            except Exception as e:
                # pub_warn (not pub_debug) so connection loss is visible without Debug Mode
                self.msg_if.pub_warn("Reader connection error: " + str(e))
                self._clear_navpose()
                nepi_sdk.sleep(1.0, 10)
            finally:
                try:
                    sock.close()
                except Exception:
                    pass

    def _handle_sentence(self, line):
        # TODO: parse your device's protocol. Example: a GGA-like fix line
        # "$GGA,<lat>,<lon>,<alt>" and a heading line "$HDT,<heading>".
        parts = line.split(',')
        try:
            with self.navpose_lock:
                t = nepi_utils.get_time()
                if parts[0].endswith('GGA') and len(parts) >= 4:
                    self.driver_navpose_dict['has_location'] = True
                    self.driver_navpose_dict['time_location'] = t
                    self.driver_navpose_dict['latitude'] = float(parts[1])
                    self.driver_navpose_dict['longitude'] = float(parts[2])
                    self.driver_navpose_dict['has_altitude'] = True
                    self.driver_navpose_dict['time_altitude'] = t
                    self.driver_navpose_dict['altitude_m'] = float(parts[3])
                elif parts[0].endswith('HDT') and len(parts) >= 2:
                    self.driver_navpose_dict['has_heading'] = True
                    self.driver_navpose_dict['time_heading'] = t
                    self.driver_navpose_dict['heading_deg'] = float(parts[1])
        except (ValueError, IndexError):
            pass  # ignore malformed lines

    def _clear_navpose(self):
        # On disconnect, mark everything stale so downstream consumers know.
        # Iterate the has_* keys rather than listing them: the dict has gained
        # sub-reports (has_pan_tilt) since this template was first written, and
        # a hardcoded list leaves any newer flag stuck True after a disconnect.
        with self.navpose_lock:
            for k in self.driver_navpose_dict.keys():
                if k.startswith('has_'):
                    self.driver_navpose_dict[k] = False

    def cleanup_actions(self):
        self._stop_evt.set()


if __name__ == '__main__':
    node = NpxTemplateNode()
