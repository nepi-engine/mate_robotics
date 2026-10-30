#!/usr/bin/env python3
#
# Measure how fresh and how fast the mate_rov_link driver's data is, from inside
# the NEPI container:
#   - Teensy line interval, from the resets of tn_age_ms in <node>/link_status
#   - how often <node>/npx/navpose/depth carries a NEW value, against how often
#     it publishes at all (the NPX republishes the last navpose between lines)
#
# Usage (in the NEPI container, ROS and nepi_engine sourced):
#   python3 sample_link_rates.py [node_namespace] [seconds]
#   default node_namespace /nepi/device1/mate_rov_link_robot, default 12 s
#
# From the laptop, without copying the file onto the device:
#   ssh nepihost@<nepi ip> 'docker exec -i <container> bash -c \
#     "source /opt/ros/noetic/setup.bash; source /opt/nepi/nepi_engine/setup.bash; python3 - <ns> 60"' \
#     < sample_link_rates.py

import json
import sys
import time

import rospy
from std_msgs.msg import String
from nepi_interfaces.msg import NavPoseDepth

NODE = '/nepi/device1/mate_rov_link_robot'
SECONDS = 12.0


def percentile(values, p):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(round(p / 100.0 * (len(ordered) - 1))))]


def main():
    node = sys.argv[1] if len(sys.argv) > 1 else NODE
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else SECONDS
    ages = []    # (receive time, tn_age_ms)
    depths = []  # (receive time, depth_m, timestamp)

    def statusCb(msg):
        try:
            ages.append((time.time(), json.loads(msg.data).get('tn_age_ms')))
        except Exception:
            pass

    def depthCb(msg):
        depths.append((time.time(), msg.depth_m, msg.timestamp))

    rospy.init_node('sample_link_rates', anonymous = True)
    rospy.Subscriber(node + '/link_status', String, statusCb)
    rospy.Subscriber(node + '/npx/navpose/depth', NavPoseDepth, depthCb)
    time.sleep(seconds)

    valid = [(t, a) for t, a in ages if a is not None and a >= 0]
    print('window: %.0f s, node: %s' % (seconds, node))
    print('link_status msgs: %d' % len(ages))
    if valid:
        age_values = [a for _, a in valid]
        print('tn_age_ms: min %d  p50 %d  p95 %d  max %d' % (min(age_values), percentile(age_values, 50),
                                                              percentile(age_values, 95), max(age_values)))
        # A new Teensy line shows as tn_age_ms dropping; its arrival = receive time - age
        arrivals = [t - a / 1000.0 for (t, a), (t0, a0) in zip(valid[1:], valid[:-1]) if a < a0]
        gaps = [b - a for a, b in zip(arrivals[:-1], arrivals[1:])]
        print('new Teensy lines seen: %d (%.2f per s)' % (len(arrivals), len(arrivals) / seconds))
        if gaps:
            print('Teensy line interval s: mean %.3f  min %.3f  max %.3f' % (sum(gaps) / len(gaps), min(gaps), max(gaps)))
    else:
        print('no tn_age_ms yet: is the ROV Pi sending status?')
    changes = sum(1 for p, q in zip(depths[:-1], depths[1:]) if p[1] != q[1])
    print('depth msgs: %d (%.1f per s), value changes: %d' % (len(depths), len(depths) / seconds, changes))
    if depths:
        values = [d for _, d, _ in depths]
        print('depth_m: min %.4f  max %.4f  distinct %d' % (min(values), max(values), len(set(values))))
        print('distinct depth timestamps: %d (float32 seconds, 128 s steps)' % len(set(s for _, _, s in depths)))


if __name__ == '__main__':
    main()
