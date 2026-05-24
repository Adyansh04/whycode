#!/usr/bin/env python3
import math
import rospy
import tf
import os
import sys
import csv
import time

COLOR_1 = "\033[1;32m"  # green
COLOR_2 = "\033[1;36m"  # cyan
COLOR_ERR = "\033[1;31m"
RESET = "\033[0m"


def get_yaw_deg(listener, parent, child):
    """
    Return (timestamp_seconds, yaw_deg) for the transform parent->child.
    On TF errors returns (None, None). On TF_OLD_DATA triggers a process restart.
    """
    try:
        try:
            t_common = listener.getLatestCommonTime(parent, child)
        except Exception:
            t_common = rospy.Time(0)

        trans, rot = listener.lookupTransform(parent, child, rospy.Time(0))
        _, _, yaw = tf.transformations.euler_from_quaternion(rot)  # radians
        stamp = t_common.to_sec() if t_common != rospy.Time(0) else rospy.Time.now().to_sec()
        return (stamp, math.degrees(yaw))
    except Exception as e:
        err_str = str(e)
        if "TF_OLD_DATA" in err_str or "ignoring data from the past" in err_str:
            rospy.logwarn(f"{COLOR_ERR}TF_OLD_DATA detected for {parent}->{child}; restarting process to recover{RESET}")
            # brief pause to flush logs
            try:
                time.sleep(0.1)
            except Exception:
                pass
            os.execv(sys.executable, [sys.executable] + sys.argv)
        # other TF errors: return None to indicate missing value
        rospy.logwarn_throttle(1.0, f"{COLOR_ERR}TF lookup failed for {parent}->{child}: {e}{RESET}")
        return (None, None)


def ensure_csv_header(path):
    if not os.path.exists(path):
        try:
            with open(path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["timestamp", "april_yaw_deg", "whycode_yaw_deg"])
        except Exception as e:
            rospy.logwarn(f"{COLOR_ERR}Failed to create CSV {path}: {e}{RESET}")


def main():
    rospy.init_node("tf_euler_logger_csv")

    frame1_parent = rospy.get_param("~frame1_parent", "camera_link")
    frame1_child = rospy.get_param("~frame1_child", "april_triangulation")
    frame2_parent = rospy.get_param("~frame2_parent", "camera_link")
    frame2_child = rospy.get_param("~frame2_child", "whycode_triangulation")
    rate_hz = float(rospy.get_param("~rate", 10.0))

    output_csv = rospy.get_param("~output_csv",
                                "/root/ros1_ws/src/whycon_whycode_localization/logs/tf_yaw_log.csv")
    os.makedirs(os.path.dirname(output_csv), exist_ok=True)
    ensure_csv_header(output_csv)

    listener = tf.TransformListener()
    # Allow TF buffer to fill
    rospy.sleep(0.5)

    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        t1, yaw1 = get_yaw_deg(listener, frame1_parent, frame1_child)
        t2, yaw2 = get_yaw_deg(listener, frame2_parent, frame2_child)

        # choose timestamp: prefer common/latest available
        stamps = [s for s in (t1, t2) if s is not None]
        if stamps:
            ts = max(stamps)
        else:
            ts = rospy.Time.now().to_sec()

        # write row: empty string for missing values
        row = [f"{ts:.6f}", f"{yaw1:.6f}" if yaw1 is not None else "", f"{yaw2:.6f}" if yaw2 is not None else ""]

        try:
            with open(output_csv, "a", newline="") as f:
                w = csv.writer(f)
                w.writerow(row)
        except Exception as e:
            rospy.logwarn(f"{COLOR_ERR}Failed to write to CSV {output_csv}: {e}{RESET}")

        # also log a compact info line
        rospy.loginfo(f"{COLOR_1}[TF CSV] t={ts:.3f} april_yaw={row[1]} whycode_yaw={row[2]}{RESET}")

        rate.sleep()


if __name__ == "__main__":
    main()