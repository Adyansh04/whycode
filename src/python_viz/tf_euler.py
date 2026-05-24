#!/usr/bin/env python3
import math
import rospy
import tf

COLOR_1 = "\033[1;32m"  # green
COLOR_2 = "\033[1;36m"  # cyan
COLOR_ERR = "\033[1;31m"
RESET = "\033[0m"

def lookup_and_log(listener, parent, child, color, tag):
    try:
        # Latest common time for stamp (optional)
        try:
            t_common = listener.getLatestCommonTime(parent, child)
        except Exception:
            t_common = rospy.Time(0)

        trans, rot = listener.lookupTransform(parent, child, rospy.Time(0))
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)  # radians

        stamp_str = f"{t_common.to_sec():.3f}" if t_common != rospy.Time(0) else "now"
        rospy.loginfo(
            f"{color}[{tag}] {parent}->{child} | "
            f"q(x,y,z,w)=({rot[0]:.4f}, {rot[1]:.4f}, {rot[2]:.4f}, {rot[3]:.4f}) | "
            f"RPY(deg)=({math.degrees(roll):.2f}, {math.degrees(pitch):.2f}, {math.degrees(yaw):.2f}) "
            f"@ {stamp_str}{RESET}"
        )
    except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logwarn_throttle(1.0, f"{COLOR_ERR}[{tag}] TF lookup failed for {parent}->{child}: {e}{RESET}")

def main():
    rospy.init_node("tf_euler_logger")

    frame1_parent = rospy.get_param("~frame1_parent", "camera_link")
    frame1_child  = rospy.get_param("~frame1_child",  "april_marker_center")
    frame2_parent = rospy.get_param("~frame2_parent", "camera_link")
    frame2_child  = rospy.get_param("~frame2_child",  "whycode_marker_center")
    rate_hz       = float(rospy.get_param("~rate", 10.0))

    listener = tf.TransformListener()
    # Allow TF buffer to fill
    rospy.sleep(0.5)

    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        lookup_and_log(listener, frame1_parent, frame1_child, COLOR_1, "TF1")
        lookup_and_log(listener, frame2_parent, frame2_child, COLOR_2, "TF2")
        rate.sleep()

if __name__ == "__main__":
    main()