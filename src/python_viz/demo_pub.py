#!/usr/bin/env python3
import rospy
from std_msgs.msg import String


def publish_10hz(event, pub, counter):
    msg = String()
    msg.data = f"Hello from 10Hz publisher: {counter[0]}"
    pub.publish(msg)
    print(f"Published to 10Hz topic: {msg.data}")
    counter[0] += 1

def publish_20hz(event, pub, counter):
    msg = String()
    msg.data = f"Hello from 20Hz publisher: {counter[0]}"
    pub.publish(msg)
    print(f"Published to 20Hz topic: {msg.data}")
    counter[0] += 1

def main():
    rospy.init_node('demo_pub', anonymous=True)
    pub_10hz = rospy.Publisher('/topic_10hz', String, queue_size=1)
    pub_20hz = rospy.Publisher('/topic_20hz', String, queue_size=1)
    x = 0
    counter_10hz = [0]
    counter_20hz = [0]

    rospy.Timer(rospy.Duration(0.1), lambda event: publish_10hz(event, pub_10hz, counter_10hz))
    rospy.Timer(rospy.Duration(0.05), lambda event: publish_20hz(event, pub_20hz, counter_20hz))

    rospy.spin()

if __name__ == '__main__':
    main()