#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import time
import threading

class FpsCounter:
    def __init__(self, label):
        self.label = label
        self.count = 0
        self.start_time = time.time()

    def tick(self):
        self.count += 1
        elapsed = time.time() - self.start_time
        if elapsed >= 1.0:
            print(f"[{self.label}] FPS: {self.count}")
            self.count = 0
            self.start_time = time.time()

# Shared variable and lock for thread safety
shared_data = {"value": 0}
# shared_lock = threading.Lock()

fps_10hz = FpsCounter("10Hz topic")
fps_20hz = FpsCounter("20Hz topic")

def callback_10hz(msg):
    print(f"[10Hz] Callback start (Thread ID: {threading.get_ident()})")
    # with shared_lock:
    shared_data["value"] += 1
    print(f"[10Hz] Shared value updated to: {shared_data['value']}")
    time.sleep(0.5)  # Simulate slow processing
    fps_10hz.tick()
    print(f"[10Hz] Callback end (Thread ID: {threading.get_ident()})")

def callback_20hz(msg):
    print(f"[20Hz] Callback start (Thread ID: {threading.get_ident()})")
    # with shared_lock:
    val = shared_data["value"]
    print(f"[20Hz] Shared value read as: {val}")
    fps_20hz.tick()
    print(f"[20Hz] Callback end (Thread ID: {threading.get_ident()})")

def main():
    rospy.init_node('demo_sub', anonymous=True)
    rospy.Subscriber('/topic_10hz', String, callback_10hz)
    rospy.Subscriber('/topic_20hz', String, callback_20hz)
    rospy.spin()

if __name__ == '__main__':
    main()