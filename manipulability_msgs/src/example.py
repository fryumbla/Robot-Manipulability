#!/usr/bin/env python

import numpy as np
import random
import rospy
from manipulability_msgs.msg import manipulability

class VisionSystemManager:
    def set(self):
        self.manipulability_msg.header.stamp = rospy.Time.now()
        self.manipulability_msg.header.frame_id = 'manipulability'




        self.manipulability_msg.pose.position.x = 1.0 + random.uniform(0, 0.25)
        self.manipulability_msg.pose.position.y = 1.0 + random.uniform(0, 0.5)
        self.manipulability_msg.pose.position.z = 1.0 + random.uniform(0, 0.3)
        self.manipulability_msg.pose.orientation.w = np.pi/4.0 + random.uniform(-np.pi/8.0, np.pi/8.0)
        self.manipulability_msg.pose.orientation.x = 10.1
        self.manipulability_msg.pose.orientation.y = 0.2
        self.manipulability_msg.pose.orientation.z = 0.3

        self.manipulability_msg.joint_values = [0,1,2,3,4,5]
        self.manipulability_msg.manipulability = 1.111

    def __init__(self):
        rospy.init_node("manipulability_index")
        self.r = rospy.Rate(100)

        self.manipulability_msg = manipulability()
        self.state_pub = rospy.Publisher('/manipulability_data', manipulability, queue_size=1)

    def loop(self):
        self.set()

        self.state_pub.publish(self.manipulability_msg)
        self.r.sleep()

if __name__ == '__main__':
        vision_system_manager = VisionSystemManager()
        while not rospy.is_shutdown():
            vision_system_manager.loop()