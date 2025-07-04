#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class MultiSourceCmdVel:
    def __init__(self):
        rospy.init_node('multi_source_cmd_vel', anonymous=True)

        self.pathfollow_msg = None  # Tuple: (timestamp, message)
        self.voice_msg = None

        # Define individual expiration windows
        self.pathfollow_max_age = rospy.Duration(0.1)  # e.g., 1 second
        self.voice_max_age = rospy.Duration(0.1)       # e.g., 0.5 seconds

        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.obstacle_detected_front = False
        self.obstacle_detected_left = False
        self.obstacle_detected_right = False
        self.obstacle_detected_back = False

        print("control center start")

        rospy.Subscriber('/front_obstacle_detect', Bool, self.obstacle_status_front)
        rospy.Subscriber('/left_obstacle_detect', Bool, self.obstacle_status_left)
        rospy.Subscriber('/right_obstacle_detect', Bool, self.obstacle_status_right)
        rospy.Subscriber('/back_obstacle_detect', Bool, self.obstacle_status_back)

        rospy.Subscriber('pathfollow_cmd_vel', Twist, self.pathfollow_callback)
        rospy.Subscriber('voice_cmd_vel', Twist, self.voice_callback)

    def obstacle_status_front(self, msg):
        self.obstacle_detected_front = msg.data

    def obstacle_status_left(self, msg):
        self.obstacle_detected_left = msg.data

    def obstacle_status_right(self, msg):
        self.obstacle_detected_right = msg.data

    def obstacle_status_back(self, msg):
        self.obstacle_detected_back = msg.data

    def publish_cmd_vel(self, twist):
        if twist.linear.x > 0.0001:
            if self.obstacle_detected_front:
                twist.linear.x = 0.0002

        if twist.linear.x < -0.0001:
            if self.obstacle_detected_back:
                twist.linear.x = -0.0002

        if twist.angular.z >= 0.0001:
            if self.obstacle_detected_left:
                twist.angular.z = 0.0001

        if twist.angular.z < -0.0001:
            if self.obstacle_detected_right:
                twist.angular.z = -0.0002

        self.cmd_pub.publish(twist)

    def pathfollow_callback(self, msg):
        self.pathfollow_msg = (rospy.Time.now(), msg)
        print("pathfollow_msg")
        #print(self.pathfollow_msg)
        self.evaluate_and_publish()
        
        # path_time, msg = self.pathfollow_msg
        # self.publish_cmd_vel(msg)

    def voice_callback(self, msg):
        self.voice_msg = (rospy.Time.now(), msg)
        print("voice_msg")
        #print(self.voice_msg)
        self.evaluate_and_publish()
        # voice_time, msg = self.voice_msg
        # self.publish_cmd_vel(msg)

    def is_valid(self, timestamp, max_age):
        return timestamp and ((rospy.Time.now() - timestamp) < max_age)

    def evaluate_and_publish(self):
        path_time, path_msg = self.pathfollow_msg if self.pathfollow_msg else (None, None)
        voice_time, voice_msg = self.voice_msg if self.voice_msg else (None, None)

        # print(path_time)
        # print(voice_time)
        # print(rospy.Time.now() - voice_time)

        path_valid = self.is_valid(path_time, self.pathfollow_max_age)
        voice_valid = self.is_valid(voice_time, self.voice_max_age)

        if voice_valid:
            self.publish_cmd_vel(voice_msg)
        elif path_valid:
            self.publish_cmd_vel(path_msg)
        # Else: Neither valid, do nothing

if __name__ == '__main__':
    try:
        MultiSourceCmdVel()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
