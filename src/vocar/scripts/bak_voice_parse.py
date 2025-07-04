#!/usr/bin/env python3
import rospy
import re
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class GoToNode:
    def __init__(self):
        rospy.init_node('go_to_node')

        # Define regex pattern for 'go to' command
        self.go_to_pattern = re.compile(r"\bgo to (?P<destination>.+)", re.IGNORECASE)

        # Predefined destinations
        self.destinations = {
            "kitchen": (8.88676452637, 0.01296043396, 0.0),  # Example: x, y, yaw
            "entrance": (10.490237236, 6.37075138092, 0.0),
            "parking": (3.83177304268, -0.190901398659, 0.0),
        }

        # Publisher to send navigation goals
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        # Subscriber to receive commands
        rospy.Subscriber('/recognized_command', String, self.command_callback)

        rospy.loginfo("GoToNode initialized. Listening for 'go to' commands.")

    def command_callback(self, msg):
        text = msg.data
        match = self.go_to_pattern.search(text)
        if match:
            destination = match.group('destination').strip().lower()
            self.handle_go_to(destination)

    def handle_go_to(self, destination):
        if destination in self.destinations:
            rospy.loginfo(f"Navigating to destination: {destination}")
            x, y, yaw = self.destinations[destination]

            # Create and publish navigation goal
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = rospy.Time.now()

            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.position.z = 0.0
            goal.pose.orientation.z = 0.0  # Simplified; real yaw-to-quaternion conversion needed
            goal.pose.orientation.w = 1.0

            self.goal_pub.publish(goal)
            rospy.loginfo(f"Navigation goal to {destination} published: x={x}, y={y}, yaw={yaw}")
        else:
            rospy.logwarn(f"Unknown destination: {destination}. Available destinations: {list(self.destinations.keys())}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = GoToNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("GoToNode terminated.")
