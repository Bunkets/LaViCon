#!/usr/bin/env python3
import rospy
import re
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from waypoints_manager.srv import ShortestPath, ShortestPathRequest
from geometry_msgs.msg import Twist
from enum import Enum

class RobotState(Enum):
    IDLE = 0
    STOPPED = 1
    FORWARD = 2
    BACKWARD = 3
    LEFT = 4
    RIGHT = 5
    TRAJ = 6


class GoToNode:
    def __init__(self):
        rospy.init_node('go_to_node')

        # Regex patterns
        self.go_to_door_pattern = re.compile(r"\bdoor\b", re.IGNORECASE)
        self.go_to_parking_pattern = re.compile(r"\bparking\b", re.IGNORECASE)
        self.stop_pattern = re.compile(r"\bstop\b", re.IGNORECASE)
        self.go_pattern = re.compile(r"\bgo\b", re.IGNORECASE)
        self.forward_pattern = re.compile(r"\bforward\b", re.IGNORECASE)
        self.backward_pattern = re.compile(r"\bbackward\b", re.IGNORECASE)
        self.left_pattern = re.compile(r"\bleft\b", re.IGNORECASE)
        self.right_pattern = re.compile(r"\bright\b", re.IGNORECASE)


        # Predefined destinations
        self.destinations = {
            "kitchen": (8.88676452637, 0.01296043396, 0.0),
            "entrance": (10.490237236, 6.37075138092, 0.0),
            "parking": (3.83177304268, -0.190901398659, 0.0),
            "door": (5.0, 1.0, 0.0),  # Add if needed for goal publishing
        }

        # Publishers
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.emergency_pub = rospy.Publisher('/front_obstacle_detect', Bool, queue_size=10)

        # Service client
        rospy.wait_for_service('/shortest_path_service')
        self.path_service = rospy.ServiceProxy('/shortest_path_service', ShortestPath)

        # # Subscriber
        rospy.Subscriber('/recognized_command', String, self.command_callback)
        # rospy.Subscriber('/recognized_command', String, self.command_callback)

        rospy.loginfo("GoToNode initialized. Listening for commands.")

        self.state = RobotState.IDLE
        self.cmd_vel_pub = rospy.Publisher('/voice_cmd_vel', Twist, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_by_state)

    def publish_by_state(self, event):
        twist = Twist()

        if self.state == RobotState.STOPPED:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        elif self.state == RobotState.FORWARD:
            twist.linear.x = 0.1
        elif self.state == RobotState.BACKWARD:
            twist.linear.x = -0.1
        elif self.state == RobotState.LEFT:
            twist.angular.z = 0.15
        elif self.state == RobotState.RIGHT:
            twist.angular.z = -0.15
        else:
            return  # IDLE: do not publish

        self.cmd_vel_pub.publish(twist)

    def command_callback(self, msg):
        text = msg.data.strip().lower()
        rospy.loginfo(f"Received command: {text}")

        if self.go_to_door_pattern.search(text):
            self.handle_shortest_path("parking", "door")
            self.state = RobotState.TRAJ
            rospy.loginfo("Follow Traj, parking => door. State: TRAJ")
        elif self.go_to_parking_pattern.search(text):
            self.handle_shortest_path("door", "parking")
            self.state = RobotState.TRAJ
            rospy.loginfo("Follow Traj, door => parking. State: TRAJ")
        elif self.stop_pattern.search(text):
            self.state = RobotState.STOPPED
            self.emergency_pub.publish(True)
            rospy.loginfo("Emergency STOP. State: STOPPED")
        elif self.go_pattern.search(text):
            self.state = RobotState.IDLE
            self.emergency_pub.publish(False)
            rospy.loginfo("Resuming operation. State: IDLE")
        elif self.forward_pattern.search(text):
            self.state = RobotState.FORWARD
            rospy.loginfo("State changed to FORWARD")
        elif self.backward_pattern.search(text):
            self.state = RobotState.BACKWARD
            rospy.loginfo("State changed to BACKWARD")
        elif self.left_pattern.search(text):
            self.state = RobotState.LEFT
            rospy.loginfo("State changed to LEFT")
        elif self.right_pattern.search(text):
            self.state = RobotState.RIGHT
            rospy.loginfo("State changed to RIGHT")
            

    def command_callback2(self, msg):
        text = msg.data.strip().lower()
        rospy.loginfo(f"Received command: {text}")

        if text == "go to door":
            self.handle_shortest_path("parking", "door")
        elif text == "go to parking":
            self.handle_shortest_path("door", "parking")
        elif text == "stop":
            self.emergency_pub.publish(True)
            rospy.loginfo("Emergency stop issued.")
        elif text == "go":
            self.emergency_pub.publish(False)
            rospy.loginfo("Resuming operation.")

    def handle_go_to(self, destination):
        if destination in self.destinations:
            rospy.loginfo(f"Navigating to destination: {destination}")
            x, y, yaw = self.destinations[destination]

            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = rospy.Time.now()

            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.position.z = 0.0
            goal.pose.orientation.z = 0.0  # Placeholder; yaw-to-quaternion recommended
            goal.pose.orientation.w = 1.0

            self.goal_pub.publish(goal)
            rospy.loginfo(f"Goal to {destination} published: x={x}, y={y}, yaw={yaw}")
        else:
            rospy.logwarn(f"Unknown destination: {destination}. Available: {list(self.destinations.keys())}")

    def handle_shortest_path(self, start, end):
        try:
            rospy.loginfo(f"Calling /shortest_path_service with start='{start}', end='{end}'")
            req = ShortestPathRequest()
            req.start = start
            req.end = end
            response = self.path_service(req)

            if response.success:
                rospy.loginfo(f"Service succeeded: {response.message}, Distance: {response.distance:.2f} meters")
            else:
                rospy.logwarn(f"Service failed: {response.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = GoToNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("GoToNode terminated.")

