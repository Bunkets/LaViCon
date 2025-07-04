#!/usr/bin/env python3
import rospy
import re
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from waypoints_manager.srv import ShortestPath, ShortestPathRequest
from geometry_msgs.msg import Twist
from enum import Enum
import time

#global variables
speed = 0

# for LLM
from transformers import AutoTokenizer, AutoModelForSequenceClassification
from peft import PeftModel
import torch
import torch.nn.functional as F

class RobotState(Enum):
    IDLE = 0
    STOPPED = 1
    FORWARD = 2
    BACKWARD = 3
    LEFT = 4
    RIGHT = 5
    TRAJ = 6
    FAST = 7
    SLOW = 8
    FAST2 = 9
    SLOW2 = 10

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

        # Load tokenizer and model
        self.tokenizer = AutoTokenizer.from_pretrained("/mnt/ros_ws/src/vocar/scripts/LLM")
        self.model = AutoModelForSequenceClassification.from_pretrained("/mnt/ros_ws/src/vocar/scripts/LLM")
        self.model.eval()

        # Energy thresholding for OOD detection
        self.energy_threshold = -3.7
        self.temperature = 0.1

        # Intent mapping
        self.id_to_intent = {
            0: "decrease_speed",
            1: "find_person",
            2: "go_to_door",
            3: "go_to_elevator",
            4: "go_to_kitchen",
            5: "go_to_parking",
            6: "go_to_restroom",
            7: "increase_speed",
            8: "move_backward",
            9: "move_forward",
            10: "stop",
            11: "turn_left",
            12: "turn_right"
        }

        # Predefined destinations
        self.destinations = {
            "kitchen": (8.88676452637, 0.01296043396, 0.0),
            "entrance": (10.490237236, 6.37075138092, 0.0),
            "parking": (3.83177304268, -0.190901398659, 0.0),
            "door": (5.0, 1.0, 0.0),
        }

        # Publishers
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.emergency_pub = rospy.Publisher('/front_obstacle_detect', Bool, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/voice_cmd_vel', Twist, queue_size=10)

        # Service client
        rospy.wait_for_service('/shortest_path_service')
        self.path_service = rospy.ServiceProxy('/shortest_path_service', ShortestPath)

        # Subscriber
        rospy.Subscriber('/recognized_command', String, self.command_callback)

        rospy.loginfo("GoToNode initialized. Listening for commands.")

        self.state = RobotState.IDLE
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_by_state)

        

    def compute_energy(self, logits, T):
        return -T * torch.logsumexp(logits / T, dim=-1)

    def predict_intent(self, text):
        inputs = self.tokenizer(text, return_tensors="pt", truncation=True, padding=True, max_length=64)
        with torch.no_grad():
            logits = self.model(**inputs).logits
            energy = self.compute_energy(logits, T=self.temperature).item()
            probs = F.softmax(logits, dim=-1)
            predicted_id = torch.argmax(probs, dim=1).item()

        if energy > self.energy_threshold:
            rospy.logwarn(f"Detected OOD input. Energy={energy:.2f} exceeds threshold.")
            return "NOT_A_COMMAND"
        return self.id_to_intent[predicted_id]

    def publish_by_state(self, event):
        twist = Twist()
        #rospy.loginfo(self.state)
        if self.state == RobotState.STOPPED:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            speed = 0.0
        elif self.state == RobotState.FORWARD:
            twist.linear.x = 0.1
            speed = 0.1
        elif self.state == RobotState.BACKWARD:
            twist.linear.x = -0.1
            speed = -0.1
        elif self.state == RobotState.LEFT:
            twist.angular.z = 0.15
        elif self.state == RobotState.RIGHT:
            twist.angular.z = -0.15
        elif self.state == RobotState.FAST:
            twist.linear.x = 0.5
        elif self.state == RobotState.SLOW:
         
            twist.linear.x = 0.05
        elif self.state == RobotState.FAST2:
           
            twist.linear.x = -0.5
        elif self.state == RobotState.SLOW2:
           
            twist.linear.x = -0.05
        else:
            return
        #rospy.loginfo(twist)
        self.cmd_vel_pub.publish(twist)

    def command_callback(self, msg):
        text = msg.data.strip()
        rospy.loginfo(f"Received command: {text}")

        intent = self.predict_intent(text)
        if intent == "NOT_A_COMMAND":
            rospy.logwarn("Filtered out unrelated or noisy input.")
            return

        rospy.loginfo(f"Predicted intent: {intent}")

        if self.go_to_door_pattern.search(text):
            self.emergency_pub.publish(False)
            self.handle_shortest_path("parking", "door")
            self.state = RobotState.TRAJ
            rospy.loginfo("Follow Traj, parking => door. State: TRAJ")
        elif self.go_to_parking_pattern.search(text):
            self.emergency_pub.publish(False)
            self.handle_shortest_path("door", "parking")
            self.state = RobotState.TRAJ
            rospy.loginfo("Follow Traj, door => parking. State: TRAJ")
        elif self.stop_pattern.search(text):
            self.state = RobotState.STOPPED
            self.emergency_pub.publish(True)
        elif self.forward_pattern.search(text):
            self.emergency_pub.publish(False)
            self.state = RobotState.FORWARD
        elif self.backward_pattern.search(text):
            self.emergency_pub.publish(False)
            self.state = RobotState.BACKWARD
        elif self.left_pattern.search(text):
            self.emergency_pub.publish(False)
            self.state = RobotState.LEFT
        elif self.right_pattern.search(text):
            self.emergency_pub.publish(False)
            self.state = RobotState.RIGHT



        elif intent == "go_to_door":
            self.emergency_pub.publish(False)
            self.handle_shortest_path("parking", "door")
            self.state = RobotState.TRAJ
        elif intent == "go_to_parking":
            self.emergency_pub.publish(False)
            self.handle_shortest_path("door", "parking")
            self.state = RobotState.TRAJ
        elif intent == "stop":
            self.state = RobotState.STOPPED
            self.emergency_pub.publish(True)
        elif intent == "move_forward":
            self.emergency_pub.publish(False)
            self.state = RobotState.FORWARD
        elif intent == "move_backward":
            self.emergency_pub.publish(False)
            self.state = RobotState.BACKWARD
        elif intent == "turn_left":
            self.emergency_pub.publish(False)
            self.state = RobotState.LEFT
        elif intent == "turn_right":
            self.emergency_pub.publish(False)
            self.state = RobotState.RIGHT
        elif intent == "increase_speed":
            if self.state == RobotState.FORWARD or self.state == RobotState.FAST:
                self.state = RobotState.FAST
                rospy.loginfo("Speed up")
            elif self.state == RobotState.BACKWARD or self.state == RobotState.FAST2:
                self.state = RobotState.FAST2
            else:
                rospy.logwarn("Can't increase speed unless already moving forward")
        elif intent == "decrease_speed":
            if self.state == RobotState.FORWARD:
                self.state = RobotState.SLOW
                rospy.loginfo("Slow down")
            elif self.state == RobotState.BACKWARD:
                self.state = RobotState.SLOW2
            else:
                rospy.logwarn("Can't decrease speed unless already moving forward")
        else:
            rospy.logwarn(f"Unrecognized intent: {intent}")

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
            goal.pose.orientation.z = 0.0
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
