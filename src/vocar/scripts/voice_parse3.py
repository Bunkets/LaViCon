#!/usr/bin/env python3

import rospy
import re
import os
import time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from waypoints_manager.srv import ShortestPath, ShortestPathRequest
from geometry_msgs.msg import Twist
from enum import Enum
import openai

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

class GoToNode:
    def __init__(self):
        rospy.init_node('go_to_node')

        # OpenAI client setup
        openai.api_key = ""
        self.system_prompt = """You are an intent classification assistant for a robot. Given a voice command, your job is to return the correct intent label. Only return the intent, nothing else. However, it is possible that the user input was not intended as a robot voice command. Thus if the voice command does not match any of the intents, output \"NONE\" instead.

Valid intents:
- go_to_door
- stop
- increase_speed
- decrease_speed
- move_backward
- turn_left
- turn_right
- go_to_parking
- move_forward
- go_to_kitchen
- go_to_restroom
- go_to_elavator
- find_person

Examples:

Command: \"uh go to the door now\"
Intent: go_to_door

Command: \"stop moving\"
Intent: stop

Command: \"speed up a bit\"
Intent: increase_speed

Command: \"take it easy you're going too fast\"
Intent: decrease_speed

Command: \"back up slowly\"
Intent: move_backward

Command: \"uh turn left here\"
Intent: turn_left

Command: \"swing right real quick\"
Intent: turn_right

Command: \"go chill in your parking spot\"
Intent: go_to_parking

Command: \"move ahead\"
Intent: move_forward

Please make indirect generalizations and inferences if possible. For example, \"let's go outside\" could indicate go_to_door."""

        # Regex patterns
        self.go_to_door_pattern = re.compile(r"\bdoor\b", re.IGNORECASE)
        self.go_to_parking_pattern = re.compile(r"\bparking\b", re.IGNORECASE)
        self.stop_pattern = re.compile(r"\bstop\b", re.IGNORECASE)
        self.go_pattern = re.compile(r"\bgo\b", re.IGNORECASE)
        self.forward_pattern = re.compile(r"\bforward\b", re.IGNORECASE)
        self.backward_pattern = re.compile(r"\bbackward\b", re.IGNORECASE)
        self.left_pattern = re.compile(r"\bleft\b", re.IGNORECASE)
        self.right_pattern = re.compile(r"\bright\b", re.IGNORECASE)

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
        #rospy.wait_for_service('/shortest_path_service')
        #self.path_service = rospy.ServiceProxy('/shortest_path_service', ShortestPath)

        # Subscriber
        rospy.Subscriber('/recognized_command', String, self.command_callback)

        rospy.loginfo("GoToNode initialized. Listening for commands.")
        self.state = RobotState.IDLE
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_by_state)

    def query_intent_llm(self, text):
        response = openai.ChatCompletion.create(
            model="gpt-4o",
            messages=[
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": f"Command: \"{text}\"\nIntent:"}
            ],
            temperature=0.5,
            max_tokens=10
        )
        return response['choices'][0]['message']['content'].strip().lower()


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
        elif self.state == RobotState.FAST:
            twist.linear.x = 1.01
        elif self.state == RobotState.SLOW:
            twist.linear.x = 0.05
        else:
            return
        self.cmd_vel_pub.publish(twist)

    


        #with open("latency_records.txt") as f:
        #    start = float(f.read())
        #end = time.time()
        #latency = end - start
        #print(f"Latency: {latency:.3f} seconds")

    def command_callback(self, msg):
        text = msg.data.strip()
        rospy.loginfo(f"Received command: {text}")
        s = time.time()
        intent = self.query_intent_llm(text)
        if intent == "none":
            rospy.logwarn("Filtered out unrelated or noisy input.")
            return

        rospy.loginfo(f"Predicted intent: {intent}")
        

        if intent == "go_to_door":
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
            if self.state == RobotState.FORWARD:
                self.state = RobotState.FAST
                rospy.loginfo("Speed up")
            else:
                rospy.logwarn("Can't increase speed unless already moving forward")
        elif intent == "decrease_speed":
            if self.state == RobotState.FORWARD:
                self.state = RobotState.SLOW
                rospy.loginfo("Slow down")
            else:
                rospy.logwarn("Can't decrease speed unless already moving forward")
        else:
            rospy.logwarn(f"Unrecognized intent: {intent}")

        with open("latency_records.txt") as f:
            start = float(f.read())
        end = time.time()
        latency = end - start
        print(f"Latency: {latency:.3f} seconds")
        print(f"Latency: {time.time() - s}")
        with open("latency_logs.txt", "a") as f:
            f.write(f"{text} -> {latency:.3f}s\n")


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
