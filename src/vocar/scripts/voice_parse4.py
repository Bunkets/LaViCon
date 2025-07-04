#!/usr/bin/env python3
import rospy
import re
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from waypoints_manager.srv import ShortestPath, ShortestPathRequest
from geometry_msgs.msg import Twist
from enum import Enum
import time
import sys
import os
import pyttsx3

# Add JointBERT to path
sys.path.append(os.path.join(os.path.dirname(__file__), 'JointBERT'))

# Torch + Hugging Face
import torch
import torch.nn.functional as F
from transformers import BertTokenizer, BertConfig
from model.modeling_jointbert import JointBERT

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

        # Initialize pyttsx3
        self.tts = pyttsx3.init()
        self.tts.setProperty('rate', 150)

        # === Load Tokenizer ===
        self.tokenizer = BertTokenizer.from_pretrained("bert-base-uncased")

        self.stop_pattern = re.compile(r"\bstop\b", re.IGNORECASE)

        # === Load intent and slot labels ===
        intent_label_path = os.path.join(os.path.dirname(__file__), "JointBERT/data/wheelchair/intent_label.txt")
        slot_label_path = os.path.join(os.path.dirname(__file__), "JointBERT/data/wheelchair/slot_label.txt")

        with open(intent_label_path) as f:
            self.intent_label_lst = [line.strip() for line in f]

        with open(slot_label_path) as f:
            self.slot_label_lst = [line.strip() for line in f]

        # === Dummy args object ===
        class Args:
            def __init__(self):
                self.use_crf = False
                self.dropout_rate = 0.1
                self.slot_loss_coef = 1.0
                self.ignore_index = -100

        args = Args()

        # === Load Model ===
        model_dir = os.path.join(os.path.dirname(__file__), "JointBERT/outputs/wheelchair")
        config = BertConfig.from_pretrained(model_dir)
        self.model = JointBERT(config=config, args=args,
                               intent_label_lst=self.intent_label_lst,
                               slot_label_lst=self.slot_label_lst)
        self.model.load_state_dict(torch.load(os.path.join(model_dir, "pytorch_model.bin"),
                                              map_location=torch.device("cpu")))
        self.model.to("cuda" if torch.cuda.is_available() else "cpu")
        self.model.eval()

        # Energy-based OOD detection
        self.energy_threshold = -6.5
        self.temperature = 1.0

        # Intent label map
        self.id_to_intent = {
            i: label for i, label in enumerate(self.intent_label_lst)
        }

        # Static destinations
        self.destinations = {
            "kitchen": (8.8868, 0.0130, 0.0),
            "entrance": (10.4902, 6.3708, 0.0),
            "parking": (3.8318, -0.1909, 0.0),
            "door": (5.0, 1.0, 0.0),
        }

        # ROS publishers/subscribers/services
        self.reference_object_pub = rospy.Publisher('/reference_object', String, queue_size=10)
        self.intent_pub = rospy.Publisher('/intent', String, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.emergency_pub = rospy.Publisher('/front_obstacle_detect', Bool, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/voice_cmd_vel', Twist, queue_size=10)
        self.tts_pub = rospy.Publisher('/tts', String, queue_size=15)

        rospy.wait_for_service('/shortest_path_service')
        self.path_service = rospy.ServiceProxy('/shortest_path_service', ShortestPath)

        rospy.Subscriber('/recognized_command', String, self.command_callback)

        rospy.loginfo("GoToNode initialized. Listening for voice commands.")

        self.state = RobotState.IDLE
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_by_state)
    
    def extract_slot_values(self, tokens, labels):
        slots = {}
        current_slot = None
        current_value = []

        for token, label in zip(tokens, labels):
            if label.startswith("B-"):
                if current_slot:
                    slots[current_slot] = " ".join(current_value)
                current_slot = label[2:]
                current_value = [token]
            elif label.startswith("I-") and current_slot == label[2:]:
                current_value.append(token)
            else:
                if current_slot:
                    slots[current_slot] = " ".join(current_value)
                current_slot = None
                current_value = []

        if current_slot:
            slots[current_slot] = " ".join(current_value)

        return slots

    def compute_energy(self, logits, T=1.0):
        return -T * torch.logsumexp(logits / T, dim=-1)

    def predict_intent(self, text):
        inputs = self.tokenizer(text, return_tensors="pt", padding=True, truncation=True, max_length=64)
        inputs = {k: v.to(self.model.device) for k, v in inputs.items()}

        dummy_intents = torch.zeros(inputs["input_ids"].size(0), dtype=torch.long).to(self.model.device)
        dummy_slots = torch.zeros_like(inputs["input_ids"]).to(self.model.device)

        with torch.no_grad():
            _, (intent_logits, slot_logits) = self.model(
                input_ids=inputs["input_ids"],
                attention_mask=inputs["attention_mask"],
                token_type_ids=inputs.get("token_type_ids", None),
                intent_label_ids=dummy_intents,
                slot_labels_ids=dummy_slots
            )

        energy = self.compute_energy(intent_logits, self.temperature).item()
        probs = F.softmax(intent_logits, dim=-1)
        predicted_id = torch.argmax(probs, dim=1).item()
        rospy.loginfo(f"Energy: {energy:.2f}")
        if energy > self.energy_threshold:
            rospy.logwarn(f"Detected OOD input. Energy={energy:.2f} exceeds threshold.")
            return "NOT_A_COMMAND", None

        predicted_intent = self.id_to_intent[predicted_id]

         # --- Slot prediction ---
        slot_preds = torch.argmax(slot_logits, dim=2).squeeze(0).tolist()
        slot_labels = [self.slot_label_lst[i] for i in slot_preds]

        # Get tokens aligned to input IDs
        input_ids = inputs["input_ids"][0]
        tokens = self.tokenizer.convert_ids_to_tokens(input_ids)

        # Log raw slots for inspection
        rospy.loginfo("==== Raw Slot Predictions ====")
        for token, label in zip(tokens, slot_labels):
            rospy.loginfo(f"{token} -> {label}")
        rospy.loginfo("==============================")

        # Remove special tokens ([CLS], [SEP]) before extracting
        clean_tokens = []
        clean_labels = []
        for token, label in zip(tokens, slot_labels):
            if token not in ["[CLS]", "[SEP]", "[PAD]"]:
                clean_tokens.append(token)
                clean_labels.append(label)

        slot_values = self.extract_slot_values(clean_tokens, clean_labels)

        return predicted_intent, slot_values

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

    def command_callback(self, msg):
        text = msg.data.strip()
        rospy.loginfo(f"Received command: {text}")

        if self.stop_pattern.search(text):
            self.state = RobotState.STOPPED
            self.emergency_pub.publish(True)

        intent, slots = self.predict_intent(text)

        
        if intent == "NOT_A_COMMAND":
            rospy.logwarn("Filtered out unrelated or noisy input.")
            return

        rospy.loginfo(f"Predicted intent: {intent}")
        self.intent_pub.publish(intent)
        if intent == "adjust_position":
            # TODO: decode slots (for now, mock)
            direction = slots.get("Direction")
            reference_object = slots.get("Reference")
            distance = slots.get("Distance")

            rospy.loginfo(f"Extracted slots: {slots}")

            if reference_object:
                rospy.loginfo(f"Publishing reference object: {reference_object}")
                self.reference_object_pub.publish(reference_object)
                self.tts_pub.publish(f"moving to the {reference_object}")
                

            if direction == "forward":
                self.state = RobotState.FORWARD
                self.tts_pub.publish("moving forward now")

            elif direction == "backward":
                self.state = RobotState.BACKWARD
                self.tts_pub.publish("moving backward now")

        # elif intent == "navigate_destination":

        elif intent == "adjust_orientation":
            direction = slots.get("Direction")
            reference_object = slots.get("Reference")
            rospy.loginfo(f"Extracted slots: {slots}")
            if reference_object:
                rospy.loginfo(f"Publishing reference object: {reference_object}")
                self.reference_object_pub.publish(reference_object)
                self.tts_pub.publish(f"turning to the {reference_object}")
               

            if direction == "left":
                self.state = RobotState.LEFT
                self.tts_pub.publish("turning left")
        
            elif direction == "right":
                self.state = RobotState.RIGHT
                self.tts_pub.publish("turning right")
            
        
        elif intent == "follow_target":
            target = slots.get("Target")
            rospy.loginfo(f"Publishing reference object: {target}")
            self.reference_object_pub.publish("person")
            self.tts_pub.publish(f"engaging following mode")
          

        
        # elif intent == "escort_person"

        # elif intent == "go_to_door":
        #     self.handle_go_to("door")
        # elif intent == "go_to_parking":
        #     self.handle_go_to("parking")
        # elif intent == "stop":
        #     self.state = RobotState.STOPPED
        #     self.emergency_pub.publish(True)
        # elif intent == "move_forward":
        #     self.emergency_pub.publish(False)
        #     self.state = RobotState.FORWARD
        # elif intent == "move_backward":
        #     self.emergency_pub.publish(False)
        #     self.state = RobotState.BACKWARD
        # elif intent == "turn_left":
        #     self.emergency_pub.publish(False)
        #     self.state = RobotState.LEFT
        # elif intent == "turn_right":
        #     self.emergency_pub.publish(False)
        #     self.state = RobotState.RIGHT
        # elif intent == "increase_speed":
        #     if self.state == RobotState.FORWARD:
        #         self.state = RobotState.FAST
        #         rospy.loginfo("Speed up")
        #     else:
        #         rospy.logwarn("Can't increase speed unless already moving forward")
        # elif intent == "decrease_speed":
        #     if self.state == RobotState.FORWARD:
        #         self.state = RobotState.SLOW
        #         rospy.loginfo("Slow down")
        #     else:
        #         rospy.logwarn("Can't decrease speed unless already moving forward")
        else:
            rospy.logwarn(f"Unrecognized or unimplemented intent: {intent}")

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
        else:
            rospy.logwarn(f"Unknown destination: {destination}")

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
