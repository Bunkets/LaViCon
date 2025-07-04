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
sys.path.append(os.path.join(os.path.dirname(__file__), 'JointBERT_v2'))

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
        self.between_objs = [] 
    
        # Initialize pyttsx3
        self.tts = pyttsx3.init()
        self.tts.setProperty('rate', 150)

        # === Load Tokenizer ===
        self.tokenizer = BertTokenizer.from_pretrained("bert-base-uncased")

        self.stop_pattern = re.compile(r"\bstop\b", re.IGNORECASE)

        # === Load intent and slot labels ===
        intent_label_path = os.path.join(os.path.dirname(__file__), "JointBERT_v2/data/wheelchair/intent_label.txt")
        slot_label_path = os.path.join(os.path.dirname(__file__), "JointBERT_v2/data/wheelchair/slot_label.txt")
        
        with open(intent_label_path) as f:
            self.intent_label_lst = [line.strip() for line in f]

        with open(slot_label_path) as f:
            self.slot_label_lst = [line.strip() for line in f]

        # Intent label map
        self.id_to_intent = {
            i: label for i, label in enumerate(self.intent_label_lst)
        }

        # === Dummy args object ===
        class Args:
            def __init__(self):
                self.use_crf = False
                self.dropout_rate = 0.1
                self.slot_loss_coef = 1.0
                self.ignore_index = -100

        args = Args()

        # === Load Model ===
        model_dir = os.path.join(os.path.dirname(__file__), "JointBERT_v2/outputs/wheelchair")
        config = BertConfig.from_pretrained(model_dir)
        self.model = JointBERT(config=config, args=args,
                               intent_label_lst=self.intent_label_lst,
                               slot_label_lst=self.slot_label_lst)
        self.model.load_state_dict(torch.load(os.path.join(model_dir, "pytorch_model.bin"),
                                              map_location=torch.device("cpu")))
        self.model.to("cuda" if torch.cuda.is_available() else "cpu")
        self.model.eval()


        # Static destinations
        self.destinations = {
            "kitchen": (8.8868, 0.0130, 0.0),
            "entrance": (10.4902, 6.3708, 0.0),
            "parking": (3.8318, -0.1909, 0.0),
            "door": (5.0, 1.0, 0.0),
        }
        self.cancel_requested = False      # ← new


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
        rospy.Subscriber('/task_status', Bool, self.status_callback)

        rospy.loginfo("GoToNode initialized. Listening for voice commands.")

        self.state = RobotState.IDLE
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_by_state)
        self.task_complete = True
    
    def extract_slot_values(self, tokens, labels):
        ordered_slots = []
        current_slot = None
        current_value = []

        for token, label in zip(tokens, labels):
            if label.startswith("B-"):
                if current_slot:
                    ordered_slots.append((current_slot, " ".join(current_value)))
                current_slot = label[2:]
                current_value = [token]
            elif label.startswith("I-") and current_slot == label[2:]:
                current_value.append(token)
            else:
                if current_slot:
                    ordered_slots.append((current_slot, " ".join(current_value)))
                    current_slot = None
                    current_value = []

        if current_slot:
            ordered_slots.append((current_slot, " ".join(current_value)))

        return ordered_slots

 

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

        probs = F.softmax(intent_logits, dim=-1)
        predicted_id = torch.argmax(probs, dim=1).item()
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

        slot_sequence = self.extract_slot_values(clean_tokens, clean_labels)

        return predicted_intent, slot_sequence

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

    def status_callback(self, msg):
        if msg.data:
            self.task_complete = True
        

    def command_callback(self, msg):
        text = msg.data.strip()
        rospy.loginfo(f"Received command: {text}")

        if self.stop_pattern.search(text):
            rospy.loginfo("STOP command received – cancelling current task.")
            self.cancel_requested = True         # signal any waiting loop
            self.task_complete   = True          # pretend task is done
            self.state           = RobotState.STOPPED
            self.emergency_pub.publish(True)
            self.intent_pub.publish("")          # clear downstream intent
            return                 

        _, slot_sequence = self.predict_intent(text)

        for slot, value in slot_sequence:
            if (self.task_complete):      
                rospy.loginfo(f"Slot: {slot}, Value: {value}")
                # Route by slot type
                if slot == "Object_destination":
                    self.task_complete = False
                    rospy.loginfo(f"MOVING TO {value}")
                    self.reference_object_pub.publish(value)
                    self.intent_pub.publish("go_to")
                    while (not self.task_complete) and (not self.cancel_requested) and (not rospy.is_shutdown()):
                        rospy.sleep(0.1)
                    self.cancel_requested = False          # clear flag for next task

                elif slot == "Obstacle":
                    self.task_complete = False
                    rospy.loginfo(f"AVOIDING {value}")
                    self.reference_object_pub.publish(value)
                    self.intent_pub.publish("obstacle")
                    while (not self.task_complete) and (not self.cancel_requested) and (not rospy.is_shutdown()):
                        rospy.sleep(0.1)
                    self.cancel_requested = False 
                elif slot == "Object_face":
                    self.task_complete = False
                    rospy.loginfo(f"FACING {value}")
                    self.reference_object_pub.publish(value)
                    self.intent_pub.publish("face")
                    while (not self.task_complete) and (not self.cancel_requested) and (not rospy.is_shutdown()):
                        rospy.sleep(0.1)
                    self.cancel_requested = False 
                elif slot == "Between_2":
                    if value not in self.between_objs:
                        self.between_objs.append(value)

                    # Once we have two, launch the “go-between” task
                    if len(self.between_objs) == 2 and self.task_complete:
                        left, right        = self.between_objs[0], self.between_objs[1]
                        rospy.loginfo(f"GOING BETWEEN '{left}' and '{right}'")

                        self.task_complete = False
                        # send both labels to the perception node; one simple way is
                        # a comma-separated string  →  “chair,table”
                        self.reference_object_pub.publish(f"{left},{right}")

                        self.intent_pub.publish("between")     # <- YoloTracker looks for this
                        self.between_objs = []                 # reset buffer for next time

                        while (not self.task_complete) and (not self.cancel_requested) and \
                            (not rospy.is_shutdown()):
                            rospy.sleep(0.1)

                        self.cancel_requested = False

                    
                elif slot == "Object_follow":
                    self.task_complete = False
                    self.reference_object_pub.publish(value)
                    self.intent_pub.publish("follow")
                    rospy.loginfo(f"FOLLOWING {value}")
                    while (not self.task_complete) and (not self.cancel_requested) and (not rospy.is_shutdown()):
                        rospy.sleep(0.1)
                    self.cancel_requested = False 
                elif slot == "step_marker":
                    rospy.loginfo(f"[AND]")
                    continue  # Used only for separation
                else:
                    rospy.logwarn(f"Unhandled slot type: {slot}")

        

     

        # if intent == "adjust_position":
        #     # TODO: decode slots (for now, mock)
        #     direction = slots.get("Direction")
        #     reference_object = slots.get("Reference")
        #     distance = slots.get("Distance")

        #     rospy.loginfo(f"Extracted slots: {slots}")

        #     if reference_object:
        #         rospy.loginfo(f"Publishing reference object: {reference_object}")
        #         self.reference_object_pub.publish(reference_object)
        #         self.tts_pub.publish(f"moving to the {reference_object}")
                

        #     if direction == "forward":
        #         self.state = RobotState.FORWARD
        #         self.tts_pub.publish("moving forward now")

        #     elif direction == "backward":
        #         self.state = RobotState.BACKWARD
        #         self.tts_pub.publish("moving backward now")

    

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
