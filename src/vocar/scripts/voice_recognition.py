#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
import pyaudio
import json
from vosk import Model, KaldiRecognizer
import re
import time

class VoiceCommandProcessor:
    def __init__(self):
        # Initialize the ROS node
        
        rospy.init_node('voice_command_processor', anonymous=True)

        # Publisher to the emergency_stop topic
        self.command_pub = rospy.Publisher('/recognized_command', String, queue_size=10)

        # Service client for the shortest_path_service
        # rospy.wait_for_service('/shortest_path_service')
        #self.shortest_path_service = rospy.ServiceProxy('/shortest_path_service', ShortestPath)

        # Load Vosk model for speech recognition
        model_path = "/home/zkwl/vocar_ws/src/vocar/models/vosk-model-small-en-us-0.15"  # Replace with the correct path to your Vosk model
        self.model = Model(model_path=model_path)
        self.rec = KaldiRecognizer(self.model, 16000)
        # Initialize PyAudio for audio input
        self.p = pyaudio.PyAudio()
        self.list_input_devices()
        device_index = self.find_device_index_by_name()
        # print(device_index)
        self.stream = self.p.open(format=pyaudio.paInt16,
                          channels=1,
                          rate=16000,
                          input=True,
                          input_device_index=device_index,  # THIS MUST BE HERE
                          frames_per_buffer=4096)
        self.stream.start_stream()
        

        self.rate = rospy.Rate(1000)
        
        # # Cooldown tracking
        self.last_publish_time = 0
        self.cooldown_sec = 1.0
        self.last_text = None

    def find_device_index_by_name(self, keyword="TONOR"):
        for i in range(self.p.get_device_count()):
            info = self.p.get_device_info_by_index(i)
            if keyword.lower() in info['name'].lower() and info['maxInputChannels'] > 0:
                print(f"Selected device: [{i}] {info['name']}")
                return i
        raise RuntimeError(f"No input device found with keyword '{keyword}'")

    def list_input_devices(self):
        for i in range(self.p.get_device_count()):
            info = self.p.get_device_info_by_index(i)
            if info['maxInputChannels'] > 0:
                print(f"[{i}] {info['name']} - {int(info['defaultSampleRate'])} Hz - {info['maxInputChannels']} channels")


    def find_valid_input_device(self):
        for i in range(self.p.get_device_count()):
            info = self.p.get_device_info_by_index(i)
            if info['maxInputChannels'] > 0:
                rospy.loginfo(f"Using input device: {info['name']} (Index {i})")
                return i
        raise RuntimeError("No valid input device found.")

    def process_voice_commands(self):
        try:
            while not rospy.is_shutdown() and self.stream.is_active():
                start_time = time.time()

            

                data = self.stream.read(4096)
                asr_start = time.time()

                if self.rec.AcceptWaveform(data):
                    asr_done = time.time()
                    res = json.loads(self.rec.Result())
                    recognized_text = res.get('text', '').strip()

                    if recognized_text:
                        now = time.time()

                        # Only publish if not duplicate and cooldown passed
                        if recognized_text != self.last_text and (now - self.last_publish_time > self.cooldown_sec):
                            self.command_pub.publish(recognized_text)

                            with open("/home/zkwl/vocar_ws/src/vocar/scripts/latency_records.txt", "w") as f:
                                f.write(str(time.time()))

                            rospy.loginfo(f"FINAL Recognized: {recognized_text}")
                            #rospy.loginfo(f"ASR latency: {asr_done - asr_start:.3f} sec")
                            #rospy.loginfo(f"Total latency (mic → publish): {now - start_time:.3f} sec")
                self.rate.sleep()

        except rospy.ROSInterruptException:
            pass
        finally:
            self.cleanup()



    def process_recognized_text2(self, text):
        # Define regex patterns for each case
        stop_pattern = re.compile(r"\bstop\b", re.IGNORECASE)
        keep_going_pattern = re.compile(r"\bkeep going\b", re.IGNORECASE)
        go_to_pattern = re.compile(r"\bgo to (?P<destination>.+)", re.IGNORECASE)  # Captures 'go to' and the destination
        spin_pattern = re.compile(r"\bspin\b", re.IGNORECASE) 
        forward_pattern = re.compile(r"\bforward\b", re.IGNORECASE) 

        # Case 1: "stop" -> publish True to /emergency_stop
        if stop_pattern.search(text):
            rospy.loginfo("Emergency stop command recognized.")
            self.emergency_stop_pub.publish(True)
            rospy.loginfo("stop published.")

        # Case 2: "keep going" -> publish False to /emergency_stop
        elif keep_going_pattern.search(text):
            rospy.loginfo("Keep going command recognized.")
            self.emergency_stop_pub.publish(False)

        # Case 3: "go to <destination>" -> extract destination and call /shortest_path_service
        # elif match := go_to_pattern.search(text):
        #     destination = match.group("destination").strip()  # Extracts the destination from the pattern
        #     start = "parking"  # You can dynamically set this
        #     rospy.loginfo(f"Calling shortest path service from {start} to {destination}")
        #     #self.call_shortest_path_service(start, destination)

    def process_recognized_text(self, text):
        # Case 1: "stop" -> publish True to /emergency_stop
        if "stop" in text:
            rospy.loginfo("Emergency stop command recognized.")
            self.emergency_stop_pub.publish(True)
        
        # Case 2: "keep going" -> publish False to /emergency_stop
        elif "keep going" in text:
            rospy.loginfo("Keep going command recognized.")
            self.emergency_stop_pub.publish(False)

        # Case 3: "go to kitchen" -> call /shortest_path_service
        elif "go to kitchen" in text:
            start = "testend"  # You can dynamically set this
            end = "Kitchen"  # The destination is "kitchen"
            rospy.loginfo(f"Calling shortest path service from {start} to {end}")
            self.call_shortest_path_service(start, end)

    def call_shortest_path_service(self, start, end):
        try:
            response = self.shortest_path_service(start, end)
            if response.success:
                rospy.loginfo(f"Shortest path found: {response.message}, Distance: {response.distance}")
            else:
                rospy.logwarn(f"Service call failed: {response.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to /shortest_path_service failed: {e}")

    def cleanup(self):
        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()

if __name__ == '__main__':
    processor = VoiceCommandProcessor()
    processor.process_voice_commands()

