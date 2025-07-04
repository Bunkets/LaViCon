#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import subprocess

def speak_callback(msg):
    text = msg.data
    rospy.loginfo(f"Speaking: {text}")
    try:
        subprocess.run(["espeak-ng", text])
    except Exception as e:
        rospy.logerr(f"TTS failed: {e}")

def tts_listener():
    rospy.init_node('tts_speaker_node')
    rospy.Subscriber('/tts', String, speak_callback)
    rospy.spin()

if __name__ == '__main__':
    tts_listener()
