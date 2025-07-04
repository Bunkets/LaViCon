#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
import sys
import select
import termios
import tty

class EmergencyStopNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('emergency_stop_node')

        # Create a publisher for the /emergency_stop topic
        self.pub = rospy.Publisher('/emergency_stop', Bool, queue_size=1)

        # Set the initial state to False (not safe to move)
        self.is_safe_to_move = False

        # Create a 20Hz rate for publishing
        self.rate = rospy.Rate(20)  # 20 Hz

        # Set up the terminal for non-blocking input
        self.old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        # Variable to track user input for the safety sentence
        self.sentence_buffer = ""

        rospy.loginfo("Emergency stop is active now.")
        rospy.loginfo("Press the space key 3 times to release the emergency stop.")


    def publish_loop(self):
        try:
            while not rospy.is_shutdown():
                # Publish the current state to the topic
                self.pub.publish(self.is_safe_to_move)

                # If not safe to move, check for sentence input
                if not self.is_safe_to_move:
                    self.check_for_sentence()

                # If safe to move, check for keypress to stop
                if self.is_safe_to_move:
                    self.check_for_keypress()

                # Sleep for the rate (20Hz)
                self.rate.sleep()

        finally:
            # Reset the terminal back to its original state on exit
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_attr)

    def check_for_sentence(self):
        # Check if there's any key press available
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
            char = sys.stdin.read(1)  # Read one character

            # Append the character to the sentence buffer
            self.sentence_buffer += char

            # Check if the entered sentence matches "it is safe to move"
            if self.sentence_buffer.endswith("   "): # 3 spaces
                rospy.loginfo("Emergency stop released. The vehicle can now move.")
                rospy.loginfo("Press any key to trigger emergency sotp.")
                self.is_safe_to_move = True
                self.sentence_buffer = ""  # Reset buffer after successful entry

    def check_for_keypress(self):
        # Check if there's any key press available
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
            key = sys.stdin.read(1)  # Read one character
            if key:
                rospy.loginfo("Key pressed, emergency stop is triggered.")
                rospy.loginfo("Press the space key 3 times to release the emergency stop.")
                self.is_safe_to_move = False

if __name__ == '__main__':
    try:
        # Create an instance of the EmergencyStopNode and start the publishing loop
        node = EmergencyStopNode()
        node.publish_loop()
    except rospy.ROSInterruptException:
        pass
