# LaViCoN: Language and Vision Cooperative Navigation for Indoor Robots
<p align="center">
  <img width="24%" alt="intro_4" src="https://github.com/user-attachments/assets/54b34830-0616-48ac-bd4f-4323df2a9168" />
  <img width="24%" alt="intro_3" src="https://github.com/user-attachments/assets/ff9ffa72-1864-4943-be44-53dac1b38781" />
  <img width="24%" alt="intro_2" src="https://github.com/user-attachments/assets/7f9d26b0-a9ac-4c1b-b445-75160b732387" />
  <img width="24%" alt="intro_1" src="https://github.com/user-attachments/assets/7434c2b2-4c39-49b5-9724-e295ba3e532b" />
</p>

## Overview
LaViCon is a modular system that lets indoor robots understand natural language commands like "go to the chair" or "navigate between the two tables." Instead of using massive AI models that need tons of training data, we built a lightweight, interpretable system that breaks the task into clear components.

Key Results:
- 87.5% task success rate across 20 navigation tasks
- 81% robustness to different ways of phrasing commands
- Real-time performance on commodity hardware

## How It Works
<img width="1724" height="362" alt="download (6)" src="https://github.com/user-attachments/assets/6930d852-40ae-4057-9d12-c3bde9d37032" />


Voice → Intent: Vosk recognizes speech, then a custom JointBERT model extracts key information (destinations, obstacles, spatial relations)
Vision: Orbbec Astra RGB-D camera + YOLOv8 finds objects in the environment
Action: Reactive motion primitives execute the commands (approach, avoid, navigate between objects)

## Hardware

Main Computer: Intel i7 Linux machine (runs ROS master, voice processing)
Jetson AGX Xavier: NVIDIA edge device (runs YOLOv8)

Sensors:
- Orbbec Astra Pro (RGB-D camera)
- Velodyne LiDAR
- USB microphone
  
Robot: Four-wheel mobile platform (RobiX from Robify)

## Dependencies
- ROS1 Melodic/Noetic
- Standard ROS packages (you'll need to set up your own catkin workspace)
- Also need Docker
```
pip install torch torchvision
pip install ultralytics  # YOLOv8
pip install transformers  # BERT
pip install vosk  # Voice recognition
pip install sounddevice pyaudio
```
## Installation
### 1. Clone Repository
```
mkdir -p ~/vocar_ws/src
cd ~/vocar_ws/src
git clone https://github.com/Bunkets/LaViCon.git
```
### 2. Set Up ROS Workspace
```
cd ~/vocar_ws
catkin_make
source devel/setup.bash
```
Note: This repo contains the core scripts but some ROS packages need to be set up separately:

four_wheel: Robot base controller
astra_camera: Camera driver
waypoints_manager: Navigation waypoints
dashboard: Vehicle interface

You'll need to install these or equivalent packages for your robot platform.

### 3. Configure Network
Set these IPs to match your setup (default values shown):
Main Computer (192.168.0.134):
```
export ROS_IP=192.168.0.134
export ROS_MASTER_URI=http://192.168.0.134:11311
```
Jetson (192.168.0.114):
```
export ROS_IP=192.168.0.114
export ROS_MASTER_URI=http://192.168.0.134:11311
```
### 4. Download Models
Place fine-tuned JointBERT model in vocar/models/
Download YOLOv8 weights (yolov8n.pt)
Download Vosk model: vosk-model-small-en-us-0.15

## Usage

### On Main Computer:

### 1. Start ROS core:
```
roscore
```
### Launch robot base (new terminal):
```
roslaunch four_wheel four_wheel.launch
```
### Start voice recognition (new terminal):
```
source ~/vocar_ws/devel/setup.bash
cd ~/vocar_ws/src/vocar/scripts
rosrun vocar voice_recognition.py
```
### Start voice parser in Docker (new terminal):
```
sudo docker run -it --rm --name voiceparse --network=host \
    --device /dev/snd \
    -e PULSE_SERVER=unix:/run/user/1000/pulse/native \
    -v /run/user/1000/pulse:/run/user/1000/pulse \
    --volume="$HOME/vocar_ws/src:/mnt/ros_ws/src" \
    llm_v3:latest /bin/bash

# Inside container:
export ROS_IP=192.168.0.134
export ROS_MASTER_URI=http://192.168.0.134:11311
cd /mnt/ros_ws
source devel/setup.bash
cd src/vocar/scripts
python3 voice_parse5.py
```
### On Jetson (SSH zebra@192.168.0.114):

### 5. Start camera node:
```
export ROS_IP=192.168.0.114
export ROS_MASTER_URI=http://192.168.0.134:11311
cd /mnt/tony/camera_ws
source devel/setup.bash
roslaunch astra_camera astra_pro.launch
```

### 6. Start object detection bridge (new terminal):
(no_bridge.py is not provided in this repo, to create your own, use a YOLOv8 script running the Astra Orbbec Camera, then publish the RGBD and depth data so the main computer can recieve).
```
docker run -it --rm --net=host --gpus all \
    --platform linux/arm64 \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v /mnt/tony:/mnt \
    yolo:success

# Inside container:
source /opt/ros/melodic/setup.bash
export ROS_IP=192.168.0.114
export ROS_MASTER_URI=http://192.168.0.134:11311
cd /mnt/
python3 no_bridge.py
```

### Back on Main Computer:

### 7. Start object detection (new terminal):
```
docker run -it --rm --network=host \
    -v ~/sensors_ws/src/yolov8_node/scripts:/mnt/yolo_scripts \
    --name yolo_node yolo:gpu /bin/bash

# Inside container:
export ROS_IP=192.168.0.134
export ROS_MASTER_URI=http://192.168.0.134:11311
cd yolo_scripts
python3 object_detection.py
```

### Example Commands
Once everything is running, try:
- "Go to the chair"
- "Face the whiteboard"
- "Go around that table"
- "Navigate between the two trash cans"
- "Go to the desk then face the door" (multi-step)
### Shutdown
To stop everything cleanly, you can create a kill script or manually:
```
# Kill all ROS nodes
rosnode kill -a
killall -9 roscore rosmaster

# Stop Docker containers
docker stop voiceparse yolo_node
```

### Key Scripts:
- voice_recognition.py: Captures audio, uses Vosk for real-time speech-to-text, publishes to /recognized_command
- voice_parse5.py: Receives text commands, uses JointBERT to extract slots (destinations, objects, obstacles), publishes intents and reference objects, controls robot motion
- object_detection.py: Runs YOLOv8 on camera feed, publishes detections
- no_bridge.py: Bridges detection data between Jetson and main computer

## Contact
Tony Yang - tony08.morgan@gmail.com





