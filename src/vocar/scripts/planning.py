#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, PoseArray, Twist
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Header

def planning_node():
    rospy.init_node('planning_node', anonymous=True)

    # Publishers for global trajectory, local trajectory, and twist control
    pub_global_trajectory = rospy.Publisher('global_trajectory', Path, queue_size=10)
    pub_local_trajectory = rospy.Publisher('local_trajectory', Path, queue_size=10)
    pub_twist_control = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # Subscribers to detected dynamic objects, occupancy grid map, current pose, and navigation target
    rospy.Subscriber('dynamic_objects', PoseArray, dynamic_objects_callback)
    rospy.Subscriber('occupancy_grid_map', OccupancyGrid, occupancy_grid_callback)
    rospy.Subscriber('current_pose', PoseStamped, current_pose_callback)
    rospy.Subscriber('target_nav_pose', PoseStamped, target_pose_callback)

    # Placeholder for storing inputs
    global detected_objects, occupancy_grid, current_pose, target_pose
    detected_objects = None
    occupancy_grid = None
    current_pose = None
    target_pose = None

    rospy.spin()  # Keep the node running

def dynamic_objects_callback(data):
    global detected_objects
    rospy.loginfo("Received dynamic objects")
    detected_objects = data
    process_planning()

def occupancy_grid_callback(data):
    global occupancy_grid
    rospy.loginfo("Received occupancy grid map")
    occupancy_grid = data
    process_planning()

def current_pose_callback(data):
    global current_pose
    rospy.loginfo("Received current pose")
    current_pose = data
    process_planning()

def target_pose_callback(data):
    global target_pose
    rospy.loginfo("Received target navigation pose")
    target_pose = data
    process_planning()

def process_planning():
    global detected_objects, occupancy_grid, current_pose, target_pose
    if detected_objects is None or occupancy_grid is None or current_pose is None or target_pose is None:
        return  # Wait until all necessary inputs are available

    # Placeholder for planning algorithms
    # This is where you'd implement the path planning and trajectory generation

    # Example of publishing a dummy global trajectory
    global_trajectory = Path()
    global_trajectory.header = Header()
    global_trajectory.header.stamp = rospy.Time.now()
    global_trajectory.header.frame_id = "map"
    
    # Add some dummy poses to the global trajectory
    # Replace this with the actual planned path
    for i in range(5):
        pose = PoseStamped()
        pose.header = global_trajectory.header
        pose.pose.position.x = current_pose.pose.position.x + i
        pose.pose.position.y = current_pose.pose.position.y + i
        global_trajectory.poses.append(pose)
    
    rospy.loginfo("Publishing global trajectory")
    pub_global_trajectory.publish(global_trajectory)

    # Example of publishing a dummy local trajectory
    local_trajectory = Path()
    local_trajectory.header = global_trajectory.header
    
    # Add some dummy poses to the local trajectory
    # Replace this with the actual refined local path
    for i in range(3):
        pose = PoseStamped()
        pose.header = local_trajectory.header
        pose.pose.position.x = current_pose.pose.position.x + 0.5 * i
        pose.pose.position.y = current_pose.pose.position.y + 0.5 * i
        local_trajectory.poses.append(pose)
    
    rospy.loginfo("Publishing local trajectory")
    pub_local_trajectory.publish(local_trajectory)

    # Example of publishing a dummy twist command
    twist = Twist()
    twist.linear.x = 0.5  # Replace with actual velocity commands from trajectory
    twist.angular.z = 0.1  # Replace with actual rotational velocity commands
    rospy.loginfo("Publishing twist control command")
    pub_twist_control.publish(twist)

if __name__ == '__main__':
    try:
        planning_node()
    except rospy.ROSInterruptException:
        pass
