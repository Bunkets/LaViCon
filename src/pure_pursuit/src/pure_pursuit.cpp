/* A ROS implementation of the Pure pursuit path tracking algorithm (Coulter 1992).
   Terminology (mostly :) follows:
   Coulter, Implementation of the pure pursuit algoritm, 1992 and 
   Sorniotti et al. Path tracking for Automated Driving, 2017.
*/

#include <string>
#include <iostream>
#include <cmath>
#include <algorithm>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include "visualization_msgs/Marker.h"
#include <kdl/frames.hpp>
#include <waypoints_manager/PathOfPosesWithVelocity.h>

using std::string;

class PurePursuit{
  public:
    PurePursuit();
    // Generate the command for the vehicle according to the current position and the waypoints
    void velocityCommandGenerator(geometry_msgs::PoseStamped newPose);
    
    //void velocityCommandGeneratorTransform(nav_msgs::Odometry odometry);

    // Listen to the waypoints topic
    //void receivePath(nav_msgs::Path path);
    void receivePath(waypoints_manager::PathOfPosesWithVelocity path);

    void obstacle_status(std_msgs::Bool status);

    void emergency_stop_status(std_msgs::Bool status);
    // Transform the pose to the base_link
    KDL::Frame transformPoseToBaseLink(const geometry_msgs::Pose& pose, const geometry_msgs::Transform& transform);
    // Eucledian distance computation; 2D
    template<typename T1, typename T2>
    double distance(T1 pointA, T2 pointB) {
      //return sqrt(pow(pointA.x - pointB.x, 2) + pow(pointA.y - pointB.y, 2) + pow(pointA.z - pointB.z, 2));
      return sqrt(pow(pointA.x - pointB.x, 2) + pow(pointA.y - pointB.y, 2));
    }
    // Ros_spin.
    void run();
    
  private:
    void generateNoVelocityCommandTwist();
    void generateNoVelocityCommandAckermannDrive();
    void publishLookaheadMarker(geometry_msgs::TransformStamped toMapTransform);
    
    // Parameters
    double wheelBase,
           lookaheadDistance, positionTolerance,
           MaximumLinearVelocity, linearVelocity, pathPoseVelocity,
           maximumAngularVelocity, steeringAngleVelocity, maximumSteeringAngle,
           acceleration, jerk;
    int idxMmemory, cmdVelRate;
    unsigned idx;
    bool isGoalReached, isPathLoaded, isObstacleDetected, canMove;
    string cmdVelTopic;
    
    //nav_msgs::Path path;
    waypoints_manager::PathOfPosesWithVelocity path;
    geometry_msgs::Twist twist;
    ackermann_msgs::AckermannDriveStamped ackermannDrive;
    visualization_msgs::Marker lookaheadMarker;
    
    // ROS
    ros::NodeHandle nodeHandle, privateNodeHandle;
    // poseSubscrier gets the pose from the (NDT) localizer
    ros::Subscriber odometrySubscriber, pathSubscriber, poseSubscriber, obstacleSubscriber, emergencyStopSubscriber ;
    ros::Publisher twistPublisher, ackermannDrivePublisher, lookaheadMarkerPublisher;
    tf2_ros::Buffer transformBuffer;
    tf2_ros::TransformListener transformListener;
    tf2_ros::TransformBroadcaster transformBroadcaster;
    geometry_msgs::TransformStamped lookaheadTransform, robotToMapTransform;
    string mapFrameId, robotFrameId, lookaheadFrameId, ackermannFrameId;
};

PurePursuit::PurePursuit() : lookaheadDistance(1.0), maximumAngularVelocity(1.0), positionTolerance(0.1), idx(0),
                             isGoalReached(false), privateNodeHandle("~"), transformListener(transformBuffer),
                             mapFrameId("map"), robotFrameId("base_link"), lookaheadFrameId("lookahead") {
  // Retrieve parameters from the parameter server
  privateNodeHandle.getParam("wheelbase", wheelBase);
  privateNodeHandle.getParam("lookahead_distance", lookaheadDistance);
  privateNodeHandle.getParam("max_linear_velocity", MaximumLinearVelocity);
  privateNodeHandle.getParam("max_rotational_velocity", maximumAngularVelocity);
  privateNodeHandle.getParam("position_tolerance", positionTolerance);
  privateNodeHandle.getParam("steering_angle_velocity", steeringAngleVelocity);
  privateNodeHandle.getParam("acceleration", acceleration);
  privateNodeHandle.getParam("jerk", jerk);
  privateNodeHandle.getParam("steering_angle_limit", maximumSteeringAngle);
  privateNodeHandle.getParam("map_frame_id", mapFrameId);
  privateNodeHandle.getParam("robot_frame_id", robotFrameId);
  privateNodeHandle.getParam("lookahead_frame_id", lookaheadFrameId);
  privateNodeHandle.getParam("ackermann_frame_id", ackermannFrameId);
  privateNodeHandle.getParam("cmd_vel_topic", cmdVelTopic);
  privateNodeHandle.getParam("cmd_vel_rate", cmdVelRate);
  linearVelocity = MaximumLinearVelocity;
  pathPoseVelocity = 0;
  isObstacleDetected = false;
  canMove = false;

  lookaheadTransform.header.frame_id = robotFrameId;
  lookaheadTransform.child_frame_id = lookaheadFrameId;

  ackermannDrive.header.frame_id = ackermannFrameId;
  ackermannDrive.drive.steering_angle_velocity = steeringAngleVelocity;
  ackermannDrive.drive.acceleration = acceleration;
  ackermannDrive.drive.jerk = jerk;

  idxMmemory = 0;
  isPathLoaded = false;
  
  pathSubscriber = nodeHandle.subscribe("/waypoints", 1, &PurePursuit::receivePath, this);
  //odometrySubscriber = nodeHandle.subscribe("/odom", 1, &PurePursuit::velocityCommandGeneratorTransform, this);
  poseSubscriber = nodeHandle.subscribe("/ndt_pose", 1, &PurePursuit::velocityCommandGenerator, this); 
  obstacleSubscriber = nodeHandle.subscribe("/front_obstacle_detect", 1, &PurePursuit::obstacle_status, this);
  emergencyStopSubscriber = nodeHandle.subscribe("/emergency_stop", 1, &PurePursuit::emergency_stop_status, this);

  twistPublisher = nodeHandle.advertise<geometry_msgs::Twist>(cmdVelTopic, 1);
  ackermannDrivePublisher = nodeHandle.advertise<ackermann_msgs::AckermannDriveStamped>("cmd_acker", 1);
  lookaheadMarkerPublisher = nodeHandle.advertise<visualization_msgs::Marker>("lookahead", 1);
}

void PurePursuit::obstacle_status(std_msgs::Bool status){
  isObstacleDetected = status.data;
  ROS_INFO("Obstacle detected: %s", isObstacleDetected ? "True" : "False");
}

void PurePursuit::emergency_stop_status(std_msgs::Bool status){
  //emergency_stop_ = status.data;
  canMove = status.data;
  // if(canMove){
  //   ROS_WARN("Emergency stop released.");
  // }else { 
  //   ROS_WARN("Emergency stop triggered.");
  // }
}

void PurePursuit::velocityCommandGenerator(geometry_msgs::PoseStamped newPose){ // provide by a localizer e.g. NDT
  ros::Rate rate(cmdVelRate);
  if(isObstacleDetected || !canMove){
    generateNoVelocityCommandTwist();
    twistPublisher.publish(twist);
    //path = waypoints_manager::PathOfPosesWithVelocity(); reset the path 
    return;
  }
  if(isPathLoaded){
    // get the current pose from the localizer pose.
    try{
      geometry_msgs::Pose currentPose = newPose.pose;
      robotToMapTransform = transformBuffer.lookupTransform(mapFrameId, robotFrameId, ros::Time(0));
      for (idx = idxMmemory; idx < path.posesWithVelocity.size(); idx++) {
        // get the path pose
        geometry_msgs::Pose currentPathPose = path.posesWithVelocity[idx].pose;
        pathPoseVelocity = path.posesWithVelocity[idx].velocity.velocity;

        std::cout << "Position: x: " << currentPathPose.position.x
          << ", y: " << currentPathPose.position.y
          << ", z: " << currentPathPose.position.z << std::endl;

        if (distance(currentPathPose.position, currentPose.position) > lookaheadDistance){
          KDL::Frame poseOffset = transformPoseToBaseLink(currentPathPose, robotToMapTransform.transform);
          lookaheadTransform.transform.translation.x = poseOffset.p.x();
          lookaheadTransform.transform.translation.y = poseOffset.p.y();
          lookaheadTransform.transform.translation.z = poseOffset.p.z();
          poseOffset.M.GetQuaternion(lookaheadTransform.transform.rotation.x, lookaheadTransform.transform.rotation.y,
                                     lookaheadTransform.transform.rotation.z, lookaheadTransform.transform.rotation.w);
          idxMmemory = idx;
          break;
        }
      }

      // If approach the goal (last waypoint)
      if (!path.posesWithVelocity.empty() && idx >= path.posesWithVelocity.size()){
        KDL::Frame goalOffset = transformPoseToBaseLink(path.posesWithVelocity.back().pose, robotToMapTransform.transform);

        // Reach the goal
        if (fabs(goalOffset.p.x()) <= positionTolerance){
          isGoalReached = true;
          path = waypoints_manager::PathOfPosesWithVelocity(); //path = nav_msgs::Path(); // Reset the path
        }
        // Not meet the position tolerance: extend the lookahead distance beyond the goal
        else{
          // Find the intersection between the circle of radius(lookaheadDistance) centered at the current pose
          // and the line defined by the last waypoint
          double roll, pitch, yaw;
          goalOffset.M.GetRPY(roll, pitch, yaw);
          double k_end = tan(yaw); // Slope of line defined by the last waypoint
          double l_end = goalOffset.p.y() - k_end * goalOffset.p.x();
          double a = 1 + pow(k_end, 2);
          double b = 2 * l_end;
          double c = pow(l_end, 2) - pow(lookaheadDistance, 2);
          double D = sqrt(pow(b, 2) - 4 * a * c);
          double x_ld = (-b + copysign(D, linearVelocity)) / (2 * a);
          double y_ld = k_end * x_ld + l_end;
            
          lookaheadTransform.transform.translation.x = x_ld;
          lookaheadTransform.transform.translation.y = y_ld;
          lookaheadTransform.transform.translation.z = goalOffset.p.z();
          goalOffset.M.GetQuaternion(lookaheadTransform.transform.rotation.x, lookaheadTransform.transform.rotation.y,
                                      lookaheadTransform.transform.rotation.z, lookaheadTransform.transform.rotation.w);
        }
      }

      // Waypoint follower
      if (!isGoalReached){
        linearVelocity = pathPoseVelocity == 0? copysign(MaximumLinearVelocity, linearVelocity):
                                                pathPoseVelocity;
        /*linearVelocity = copysign(MaximumLinearVelocity, linearVelocity);*/

        double lateralOffset = lookaheadTransform.transform.translation.y;
        twist.angular.z = std::min(3 * linearVelocity / lookaheadDistance * lookaheadDistance * lateralOffset, maximumAngularVelocity);
        // Desired Ackermann steering_angle
        ackermannDrive.drive.steering_angle = std::min(atan2(2 * lateralOffset * wheelBase, lookaheadDistance * lookaheadDistance), maximumSteeringAngle);
        
        // Linear velocity
        twist.linear.x = linearVelocity;
        ackermannDrive.drive.speed = linearVelocity;
        ackermannDrive.header.stamp = ros::Time::now();
      }
      // Reach the goal: stop the vehicle
      else{
        generateNoVelocityCommandTwist();
        generateNoVelocityCommandAckermannDrive();
      }

      // Publish the lookahead target transform.
      lookaheadTransform.header.stamp = ros::Time::now();
      transformBroadcaster.sendTransform(lookaheadTransform);
      // Publish the velocity command
      twistPublisher.publish(twist);
      // Publish the ackerman_steering command
      ackermannDrivePublisher.publish(ackermannDrive);
      // Publish the lookahead_marker for visualization
      publishLookaheadMarker(robotToMapTransform);
    }catch (tf2::TransformException &ex) {
      ROS_WARN_STREAM(ex.what());
    }
  }
  rate.sleep();
}

void PurePursuit::receivePath(waypoints_manager::PathOfPosesWithVelocity newPath){
  if (newPath.header.frame_id == mapFrameId){
    path = newPath;
    idx = 0;
    if (newPath.posesWithVelocity.size() > 0){
      std::cout << "Received Waypoints" << std::endl;
      isPathLoaded = true;
    } else {
      ROS_WARN_STREAM("Received empty waypoint!");
    }
  } else {
    ROS_WARN_STREAM("The waypoints must be published in the " << mapFrameId << " frame! Ignoring path in " << newPath.header.frame_id << " frame!");
  }
}

KDL::Frame PurePursuit::transformPoseToBaseLink(const geometry_msgs::Pose& pose, const geometry_msgs::Transform& transform) {
  // Pose in map
  KDL::Frame poseInMap(KDL::Rotation::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                        KDL::Vector(pose.position.x, pose.position.y, pose.position.z));
  // base_link in map
  KDL::Frame baseLinkInMap(KDL::Rotation::Quaternion(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w),
                      KDL::Vector(transform.translation.x, transform.translation.y, transform.translation.z));

  return baseLinkInMap.Inverse() * poseInMap;
}

void PurePursuit::run(){
  ros::spin();
}

void PurePursuit::publishLookaheadMarker(geometry_msgs::TransformStamped toMapTransform){
  lookaheadMarker.header.frame_id = mapFrameId;
  lookaheadMarker.header.stamp = ros::Time::now();
  lookaheadMarker.type = visualization_msgs::Marker::SPHERE;
  lookaheadMarker.action = visualization_msgs::Marker::ADD;
  lookaheadMarker.scale.x = 0.25;
  lookaheadMarker.scale.y = 0.25;
  lookaheadMarker.scale.z = 0.25;
  lookaheadMarker.pose.orientation.x = 0.0;
  lookaheadMarker.pose.orientation.y = 0.0;
  lookaheadMarker.pose.orientation.z = 0.0;
  lookaheadMarker.pose.orientation.w = 1.0;
  lookaheadMarker.color.a = 1.0;
  if (!isGoalReached){
    lookaheadMarker.id = idx;
    lookaheadMarker.pose.position.x = path.posesWithVelocity[idx].pose.position.x;
    lookaheadMarker.pose.position.y = path.posesWithVelocity[idx].pose.position.y;
    lookaheadMarker.pose.position.z = path.posesWithVelocity[idx].pose.position.z;
    lookaheadMarker.color.r = 0.0;
    lookaheadMarker.color.g = 1.0;
    lookaheadMarker.color.b = 0.0;
    lookaheadMarkerPublisher.publish(lookaheadMarker);
  }else {
    lookaheadMarker.id = idxMmemory;
    idxMmemory += 1;
    lookaheadMarker.pose.position.x = toMapTransform.transform.translation.x;
    lookaheadMarker.pose.position.y = toMapTransform.transform.translation.y;
    lookaheadMarker.pose.position.z = toMapTransform.transform.translation.z;
    lookaheadMarker.color.r = 1.0;
    lookaheadMarker.color.g = 0.0;
    lookaheadMarker.color.b = 0.0;
    if (idxMmemory % 5 == 0){
      lookaheadMarkerPublisher.publish(lookaheadMarker); 
    }
    isGoalReached = false;
    isPathLoaded = false;
    idxMmemory = 0;
  }
}

void PurePursuit::generateNoVelocityCommandTwist(){
  twist.linear.x = 0.00;
  twist.linear.y = 0.00;
  twist.linear.z = 0.00;
  twist.angular.x = 0.00;
  twist.angular.y = 0.00;
  twist.angular.z = 0.00;
}

void PurePursuit::generateNoVelocityCommandAckermannDrive(){
    ackermannDrive.header.stamp = ros::Time::now();
    ackermannDrive.drive.steering_angle = 0.00;
    ackermannDrive.drive.speed = 0.00;
}

int main(int argc, char**argv){
  ros::init(argc, argv, "pure_pursuit");

  PurePursuit controller;
  controller.run();
  
  return 0;
}
