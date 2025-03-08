#include "cw1_class.h"
#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


cw1::cw1(ros::NodeHandle nh): nh_(nh),
  obj_rec(nh),
  move_and_place(nh),
  find_objects(nh)
{
  t1_service_ = nh_.advertiseService("/task1_start", &cw1::t1_callback, this);
  t2_service_ = nh_.advertiseService("/task2_start", &cw1::t2_callback, this);
  t3_service_ = nh_.advertiseService("/task3_start", &cw1::t3_callback, this);

  ROS_INFO("cw1 class initialised. Services for Task1/2/3 ready.");
}

// ========== 打印函数 ==========


bool cw1::t1_callback(cw1_world_spawner::Task1Service::Request &request,
                      cw1_world_spawner::Task1Service::Response &response)
{
  ROS_INFO("Task 1 callback triggered");
  return move_and_place.performPickAndPlace(request.object_loc, request.goal_loc);
}


bool cw1::t2_callback(cw1_world_spawner::Task2Service::Request &request,
cw1_world_spawner::Task2Service::Response &response)
{
  ROS_INFO("Task 2 callback triggered");
  // Subscribe to the point cloud topic and get one message at 30fps
  ros::Rate rate(30);
  while (ros::ok()) {
  boost::shared_ptr<const sensor_msgs::PointCloud2> cloud_msg =
  ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/r200/camera/depth_registered/points", nh_);
  if (cloud_msg != nullptr) {
  obj_rec.cloudCallBackOne(cloud_msg);
  } else {
  ROS_ERROR("Failed to receive point cloud message");
  return false;
  }
  rate.sleep();
  }
  return true;
}

bool cw1::t3_callback(cw1_world_spawner::Task3Service::Request &request,
                      cw1_world_spawner::Task3Service::Response &response)
{
  ROS_INFO("Task 3 callback triggered");
  // TODO
  find_objects.visitAllPositions();
  return true;
}

// ========== 机械臂 / 手爪动作的具体实现不变 ==========


