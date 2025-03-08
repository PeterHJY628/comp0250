#include "cw1_class.h"
#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_ros/transforms.h>


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
  
  // 每次等待一帧点云，这里用30 Hz循环只是为了演示
  ros::Rate rate(30);

  while (ros::ok()) {

  // 等待订阅 "/r200/camera/depth_registered/points" 话题
  boost::shared_ptr<const sensor_msgs::PointCloud2> cloud_msg =
    ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/r200/camera/depth_registered/points", nh_);

  if (!cloud_msg) {
    ROS_ERROR("Failed to receive point cloud message");
    return false;
  }

  // 获取变换
  tf::StampedTransform transform;
  try {
    tfListener_.lookupTransform("world", cloud_msg->header.frame_id, ros::Time(0), transform);
  } 
  catch (tf::TransformException &ex) {
    ROS_ERROR("Could NOT transform: %s", ex.what());
    return false;
  }

  // 将点云变换到 world 坐标系
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

  pcl_ros::transformPointCloud(*temp_cloud, *cloud_transformed, transform);

  sensor_msgs::PointCloud2 cloud_publish;
  pcl::toROSMsg(*cloud_transformed, cloud_publish);
  cloud_publish.header = cloud_msg->header;
  cloud_publish.header.frame_id = "world";

  // 现在 cloud_publish 就是以 "world" 为坐标系的点云
  // 继续对转换好的点云做后续处理或发布
  obj_rec.cloudCallBackOne(boost::make_shared<sensor_msgs::PointCloud2>(cloud_publish));

  rate.sleep();
  }
  return true;
}


// ========== 机械臂 / 手爪动作的具体实现不变 ==========


