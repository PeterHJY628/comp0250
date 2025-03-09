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

// Create a global point cloud to accumulate merged clouds.
pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
ros::Rate rate(30);

for (int i = 0; i < 5; i++) {
// Move arm to position i.
find_objects.visitPosition(i);

// Wait for one point cloud message.
boost::shared_ptr<const sensor_msgs::PointCloud2> cloud_msg =
ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/r200/camera/depth_registered/points", nh_);

if (!cloud_msg) {
ROS_ERROR("Failed to receive point cloud message");
return false;
}

// Get the transformation from the point cloud frame to "world" frame.
tf::StampedTransform transform;
try {
tfListener_.lookupTransform("world", cloud_msg->header.frame_id, ros::Time(0), transform);
}
catch (tf::TransformException &ex) {
ROS_ERROR("Could NOT transform: %s", ex.what());
return false;
}

// Convert sensor_msgs point cloud into PCL format.
pcl::PointCloud<pcl::PointXYZRGB>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PCLPointCloud2 pcl_pc2;
pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
pcl::fromPCLPointCloud2(pcl_pc2, *local_cloud);

// Transform the local point cloud to the "world" frame.
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl_ros::transformPointCloud(*local_cloud, *cloud_transformed, transform);

// Merge the transformed local cloud into the global cloud.
*global_cloud += *cloud_transformed;

rate.sleep();
}

// Convert the merged global point cloud back into a ROS message.
sensor_msgs::PointCloud2 merged_cloud_msg;
pcl::toROSMsg(*global_cloud, merged_cloud_msg);
merged_cloud_msg.header.frame_id = "world";
merged_cloud_msg.header.stamp = ros::Time::now();

// Finally, process the merged point cloud with obj_rec.cloudCallBackOne.
obj_rec.cloudCallBackOne(boost::make_shared<sensor_msgs::PointCloud2>(merged_cloud_msg));


std::cout << "Debug info for obj_rec:" << std::endl;

std::cout << "Red box position: " << obj_rec.red_box_pos.transpose() << std::endl;
std::cout << "Blue box position: " << obj_rec.blue_box_pos.transpose() << std::endl;
std::cout << "Purple box position: " << obj_rec.purple_box_pos.transpose() << std::endl;

std::cout << "Red box found: " << (obj_rec.box_found[0] ? "true" : "false") << std::endl;
std::cout << "Blue box found: " << (obj_rec.box_found[1] ? "true" : "false") << std::endl;
std::cout << "Purple box found: " << (obj_rec.box_found[2] ? "true" : "false") << std::endl;

std::cout << "Block positions:" << std::endl;
for (size_t i = 0; i < obj_rec.block_pos.size(); ++i) {
  std::cout << "  Block " << i << " position: " << obj_rec.block_pos[i].transpose() << std::endl;
}

std::cout << "Block colors:" << std::endl;
for (size_t i = 0; i < obj_rec.block_color.size(); ++i) {
  std::cout << "  Block " << i << " color: " << obj_rec.block_color[i] << std::endl;
}

std::cout << "Current block index: " << obj_rec.current_block_idx << std::endl;

return true;
}


// ========== 机械臂 / 手爪动作的具体实现不变 ==========


