#include "cw1_class.h"
#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_ros/transforms.h>

cw1::cw1(ros::NodeHandle nh) : nh_(nh),
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

  std::vector<geometry_msgs::PointStamped> basket_locs = request.basket_locs;
  for (size_t i = 0; i < basket_locs.size(); ++i)
  {
    tf2::Quaternion downQ, yawQ, finalQ;

    downQ.setRPY(-M_PI, 0.0, 0.0);
    yawQ.setRPY(0.0, 0.0, M_PI / 4.0); // 45 deg

    finalQ = yawQ * downQ;
    finalQ.normalize(); // 归一化

    // 把它变成 geometry_msgs/Quaternion
    geometry_msgs::Quaternion corrected_orientation = tf2::toMsg(finalQ);

    // ====================== 4) place pose ======================
    geometry_msgs::PoseStamped place_pose;
    place_pose.header = basket_locs[i].header;
    place_pose.pose.position = basket_locs[i].point;
    place_pose.pose.position.z += 0.30;
    place_pose.pose.orientation = corrected_orientation;

    if (!move_and_place.moveArm(place_pose.pose))
    {
      ROS_ERROR("Failed to move arm to place pose");
      return false;
    }
    // detect color
    ros::Time t0 = ros::Time::now();
    boost::shared_ptr<const sensor_msgs::PointCloud2> tmp_msg;
    while (ros::ok())
    {
      tmp_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
          "/r200/camera/depth_registered/points", nh_, ros::Duration(5.0));
      if (!tmp_msg)
      {
        ROS_ERROR("Failed to receive point cloud message within 5 seconds, retrying...");
        continue;
      }
      if (tmp_msg->header.stamp <= t0)
      {
        ROS_WARN("Stale point cloud received (stamp: %.3f, t0: %.3f), retrying...",
                 tmp_msg->header.stamp.toSec(), t0.toSec());
        continue;
      }
      break;
    }
    boost::shared_ptr<const sensor_msgs::PointCloud2> cloud_msg = tmp_msg;
    // Convert sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZRGBA>
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud_rgb);
    // Process the point cloud through segmentation.
    obj_rec.segColor(cloud_rgb);

    // Retrieve the number of points segmented for each color.
    size_t num_red = obj_rec.g_cloud_red->points.size();
    size_t num_blue = obj_rec.g_cloud_blue->points.size();
    size_t num_purple = obj_rec.g_cloud_purple->points.size();

    ROS_WARN("Basket index %zu: Segmented points - Red: %zu, Blue: %zu, Purple: %zu", i, num_red, num_blue, num_purple);

    // Determine which color has been detected (i.e. has more than 0 points).
    std::string detected_color = "none";
    if (num_red > 0)
      detected_color = "red";
    else if (num_blue > 0)
      detected_color = "blue";
    else if (num_purple > 0)
      detected_color = "purple";

    ROS_WARN("Detected color: %s", detected_color.c_str());
  }

  // TODO return result in vector format
  // Done - response.basket_colors now has one color string per basket_locs
  return true;
}

bool cw1::t3_callback(cw1_world_spawner::Task3Service::Request &request,
                      cw1_world_spawner::Task3Service::Response &response)
{
  ROS_INFO("Task 3 callback triggered");

  // Create a global point cloud to accumulate merged clouds.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  ros::Rate rate(30);

  // 1) Visit several positions and merge point clouds
  for (int i = 0; i < 5; i++)
  {
    // Move arm to position i.
    find_objects.visitPosition(i);

    // --- [OPTIONAL] Small sleep to let the arm/camera settle ---
    ros::Duration(1.0).sleep();

    // ------------------------------------------------------------------------
    // Wait for a single NEW point cloud message (5 seconds as an example).
    // We do NOT flush in a loop. Instead, rely on "waitForMessage" once here.
    // ------------------------------------------------------------------------
    // Record current time to use as t0.
    ros::Time t0 = ros::Time::now();

    boost::shared_ptr<const sensor_msgs::PointCloud2> tmp_msg;
    while (ros::ok())
    {
      tmp_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
          "/r200/camera/depth_registered/points", nh_, ros::Duration(5.0));
      if (!tmp_msg)
      {
        ROS_ERROR("Failed to receive point cloud message within 5 seconds, retrying...");
        continue;
      }
      if (tmp_msg->header.stamp <= t0)
      {
        ROS_WARN("Stale point cloud received (stamp: %.3f, t0: %.3f), retrying...",
                 tmp_msg->header.stamp.toSec(), t0.toSec());
        continue;
      }
      break;
    }
    boost::shared_ptr<const sensor_msgs::PointCloud2> cloud_msg = tmp_msg;

    ros::Time t1 = ros::Time::now();
    tf::StampedTransform transform;
    while (cloud_msg->header.stamp <= t1 && ros::ok())
    {
      ROS_WARN("Received point cloud timestamp (%.3f) is not later than t1 (%.3f), waiting for a newer message...", cloud_msg->header.stamp.toSec(), t1.toSec());
      cloud_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
          "/r200/camera/depth_registered/points", nh_, ros::Duration(5.0));
      if (!cloud_msg)
      {
        ROS_ERROR("Failed to receive point cloud message.");
        return false;
      }
    }

    try
    {
      tfListener_.waitForTransform("world", cloud_msg->header.frame_id,
                                   ros::Time(0), ros::Duration(5.0));
      tfListener_.lookupTransform("world", cloud_msg->header.frame_id,
                                  ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("Could not get TF transform (world -> %s). Error: %s",
                cloud_msg->header.frame_id.c_str(), ex.what());
      return false;
    }

    // Convert sensor_msgs point cloud into PCL format.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    {
      pcl::PCLPointCloud2 pcl_pc2;
      pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
      pcl::fromPCLPointCloud2(pcl_pc2, *local_cloud);
    }

    // Transform the local point cloud to the "world" frame.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl_ros::transformPointCloud(*local_cloud, *cloud_transformed, transform);

    // Merge the transformed local cloud into the global cloud.
    *global_cloud += *cloud_transformed;

    rate.sleep();
  }

  // 2) Convert the merged global point cloud back into a ROS message.
  sensor_msgs::PointCloud2 merged_cloud_msg;
  pcl::toROSMsg(*global_cloud, merged_cloud_msg);
  merged_cloud_msg.header.frame_id = "world";
  merged_cloud_msg.header.stamp = ros::Time::now();

  // 3) Process the merged point cloud with obj_rec.
  obj_rec.cloudCallBackOne(boost::make_shared<sensor_msgs::PointCloud2>(merged_cloud_msg));

  // 4) Debug info:
  ROS_INFO("---- Debug info for obj_rec ----");
  ROS_INFO_STREAM("Red box position: " << obj_rec.red_box_pos.transpose());
  ROS_INFO_STREAM("Blue box position: " << obj_rec.blue_box_pos.transpose());
  ROS_INFO_STREAM("Purple box position: " << obj_rec.purple_box_pos.transpose());

  ROS_INFO_STREAM("Red box found: " << (obj_rec.box_found[0] ? "true" : "false"));
  ROS_INFO_STREAM("Blue box found: " << (obj_rec.box_found[1] ? "true" : "false"));
  ROS_INFO_STREAM("Purple box found: " << (obj_rec.box_found[2] ? "true" : "false"));

  ROS_INFO("Block positions:");
  for (size_t i = 0; i < obj_rec.block_pos.size(); ++i)
  {
    ROS_INFO_STREAM("  Block " << i << " position: " << obj_rec.block_pos[i].transpose());
  }

  ROS_INFO("Block colors:");
  for (size_t i = 0; i < obj_rec.block_color.size(); ++i)
  {
    ROS_INFO_STREAM("  Block " << i << " color: " << obj_rec.block_color[i]);
  }

  ROS_INFO_STREAM("Current block index: " << obj_rec.current_block_idx);
  ROS_INFO("--------------------------------");

  // 5) For each recognized block, pick it from its position and place it on the box of matching color.
  for (size_t i = 0; i < obj_rec.block_pos.size(); ++i)
  {
    const std::string &color = obj_rec.block_color[i];

    // Prepare pick point
    geometry_msgs::Point pick_point;
    pick_point.x = obj_rec.block_pos[i](0);
    pick_point.y = obj_rec.block_pos[i](1);
    pick_point.z = obj_rec.block_pos[i](2);

    geometry_msgs::Point place_point;

    // Decide which box to place on based on color
    if (color == "red")
    {
      if (!obj_rec.box_found[0])
      {
        ROS_WARN("Red box not found! Skipping block %zu (color red).", i);
        continue;
      }
      place_point.x = obj_rec.red_box_pos(0);
      place_point.y = obj_rec.red_box_pos(1);
      place_point.z = obj_rec.red_box_pos(2);
    }
    else if (color == "blue")
    {
      if (!obj_rec.box_found[1])
      {
        ROS_WARN("Blue box not found! Skipping block %zu (color blue).", i);
        continue;
      }
      place_point.x = obj_rec.blue_box_pos(0);
      place_point.y = obj_rec.blue_box_pos(1);
      place_point.z = obj_rec.blue_box_pos(2);
    }
    else if (color == "purple")
    {
      if (!obj_rec.box_found[2])
      {
        ROS_WARN("Purple box not found! Skipping block %zu (color purple).", i);
        continue;
      }
      place_point.x = obj_rec.purple_box_pos(0);
      place_point.y = obj_rec.purple_box_pos(1);
      place_point.z = obj_rec.purple_box_pos(2);
    }
    else
    {
      ROS_WARN("Unknown block color '%s' for block %zu. Skipping.", color.c_str(), i);
      continue;
    }

    ROS_INFO("Picking block %zu (color %s) from (%.3f, %.3f, %.3f) -> place at (%.3f, %.3f, %.3f)",
             i, color.c_str(),
             pick_point.x, pick_point.y, pick_point.z,
             place_point.x, place_point.y, place_point.z);

    // 6) Attempt the pick-and-place.
    geometry_msgs::PoseStamped pick_pose;
    pick_pose.header.frame_id = "world";
    pick_pose.header.stamp = ros::Time::now();
    pick_pose.pose.position = pick_point;
    pick_pose.pose.position.z = 0.02;
    pick_pose.pose.orientation.w = 1.0; // Identity orientation

    geometry_msgs::PointStamped place_point_stamped;
    place_point_stamped.header.frame_id = "world";
    place_point_stamped.header.stamp = ros::Time::now();
    place_point_stamped.point = place_point;

    bool success = move_and_place.performPickAndPlace(pick_pose, place_point_stamped);
    if (!success)
    {
      ROS_ERROR("Failed to pick and place block %zu (color %s).", i, color.c_str());
    }
    else
    {
      ROS_INFO("Successfully picked and placed block %zu (color %s).", i, color.c_str());
    }
  }

  return true;
}