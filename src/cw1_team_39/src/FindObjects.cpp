#include "FindObjects.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <iostream>

FindObjects::FindObjects(ros::NodeHandle &nh)
  : nh_(nh),
    arm_group_("panda_arm")
{
  // Set the planning time and reference frame
  arm_group_.setPlanningTime(10.0);
  arm_group_.setPoseReferenceFrame("panda_link0");

  // ------------------------------------------------------------
  // Create the "down + 45° around Z" orientation (same as move_and_place)
  // ------------------------------------------------------------
  // 先翻转180° (roll = -M_PI)，让Z轴朝下
  // 再绕Z轴 +45° (yaw = M_PI/4)
  // 最后归一化
  tf2::Quaternion downQ, yawQ, finalQ;
  downQ.setRPY(-M_PI, 0.0, 0.0);        // Roll = -π
  yawQ.setRPY(0.0, 0.0, M_PI / 4.0);    // Yaw = π/4
  finalQ = yawQ * downQ;               // Apply downQ first, then yawQ
  finalQ.normalize();

  // 转成 geometry_msgs::Quaternion
  geometry_msgs::Quaternion corrected_orientation = tf2::toMsg(finalQ);

  // ------------------------------------------------------------
  // Initialize the list of 5 predefined target positions
  // and use the same orientation for all.
  // ------------------------------------------------------------
  geometry_msgs::Pose pose;

  // Position 1
  pose.position.x = 0.29;
  pose.position.y = -0.34;
  pose.position.z = 0.6;
  pose.orientation = corrected_orientation;
  target_positions_.push_back(pose);

  // Position 2
  pose.position.x = 0.29;
  pose.position.y = 0.34;
  pose.position.z = 0.6;
  pose.orientation = corrected_orientation;
  target_positions_.push_back(pose);

  // Position 3
  pose.position.x = 0.6;
  pose.position.y = 0.34;
  pose.position.z = 0.6;
  pose.orientation = corrected_orientation;
  target_positions_.push_back(pose);

  // Position 4
  pose.position.x = 0.6;
  pose.position.y = -0.34;
  pose.position.z = 0.6;
  pose.orientation = corrected_orientation;
  target_positions_.push_back(pose);

  // Position 5
  pose.position.x = 0.445;
  pose.position.y = 0.0;
  pose.position.z = 0.6;
  pose.orientation = corrected_orientation;
  target_positions_.push_back(pose);
}

bool FindObjects::moveArm(const geometry_msgs::Pose target_pose)
{
  ROS_INFO("Setting target pose: x=%f, y=%f, z=%f",
           target_pose.position.x,
           target_pose.position.y,
           target_pose.position.z);
  arm_group_.setPoseTarget(target_pose);

  std::string planning_frame = arm_group_.getPlanningFrame();
  ROS_INFO("Planning frame: %s", planning_frame.c_str());

  ROS_INFO("Attempting to plan the path...");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_group_.plan(my_plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Plan %s", success ? "SUCCEEDED" : "FAILED");

  if (success)
  {
    arm_group_.move();
    ros::Duration(1.0).sleep(); // Wait for the motion to complete
    return true;
  }
  else
  {
    return false;
  }
}

bool FindObjects::visitAllPositions()
{
  ROS_INFO("Start visiting predefined positions...");
  for (size_t i = 0; i < target_positions_.size(); ++i)
  {
    ROS_INFO("Moving to position %zu...", i + 1);
    if (!moveArm(target_positions_[i]))
    {
      ROS_ERROR("Failed to move to position %zu", i + 1);
      return false;
    }
    ros::Duration(1.0).sleep();  // Pause to observe the position
  }
  ROS_INFO("Successfully visited all positions.");
  return true;
}

bool FindObjects::visitPosition(size_t position_index)
{
  if (position_index >= target_positions_.size())
  {
    ROS_ERROR("Invalid position index: %zu. Valid range is 0 to %zu.",
              position_index,
              target_positions_.size() - 1);
    return false;
  }

  ROS_INFO("Moving to position %zu...", position_index + 1);
  if (!moveArm(target_positions_[position_index]))
  {
    ROS_ERROR("Failed to move to position %zu", position_index + 1);
    return false;
  }
  return true;
}
