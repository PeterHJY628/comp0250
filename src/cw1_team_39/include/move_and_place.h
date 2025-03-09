#ifndef MOVE_AND_PLACE_H_
#define MOVE_AND_PLACE_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <string>
#include <vector>

// MoveIt includes
#include "cw1_world_spawner/Task1Service.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class MoveAndPlace
{
public:
  MoveAndPlace(ros::NodeHandle &nh);
    /**
   * @brief Move the arm to a desired pose (wrapper for MoveGroupInterface).
   * @param target_pose Desired pose for the end-effector
   * @return true if success
  */
  bool moveArm(const geometry_msgs::Pose target_pose);
  bool moveArm(const geometry_msgs::Pose target_pose, float width);

  /**
   * @brief Move the gripper fingers to a certain width.
   * @param width Distance between fingers [m].
   * @return true if success
   */
  bool moveGripper(float width);
  /**
   * @brief Perform a pick-and-place sequence:
   *        - Move above object
   *        - Descend & grasp
   *        - Move above again
   *        - Move above basket
   *        - Release the object
   *
   * @param object_pose pose of the object
   * @param basket_point position of the basket
   * @return true if succeeded
   */
  bool performPickAndPlace(geometry_msgs::PoseStamped object_pose,
                           geometry_msgs::PointStamped basket_point);
private:
  /**
   * @brief Print contents of a PoseStamped, for debugging
   * @param pose PoseStamped to print
   */
  void printPoseStamped(const geometry_msgs::PoseStamped &pose);

  /**
   * @brief Print contents of a PointStamped, for debugging
   * @param point PointStamped to print
   */
  void printPointStamped(const geometry_msgs::PointStamped &point);

  // MoveIt planning interfaces
  // -- here we use normal objects, not pointers --
  ros::NodeHandle nh_;
  moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
  moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  // Some default finger positions for fully open/closed
  double gripper_open_   = 0.07; ///< ~7cm open
  double gripper_closed_ = 0.01; ///< ~1cm (close)

  // Base frame for referencing
  std::string base_frame_ = "panda_link0";
};

#endif // MOVE_AND_PLACE_H_