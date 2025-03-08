#ifndef CW1_CLASS_H_
#define CW1_CLASS_H_

/* 
 * cw1_class.h
 * 
 * Header file defining the cw1 class, which advertises ROS services for
 * tasks 1, 2, and 3. It uses MoveIt! to plan and execute motions of
 * the "panda_arm" and "hand" groups.
 */

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <string>
#include <vector>

// MoveIt includes
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Coursework service definitions
#include "cw1_world_spawner/Task1Service.h"
#include "cw1_world_spawner/Task2Service.h"
#include "cw1_world_spawner/Task3Service.h"
#include "obj_rec_tutorial.h"

class cw1
{
public:
  /**
   * @brief Constructor: sets up the MoveGroupInterfaces and advertises services
   * @param nh ROS NodeHandle
   */
  cw1(ros::NodeHandle nh);

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

  /**
   * @brief Callback for Task1 (pick & place)
   * @param request includes object_loc & goal_loc
   * @param response (empty for Task1)
   */
  bool t1_callback(cw1_world_spawner::Task1Service::Request &request,
                   cw1_world_spawner::Task1Service::Response &response);

  /**
   * @brief Callback for Task2 (detect color)
   */
  bool t2_callback(cw1_world_spawner::Task2Service::Request &request,
                   cw1_world_spawner::Task2Service::Response &response);

  /**
   * @brief Callback for Task3 (multi-object pick & place)
   */
  bool t3_callback(cw1_world_spawner::Task3Service::Request &request,
                   cw1_world_spawner::Task3Service::Response &response);

  /**
   * @brief Move the arm to a desired pose (wrapper for MoveGroupInterface).
   * @param target_pose Desired pose for the end-effector
   * @return true if success
   */
  bool moveArm(const geometry_msgs::Pose target_pose);

  /**
   * @brief Move the gripper fingers to a certain width.
   * @param width Distance between fingers [m].
   * @return true if success
   */
  bool moveGripper(float width);

private:
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

  // Node
  ros::NodeHandle nh_;
  ObjRecTutorial obj_rec_tutorial;
  // Advertised services for tasks 1,2,3
  ros::ServiceServer t1_service_;
  ros::ServiceServer t2_service_;
  ros::ServiceServer t3_service_;

  // MoveIt planning interfaces
  // -- here we use normal objects, not pointers --
  moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
  moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  // Some default finger positions for fully open/closed
  double gripper_open_   = 0.07; ///< ~7cm open
  double gripper_closed_ = 0.01; ///< ~1cm (close)

  // Base frame for referencing
  std::string base_frame_ = "panda_link0";
};

#endif  // CW1_CLASS_H_
