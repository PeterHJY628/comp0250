#ifndef FIND_OBJECTS_H_
#define FIND_OBJECTS_H_

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>

class FindObjects
{
public:
  /**
   * @brief Constructor: initializes the MoveIt! interface and the list of target positions.
   * @param nh ROS NodeHandle.
   */
  FindObjects(ros::NodeHandle &nh);

  /**
   * @brief Sequentially moves the arm to the five predefined positions.
   * @return true if the arm successfully reaches all positions; false if any movement fails.
   */
  bool visitAllPositions();

private:
  /**
   * @brief Moves the arm to the specified target pose.
   * @param target_pose The target pose.
   * @return true if planning and motion execution succeed; false otherwise.
   */
  bool moveArm(const geometry_msgs::Pose target_pose);

  ros::NodeHandle nh_;
  moveit::planning_interface::MoveGroupInterface arm_group_;
  std::vector<geometry_msgs::Pose> target_positions_;
};

#endif // FIND_OBJECTS_H_
