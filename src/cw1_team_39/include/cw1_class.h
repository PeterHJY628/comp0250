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

// Coursework service definitions
#include "cw1_world_spawner/Task1Service.h"
#include "cw1_world_spawner/Task2Service.h"
#include "cw1_world_spawner/Task3Service.h"
#include "obj_rec.h"
#include "move_and_place.h"

class cw1
{
public:
  /**
   * @brief Constructor: sets up the MoveGroupInterfaces and advertises services
   * @param nh ROS NodeHandle
   */
  cw1(ros::NodeHandle nh);

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


private:

  // Node
  ros::NodeHandle nh_;
  ObjRec obj_rec;
  MoveAndPlace move_and_place;
  // Advertised services for tasks 1,2,3
  ros::ServiceServer t1_service_;
  ros::ServiceServer t2_service_;
  ros::ServiceServer t3_service_;
  tf2_ros::Buffer tfBuffer_;
  tf::TransformListener tfListener_;
  
};

#endif  // CW1_CLASS_H_
