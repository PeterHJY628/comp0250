#include "cw1_class.h"
#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


cw1::cw1(ros::NodeHandle nh)
  : nh_(nh)
  , arm_group_("panda_arm")
  , hand_group_("hand")
  , obj_rec_tutorial(nh)
{
  t1_service_ = nh_.advertiseService("/task1_start", &cw1::t1_callback, this);
  t2_service_ = nh_.advertiseService("/task2_start", &cw1::t2_callback, this);
  t3_service_ = nh_.advertiseService("/task3_start", &cw1::t3_callback, this);

  arm_group_.setPlanningTime(10.0);
  hand_group_.setPlanningTime(5.0);
  arm_group_.setPoseReferenceFrame("panda_link0");

  ROS_INFO("cw1 class initialised. Services for Task1/2/3 ready.");
}

// ========== 打印函数 ==========

void cw1::printPoseStamped(const geometry_msgs::PoseStamped &pose)
{
  std::cout << "PoseStamped:\n"
            << "  Header:\n"
            << "    seq: " << pose.header.seq << "\n"
            << "    stamp: " << pose.header.stamp << "\n"
            << "    frame_id: " << pose.header.frame_id << "\n"
            << "  Pose:\n"
            << "    Position: ("
            << pose.pose.position.x << ", "
            << pose.pose.position.y << ", "
            << pose.pose.position.z << ")\n"
            << "    Orientation: ("
            << pose.pose.orientation.x << ", "
            << pose.pose.orientation.y << ", "
            << pose.pose.orientation.z << ", "
            << pose.pose.orientation.w << ")\n";
}

void cw1::printPointStamped(const geometry_msgs::PointStamped &point)
{
  std::cout << "PointStamped:\n"
            << "  Header:\n"
            << "    seq: " << point.header.seq << "\n"
            << "    stamp: " << point.header.stamp << "\n"
            << "    frame_id: " << point.header.frame_id << "\n"
            << "  Point: ("
            << point.point.x << ", "
            << point.point.y << ", "
            << point.point.z << ")\n";
}

// ========== Task1 回调，内部只调用performPickAndPlace ==========

bool cw1::t1_callback(cw1_world_spawner::Task1Service::Request &request,
                      cw1_world_spawner::Task1Service::Response &response)
{
  ROS_INFO("Task 1 callback triggered");
  return performPickAndPlace(request.object_loc, request.goal_loc);
}

// ========== 核心抓取放置逻辑写在这个函数里 ==========
bool cw1::performPickAndPlace(geometry_msgs::PoseStamped object_pose,
                              geometry_msgs::PointStamped basket_point)
{
  // 1) 打印信息
  printPoseStamped(object_pose);
  printPointStamped(basket_point);

  // ====================== 生成“末端朝下 + 45°绕Z”的四元数 ======================
  // 例如：先翻转180° (roll=M_PI) 让Z轴朝下，再绕Z轴+45°(yaw=M_PI/4)
  // 在你同学python里是: down_orientation = euler(-3.14,0,0), yaw=3.14/4 => multiply
  tf2::Quaternion downQ, yawQ, finalQ;
  // 或者 roll = -M_PI, pitch=0, yaw=0 也可以，只要保证“朝下”
  downQ.setRPY(-M_PI, 0.0, 0.0);   
  yawQ.setRPY(0.0, 0.0, M_PI/4.0); // 45 deg
  // tf docs: final = yawQ * downQ => 先应用 down，再叠加 yaw
  finalQ = yawQ * downQ;
  finalQ.normalize(); // 归一化

  // 把它变成 geometry_msgs/Quaternion
  geometry_msgs::Quaternion corrected_orientation = tf2::toMsg(finalQ);

  // ====================== 2) pre_grasp_pose ======================
  geometry_msgs::PoseStamped pre_grasp_pose = object_pose;
  pre_grasp_pose.pose.position.z += 0.20;
  // 让 pre_grasp_pose 始终使用上面计算的 corrected_orientation
  pre_grasp_pose.pose.orientation = corrected_orientation;

  // ====================== 3) 抓取 pose ======================
  object_pose.pose.position.z += 0.13;
  object_pose.pose.orientation = corrected_orientation;

  // ====================== 4) place pose ======================
  geometry_msgs::PoseStamped place_pose;
  place_pose.header = basket_point.header;
  place_pose.pose.position = basket_point.point;
  place_pose.pose.position.z += 0.30;
  place_pose.pose.orientation = corrected_orientation;

  // ===== 原本的抓取逻辑不变(先打开手爪→移动到 pre_grasp→下落→抓取→抬起→移动到 place→松爪) =====
  ROS_WARN("Using corrected_orientation = (Down + 45deg in world) for pre_grasp/obj/place");

  // 5) 动作流程：先打开夹爪
  if(!moveGripper(gripper_open_))
  {
    ROS_ERROR("Failed to open gripper before pick");
    return false;
  }

  // 6) 移动到 pre_grasp_pose
  if(!moveArm(pre_grasp_pose.pose))
  {
    ROS_ERROR("Failed to move arm to pre-grasp");
    return false;
  }

  // 7) 下落到 object_pose
  if(!moveArm(object_pose.pose))
  {
    ROS_ERROR("Failed to move arm to object");
    return false;
  }

  // 8) 闭合夹爪
  if(!moveGripper(gripper_closed_))
  {
    ROS_ERROR("Failed to close gripper on object");
    return false;
  }

  // 9) 抬回 pre_grasp_pose
  if(!moveArm(pre_grasp_pose.pose))
  {
    ROS_ERROR("Failed to retreat arm to pre-grasp");
    return false;
  }

  // 10) 移动到 place_pose
  if(!moveArm(place_pose.pose))
  {
    ROS_ERROR("Failed to move arm to place pose");
    return false;
  }

  // 11) 松开手爪
  if(!moveGripper(gripper_open_))
  {
    ROS_ERROR("Failed to open gripper at basket");
    return false;
  }

  ROS_INFO("pick & place completed successfully with 45° yaw wrt world & downward orientation");
  return true;
}


// ========== 其余任务 (Task2, Task3) 保留空实现 ==========

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
  obj_rec_tutorial.cloudCallBackOne(cloud_msg);
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
  return true;
}

// ========== 机械臂 / 手爪动作的具体实现不变 ==========

bool cw1::moveArm(const geometry_msgs::Pose target_pose)
{
  ROS_INFO("Setting pose target");
  arm_group_.setPoseTarget(target_pose);

  std::string planning_frame = arm_group_.getPlanningFrame();
  ROS_INFO("Planning frame: %s", planning_frame.c_str());

  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_group_.plan(my_plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");
  if (success)
  {
    arm_group_.move();
    return true;
  }
  else
  {
    return false;
  }
}

bool cw1::moveGripper(float width)
{
  if (width > gripper_open_)  width = gripper_open_;
  if (width < gripper_closed_) width = gripper_closed_;

  double eachJoint = width / 2.0;
  std::vector<double> gripperJointTargets(2, eachJoint);

  hand_group_.setJointValueTarget(gripperJointTargets);

  ROS_INFO("Attempting to plan the path (gripper)");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (hand_group_.plan(my_plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success)
  {
    hand_group_.move();
    return true;
  }
  else
  {
    return false;
  }
}

