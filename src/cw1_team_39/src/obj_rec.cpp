/* Software License Agreement (MIT License)
*
*  Copyright (c) 2019-, Dimitrios Kanoulas
*
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the copyright holder(s) nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*/

#include "obj_rec.h"
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC; 
typedef PointC::Ptr PointCPtr;

////////////////////////////////////////////////////////////////////////////////
ObjRec::ObjRec (ros::NodeHandle &nh):
  g_cloud_ptr (new PointC), // input point cloud
  g_cloud_filtered (new PointC), // filtered point cloud
  g_cloud_filtered2 (new PointC), // filtered point cloud
  g_cloud_plane (new PointC), // plane point cloud
  g_cloud_cylinder (new PointC), // cylinder point cloud
  g_cloud_box (new PointC), // box point cloud
  g_cloud_blue (new PointC), // blue point cloud
  g_cloud_red (new PointC), // red point cloud
  g_cloud_purple (new PointC), // purple point cloud
  g_tree_ptr (new pcl::search::KdTree<PointT> ()), // KdTree
  g_cloud_normals (new pcl::PointCloud<pcl::Normal>), // segmentation
  g_cloud_normals2 (new pcl::PointCloud<pcl::Normal>), // segmentation
  g_inliers_plane (new pcl::PointIndices), // plane seg
  g_inliers_cylinder (new pcl::PointIndices), // cylidenr seg
  g_coeff_plane (new pcl::ModelCoefficients), // plane coeff
  g_coeff_cylinder (new pcl::ModelCoefficients), // cylinder coeff
  g_coeff_box (new pcl::ModelCoefficients), // box coeff
  debug_ (false)
{
  g_nh = nh;

  // Define the publishers
  g_pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("filtered_cloud", 1, true);
  g_pub_cloud_red = nh.advertise<sensor_msgs::PointCloud2> ("filtered_cloud_red", 1, true);
  g_pub_cloud_blue = nh.advertise<sensor_msgs::PointCloud2> ("filtered_cloud_blue", 1, true);
  g_pub_cloud_purple = nh.advertise<sensor_msgs::PointCloud2> ("filtered_cloud_purple", 1, true);
  g_pub_pose = nh.advertise<geometry_msgs::PointStamped> ("detected_pos", 1, true);

  pub_box_poses_  = nh.advertise<geometry_msgs::PoseArray>("box_poses",  1);
  pub_block_poses_ = nh.advertise<geometry_msgs::PoseArray>("block_poses", 1);

  
  // Define public variables
  g_vg_leaf_sz = 0.01; // VoxelGrid leaf size: Better in a config file
  g_pt_thrs_min = -0.01; // PassThrough min thres: Better in a config file
  g_pt_thrs_max = 0.06; // PassThrough max thres: Better in a config file
  g_k_nn = 50; // Normals nn size: Better in a config file
}

////////////////////////////////////////////////////////////////////////////////
void
ObjRec::cloudCallBackOne
  (const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{
  // Extract inout point cloud info
  g_input_pc_frame_id_ = cloud_input_msg->header.frame_id;
    
  // Convert to PCL data type
  pcl_conversions::toPCL (*cloud_input_msg, g_pcl_pc);
  pcl::fromPCLPointCloud2 (g_pcl_pc, *g_cloud_ptr);

  // Perform the filtering
  // applyVX (g_cloud_ptr, g_cloud_filtered);
  // applyPT (g_cloud_ptr, g_cloud_filtered);
  // Copy the point cloud
  pcl::copyPointCloud(*g_cloud_ptr, *g_cloud_filtered);
  

  findNormals (g_cloud_ptr);
  segPlane (g_cloud_ptr);
  segColor (g_cloud_filtered2);
  findObjectsPose();
  publishObjectsInfo();
  // Publish the data
  ROS_INFO ("Publishing Filtered Cloud 2");
  pubFilteredPCMsg (g_pub_cloud, *g_cloud_filtered2);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
ObjRec::applyVX (PointCPtr &in_cloud_ptr,
                      PointCPtr &out_cloud_ptr)
{
  g_vx.setInputCloud (in_cloud_ptr);
  g_vx.setLeafSize (g_vg_leaf_sz, g_vg_leaf_sz, g_vg_leaf_sz);
  g_vx.filter (*out_cloud_ptr);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
ObjRec::applyPT (PointCPtr &in_cloud_ptr,
                      PointCPtr &out_cloud_ptr)
{
  g_pt.setInputCloud (in_cloud_ptr);
  g_pt.setFilterFieldName ("y");
  g_pt.setFilterLimits (g_pt_thrs_min, g_pt_thrs_max);
  g_pt.filter (*out_cloud_ptr);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
ObjRec::findNormals (PointCPtr &in_cloud_ptr)
{
  // Estimate point normals
  g_ne.setInputCloud (in_cloud_ptr);
  g_ne.setSearchMethod (g_tree_ptr);
  g_ne.setKSearch (g_k_nn);
  g_ne.compute (*g_cloud_normals);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
ObjRec::segPlane (PointCPtr &in_cloud_ptr)
{
  // Create the segmentation object for the planar model
  // and set all the params
  g_seg.setOptimizeCoefficients (true);
  g_seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  g_seg.setNormalDistanceWeight (0.1); //bad style
  g_seg.setMethodType (pcl::SAC_RANSAC);
  g_seg.setMaxIterations (100); //bad style
  g_seg.setDistanceThreshold (0.03); //bad style
  g_seg.setInputCloud (in_cloud_ptr);
  g_seg.setInputNormals (g_cloud_normals);
  // Obtain the plane inliers and coefficients
  g_seg.segment (*g_inliers_plane, *g_coeff_plane);
  
  // Extract the planar inliers from the input cloud
  g_extract_pc.setInputCloud (in_cloud_ptr);
  g_extract_pc.setIndices (g_inliers_plane);
  g_extract_pc.setNegative (false);
  
  // Write the planar inliers to disk
  g_extract_pc.filter (*g_cloud_plane);
  
  // Remove the planar inliers, extract the rest
  g_extract_pc.setNegative (true);
  g_extract_pc.filter (*g_cloud_filtered2);
  g_extract_normals.setNegative (true);
  g_extract_normals.setInputCloud (g_cloud_normals);
  g_extract_normals.setIndices (g_inliers_plane);
  g_extract_normals.filter (*g_cloud_normals2);

  //ROS_INFO_STREAM ("Plane coefficients: " << *g_coeff_plane);
  ROS_INFO_STREAM ("PointCloud representing the planar component: "
                   << g_cloud_plane->size ()
                   << " data points.");
}
////////////////////////////////////////////////////////////////////////////////
void 
ObjRec::segBox(PointCPtr &in_cloud_ptr)
{
  // 1. 清空用于临时存储分割后结果的 g_cloud_box
  g_cloud_box->points.clear();

  // 2. 颜色阈值
  float thresh = 0.1f;

  // 3. 定义目标 RGB，与 segBox 原逻辑一致
  std::map<std::string, Eigen::Vector3f> color_map = {
      {"blue",   Eigen::Vector3f(0.1f, 0.1f, 0.8f)},
      {"red",    Eigen::Vector3f(0.8f, 0.1f, 0.1f)},
      {"purple", Eigen::Vector3f(0.8f, 0.1f, 0.8f)}
  };

  // 4. 用于记录本次分割得到的实际颜色（可能为 "unknown"）
  std::string detected_color = "unknown";

  // ---- 先做颜色过滤 ----
  // 遍历输入点云，对每个点进行颜色比较
  for (const auto &point : in_cloud_ptr->points) {
    float r = static_cast<float>(point.r) / 255.0f;
    float g = static_cast<float>(point.g) / 255.0f;
    float b = static_cast<float>(point.b) / 255.0f;

    Eigen::Vector3f point_rgb(r, g, b);

    // 遍历 color_map，看该点是否满足阈值
    for (const auto& [color_name, target_rgb] : color_map) {
      float dist = (point_rgb - target_rgb).norm();
      // 如果点与某颜色足够接近，且此前尚未确定最终颜色，则确定其颜色
      if (dist < thresh && detected_color == "unknown") {
        detected_color = color_name;
      }
      // 如果该点与 detected_color 对应的目标颜色接近，则将该点收入 g_cloud_box
      if (dist < thresh && color_name == detected_color) {
        g_cloud_box->points.push_back(point);
      }
    }
  }

  // 如果没有检测到任何颜色或者没有任何点进入 g_cloud_box，直接返回
  if (detected_color == "unknown" || g_cloud_box->points.empty()) {
    ROS_INFO_STREAM("No color or no points matched in segBox!");
    return;
  }

  // 更新一下 g_cloud_box 的宽高等
  g_cloud_box->width = g_cloud_box->points.size();
  g_cloud_box->height = 1;
  g_cloud_box->is_dense = false;

  // ---- 接下来做 box / block 区分 ----
  // 1) 按 z 坐标对 g_cloud_box->points 排序
  //    这里可以直接复制到一个临时 vector 里排序，也可以在原地排序
  std::vector<PointT, Eigen::aligned_allocator<PointT>> sorted_points = g_cloud_box->points;
  std::sort(sorted_points.begin(), sorted_points.end(),
            [](const PointT &a, const PointT &b)
            {
              return a.z < b.z;
            });

  // 2) 取最高的前 10% 点
  size_t n_points = sorted_points.size();
  // 若小于10个点，则 10% 可能会是 0，这里做个保护
  size_t n_top = static_cast<size_t>(std::ceil(n_points * 0.1));
  if (n_top == 0) {
    // 如果还不足以取 10%，这里简单认为是 block 或根据需要做处理
    n_top = 1; // 至少取一个点来判断
  }

  // 取末尾 n_top 个点
  float sum_z = 0.0f;
  for (size_t i = n_points - n_top; i < n_points; ++i) {
    sum_z += sorted_points[i].z;
  }
  float avg_top_z = sum_z / static_cast<float>(n_top);

  // 3) 大于 0.06 => box，否则 => block
  bool is_box = (avg_top_z > 0.08f);

  // ---- 计算该聚类的中心 ----
  // 注意，这里取的是 g_cloud_box 里所有点的中心；也可按需求仅对顶部点计算。
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*g_cloud_box, centroid);

  // ---- 根据是否为 box，存储到相应的数据结构 ----
  if (is_box) {
    // 根据颜色更新对应 box 的中心
    // 有三个可选 color: red / blue / purple
    // 维护一个索引，0: red, 1: blue, 2: purple
    int color_idx = -1;
    if      (detected_color == "red")    color_idx = 0;
    else if (detected_color == "blue")   color_idx = 1;
    else if (detected_color == "purple") color_idx = 2;

    // 未被支持的颜色，或者未知，则直接返回也行
    if (color_idx < 0) {
      // ROS_WARN_STREAM("Detected color is not in {red,blue,purple}, skip storing box.");
      return;
    }

    // 如果之前没有检测到该颜色 box，则直接存储
    if (!box_found[color_idx]) {
      box_found[color_idx] = true;
      if (color_idx == 0) {
        red_box_pos = centroid;
      } else if (color_idx == 1) {
        blue_box_pos = centroid;
      } else if (color_idx == 2) {
        purple_box_pos = centroid;
      }
    } 
    else {
      // 如果已经检测到过该颜色的 box，则比较与原点的距离，保留更近的
      Eigen::Vector3f old_center, new_center;
      if (color_idx == 0) {
        old_center = red_box_pos.head<3>();
      } else if (color_idx == 1) {
        old_center = blue_box_pos.head<3>();
      } else {
        old_center = purple_box_pos.head<3>();
      }
      new_center = centroid.head<3>();

      float old_dist = old_center.norm();
      float new_dist = new_center.norm();

      if (new_dist < old_dist) {
        // 覆盖
        if (color_idx == 0) {
          red_box_pos = centroid;
        } else if (color_idx == 1) {
          blue_box_pos = centroid;
        } else {
          purple_box_pos = centroid;
        }
      }
    }
    // 可以打印一些日志信息
    ROS_INFO_STREAM("Detected box color: " << detected_color << ", center = " << centroid.transpose());

  } else {
    // 如果是 block，则把中心存入 block_pos
    block_pos.push_back(centroid);
    // 如果想要索引或其他信息，也可以记录
    // current_block_idx++;
    ROS_INFO_STREAM("Detected block (color=" << detected_color << "), center = " << centroid.transpose());
  }
}

////////////////////////////////////////////////////////////////////////////////
void ObjRec::segColor(PointCPtr &in_cloud_ptr)
{
  g_cloud_blue->points.clear();
  g_cloud_red->points.clear();
  g_cloud_purple->points.clear();

  float thresh = 0.1f;
  std::map<std::string, Eigen::Vector3f> color_map = {
      {"blue",   Eigen::Vector3f(0.1f, 0.1f, 0.8f)},
      {"red",    Eigen::Vector3f(0.8f, 0.1f, 0.1f)},
      {"purple", Eigen::Vector3f(0.8f, 0.1f, 0.8f)}
  };

  for (const auto &point : in_cloud_ptr->points) {
    if (point.z < 0.03) {
      continue;
    }

    float r = static_cast<float>(point.r) / 255.0f;
    float g = static_cast<float>(point.g) / 255.0f;
    float b = static_cast<float>(point.b) / 255.0f;

    Eigen::Vector3f point_rgb(r, g, b);

    if ((point_rgb - color_map["blue"]).norm() < thresh) {
      g_cloud_blue->points.push_back(point);
    }
    if ((point_rgb - color_map["red"]).norm() < thresh) {
      g_cloud_red->points.push_back(point);
    }
    if ((point_rgb - color_map["purple"]).norm() < thresh) {
      g_cloud_purple->points.push_back(point);
    }
  }

  std::map<std::string, PointCPtr> cloud_map = {
      {"blue", g_cloud_blue},
      {"red", g_cloud_red},
      {"purple", g_cloud_purple}
  };

  for (auto& [color_name, cloud_ptr] : cloud_map) {
    cloud_ptr->width = cloud_ptr->points.size();
    cloud_ptr->height = 1;
    cloud_ptr->is_dense = false;

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_ptr, cloud_msg);
    cloud_msg.header.frame_id = g_input_pc_frame_id_;

    if (color_name == "blue") {
      g_pub_cloud_blue.publish(cloud_msg);
    } else if (color_name == "red") {
      g_pub_cloud_red.publish(cloud_msg);
    } else if (color_name == "purple") {
      g_pub_cloud_purple.publish(cloud_msg);
    }
  }

  ROS_INFO_STREAM("Segmented color clouds: "
                  << "Blue: " << g_cloud_blue->size() << " points, "
                  << "Red: " << g_cloud_red->size() << " points, "
                  << "Purple: " << g_cloud_purple->size() << " points.");
}
////////////////////////////////////////////////////////////////////////////////
void ObjRec::findObjectsPose()
{
  // 这里的 cluster_radius 是欧式聚类的距离阈值，影响一个点和另一个点
  // 是否判定为同一 cluster。可以根据实际情况调整。
  float cluster_radius = 0.005f;

  // 几个常用的聚类相关参数，实际项目中可根据效果进行调优
  int min_cluster_size = 30;    // 最小点数，小于这个数量的 cluster 将被忽略
  int max_cluster_size = 25000; // 最大点数，大于这个数量的 cluster 将被忽略

  // 建立颜色名称到对应点云的映射
  std::map<std::string, PointCPtr> cloud_map = {
      {"blue",   g_cloud_blue},
      {"red",    g_cloud_red},
      {"purple", g_cloud_purple}
  };

  // 对三种颜色分别进行处理
  for (auto& [color_name, cloud_ptr] : cloud_map) {
    // 如果对应颜色的点云里有数据，则先进行聚类，再对每个 cluster 调用 segBox
    if (cloud_ptr->size() > 0) {
      // 1. 构造一个 kd-tree，以便进行聚类搜索
      ROS_INFO_STREAM("[" << color_name << "] Start clustering..." << cloud_ptr->size() << " points.");
      pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
      tree->setInputCloud(cloud_ptr);

      // 2. 欧式聚类提取器
      pcl::EuclideanClusterExtraction<PointT> ec;
      ec.setClusterTolerance(cluster_radius);
      ec.setMinClusterSize(min_cluster_size);
      ec.setMaxClusterSize(max_cluster_size);
      ec.setSearchMethod(tree);
      ec.setInputCloud(cloud_ptr);

      // 3. 执行聚类，得到若干个 cluster 的索引
      std::vector<pcl::PointIndices> cluster_indices;
      ec.extract(cluster_indices);
      ROS_INFO_STREAM("[" << color_name << "] Number of clusters: " << cluster_indices.size());
      // 4. 遍历每个 cluster，将每个 cluster 的点提取出来形成一个新的点云
      int cluster_id = 0;
      for (const auto& indices : cluster_indices) {
        // 创建一个新的点云来保存当前 cluster
        PointCPtr cluster_cloud(new pcl::PointCloud<PointT>());
        cluster_cloud->points.reserve(indices.indices.size());

        for (auto idx : indices.indices) {
          cluster_cloud->points.push_back(cloud_ptr->points[idx]);
        }
        cluster_cloud->width = cluster_cloud->points.size();
        cluster_cloud->height = 1;
        cluster_cloud->is_dense = true;

        // 对该 cluster 调用 segBox（或你自己的后续处理函数）
        segBox(cluster_cloud);

        // 这里可以记录 log 或发布一些信息
        ROS_INFO_STREAM("[" << color_name << "] cluster " << cluster_id
                            << " => " << cluster_cloud->size() << " points.");

        cluster_id++;
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void ObjRec::publishObjectsInfo()
{
  //--- 1) 发布 box 信息 ---
  //   使用 PoseArray 来一次性发布多个 box 的位姿
  geometry_msgs::PoseArray box_poses;
  box_poses.header.frame_id = "map";               // 或者您所使用的坐标系
  box_poses.header.stamp = ros::Time::now();

  // 依次检查 red/blue/purple 是否存在，如果存在就存入 PoseArray
  // 注意：我们在示例中设定 box_found[0] 对应 red_box_pos,
  //                         box_found[1] 对应 blue_box_pos,
  //                         box_found[2] 对应 purple_box_pos

  if (box_found[0])  // red
  {
    geometry_msgs::Pose pose_red;
    pose_red.position.x = red_box_pos[0];
    pose_red.position.y = red_box_pos[1];
    pose_red.position.z = red_box_pos[2];
    // Orientation 如无特殊需求，可设成单位四元数
    pose_red.orientation.x = 0.0;
    pose_red.orientation.y = 0.0;
    pose_red.orientation.z = 0.0;
    pose_red.orientation.w = 1.0;

    box_poses.poses.push_back(pose_red);
  }

  if (box_found[1])  // blue
  {
    geometry_msgs::Pose pose_blue;
    pose_blue.position.x = blue_box_pos[0];
    pose_blue.position.y = blue_box_pos[1];
    pose_blue.position.z = blue_box_pos[2];

    pose_blue.orientation.w = 1.0;

    box_poses.poses.push_back(pose_blue);
  }

  if (box_found[2])  // purple
  {
    geometry_msgs::Pose pose_purple;
    pose_purple.position.x = purple_box_pos[0];
    pose_purple.position.y = purple_box_pos[1];
    pose_purple.position.z = purple_box_pos[2];

    pose_purple.orientation.w = 0.0;

    box_poses.poses.push_back(pose_purple);
  }

  // 将 box_poses 发布
  pub_box_poses_.publish(box_poses);

  //--- 2) 发布 block 信息 ---
  //   可能有多个 block，因此也用 PoseArray
  geometry_msgs::PoseArray block_poses;
  block_poses.header.frame_id = "map"; // 同理
  block_poses.header.stamp = ros::Time::now();

  for (auto &pos : block_pos) {
    geometry_msgs::Pose p;
    p.position.x = pos[0];
    p.position.y = pos[1];
    p.position.z = pos[2];
    p.orientation.w = 0.0;

    block_poses.poses.push_back(p);
  }

  // 将 block_poses 发布
  pub_block_poses_.publish(block_poses);
}
////////////////////////////////////////////////////////////////////////////////
void
ObjRec::pubFilteredPCMsg (ros::Publisher &pc_pub,
                               PointC &pc)
{
  // Publish the data
  pcl::toROSMsg(pc, g_cloud_filtered_msg);
  pc_pub.publish (g_cloud_filtered_msg);
  
  return;
}
