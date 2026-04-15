// Copyright (c) 2020, Map IV, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the Map IV, Inc. nor the names of its contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "core/types.hpp"
#include "map/map.hpp"
#include "publish/publish.hpp"
#include "system/system.hpp"
#include <chrono>
#include <cinttypes>
#include <fstream>
#include <iomanip>
#include <limits>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class IrisNode : public rclcpp::Node
{
public:
  IrisNode()
  : Node("iris_node"),
    vslam_data_(new pcl::PointCloud<pcl::PointXYZINormal>),
    T_recover_(Eigen::Matrix4f::Zero())
  {
    // Parameters
    declare_parameter("iris_config_path", "");
    declare_parameter("pcd_path", "");
    const std::string config_path = get_parameter("iris_config_path").as_string();
    const std::string pcd_path = get_parameter("pcd_path").as_string();
    RCLCPP_INFO(get_logger(), "config_path: %s, pcd_path: %s",
      config_path.c_str(), pcd_path.c_str());

    // Initialize algorithm
    iris::Config config(config_path);
    iris::map::Parameter map_param(
      pcd_path, config.voxel_grid_leaf, config.normal_search_leaf, config.submap_grid_leaf);
    map_ = std::make_shared<iris::map::Map>(map_param, config.T_init);
    system_ = std::make_shared<iris::System>(config, map_);

    offseted_vslam_pose_ = config.T_init;
    iris_pose_ = config.T_init;

    // TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    // Subscribers
    vslam_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "iris/vslam_data", 5,
      [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) {
        pcl::fromROSMsg(*msg, *vslam_data_);
        if (vslam_data_->size() > 0) vslam_update_ = true;
      });

    recover_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 5,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg) {
        onRecoverPose(msg);
      });

    // Publishers (latched QoS for map pointclouds)
    auto latch_qos = rclcpp::QoS(1).transient_local();
    target_pc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("iris/target_pointcloud", latch_qos);
    whole_pc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("iris/whole_pointcloud", latch_qos);
    source_pc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("iris/source_pointcloud", 1);
    iris_path_pub_ = create_publisher<nav_msgs::msg::Path>("iris/iris_path", 1);
    vslam_path_pub_ = create_publisher<nav_msgs::msg::Path>("iris/vslam_path", 1);
    correspondences_pub_ = create_publisher<visualization_msgs::msg::Marker>("iris/correspondences", 1);
    scale_pub_ = create_publisher<std_msgs::msg::Float32>("iris/align_scale", 1);
    time_pub_ = create_publisher<std_msgs::msg::Float32>("iris/processing_time", 1);

    // Publish initial map
    auto now = this->now();
    iris::publishPointcloud(whole_pc_pub_, map_->getSparseCloud(), now);
    iris::publishPointcloud(target_pc_pub_, map_->getTargetCloud(), now);
    whole_pointcloud_ = map_->getSparseCloud();

    // Main loop timer at 20 Hz
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&IrisNode::onTimer, this));

    RCLCPP_INFO(get_logger(), "start main loop.");
  }

private:
  void onRecoverPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg)
  {
    RCLCPP_INFO(get_logger(), "/initialpose subscribed");

    float x = static_cast<float>(msg->pose.pose.position.x);
    float y = static_cast<float>(msg->pose.pose.position.y);
    float qw = static_cast<float>(msg->pose.pose.orientation.w);
    float qz = static_cast<float>(msg->pose.pose.orientation.z);

    float z = std::numeric_limits<float>::max();
    if (whole_pointcloud_ == nullptr) {
      z = 0;
    } else {
      for (const pcl::PointXYZ & p : *whole_pointcloud_) {
        constexpr float r2 = 5 * 5;
        float dx = x - p.x;
        float dy = y - p.y;
        if (dx * dx + dy * dy < r2) z = std::min(z, p.z);
      }
    }

    T_recover_.setIdentity();
    T_recover_(0, 3) = x;
    T_recover_(1, 3) = y;
    T_recover_(2, 3) = z;
    float theta = 2 * std::atan2(qz, qw);
    Eigen::Matrix3f R;
    R << 0, 0, 1,
        -1, 0, 0,
        0, -1, 0;
    T_recover_.topLeftCorner(3, 3) =
      Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()).toRotationMatrix() * R;
  }

  Eigen::Matrix4f listenTransform()
  {
    try {
      auto t = tf_buffer_->lookupTransform("world", "iris/vslam_pose", tf2::TimePointZero);
      Eigen::Quaternionf q(
        static_cast<float>(t.transform.rotation.w),
        static_cast<float>(t.transform.rotation.x),
        static_cast<float>(t.transform.rotation.y),
        static_cast<float>(t.transform.rotation.z));
      Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
      T.topLeftCorner(3, 3) = q.toRotationMatrix();
      T(0, 3) = static_cast<float>(t.transform.translation.x);
      T(1, 3) = static_cast<float>(t.transform.translation.y);
      T(2, 3) = static_cast<float>(t.transform.translation.z);
      return T;
    } catch (...) {
      return Eigen::Matrix4f::Identity();
    }
  }

  void onTimer()
  {
    auto now = this->now();
    Eigen::Matrix4f T_vslam = listenTransform();

    if (!T_recover_.isZero()) {
      system_->specifyTWorld(T_recover_);
      T_recover_.setZero();
    }

    if (vslam_update_) {
      vslam_update_ = false;
      auto m_start = std::chrono::system_clock::now();

      rclcpp::Time process_stamp = pcl_conversions::fromPCL(vslam_data_->header.stamp);

      system_->execute(2, T_vslam, vslam_data_);

      iris::Publication publication;
      system_->popPublication(publication);

      iris::publishPointcloud(source_pc_pub_, publication.cloud, now);
      iris::publishPath(iris_path_pub_, publication.iris_trajectory, now);
      iris::publishPath(vslam_path_pub_, publication.offset_trajectory, now);
      iris::publishCorrespondences(
        correspondences_pub_, publication.cloud, map_->getTargetCloud(),
        publication.correspondences, now);

      if (last_map_info_ != map_->getLocalmapInfo()) {
        iris::publishPointcloud(target_pc_pub_, map_->getTargetCloud(), now);
      }
      last_map_info_ = map_->getLocalmapInfo();

      int64_t time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now() - m_start).count();
      RCLCPP_INFO(get_logger(), "Iris/ALIGN: processing time= %" PRId64 " ms", time_ms);

      std_msgs::msg::Float32 scale_msg;
      scale_msg.data = iris::util::getScale(publication.T_align);
      scale_pub_->publish(scale_msg);

      std_msgs::msg::Float32 time_msg;
      time_msg.data = static_cast<float>(time_ms);
      time_pub_->publish(time_msg);

      offseted_vslam_pose_ = publication.offset_camera;
      iris_pose_ = publication.iris_camera;

      writeCsv(process_stamp, iris_pose_);
    }

    iris::publishPose(*tf_broadcaster_, offseted_vslam_pose_, "iris/offseted_vslam_pose", now);
    iris::publishPose(*tf_broadcaster_, iris_pose_, "iris/iris_pose", now);
  }

  void writeCsv(const rclcpp::Time & stamp, const Eigen::Matrix4f & iris_pose)
  {
    if (!ofs_track_.is_open()) ofs_track_.open("trajectory.csv");
    auto convert = [](const Eigen::MatrixXf & mat) -> Eigen::VectorXf {
      Eigen::MatrixXf tmp = mat.transpose();
      return Eigen::VectorXf(Eigen::Map<Eigen::VectorXf>(tmp.data(), mat.size()));
    };
    ofs_track_ << std::fixed
               << std::setprecision(std::numeric_limits<double>::max_digits10)
               << stamp.seconds() << " "
               << convert(iris_pose).transpose() << "\n";
  }

  // State
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr vslam_data_;
  bool vslam_update_{false};
  Eigen::Matrix4f T_recover_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr whole_pointcloud_{nullptr};
  Eigen::Matrix4f offseted_vslam_pose_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f iris_pose_{Eigen::Matrix4f::Identity()};
  iris::map::Info last_map_info_;
  std::ofstream ofs_track_;

  // Algorithm
  std::shared_ptr<iris::map::Map> map_;
  std::shared_ptr<iris::System> system_;

  // ROS
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr vslam_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr recover_sub_;
  iris::PointCloud2Pub target_pc_pub_, whole_pc_pub_, source_pc_pub_;
  iris::PathPub iris_path_pub_, vslam_path_pub_;
  iris::MarkerPub correspondences_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr scale_pub_, time_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IrisNode>());
  rclcpp::shutdown();
  return 0;
}
