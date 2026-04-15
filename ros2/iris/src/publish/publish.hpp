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

#pragma once
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <pcl/correspondence.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace iris
{
using PointCloud2Pub = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr;
using MarkerPub = rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr;
using MarkerArrayPub = rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr;
using PathPub = rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr;

void publishImage(image_transport::Publisher & publisher, const cv::Mat & image);

void publishPose(
  tf2_ros::TransformBroadcaster & br,
  const Eigen::Matrix4f & T,
  const std::string & child_frame_id,
  const rclcpp::Time & stamp);

void publishPointcloud(
  PointCloud2Pub & publisher,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const rclcpp::Time & stamp);

void publishPath(
  PathPub & publisher,
  const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> & path,
  const rclcpp::Time & stamp);

void publishCorrespondences(
  MarkerPub & publisher,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & source,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & target,
  const pcl::CorrespondencesPtr & correspondences,
  const rclcpp::Time & stamp);

void publishNormal(
  MarkerArrayPub & publisher,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const pcl::PointCloud<pcl::Normal>::Ptr & normals,
  const rclcpp::Time & stamp);

void publishCovariance(
  MarkerArrayPub & publisher,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const pcl::PointCloud<pcl::Normal>::Ptr & normals,
  const rclcpp::Time & stamp);

void publishResetPointcloud(PointCloud2Pub & publisher, const rclcpp::Time & stamp);
void publishResetCorrespondences(MarkerPub & publisher, const rclcpp::Time & stamp);

}  // namespace iris
