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

#include "stereo_bridge.hpp"
#include <chrono>
#include <functional>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <image_transport/image_transport.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using ApproxSync = message_filters::sync_policies::ApproximateTime<
  sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage>;

class OpenVSLAMStereoBridgeNode : public rclcpp::Node
{
public:
  OpenVSLAMStereoBridgeNode()
  : Node("openvslam_stereo_bridge_node"),
    vslam_data_(new pcl::PointCloud<pcl::PointXYZINormal>)
  {
    // Parameters
    declare_parameter("vocab_path", "");
    declare_parameter("vslam_config_path", "");
    declare_parameter("image_topic_name0", "");
    declare_parameter("image_topic_name1", "");
    declare_parameter("is_image_compressed", true);
    declare_parameter("is_image_color", true);
    declare_parameter("keyframe_recollection", 30);
    declare_parameter("max_height", std::numeric_limits<float>::max());

    const std::string vocab_path = get_parameter("vocab_path").as_string();
    const std::string vslam_config_path = get_parameter("vslam_config_path").as_string();
    const std::string topic0 = get_parameter("image_topic_name0").as_string();
    const std::string topic1 = get_parameter("image_topic_name1").as_string();
    is_color_ = get_parameter("is_image_color").as_bool();
    recollection_ = get_parameter("keyframe_recollection").as_int();
    height_ = static_cast<float>(get_parameter("max_height").as_double());

    const bool is_compressed = get_parameter("is_image_compressed").as_bool();
    if (!is_compressed) {
      RCLCPP_ERROR(get_logger(), "Only compressed image is supported");
      throw std::runtime_error("Only compressed image is acceptable");
    }

    RCLCPP_INFO(get_logger(),
      "vocab_path: %s, config: %s, topic0: %s",
      vocab_path.c_str(), vslam_config_path.c_str(), topic0.c_str());

    bridge_.setup(vslam_config_path, vocab_path);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    // Synchronized stereo subscribers
    infra0_sub_.subscribe(this, topic0);
    infra1_sub_.subscribe(this, topic1);
    sync_ = std::make_shared<message_filters::Synchronizer<ApproxSync>>(
      ApproxSync(10), infra0_sub_, infra1_sub_);
    sync_->registerCallback(
      std::bind(&OpenVSLAMStereoBridgeNode::onStereoImage, this,
        std::placeholders::_1, std::placeholders::_2));

    // Publishers
    vslam_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("iris/vslam_data", 1);
    image_pub_ = image_transport::create_publisher(this, "iris/processed_image");

    // Timer at 20 Hz
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&OpenVSLAMStereoBridgeNode::onTimer, this));

    RCLCPP_INFO(get_logger(), "start main loop.");
  }

private:
  void onStereoImage(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & img0,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & img1)
  {
    int flag = is_color_ ? 1 : 0;
    subscribed_image0_ = cv::imdecode(cv::Mat(img0->data), flag);
    subscribed_image1_ = cv::imdecode(cv::Mat(img1->data), flag);
    subscribed_stamp_ = img0->header.stamp;
  }

  void publishPose(const Eigen::Matrix4f & T, const std::string & child_frame_id)
  {
    Eigen::Matrix3f R = T.topLeftCorner(3, 3);
    Eigen::Quaternionf q(R);
    geometry_msgs::msg::TransformStamped ts;
    ts.header.stamp = now();
    ts.header.frame_id = "world";
    ts.child_frame_id = child_frame_id;
    ts.transform.translation.x = T(0, 3);
    ts.transform.translation.y = T(1, 3);
    ts.transform.translation.z = T(2, 3);
    ts.transform.rotation.x = q.x();
    ts.transform.rotation.y = q.y();
    ts.transform.rotation.z = q.z();
    ts.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(ts);
  }

  void onTimer()
  {
    if (!subscribed_image0_.empty() && !subscribed_image1_.empty()) {
      auto m_start = std::chrono::system_clock::now();
      rclcpp::Time process_stamp(subscribed_stamp_);

      bridge_.execute(subscribed_image0_, subscribed_image1_);
      bridge_.setCriteria(recollection_, accuracy_);
      bridge_.getLandmarksAndNormals(vslam_data_, height_);
      subscribed_image0_ = cv::Mat();
      subscribed_image1_ = cv::Mat();

      // Adjust accuracy
      if (vslam_data_->size() < lower_threshold_ && accuracy_ > 0.10f) accuracy_ -= 0.01f;
      if (vslam_data_->size() > upper_threshold_ && accuracy_ < 0.90f) accuracy_ += 0.01f;

      // Publish processed image
      {
        cv::Mat frame = bridge_.getFrame();
        if (!frame.empty()) {
          std_msgs::msg::Header header;
          header.stamp = process_stamp;
          auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
          image_pub_.publish(*msg);
        }
      }

      // Publish vslam data
      {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*vslam_data_, cloud_msg);
        cloud_msg.header.frame_id = "world";
        cloud_msg.header.stamp = process_stamp;
        vslam_pub_->publish(cloud_msg);
      }

      long time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now() - m_start).count();
      RCLCPP_INFO(get_logger(), "processing time= %ld ms", time_ms);
    }

    publishPose(bridge_.getCameraPose().inverse(), "iris/vslam_pose");
  }

  // State
  cv::Mat subscribed_image0_, subscribed_image1_;
  builtin_interfaces::msg::Time subscribed_stamp_;
  float accuracy_{0.5f};
  float height_{std::numeric_limits<float>::max()};
  int recollection_{30};
  bool is_color_{true};
  static constexpr int lower_threshold_ = 1000;
  static constexpr int upper_threshold_ = 1500;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr vslam_data_;

  // Algorithm
  iris::BridgeStereoOpenVSLAM bridge_;

  // ROS
  rclcpp::TimerBase::SharedPtr timer_;
  message_filters::Subscriber<sensor_msgs::msg::CompressedImage> infra0_sub_, infra1_sub_;
  std::shared_ptr<message_filters::Synchronizer<ApproxSync>> sync_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr vslam_pub_;
  image_transport::Publisher image_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OpenVSLAMStereoBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
