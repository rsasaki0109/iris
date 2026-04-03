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

#include "bridge.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <image_transport/image_transport.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class OpenVSLAMBridgeNode : public rclcpp::Node
{
public:
  OpenVSLAMBridgeNode()
  : Node("openvslam_bridge_node"),
    vslam_data_(new pcl::PointCloud<pcl::PointXYZINormal>)
  {
    // Parameters
    declare_parameter("vocab_path", "");
    declare_parameter("vslam_config_path", "");
    declare_parameter("image_topic_name0", "");
    declare_parameter("is_image_compressed", false);
    declare_parameter("keyframe_recollection", 30);

    const std::string vocab_path = get_parameter("vocab_path").as_string();
    const std::string vslam_config_path = get_parameter("vslam_config_path").as_string();
    const std::string image_topic = get_parameter("image_topic_name0").as_string();
    const bool is_compressed = get_parameter("is_image_compressed").as_bool();
    recollection_ = get_parameter("keyframe_recollection").as_int();

    RCLCPP_INFO(get_logger(),
      "vocab_path: %s, config: %s, topic: %s, compressed: %d",
      vocab_path.c_str(), vslam_config_path.c_str(), image_topic.c_str(), is_compressed);

    // Setup OpenVSLAM
    bridge_.setup(vslam_config_path, vocab_path);

    // TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    // Subscriber
    if (is_compressed) {
      compressed_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        image_topic + "/compressed", 5,
        [this](const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg) {
          subscribed_image_ = cv::imdecode(cv::Mat(msg->data), 1);
          subscribed_stamp_ = msg->header.stamp;
        });
    } else {
      image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic, 5,
        [this](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
          auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
          subscribed_image_ = cv_ptr->image.clone();
          subscribed_stamp_ = msg->header.stamp;
        });
    }

    // Publishers
    vslam_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("iris/vslam_data", 1);
    image_pub_ = image_transport::create_publisher(this, "iris/processed_image");

    // Timer at 20 Hz
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&OpenVSLAMBridgeNode::onTimer, this));

    RCLCPP_INFO(get_logger(), "start main loop.");
  }

private:
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
    if (!subscribed_image_.empty()) {
      auto m_start = std::chrono::system_clock::now();
      rclcpp::Time process_stamp(subscribed_stamp_);

      bridge_.execute(subscribed_image_);
      bridge_.setCriteria(recollection_, accuracy_);
      bridge_.getLandmarksAndNormals(vslam_data_, std::numeric_limits<float>::max());
      subscribed_image_ = cv::Mat();

      // Adjust accuracy to keep point count in range
      if (vslam_data_->size() < lower_threshold_ && accuracy_ > 0.10f) accuracy_ -= 0.01f;
      if (vslam_data_->size() > upper_threshold_ && accuracy_ < 0.90f) accuracy_ += 0.01f;

      // Publish processed image
      {
        std_msgs::msg::Header header;
        header.stamp = process_stamp;
        auto msg = cv_bridge::CvImage(header, "bgr8", bridge_.getFrame()).toImageMsg();
        image_pub_.publish(*msg);
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
  cv::Mat subscribed_image_;
  builtin_interfaces::msg::Time subscribed_stamp_;
  float accuracy_{0.5f};
  int recollection_{30};
  static constexpr int lower_threshold_ = 1500;
  static constexpr int upper_threshold_ = 2000;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr vslam_data_;

  // Algorithm
  iris::BridgeOpenVSLAM bridge_;

  // ROS
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr vslam_pub_;
  image_transport::Publisher image_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OpenVSLAMBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
