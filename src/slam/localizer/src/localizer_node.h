#pragma once

#include <queue>
#include <mutex>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "localizers/tic_toc.h"
#include "localizers/commons.h"
#include "localizers/icp_localizer.h"
#include "interface/srv/relocalize.hpp"
#include "interface/srv/is_valid.hpp"
#include <yaml-cpp/yaml.h>

// using namespace std::chrono_literals;

struct NodeConfig
{
  std::string cloud_topic = "/fastlio2/body_cloud";
  std::string odom_topic = "/fastlio2/lio_odom";
  std::string map_frame = "map";
  std::string local_frame = "lidar";
  std::string map_path = "pointcloud.pcd";
  double update_hz = 1.0;
};

struct NodeState
{
  std::mutex message_mutex;
  std::mutex service_mutex;

  bool message_received = false;
  bool service_received = false;
  bool localize_success = false;
  rclcpp::Time last_send_tf_time = rclcpp::Clock().now();
  builtin_interfaces::msg::Time last_message_time;
  CloudType::Ptr last_cloud = std::make_shared<CloudType>();
  M3D last_r;                          // localmap_body_r
  V3D last_t;                          // localmap_body_t
  M3D last_offset_r = M3D::Identity(); // map_localmap_r
  V3D last_offset_t = V3D::Zero();     // map_localmap_t
  M4F initial_guess = M4F::Identity();
};

class LocalizerNode : public rclcpp::Node {
 public:
  LocalizerNode(/* args */);
  ~LocalizerNode() {};

  void loadParameters();
  void timerCB();
  void syncCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg, 
              const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg);
  void sendBroadCastTF(builtin_interfaces::msg::Time &time);
  void pointBodyToWorld(PointType const * const pi, PointType * const po, 
                        const M3D& rot, const V3D& pos);
  Eigen::Vector3d QuatnionToEulerAngles(Eigen::Quaterniond quat);
  void InitialPoseCB(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr init_pose);
  void relocCB(const std::shared_ptr<interface::srv::Relocalize::Request> request, 
               std::shared_ptr<interface::srv::Relocalize::Response> response);
  void relocCheckCB(const std::shared_ptr<interface::srv::IsValid::Request> request, 
                    std::shared_ptr<interface::srv::IsValid::Response> response);
  void publishMapCloud(builtin_interfaces::msg::Time &time);
  
 private:
  NodeConfig m_config;
  NodeState m_state;

  ICPConfig m_localizer_config;
  std::shared_ptr<ICPLocalizer> m_localizer;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  Eigen::Quaterniond q_body_sensor_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
  message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
  rclcpp::TimerBase::SharedPtr m_timer;
  std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>> m_sync;
  std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
  rclcpp::Service<interface::srv::Relocalize>::SharedPtr m_reloc_srv;
  rclcpp::Service<interface::srv::IsValid>::SharedPtr m_reloc_check_srv;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_map_cloud_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_scan_cloud_pub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_initial_pose_sub;

};
