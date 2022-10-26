#ifndef _NDT_SCAN_MATCHING_HPP_
#define _NDT_SCAN_MATCHING_HPP_

#include "ndt_scan_matching/ndt.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class NDTScanMatching : public rclcpp::Node
{
public:
  NDTScanMatching(const rclcpp::NodeOptions & node_options);
  ~NDTScanMatching() = default;

  void mapCallback(const sensor_msgs::msg::PointCloud2 msg);
  void sensorCallback(const sensor_msgs::msg::PointCloud2 msg);
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped msg);

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initialpose_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_points_subscriber_;

  std::shared_ptr<NDT<pcl::PointXYZ>> ndt_ptr_;

  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_;
};

#endif
