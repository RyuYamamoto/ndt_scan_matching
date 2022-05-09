#ifndef _NDT_SCAN_MATCHING_HPP_
#define _NDT_SCAN_MATCHING_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "ndt.hpp"

class NDTScanMatching : public rclcpp::Node
{
public:
  NDTScanMatching(const rclcpp::NodeOptions & node_options);
  ~NDTScanMatching() = default;
private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_points_subscriber_;
};

#endif
