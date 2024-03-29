#include "ndt_scan_matching/ndt_scan_matching.hpp"

NDTScanMatching::NDTScanMatching(const rclcpp::NodeOptions & node_options)
: Node("ndt_scan_matching", node_options)
{
  ndt_ptr_ = std::make_shared<NDT<pcl::PointXYZ>>();

  const double resolution = this->declare_parameter("resolution", 2.0);

  ndt_ptr_->setResolution(resolution);

  map_points_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&NDTScanMatching::mapCallback, this, std::placeholders::_1));

  sensor_points_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_raw", rclcpp::SensorDataQoS().keep_last(5),
    std::bind(&NDTScanMatching::sensorCallback, this, std::placeholders::_1));

  initialpose_subscriber_ =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 1,
      std::bind(&NDTScanMatching::initialPoseCallback, this, std::placeholders::_1));
}

void NDTScanMatching::mapCallback(const sensor_msgs::msg::PointCloud2 msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(msg, *map);

  ndt_ptr_->setInputTarget(map);
}

void NDTScanMatching::sensorCallback(const sensor_msgs::msg::PointCloud2 msg) {}

void NDTScanMatching::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped msg)
{
  initial_pose_ = msg;

  // fot demo
  pcl::PointXYZ p;
  p.x = msg.pose.pose.position.x;
  p.y = msg.pose.pose.position.y;
  p.z = 0.0;

  Eigen::Affine3d initialpose_affine;
  tf2::fromMsg(msg.pose.pose, initialpose_affine);
  Eigen::Matrix4f init_guess = initialpose_affine.matrix().cast<float>();

  pcl::PointCloud<pcl::PointXYZ> input_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  ndt_ptr_->align(output_cloud, init_guess);

  std::vector<Leaf> leaf_vec;
  ndt_ptr_->radiusSearch(p, 2.0, leaf_vec);

  utils::transform_point_cloud<pcl::PointXYZ>(input_cloud, *output_cloud, init_guess);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(NDTScanMatching)
