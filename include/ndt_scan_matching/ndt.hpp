#ifndef _NDT_HPP_
#define _NDT_HPP_

#include "ndt_scan_matching/voxel_grid.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

template <typename PointType>
class NDT
{
public:
  NDT()
  {
    voxel_grid_covariance_ = std::make_shared<VoxelGridCovariance<PointType>>();
  }
  ~NDT() = default;

  void align(const Eigen::Matrix4f init_guess) {}
  void align(typename pcl::PointCloud<PointType>::Ptr output, const Eigen::Matrix4f init_guess) {}
  void setInputSource(typename const pcl::PointCloud<PointType>::Ptr input) {}
  void setInputTarget(typename const pcl::PointCloud<PointType>::Ptr target) {}
  Eigen::Matrix4f getFinalTransformation() {}

private:
  std::shared_ptr<VoxelGridCovariance> voxel_grid_covariance_;
};

#endif
