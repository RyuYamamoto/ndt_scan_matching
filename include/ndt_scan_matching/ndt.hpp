#ifndef _NDT_HPP_
#define _NDT_HPP_

#include "ndt_scan_matching/voxel_grid.hpp"
#include "ndt_scan_matching/utils.hpp"

#include <pcl_ros/transforms.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

template <typename PointType>
class NDT
{
public:
  NDT() : resolution_(2.0), input_(nullptr)
  {
    voxel_grid_covariance_ = std::make_shared<VoxelGridCovariance<PointType>>();
  }
  ~NDT() = default;

  void setResolution(const double resolution) { resolution_ = resolution; }

  void align(typename pcl::PointCloud<PointType>::Ptr output, const Eigen::Matrix4f init_guess)
  {
    if (input_ == nullptr) return;

    // 与えた初期位置から最初に点setInputSourceで与えられた点群を座標変換しておく
    typename pcl::PointCloud<PointType>::Ptr transform_cloud_ptr(new pcl::PointCloud<PointType>);
    pcl::transformPointCloud(*input_, *transform_cloud_ptr, init_guess);

    output = transform_cloud_ptr;
  }
  void setInputSource(const typename pcl::PointCloud<PointType>::Ptr input) { input_ = input; }
  void setInputTarget(const typename pcl::PointCloud<PointType>::Ptr target)
  {
    // NDボクセル生成・KDTree構築
    voxel_grid_covariance_->setLeafSize(resolution_, resolution_, resolution_);
    voxel_grid_covariance_->setInputCloud(target);
  }
  Eigen::Matrix4f getFinalTransformation() {}

  // for demo
  void radiusSearch(const PointType point, const double radius, std::vector<Leaf> & leaf_vec)
  {
    voxel_grid_covariance_->radiusSearch(point, radius, leaf_vec);
  }

private:
  std::shared_ptr<VoxelGridCovariance<PointType>> voxel_grid_covariance_;

  typename pcl::PointCloud<PointType>::Ptr input_;

  double resolution_;
};

#endif
