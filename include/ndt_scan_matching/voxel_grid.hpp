#ifndef _VOXEL_GRID_HPP_
#define _VOXEL_GRID_HPP_

#include "ndt_scan_matching/kd_tree.hpp"

#include <Eigen/Dense>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <map>
#include <vector>

struct Leaf
{
  int num_points;
  Eigen::Matrix3f eigenvec;
  Eigen::Vector3f eigenvalues;
  Eigen::Vector3f mean;
  Eigen::Matrix3f covariance;
  Leaf()
  {
    num_points = 0;
    eigenvalues.setZero();
    mean.setZero();
    covariance.setZero();
  }
};

template <class PointType>
class VoxelGridCovariance
{
public:
  VoxelGridCovariance() : num_points_per_voxel_(3)
  {
    output_.reset(new pcl::PointCloud<PointType>);

    kd_tree_ptr_ = std::make_shared<KDTree<PointType>>();
  }
  ~VoxelGridCovariance() = default;

  void setLeafSize(const double leaf_size_x, const double leaf_size_y, const double leaf_size_z)
  {
    resolution_[0] = leaf_size_x;
    resolution_[1] = leaf_size_y;
    resolution_[2] = leaf_size_z;
  }

  void setInputCloud(const typename pcl::PointCloud<PointType>::Ptr input)
  {
    if (input->points.size() == 0) return;

    buildVoxelGrid(input);

    kd_tree_ptr_->setInputCloud(getPoints());
  }

  void radiusSearch(const PointType point, const double radius, std::vector<Leaf> & leaf_vec)
  {
    std::vector<int> indices;
    kd_tree_ptr_->radiusSearch(point, radius, indices);

    if (indices.empty()) return;

    auto leafs = getLeafMap();
    auto leafs_index = getVoxelGridIndex();

    for (const auto indice : indices) {
      auto leaf = leafs.find(leafs_index[indice]);
      leaf_vec.emplace_back(leaf->second);
    }
  }

  std::vector<int> getVoxelGridIndex() { return indices_; }
  typename pcl::PointCloud<PointType>::Ptr getPoints() { return output_; }

  void buildVoxelGrid(const typename pcl::PointCloud<PointType>::Ptr input)
  {
    leaf_map_.clear();
    output_->points.clear();

    // get min and max coordinate of points
    Eigen::Vector4f min, max;
    pcl::getMinMax3D<PointType>(*input, min, max);

    // min and max of voxel
    // maximum and minimum coordinate values obtained by getMinMax3D are used to determine the
    // resolution.
    Eigen::Vector4i min_b, max_b;
    for (std::size_t idx = 0; idx < 3; idx++) {
      min_b[idx] = static_cast<int>(std::floor(min[idx] / resolution_[idx]));
      max_b[idx] = static_cast<int>(std::floor(max[idx] / resolution_[idx]));
    }

    Eigen::Vector4i div_b = max_b - min_b + Eigen::Vector4i::Ones();
    Eigen::Vector4i divb_mul = Eigen::Vector4i(1, div_b[0], div_b[0] * div_b[1], 0);

    leaf_map_.clear();
    for (auto p : input->points) {
      // which index does the point cloud position of each axis belong?
      // min_b from p.axis / resolution to make it the origin.
      int idx0 = static_cast<int>(std::floor(p.x / resolution_[0]) - static_cast<float>(min_b[0]));
      int idx1 = static_cast<int>(std::floor(p.y / resolution_[1]) - static_cast<float>(min_b[1]));
      int idx2 = static_cast<int>(std::floor(p.z / resolution_[2]) - static_cast<float>(min_b[2]));

      // Index access in one-dimensional arrays
      int idx = idx0 * divb_mul[0] + idx1 * divb_mul[1] + idx2 * divb_mul[2];

      Eigen::Vector3f vec(p.x, p.y, p.z);
      leaf_map_[idx].mean += vec;
      leaf_map_[idx].covariance += (vec * vec.transpose());
      leaf_map_[idx].num_points++;
    }

    for (auto & leaf : leaf_map_) {
      const float size_per_voxel = static_cast<float>(leaf.second.num_points);
      leaf.second.mean = (leaf.second.mean / size_per_voxel);

      if (num_points_per_voxel_ <= leaf.second.num_points) {
        PointType p;
        p.x = leaf.second.mean[0];
        p.y = leaf.second.mean[1];
        p.z = leaf.second.mean[2];
        output_->points.emplace_back(p);

        indices_.emplace_back(leaf.first);
        leaf.second.covariance =
          ((leaf.second.covariance / size_per_voxel) -
           leaf.second.mean * leaf.second.mean.transpose());
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(leaf.second.covariance);
        if (solver.info() != Eigen::Success) continue;
        leaf.second.eigenvec = solver.eigenvectors();
        leaf.second.eigenvalues = solver.eigenvalues().asDiagonal().diagonal();
      }
    }
  }

  std::map<std::size_t, Leaf> getLeafMap() { return leaf_map_; }

private:
  std::shared_ptr<KDTree<PointType>> kd_tree_ptr_;

  Eigen::Vector3f resolution_;

  typename pcl::PointCloud<PointType>::Ptr output_;

  int num_points_per_voxel_;

  std::map<std::size_t, Leaf> leaf_map_;
  std::vector<int> indices_;
};

#endif
