#ifndef _UTILS_HPP_
#define _UTILS_HPP_

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>

namespace utils
{

template <typename PointT>
void transform_point_cloud(
  const typename pcl::PointCloud<PointT> cloud_in, typename pcl::PointCloud<PointT> & cloud_out,
  const Eigen::Matrix4f matrix)
{
  cloud_out.points.resize(cloud_in.points.size());

  Eigen::Vector4f point;
  for (std::size_t i = 0; i < cloud_in.points.size(); i++) {
    point << cloud_in.points[i].x, cloud_in.points[i].y, cloud_in.points[i].z, 1;
    Eigen::Vector4f trans_point = matrix * point;

    cloud_out.points[i].x = trans_point.x();
    cloud_out.points[i].y = trans_point.y();
    cloud_out.points[i].z = trans_point.z();
  }
}

}  // namespace utils
#endif