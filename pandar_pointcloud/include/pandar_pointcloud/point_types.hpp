#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
// #include <pcl_ros/point_cloud.h>

namespace pandar_pointcloud
{
struct PointXYZIR
{
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

struct PointXYZIRADT
{
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring;
  float azimuth;
  float distance;
  int8_t return_type;
  double time_stamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

using PointcloudXYZIRADT = pcl::PointCloud<PointXYZIRADT>::Ptr;

}  // namespace pandar_pointcloud

POINT_CLOUD_REGISTER_POINT_STRUCT(pandar_pointcloud::PointXYZIR,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t,
                                                                                                       ring, ring))

POINT_CLOUD_REGISTER_POINT_STRUCT(pandar_pointcloud::PointXYZIRADT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                      std::uint16_t, ring, ring)(float, azimuth, azimuth)(float, distance,
                                                                                          distance)(std::int8_t, return_type,
                                                                                          return_type)(double, time_stamp,
                                                                                                        time_stamp))