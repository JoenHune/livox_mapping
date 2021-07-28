//
// Created by joen on 2021/7/16.
//

#ifndef LIVOX_MAPPING_MISC_H
#define LIVOX_MAPPING_MISC_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/registration/gicp.h>

#include <utility>

#include "Eigen/Dense"

typedef pcl::PointXYZ PointType;

typedef std::pair<Eigen::Quaternionf, Eigen::Vector3f> StateType;

#endif //LIVOX_MAPPING_MISC_H
