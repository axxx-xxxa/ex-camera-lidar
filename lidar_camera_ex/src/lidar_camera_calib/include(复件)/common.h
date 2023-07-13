#ifndef LIDAR_CAMERA_COMMON_H
#define LIDAR_CAMERA_COMMON_H
#include <Eigen/Core>
#include <cv_bridge/cv_bridge.h>
#include <pcl/common/io.h>
#include <stdio.h>
#include <string>
#include <unordered_map>

struct PnPData {
  double x, y, z, u, v;
};

struct VPnPData {
  double x, y, z, u, v;
  Eigen::Vector2d direction;
  Eigen::Vector2d direction_lidar;
  int number;
};

#endif
