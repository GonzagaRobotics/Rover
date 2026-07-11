// Implementation of a Pillar Ground Removal algorithm, from: https://arxiv.org/html/2410.00582v1

#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

class PGR
{
private:
  float pillar_resolution_ = 0.4f;
  float env_radius_ = 1.8f;
  float delta_minmax_ = 0.4f;
  float delta_env_ = 0.4f;

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_;
  Eigen::Vector2f points_extent_min_;
  Eigen::Vector2f points_extent_max_;

  int num_pillars_x_;
  int num_pillars_y_;
  std::vector<int> points_pillars_;

  void set_extents();
  void set_points_to_pillars();
  bool is_pillar_within_lgb(int pillar_index, const std::vector<float> & mins) const;

public:
  PGR() = default;

  pcl::PointCloud<pcl::PointXYZ> remove_ground(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud);
};