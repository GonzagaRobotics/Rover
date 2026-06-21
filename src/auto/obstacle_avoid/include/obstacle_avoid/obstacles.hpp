#pragma once

#include <Eigen/Dense>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

class Obstacles
{
private:
  float extent_x_ = 10.0f;
  float extent_y_ = 10.0f;
  float resolution_ = 0.1f;

  Eigen::Vector2f goal_;

  float min_z_ = -1.0f;
  float max_z_ = 2.0f;
  int min_points_per_cell_ = 2;
  float distance_influence_ = 2.0f;
  float attractive_multiplier_ = 1.0f;
  float attractive_max_ = 10.0f;
  float repulsive_multiplier_ = 1.0f;
  float repulsive_max_ = 100.0f;

  int grid_x_ = (extent_x_ * 2) / resolution_;
  int grid_y_ = (extent_y_ * 2) / resolution_;

  Eigen::MatrixXi occ_ = Eigen::MatrixXi::Zero(grid_x_, grid_y_);
  Eigen::MatrixXf grid_ = Eigen::MatrixXf::Zero(grid_x_, grid_y_);

  void update_repulsive_cost(int x, int y);
  void update_attractive_potential(int x, int y);

  float repulsive_cost(int x0, int y0, int x1, int y1) const
  {
    float dist = std::sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0)) * resolution_;

    if (dist > distance_influence_) {
      return 0.0f;
    }

    // Avoid division by zero
    if (dist < 0.01f) {
      return repulsive_max_;
    }

    float dist_cost = (1.f / dist) - (1.f / distance_influence_);
    float cost = 0.5f * repulsive_multiplier_ * dist_cost * dist_cost;
    return std::min(cost, repulsive_max_);
  }

  Eigen::Vector2f compute_attractive_vector();
  Eigen::Vector2f compute_repulsive_vector();

public:
  Obstacles() = default;

  void update(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, Eigen::Vector2f goal);

  Eigen::Vector2f compute_vector();
};