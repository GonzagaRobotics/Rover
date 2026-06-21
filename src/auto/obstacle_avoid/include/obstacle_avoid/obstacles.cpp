#include "obstacles.hpp"

void Obstacles::update_repulsive_cost(int x0, int y0)
{
  int num_near = std::ceil(distance_influence_ / resolution_);
  float nearest_dist = std::numeric_limits<float>::max();
  int nearest_x = -1;
  int nearest_y = -1;

  for (int dx = -num_near; dx <= num_near; ++dx) {
    for (int dy = -num_near; dy <= num_near; ++dy) {
      int x1 = x0 + dx;
      int y1 = y0 + dy;

      if (x1 < 0 || x1 >= grid_x_ || y1 < 0 || y1 >= grid_y_) {
        continue;
      }

      if (occ_(x1, y1) == 0) {
        continue;
      }

      float dist = std::sqrt(dx * dx + dy * dy);
      if (dist < nearest_dist) {
        nearest_dist = dist;
        nearest_x = x1;
        nearest_y = y1;
      }
    }
  }

  if (nearest_x >= 0 && nearest_y >= 0) {
    grid_(x0, y0) += repulsive_cost(x0, y0, nearest_x, nearest_y);
  }
}

void Obstacles::update_attractive_potential(int x, int y)
{
  Eigen::Vector2f cell_pos(
    (x + 0.5f) * resolution_ - extent_x_, (y + 0.5f) * resolution_ - extent_y_);

  float dist = (cell_pos - goal_).norm();
  float potential = 0.5f * attractive_multiplier_ * dist * dist;
  grid_(x, y) -= std::min(potential, attractive_max_);
}

Eigen::Vector2f Obstacles::compute_attractive_vector() {}

Eigen::Vector2f Obstacles::compute_repulsive_vector() { return Eigen::Vector2f(); }

void Obstacles::update(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, Eigen::Vector2f goal)
{
  goal_ = goal;
  occ_ = Eigen::MatrixXi::Zero(grid_x_, grid_y_);
  grid_ = Eigen::MatrixXf::Zero(grid_x_, grid_y_);

  // Create an occupancy grid when enough points are in a cell
  for (const auto & point : cloud->points) {
    if (point.z < min_z_ || point.z > max_z_) {
      continue;
    }

    int x_idx = static_cast<int>((point.x + extent_x_) / resolution_);
    int y_idx = static_cast<int>((point.y + extent_y_) / resolution_);

    if (x_idx >= 0 && x_idx < grid_x_ && y_idx >= 0 && y_idx < grid_y_) {
      occ_(x_idx, y_idx) += 1;
    }
  }

  // Require a minimum number of points in a cell to consider it occupied
  for (int i = 0; i < grid_x_; ++i) {
    for (int j = 0; j < grid_y_; ++j) {
      occ_(i, j) = (occ_(i, j) >= min_points_per_cell_) ? 1 : 0;
    }
  }

  for (int i = 0; i < grid_x_; ++i) {
    for (int j = 0; j < grid_y_; ++j) {
      update_repulsive_cost(i, j);
      update_attractive_potential(i, j);
    }
  }
}

Eigen::Vector2f Obstacles::compute_vector() {
  
}
