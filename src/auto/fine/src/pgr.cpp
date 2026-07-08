#include "pgr.hpp"

void PGR::set_extents()
{
  float min_x = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float min_y = std::numeric_limits<float>::max();
  float max_y = std::numeric_limits<float>::lowest();

  for (const auto & point : *input_) {
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
    min_y = std::min(min_y, point.y);
    max_y = std::max(max_y, point.y);
  }

  points_extent_min_ = Eigen::Vector2f(min_x, min_y);
  points_extent_max_ = Eigen::Vector2f(max_x, max_y);

  num_pillars_x_ = std::ceil((max_x - min_x) / pillar_resolution_);
  num_pillars_y_ = std::ceil((max_y - min_y) / pillar_resolution_);
}

void PGR::set_points_to_pillars()
{
  for (const auto & point : *input_) {
    int pillar_x = std::floor((point.x - points_extent_min_.x()) / pillar_resolution_);
    int pillar_y = std::floor((point.y - points_extent_min_.y()) / pillar_resolution_);

    if (pillar_x < 0) {
      pillar_x = 0;
    } else if (pillar_x >= num_pillars_x_) {
      pillar_x = num_pillars_x_ - 1;
    }

    if (pillar_y < 0) {
      pillar_y = 0;
    } else if (pillar_y >= num_pillars_y_) {
      pillar_y = num_pillars_y_ - 1;
    }

    points_pillars_.push_back(pillar_y * num_pillars_x_ + pillar_x);
  }
}

bool PGR::is_pillar_within_lgb(int pillar_index, const std::vector<float> & mins) const
{
  int num_check = std::ceil(env_radius_ / pillar_resolution_);
  int pillar_x = pillar_index % num_pillars_x_;
  int pillar_y = pillar_index / num_pillars_x_;

  for (int dx = -num_check; dx <= num_check; ++dx) {
    for (int dy = -num_check; dy <= num_check; ++dy) {
      int check_x = pillar_x + dx;
      int check_y = pillar_y + dy;

      if (check_x < 0 || check_x >= num_pillars_x_ || check_y < 0 || check_y >= num_pillars_y_) {
        continue;
      }

      int check_index = check_y * num_pillars_x_ + check_x;
      // Ignore if the neighboring pillar is empty
      if (mins[check_index] == std::numeric_limits<float>::max()) {
        continue;
      }

      if (mins[pillar_index] - mins[check_index] < delta_env_) {
        return false;
      }
    }
  }

  return true;
}

pcl::PointCloud<pcl::PointXYZ> PGR::remove_ground(
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud)
{
  input_ = input_cloud;
  points_pillars_.clear();

  set_extents();
  set_points_to_pillars();

  pcl::Indices kept_indices;

  std::vector<bool> removed_pillars(num_pillars_x_ * num_pillars_y_, false);
  std::vector<float> pillars_min(
    num_pillars_x_ * num_pillars_y_, std::numeric_limits<float>::max());
  std::vector<float> pillars_max(
    num_pillars_x_ * num_pillars_y_, std::numeric_limits<float>::lowest());

  // Calculate min and max z for each pillar for later
  for (size_t i = 0; i < input_->size(); ++i) {
    int pillar_index = points_pillars_[i];

    // Skip invalid points
    if (pillar_index < 0) {
      continue;
    }
    float z = input_->points[i].z;

    pillars_min[pillar_index] = std::min(pillars_min[pillar_index], z);
    pillars_max[pillar_index] = std::max(pillars_max[pillar_index], z);
  }

  for (size_t i = 0; i < removed_pillars.size(); ++i) {
    // Ignore if the pillar is empty
    if (pillars_min[i] == std::numeric_limits<float>::max()) {
      continue;
    }
    // Or if the pillar has enough height variation to be considered non-ground
    if (pillars_max[i] - pillars_min[i] > delta_minmax_) {
      continue;
    }

    // Now, do the more expensive check if the pillar is high enough above the local ground baseline
    if (!is_pillar_within_lgb(i, pillars_min)) {
      removed_pillars[i] = true;
    }
  }

  // Collect points that are not in removed pillars
  for (size_t i = 0; i < input_->size(); ++i) {
    int pillar_index = points_pillars_[i];

    if (pillar_index >= 0 && !removed_pillars[pillar_index]) {
      kept_indices.push_back(i);
    }
  }

  // Clear input pointer so it can be released after we are done with it
  input_ = nullptr;

  return pcl::PointCloud<pcl::PointXYZ>(*input_cloud, kept_indices);
}