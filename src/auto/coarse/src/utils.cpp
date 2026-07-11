#include "utils.hpp"

double loc_dist(const auto_msgs::msg::Location & a, const auto_msgs::msg::Location & b)
{
  double mid_lat = (a.latitude + b.latitude) / 2.0;
  double k1 = 111.13209 - 0.56605 * std::cos(2 * mid_lat) + 0.00120 * std::cos(4 * mid_lat);
  double k2 = 111.41513 * std::cos(mid_lat) - 0.09455 * std::cos(3 * mid_lat) +
              0.00012 * std::cos(5 * mid_lat);

  double squared =
    std::pow(k1 * (a.latitude - b.latitude), 2) + std::pow(k2 * (a.longitude - b.longitude), 2);
  return std::sqrt(squared) * 1000;  // Convert from km to m
}
