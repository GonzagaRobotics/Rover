#pragma once

#include <cmath>

#include "auto_msgs/msg/location.hpp"

/// @brief FCC distance formula for two locations on the Earth. Only suitable for relatively short distances.
/// @param a The first location
/// @param b The second location
/// @return The distance in meters between the two locations
/// @note See: https://en.wikipedia.org/wiki/Geographical_distance
double loc_dist(const auto_msgs::msg::Location & a, const auto_msgs::msg::Location & b);