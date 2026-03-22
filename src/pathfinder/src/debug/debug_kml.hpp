#pragma once

#include <chrono>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>

#include "auto_msgs/Types.hpp"

/**
 * Writes a KML file containing the given plan.
 *
 * @param static_dir The directory the template KML file is in and the output directory.
 * @param start The starting location of the plan.
 * @param plan The plan to write to the KML file.
 *
 * @throws std::runtime_error If the template file or output file cannot be opened.
 */
void debugKML(const std::string & static_dir, const Location & start, const Plan & plan);