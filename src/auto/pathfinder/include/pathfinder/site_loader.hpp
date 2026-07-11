#pragma once

#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

#include "site.hpp"
#include "stb_image.h"

class SiteLoader
{
private:
  std::string pathToDir;

  void loadBounds(
    const std::string & name, double & latNorth, double & lngEast, double & latSouth,
    double & lngWest);

public:
  SiteLoader() = delete;

  SiteLoader(const std::string & pathToDir);

  std::shared_ptr<Site> load(const std::string & name);
};