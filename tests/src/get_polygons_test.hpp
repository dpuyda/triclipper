#pragma once

#include <gtest/gtest.h>

#include "types.hpp"

struct Parameters {
  std::vector<Triangle32S> triangles;
  std::vector<Polygon32S> polygons;
};

std::ostream& operator<<(std::ostream& stream, const Parameters& parameters);

using GetPolygonsTest = testing::TestWithParam<Parameters>;
