#pragma once

#include <gtest/gtest.h>

#include "types.hpp"

template <triclipper::OperationType Operation>
struct Parameters {
  std::vector<Triangle32S> triangles;
  std::vector<Polygon32S> polygons;
};

using MergeTest =
    testing::TestWithParam<Parameters<triclipper::OperationType::kMerge>>;

using UnionTest =
    testing::TestWithParam<Parameters<triclipper::OperationType::kUnion>>;

template <triclipper::OperationType Operation>
std::ostream& operator<<(std::ostream& stream,
                         const Parameters<Operation>& parameters) {
  return stream << parameters.triangles;
}
