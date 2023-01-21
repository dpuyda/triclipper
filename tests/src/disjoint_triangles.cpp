#include "get_monotone_polygons_test.h"

namespace {
using namespace triclipper;

template <OperationType Operation>
std::vector<Parameters<Operation>> CreateParameters();

template <>
std::vector<Parameters<OperationType::kMerge>> CreateParameters() {
  std::vector<Parameters<OperationType::kMerge>> tests;

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 40}});
    triangles.push_back(
        {Vertex32S{30, 10}, Vertex32S{40, 30}, Vertex32S{50, 20}});

    polygons.push_back({{20, 0}, {0, 20}, {40, 40}});
    polygons.push_back({{30, 10}, {40, 30}, {50, 20}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{10, 0}, Vertex32S{0, 30}, Vertex32S{20, 30}});
    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{30, 30}, Vertex32S{40, 0}});
    triangles.push_back(
        {Vertex32S{50, 0}, Vertex32S{40, 30}, Vertex32S{60, 30}});

    polygons.push_back({{10, 0}, {0, 30}, {20, 30}});
    polygons.push_back({{20, 0}, {30, 30}, {40, 0}});
    polygons.push_back({{50, 0}, {40, 30}, {60, 30}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{10, 0}, Vertex32S{0, 10}, Vertex32S{20, 10}});
    triangles.push_back(
        {Vertex32S{10, 0}, Vertex32S{20, 10}, Vertex32S{30, 0}});

    polygons.push_back({{10, 0}, {0, 10}, {20, 10}});
    polygons.push_back({{10, 0}, {20, 10}, {30, 0}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{10, 10}, Vertex32S{30, 10}});
    triangles.push_back(
        {Vertex32S{10, 10}, Vertex32S{0, 20}, Vertex32S{20, 20}});
    triangles.push_back(
        {Vertex32S{30, 10}, Vertex32S{20, 20}, Vertex32S{40, 20}});

    polygons.push_back({{20, 0}, {10, 10}, {30, 10}});
    polygons.push_back({{10, 10}, {0, 20}, {20, 20}});
    polygons.push_back({{30, 10}, {20, 20}, {40, 20}});
  }

  return tests;
}

template <>
std::vector<Parameters<OperationType::kUnion>> CreateParameters() {
  std::vector<Parameters<OperationType::kUnion>> tests;

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 40}});
    triangles.push_back(
        {Vertex32S{30, 10}, Vertex32S{40, 30}, Vertex32S{50, 20}});

    polygons.push_back({{20, 0}, {0, 20}, {40, 40}});
    polygons.push_back({{30, 10}, {40, 30}, {50, 20}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{10, 0}, Vertex32S{0, 30}, Vertex32S{20, 30}});
    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{30, 30}, Vertex32S{40, 0}});
    triangles.push_back(
        {Vertex32S{50, 0}, Vertex32S{40, 30}, Vertex32S{60, 30}});

    polygons.push_back({{10, 0}, {0, 30}, {20, 30}});
    polygons.push_back({{20, 0}, {30, 30}, {40, 0}});
    polygons.push_back({{50, 0}, {40, 30}, {60, 30}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{10, 0}, Vertex32S{0, 10}, Vertex32S{20, 10}});
    triangles.push_back(
        {Vertex32S{10, 0}, Vertex32S{20, 10}, Vertex32S{30, 0}});

    polygons.push_back({{10, 0}, {0, 10}, {20, 10}, {30, 0}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{10, 10}, Vertex32S{30, 10}});
    triangles.push_back(
        {Vertex32S{10, 10}, Vertex32S{0, 20}, Vertex32S{20, 20}});
    triangles.push_back(
        {Vertex32S{30, 10}, Vertex32S{20, 20}, Vertex32S{40, 20}});

    polygons.push_back({{20, 0}, {10, 10}, {30, 10}});
    polygons.push_back({{10, 10}, {0, 20}, {20, 20}});
    polygons.push_back({{30, 10}, {20, 20}, {40, 20}});
  }

  return tests;
}

INSTANTIATE_TEST_SUITE_P(
    DisjointTriangles, MergeTest,
    testing::ValuesIn(CreateParameters<OperationType::kMerge>()));

INSTANTIATE_TEST_SUITE_P(
    DisjointTriangles, UnionTest,
    testing::ValuesIn(CreateParameters<OperationType::kUnion>()));
}  // anonymous namespace
