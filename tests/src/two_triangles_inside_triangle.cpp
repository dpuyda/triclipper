#include "get_polygons_test.h"

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
        {Vertex32S{0, 0}, Vertex32S{50, 50}, Vertex32S{100, 0}});
    triangles.push_back(
        {Vertex32S{20, 10}, Vertex32S{30, 20}, Vertex32S{40, 10}});
    triangles.push_back(
        {Vertex32S{60, 10}, Vertex32S{70, 20}, Vertex32S{80, 10}});

    polygons.push_back({{0, 0}, {50, 50}, {30, 20}, {20, 10}, {100, 0}});
    polygons.push_back({{20, 10}, {30, 20}, {40, 10}});
    polygons.push_back(
        {{100, 0}, {20, 10}, {40, 10}, {30, 20}, {50, 50}, {70, 20}, {60, 10}});
    polygons.push_back({{60, 10}, {70, 20}, {80, 10}});
    polygons.push_back({{100, 0}, {60, 10}, {80, 10}, {70, 20}, {50, 50}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{50, 50}, Vertex32S{100, 0}});
    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{30, 10}, Vertex32S{40, 0}});
    triangles.push_back(
        {Vertex32S{60, 10}, Vertex32S{70, 20}, Vertex32S{80, 10}});

    polygons.push_back({{0, 0}, {50, 50}, {30, 10}, {20, 0}});
    polygons.push_back({{20, 0}, {30, 10}, {40, 0}});
    polygons.push_back(
        {{40, 0}, {30, 10}, {50, 50}, {70, 20}, {60, 10}, {100, 0}});
    polygons.push_back({{60, 10}, {70, 20}, {80, 10}});
    polygons.push_back({{100, 0}, {60, 10}, {80, 10}, {70, 20}, {50, 50}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{50, 50}, Vertex32S{100, 0}});
    triangles.push_back(
        {Vertex32S{20, 10}, Vertex32S{30, 20}, Vertex32S{40, 10}});
    triangles.push_back(
        {Vertex32S{60, 0}, Vertex32S{70, 10}, Vertex32S{80, 0}});

    polygons.push_back({{0, 0}, {50, 50}, {30, 20}, {20, 10}, {60, 0}});
    polygons.push_back({{60, 0}, {70, 10}, {80, 0}});
    polygons.push_back({{80, 0}, {70, 10}, {50, 50}, {100, 0}});
    polygons.push_back({{20, 10}, {30, 20}, {40, 10}});
    polygons.push_back(
        {{60, 0}, {20, 10}, {40, 10}, {30, 20}, {50, 50}, {70, 10}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{50, 50}, Vertex32S{100, 0}});
    triangles.push_back(
        {Vertex32S{40, 10}, Vertex32S{50, 20}, Vertex32S{60, 10}});
    triangles.push_back(
        {Vertex32S{40, 30}, Vertex32S{50, 40}, Vertex32S{60, 30}});

    polygons.push_back(
        {{0, 0}, {50, 50}, {50, 40}, {40, 30}, {50, 20}, {40, 10}, {100, 0}});
    polygons.push_back({{40, 10}, {50, 20}, {60, 10}});
    polygons.push_back({{100, 0},
                        {40, 10},
                        {60, 10},
                        {50, 20},
                        {40, 30},
                        {60, 30},
                        {50, 40},
                        {50, 50}});
    polygons.push_back({{40, 30}, {50, 40}, {60, 30}});
  }

  return tests;
}

template <>
std::vector<Parameters<OperationType::kUnion>> CreateParameters() {
  std::vector<Parameters<OperationType::kUnion>> tests;

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{50, 50}, Vertex32S{100, 0}});
    triangles.push_back(
        {Vertex32S{20, 10}, Vertex32S{30, 20}, Vertex32S{40, 10}});
    triangles.push_back(
        {Vertex32S{60, 10}, Vertex32S{70, 20}, Vertex32S{80, 10}});

    polygons.push_back({{0, 0}, {50, 50}, {100, 0}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{50, 50}, Vertex32S{100, 0}});
    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{30, 10}, Vertex32S{40, 0}});
    triangles.push_back(
        {Vertex32S{60, 10}, Vertex32S{70, 20}, Vertex32S{80, 10}});

    polygons.push_back({{0, 0}, {50, 50}, {100, 0}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{50, 50}, Vertex32S{100, 0}});
    triangles.push_back(
        {Vertex32S{20, 10}, Vertex32S{30, 20}, Vertex32S{40, 10}});
    triangles.push_back(
        {Vertex32S{60, 0}, Vertex32S{70, 10}, Vertex32S{80, 0}});

    polygons.push_back({{0, 0}, {50, 50}, {100, 0}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{50, 50}, Vertex32S{100, 0}});
    triangles.push_back(
        {Vertex32S{40, 10}, Vertex32S{50, 20}, Vertex32S{60, 10}});
    triangles.push_back(
        {Vertex32S{40, 30}, Vertex32S{50, 40}, Vertex32S{60, 30}});

    polygons.push_back({{0, 0}, {50, 50}, {100, 0}});
  }

  return tests;
}

INSTANTIATE_TEST_SUITE_P(
    TwoTrianglesInsideTriangle, MergeTest,
    testing::ValuesIn(CreateParameters<OperationType::kMerge>()));

INSTANTIATE_TEST_SUITE_P(
    TwoTrianglesInsideTriangle, UnionTest,
    testing::ValuesIn(CreateParameters<OperationType::kUnion>()));
}  // anonymous namespace
