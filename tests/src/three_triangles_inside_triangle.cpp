#include "get_polygons_test.hpp"

namespace {
using namespace triclipper;

template <OperationType Operation>
std::vector<Parameters<Operation>> CreateParameters();

template <>
std::vector<Parameters<OperationType::kMerge>> CreateParameters() {
  std::vector<Parameters<OperationType::kMerge>> tests;

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{50, 50}, Vertex32S{100, 0}});
    test.triangles.push_back(
        {Vertex32S{20, 10}, Vertex32S{30, 20}, Vertex32S{40, 10}});
    test.triangles.push_back(
        {Vertex32S{50, 10}, Vertex32S{60, 20}, Vertex32S{70, 10}});
    test.triangles.push_back(
        {Vertex32S{40, 30}, Vertex32S{50, 40}, Vertex32S{60, 30}});

    test.polygons.push_back(
        {{0, 0}, {50, 50}, {50, 40}, {40, 30}, {30, 20}, {20, 10}, {100, 0}});
    test.polygons.push_back({{20, 10}, {30, 20}, {40, 10}});
    test.polygons.push_back(
        {{100, 0}, {20, 10}, {40, 10}, {30, 20}, {40, 30}, {60, 20}, {50, 10}});
    test.polygons.push_back({{50, 10}, {60, 20}, {70, 10}});
    test.polygons.push_back({{100, 0},
                             {50, 10},
                             {70, 10},
                             {60, 20},
                             {40, 30},
                             {60, 30},
                             {50, 40},
                             {50, 50}});
    test.polygons.push_back({{40, 30}, {50, 40}, {60, 30}});
  }

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{50, 50}, Vertex32S{100, 0}});
    test.triangles.push_back(
        {Vertex32S{20, 10}, Vertex32S{30, 20}, Vertex32S{40, 10}});
    test.triangles.push_back(
        {Vertex32S{50, 20}, Vertex32S{40, 30}, Vertex32S{60, 30}});
    test.triangles.push_back(
        {Vertex32S{60, 10}, Vertex32S{70, 20}, Vertex32S{80, 10}});

    test.polygons.push_back({{0, 0}, {50, 50}, {30, 20}, {20, 10}, {100, 0}});
    test.polygons.push_back({{20, 10}, {30, 20}, {40, 10}});
    test.polygons.push_back({{100, 0},
                             {20, 10},
                             {40, 10},
                             {30, 20},
                             {50, 50},
                             {60, 30},
                             {40, 30},
                             {50, 20},
                             {60, 10}});
    test.polygons.push_back({{60, 10}, {70, 20}, {80, 10}});
    test.polygons.push_back({{100, 0}, {60, 10}, {80, 10}, {70, 20}, {50, 50}});
    test.polygons.push_back({{50, 20}, {40, 30}, {60, 30}});
    test.polygons.push_back({{60, 10}, {50, 20}, {60, 30}, {50, 50}, {70, 20}});
  }

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{50, 50}, Vertex32S{100, 0}});
    test.triangles.push_back(
        {Vertex32S{20, 10}, Vertex32S{30, 20}, Vertex32S{40, 10}});
    test.triangles.push_back(
        {Vertex32S{50, 10}, Vertex32S{40, 20}, Vertex32S{60, 20}});
    test.triangles.push_back(
        {Vertex32S{60, 10}, Vertex32S{70, 20}, Vertex32S{80, 10}});

    test.polygons.push_back({{0, 0}, {50, 50}, {30, 20}, {20, 10}, {100, 0}});
    test.polygons.push_back({{20, 10}, {30, 20}, {40, 10}});
    test.polygons.push_back({{100, 0},
                             {20, 10},
                             {40, 10},
                             {30, 20},
                             {50, 50},
                             {60, 20},
                             {40, 20},
                             {50, 10}});
    test.polygons.push_back({{50, 10}, {40, 20}, {60, 20}});
    test.polygons.push_back(
        {{100, 0}, {50, 10}, {60, 20}, {50, 50}, {70, 20}, {60, 10}});
    test.polygons.push_back({{60, 10}, {70, 20}, {80, 10}});
    test.polygons.push_back({{100, 0}, {60, 10}, {80, 10}, {70, 20}, {50, 50}});
  }

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{50, 50}, Vertex32S{100, 0}});
    test.triangles.push_back(
        {Vertex32S{20, 10}, Vertex32S{30, 20}, Vertex32S{40, 10}});
    test.triangles.push_back(
        {Vertex32S{60, 20}, Vertex32S{50, 30}, Vertex32S{70, 30}});
    test.triangles.push_back(
        {Vertex32S{60, 10}, Vertex32S{70, 20}, Vertex32S{80, 10}});

    test.polygons.push_back({{0, 0}, {50, 50}, {30, 20}, {20, 10}, {100, 0}});
    test.polygons.push_back({{20, 10}, {30, 20}, {40, 10}});
    test.polygons.push_back({{100, 0},
                             {20, 10},
                             {40, 10},
                             {30, 20},
                             {50, 50},
                             {70, 30},
                             {50, 30},
                             {60, 20},
                             {60, 10}});
    test.polygons.push_back({{60, 10}, {70, 20}, {80, 10}});
    test.polygons.push_back({{100, 0}, {60, 10}, {80, 10}, {70, 20}, {70, 30}});
    test.polygons.push_back({{60, 20}, {50, 30}, {70, 30}});
    test.polygons.push_back({{60, 10}, {60, 20}, {70, 30}, {70, 20}});
  }

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{50, 50}, Vertex32S{100, 0}});
    test.triangles.push_back(
        {Vertex32S{20, 10}, Vertex32S{30, 20}, Vertex32S{40, 10}});
    test.triangles.push_back(
        {Vertex32S{40, 20}, Vertex32S{30, 30}, Vertex32S{50, 30}});
    test.triangles.push_back(
        {Vertex32S{60, 10}, Vertex32S{70, 20}, Vertex32S{80, 10}});

    test.polygons.push_back({{0, 0}, {30, 30}, {30, 20}, {20, 10}, {100, 0}});
    test.polygons.push_back({{20, 10}, {30, 20}, {40, 10}});
    test.polygons.push_back(
        {{100, 0}, {20, 10}, {40, 10}, {30, 20}, {30, 30}, {40, 20}, {60, 10}});
    test.polygons.push_back({{60, 10}, {70, 20}, {80, 10}});
    test.polygons.push_back({{100, 0}, {60, 10}, {80, 10}, {70, 20}, {50, 50}});
    test.polygons.push_back({{40, 20}, {30, 30}, {50, 30}});
    test.polygons.push_back(
        {{60, 10}, {40, 20}, {50, 30}, {30, 30}, {50, 50}, {70, 20}});
  }

  return tests;
}

template <>
std::vector<Parameters<OperationType::kUnion>> CreateParameters() {
  std::vector<Parameters<OperationType::kUnion>> tests;

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{50, 50}, Vertex32S{100, 0}});
    test.triangles.push_back(
        {Vertex32S{20, 10}, Vertex32S{30, 20}, Vertex32S{40, 10}});
    test.triangles.push_back(
        {Vertex32S{50, 10}, Vertex32S{60, 20}, Vertex32S{70, 10}});
    test.triangles.push_back(
        {Vertex32S{40, 30}, Vertex32S{50, 40}, Vertex32S{60, 30}});

    test.polygons.push_back({{0, 0}, {50, 50}, {100, 0}});
  }

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{50, 50}, Vertex32S{100, 0}});
    test.triangles.push_back(
        {Vertex32S{20, 10}, Vertex32S{30, 20}, Vertex32S{40, 10}});
    test.triangles.push_back(
        {Vertex32S{50, 20}, Vertex32S{40, 30}, Vertex32S{60, 30}});
    test.triangles.push_back(
        {Vertex32S{60, 10}, Vertex32S{70, 20}, Vertex32S{80, 10}});

    test.polygons.push_back({{0, 0}, {50, 50}, {100, 0}});
  }

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{50, 50}, Vertex32S{100, 0}});
    test.triangles.push_back(
        {Vertex32S{20, 10}, Vertex32S{30, 20}, Vertex32S{40, 10}});
    test.triangles.push_back(
        {Vertex32S{50, 10}, Vertex32S{40, 20}, Vertex32S{60, 20}});
    test.triangles.push_back(
        {Vertex32S{60, 10}, Vertex32S{70, 20}, Vertex32S{80, 10}});

    test.polygons.push_back({{0, 0}, {50, 50}, {100, 0}});
  }

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{50, 50}, Vertex32S{100, 0}});
    test.triangles.push_back(
        {Vertex32S{20, 10}, Vertex32S{30, 20}, Vertex32S{40, 10}});
    test.triangles.push_back(
        {Vertex32S{60, 20}, Vertex32S{50, 30}, Vertex32S{70, 30}});
    test.triangles.push_back(
        {Vertex32S{60, 10}, Vertex32S{70, 20}, Vertex32S{80, 10}});

    test.polygons.push_back({{0, 0}, {50, 50}, {100, 0}});
  }

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{50, 50}, Vertex32S{100, 0}});
    test.triangles.push_back(
        {Vertex32S{20, 10}, Vertex32S{30, 20}, Vertex32S{40, 10}});
    test.triangles.push_back(
        {Vertex32S{40, 20}, Vertex32S{30, 30}, Vertex32S{50, 30}});
    test.triangles.push_back(
        {Vertex32S{60, 10}, Vertex32S{70, 20}, Vertex32S{80, 10}});

    test.polygons.push_back({{0, 0}, {50, 50}, {100, 0}});
  }

  return tests;
}

INSTANTIATE_TEST_SUITE_P(
    ThreeTrianglesInsideTriangle, MergeTest,
    testing::ValuesIn(CreateParameters<OperationType::kMerge>()));

INSTANTIATE_TEST_SUITE_P(
    ThreeTrianglesInsideTriangle, UnionTest,
    testing::ValuesIn(CreateParameters<OperationType::kUnion>()));
}  // anonymous namespace
