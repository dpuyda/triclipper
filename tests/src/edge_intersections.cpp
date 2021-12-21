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
        {Vertex32S{30, 0}, Vertex32S{0, 30}, Vertex32S{60, 60}});
    triangles.push_back(
        {Vertex32S{50, 10}, Vertex32S{30, 30}, Vertex32S{70, 50}});

    polygons.push_back(
        {{30, 0}, {0, 30}, {60, 60}, {50, 40}, {30, 30}, {40, 20}});
    polygons.push_back({{50, 10}, {40, 20}, {50, 40}, {70, 50}});
    polygons.push_back({{40, 20}, {30, 30}, {50, 40}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 20}});
    triangles.push_back(
        {Vertex32S{40, 0}, Vertex32S{20, 20}, Vertex32S{60, 20}});

    polygons.push_back({{20, 0}, {0, 20}, {20, 20}, {30, 10}});
    polygons.push_back({{40, 0}, {30, 10}, {40, 20}, {60, 20}});
    polygons.push_back({{30, 10}, {20, 20}, {40, 20}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{30, 0}, Vertex32S{10, 20}, Vertex32S{50, 20}});
    triangles.push_back(
        {Vertex32S{20, 10}, Vertex32S{0, 30}, Vertex32S{40, 30}});

    polygons.push_back({{30, 0}, {20, 10}, {30, 20}, {50, 20}});
    polygons.push_back({{20, 10}, {10, 20}, {30, 20}});
    polygons.push_back({{10, 20}, {0, 30}, {40, 30}, {30, 20}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 20}});
    triangles.push_back(
        {Vertex32S{30, 10}, Vertex32S{10, 30}, Vertex32S{50, 30}});

    polygons.push_back({{20, 0}, {0, 20}, {20, 20}, {30, 10}});
    polygons.push_back({{30, 10}, {20, 20}, {40, 20}});
    polygons.push_back({{20, 20}, {10, 30}, {50, 30}, {40, 20}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{30, 0}, Vertex32S{10, 20}, Vertex32S{50, 20}});
    triangles.push_back(
        {Vertex32S{20, 10}, Vertex32S{0, 30}, Vertex32S{40, 30}});
    triangles.push_back(
        {Vertex32S{40, 10}, Vertex32S{20, 30}, Vertex32S{60, 30}});

    polygons.push_back({{30, 0}, {20, 10}, {30, 20}, {40, 10}});
    polygons.push_back({{20, 10}, {10, 20}, {30, 20}});
    polygons.push_back({{40, 10}, {30, 20}, {50, 20}});
    polygons.push_back({{10, 20}, {0, 30}, {20, 30}, {30, 20}});
    polygons.push_back({{30, 20}, {20, 30}, {40, 30}});
    polygons.push_back({{30, 20}, {40, 30}, {60, 30}, {50, 20}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{0, 20}, Vertex32S{40, 0}});
    triangles.push_back({Vertex32S{0, 0}, Vertex32S{40, 20}, Vertex32S{40, 0}});
    triangles.push_back(
        {Vertex32S{10, 0}, Vertex32S{10, 20}, Vertex32S{30, 0}});

    polygons.push_back({{0, 0}, {0, 20}, {10, 15}, {10, 5}});
    polygons.push_back({{0, 0}, {10, 5}, {10, 0}});
    polygons.push_back({{10, 0}, {10, 5}, {20, 10}, {30, 0}});
    polygons.push_back({{30, 0}, {20, 10}, {40, 0}});
    polygons.push_back({{40, 0}, {20, 10}, {40, 20}});
    polygons.push_back({{10, 5}, {10, 15}, {20, 10}});
    polygons.push_back({{20, 10}, {10, 15}, {10, 20}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{0, 20}, Vertex32S{40, 0}});
    triangles.push_back({Vertex32S{0, 0}, Vertex32S{40, 20}, Vertex32S{40, 0}});
    triangles.push_back(
        {Vertex32S{10, 0}, Vertex32S{10, 20}, Vertex32S{30, 0}});
    triangles.push_back(
        {Vertex32S{10, 0}, Vertex32S{30, 20}, Vertex32S{30, 0}});

    polygons.push_back({{0, 0}, {0, 20}, {10, 15}, {10, 5}});
    polygons.push_back({{0, 0}, {10, 5}, {10, 0}});
    polygons.push_back({{10, 0}, {10, 5}, {20, 10}});
    polygons.push_back({{10, 0}, {20, 10}, {30, 0}});
    polygons.push_back({{30, 0}, {20, 10}, {30, 5}});
    polygons.push_back({{30, 0}, {30, 5}, {40, 0}});
    polygons.push_back({{40, 0}, {30, 5}, {30, 15}, {40, 20}});
    polygons.push_back({{10, 5}, {10, 15}, {20, 10}});
    polygons.push_back({{30, 5}, {20, 10}, {30, 15}});
    polygons.push_back({{20, 10}, {10, 15}, {10, 20}});
    polygons.push_back({{20, 10}, {30, 20}, {30, 15}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{0, 20}, Vertex32S{40, 0}});
    triangles.push_back({Vertex32S{0, 0}, Vertex32S{40, 20}, Vertex32S{40, 0}});
    triangles.push_back(
        {Vertex32S{10, 0}, Vertex32S{10, 20}, Vertex32S{30, 0}});
    triangles.push_back(
        {Vertex32S{10, 0}, Vertex32S{30, 20}, Vertex32S{30, 0}});
    triangles.push_back(
        {Vertex32S{0, 10}, Vertex32S{20, 30}, Vertex32S{40, 10}});

    polygons.push_back({{0, 0}, {0, 10}, {10, 10}, {10, 5}});
    polygons.push_back({{0, 0}, {10, 5}, {10, 0}});
    polygons.push_back({{10, 0}, {10, 5}, {20, 10}});
    polygons.push_back({{10, 0}, {20, 10}, {30, 0}});
    polygons.push_back({{30, 0}, {20, 10}, {30, 5}});
    polygons.push_back({{30, 0}, {30, 5}, {40, 0}});
    polygons.push_back({{40, 0}, {30, 5}, {30, 10}, {40, 10}});
    polygons.push_back({{10, 5}, {10, 10}, {20, 10}});
    polygons.push_back({{30, 5}, {20, 10}, {30, 10}});
    polygons.push_back({{0, 10}, {0, 20}, {7, 17}});
    polygons.push_back({{0, 10}, {7, 17}, {10, 15}, {10, 10}});
    polygons.push_back({{10, 10}, {10, 15}, {20, 10}});
    polygons.push_back({{20, 10}, {10, 15}, {10, 20}});
    polygons.push_back({{20, 10}, {10, 20}, {20, 30}, {30, 20}});
    polygons.push_back({{20, 10}, {30, 20}, {30, 15}});
    polygons.push_back({{20, 10}, {30, 15}, {30, 10}});
    polygons.push_back({{30, 10}, {30, 15}, {34, 17}, {40, 10}});
    polygons.push_back({{40, 10}, {34, 17}, {40, 20}});
    polygons.push_back({{10, 15}, {7, 17}, {10, 20}});
    polygons.push_back({{30, 15}, {30, 20}, {34, 17}});
  }

  return tests;
}

template <>
std::vector<Parameters<OperationType::kUnion>> CreateParameters() {
  std::vector<Parameters<OperationType::kUnion>> tests;

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{30, 0}, Vertex32S{0, 30}, Vertex32S{60, 60}});
    triangles.push_back(
        {Vertex32S{50, 10}, Vertex32S{30, 30}, Vertex32S{70, 50}});

    polygons.push_back({{30, 0}, {0, 30}, {60, 60}, {50, 40}, {40, 20}});
    polygons.push_back({{50, 10}, {40, 20}, {50, 40}, {70, 50}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 20}});
    triangles.push_back(
        {Vertex32S{40, 0}, Vertex32S{20, 20}, Vertex32S{60, 20}});

    polygons.push_back({{20, 0}, {0, 20}, {60, 20}, {30, 10}});
    polygons.push_back({{40, 0}, {30, 10}, {60, 20}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{30, 0}, Vertex32S{10, 20}, Vertex32S{50, 20}});
    triangles.push_back(
        {Vertex32S{20, 10}, Vertex32S{0, 30}, Vertex32S{40, 30}});

    polygons.push_back({{30, 0}, {10, 20}, {50, 20}});
    polygons.push_back({{10, 20}, {0, 30}, {40, 30}, {30, 20}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 20}});
    triangles.push_back(
        {Vertex32S{30, 10}, Vertex32S{10, 30}, Vertex32S{50, 30}});

    polygons.push_back({{20, 0}, {0, 20}, {40, 20}});
    polygons.push_back({{20, 20}, {10, 30}, {50, 30}, {40, 20}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{30, 0}, Vertex32S{10, 20}, Vertex32S{50, 20}});
    triangles.push_back(
        {Vertex32S{20, 10}, Vertex32S{0, 30}, Vertex32S{40, 30}});
    triangles.push_back(
        {Vertex32S{40, 10}, Vertex32S{20, 30}, Vertex32S{60, 30}});

    polygons.push_back({{30, 0}, {10, 20}, {50, 20}});
    polygons.push_back({{10, 20}, {0, 30}, {60, 30}, {50, 20}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{0, 20}, Vertex32S{40, 0}});
    triangles.push_back({Vertex32S{0, 0}, Vertex32S{40, 20}, Vertex32S{40, 0}});
    triangles.push_back(
        {Vertex32S{10, 0}, Vertex32S{10, 20}, Vertex32S{30, 0}});

    polygons.push_back({{0, 0}, {0, 20}, {10, 15}, {20, 10}, {40, 0}});
    polygons.push_back({{40, 0}, {20, 10}, {40, 20}});
    polygons.push_back({{20, 10}, {10, 15}, {10, 20}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{0, 20}, Vertex32S{40, 0}});
    triangles.push_back({Vertex32S{0, 0}, Vertex32S{40, 20}, Vertex32S{40, 0}});
    triangles.push_back(
        {Vertex32S{10, 0}, Vertex32S{10, 20}, Vertex32S{30, 0}});
    triangles.push_back(
        {Vertex32S{10, 0}, Vertex32S{30, 20}, Vertex32S{30, 0}});

    polygons.push_back({{0, 0}, {0, 20}, {10, 15}, {20, 10}, {40, 0}});
    polygons.push_back({{40, 0}, {20, 10}, {30, 20}, {30, 15}});
    polygons.push_back({{20, 10}, {10, 15}, {10, 20}});
    polygons.push_back({{40, 0}, {30, 15}, {40, 20}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{0, 20}, Vertex32S{40, 0}});
    triangles.push_back({Vertex32S{0, 0}, Vertex32S{40, 20}, Vertex32S{40, 0}});
    triangles.push_back(
        {Vertex32S{10, 0}, Vertex32S{10, 20}, Vertex32S{30, 0}});
    triangles.push_back(
        {Vertex32S{10, 0}, Vertex32S{30, 20}, Vertex32S{30, 0}});
    triangles.push_back(
        {Vertex32S{0, 10}, Vertex32S{20, 30}, Vertex32S{40, 10}});

    polygons.push_back({{0, 0}, {0, 10}, {40, 10}, {40, 0}});
    polygons.push_back({{0, 10}, {0, 20}, {7, 17}, {40, 10}});
    polygons.push_back({{40, 10}, {7, 17}, {20, 30}, {34, 17}});
    polygons.push_back({{40, 10}, {34, 17}, {40, 20}});
  }

  return tests;
}

INSTANTIATE_TEST_SUITE_P(
    EdgeIntersections, MergeTest,
    testing::ValuesIn(CreateParameters<OperationType::kMerge>()));

INSTANTIATE_TEST_SUITE_P(
    EdgeIntersections, UnionTest,
    testing::ValuesIn(CreateParameters<OperationType::kUnion>()));
}  // anonymous namespace
