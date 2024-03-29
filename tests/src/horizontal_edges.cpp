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

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{20, 20}, Vertex32S{40, 0}});
    triangles.push_back(
        {Vertex32S{10, 0}, Vertex32S{20, 10}, Vertex32S{30, 0}});

    polygons.push_back({{0, 0}, {20, 20}, {20, 10}, {10, 0}});
    polygons.push_back({{10, 0}, {20, 10}, {30, 0}});
    polygons.push_back({{30, 0}, {20, 10}, {20, 20}, {40, 0}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{20, 20}, Vertex32S{40, 0}});
    triangles.push_back({Vertex32S{0, 0}, Vertex32S{10, 10}, Vertex32S{20, 0}});

    polygons.push_back({{0, 0}, {10, 10}, {20, 0}});
    polygons.push_back({{20, 0}, {10, 10}, {20, 20}, {40, 0}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{20, 20}, Vertex32S{40, 0}});
    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{30, 10}, Vertex32S{40, 0}});

    polygons.push_back({{0, 0}, {20, 20}, {30, 10}, {20, 0}});
    polygons.push_back({{20, 0}, {30, 10}, {40, 0}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{20, 20}, Vertex32S{40, 0}});
    triangles.push_back(
        {Vertex32S{10, 10}, Vertex32S{20, 20}, Vertex32S{30, 10}});

    polygons.push_back({{0, 0}, {10, 10}, {30, 10}, {40, 0}});
    polygons.push_back({{10, 10}, {20, 20}, {30, 10}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{30, 30}, Vertex32S{60, 0}});
    triangles.push_back(
        {Vertex32S{20, 10}, Vertex32S{30, 20}, Vertex32S{40, 10}});

    polygons.push_back({{0, 0}, {30, 30}, {30, 20}, {20, 10}, {60, 0}});
    polygons.push_back({{20, 10}, {30, 20}, {40, 10}});
    polygons.push_back({{60, 0}, {20, 10}, {40, 10}, {30, 20}, {30, 30}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{40, 40}, Vertex32S{80, 0}});
    triangles.push_back(
        {Vertex32S{30, 20}, Vertex32S{50, 20}, Vertex32S{40, 10}});

    polygons.push_back(
        {{0, 0}, {40, 40}, {50, 20}, {30, 20}, {40, 10}, {80, 0}});
    polygons.push_back({{40, 10}, {30, 20}, {50, 20}});
    polygons.push_back({{80, 0}, {40, 10}, {50, 20}, {40, 40}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{30, 30}, Vertex32S{60, 0}});
    triangles.push_back(
        {Vertex32S{10, 10}, Vertex32S{20, 20}, Vertex32S{30, 10}});

    polygons.push_back(
        {{0, 0}, {10, 10}, {30, 10}, {20, 20}, {30, 30}, {60, 0}});
    polygons.push_back({{10, 10}, {20, 20}, {30, 10}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{30, 30}, Vertex32S{60, 0}});
    triangles.push_back(
        {Vertex32S{30, 10}, Vertex32S{40, 20}, Vertex32S{50, 10}});

    polygons.push_back(
        {{0, 0}, {30, 30}, {40, 20}, {30, 10}, {50, 10}, {60, 0}});
    polygons.push_back({{30, 10}, {40, 20}, {50, 10}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{40, 40}, Vertex32S{80, 0}});
    triangles.push_back(
        {Vertex32S{20, 20}, Vertex32S{40, 20}, Vertex32S{30, 10}});

    polygons.push_back({{0, 0}, {20, 20}, {30, 10}, {80, 0}});
    polygons.push_back({{30, 10}, {20, 20}, {40, 20}});
    polygons.push_back({{80, 0}, {30, 10}, {40, 20}, {20, 20}, {40, 40}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{40, 40}, Vertex32S{80, 0}});
    triangles.push_back(
        {Vertex32S{40, 20}, Vertex32S{60, 20}, Vertex32S{50, 10}});

    polygons.push_back(
        {{0, 0}, {40, 40}, {60, 20}, {40, 20}, {50, 10}, {80, 0}});
    polygons.push_back({{50, 10}, {40, 20}, {60, 20}});
    polygons.push_back({{80, 0}, {50, 10}, {60, 20}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{30, 30}, Vertex32S{60, 0}});
    triangles.push_back(
        {Vertex32S{20, 20}, Vertex32S{40, 20}, Vertex32S{30, 10}});

    polygons.push_back({{0, 0}, {20, 20}, {30, 10}, {60, 0}});
    polygons.push_back({{30, 10}, {20, 20}, {40, 20}});
    polygons.push_back({{60, 0}, {30, 10}, {40, 20}});
    polygons.push_back({{20, 20}, {30, 30}, {40, 20}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{30, 30}, Vertex32S{60, 0}});
    triangles.push_back(
        {Vertex32S{10, 10}, Vertex32S{30, 10}, Vertex32S{20, 0}});

    polygons.push_back({{0, 0}, {10, 10}, {20, 0}});
    polygons.push_back({{20, 0}, {10, 10}, {30, 10}});
    polygons.push_back({{20, 0}, {30, 10}, {10, 10}, {30, 30}, {60, 0}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{30, 30}, Vertex32S{60, 0}});
    triangles.push_back(
        {Vertex32S{20, 10}, Vertex32S{40, 10}, Vertex32S{30, 0}});

    polygons.push_back({{0, 0}, {30, 30}, {40, 10}, {20, 10}, {30, 0}});
    polygons.push_back({{30, 0}, {20, 10}, {40, 10}});
    polygons.push_back({{30, 0}, {40, 10}, {30, 30}, {60, 0}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{30, 30}, Vertex32S{60, 0}});
    triangles.push_back(
        {Vertex32S{30, 10}, Vertex32S{50, 10}, Vertex32S{40, 0}});

    polygons.push_back({{0, 0}, {30, 30}, {50, 10}, {30, 10}, {40, 0}});
    polygons.push_back({{40, 0}, {30, 10}, {50, 10}});
    polygons.push_back({{40, 0}, {50, 10}, {60, 0}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{20, 20}, Vertex32S{40, 0}});
    triangles.push_back(
        {Vertex32S{10, 10}, Vertex32S{30, 10}, Vertex32S{20, 0}});

    polygons.push_back({{0, 0}, {10, 10}, {20, 0}});
    polygons.push_back({{20, 0}, {10, 10}, {30, 10}});
    polygons.push_back({{20, 0}, {30, 10}, {40, 0}});
    polygons.push_back({{10, 10}, {20, 20}, {30, 10}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{100, 0}, Vertex32S{100, 100}});
    triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{100, 100}, Vertex32S{0, 100}});
    triangles.push_back(
        {Vertex32S{50, 50}, Vertex32S{150, 50}, Vertex32S{150, 150}});
    triangles.push_back(
        {Vertex32S{50, 50}, Vertex32S{150, 150}, Vertex32S{50, 150}});

    polygons.push_back({{0, 0}, {0, 100}, {50, 100}, {50, 50}});
    polygons.push_back({{0, 0}, {50, 50}, {100, 50}, {100, 0}});
    polygons.push_back({{50, 50}, {50, 100}, {100, 100}});
    polygons.push_back({{50, 50}, {100, 100}, {100, 50}});
    polygons.push_back({{100, 50}, {100, 100}, {150, 150}, {150, 50}});
    polygons.push_back({{50, 100}, {50, 150}, {150, 150}, {100, 100}});
  }

  return tests;
}

template <>
std::vector<Parameters<OperationType::kUnion>> CreateParameters() {
  std::vector<Parameters<OperationType::kUnion>> tests;

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{20, 20}, Vertex32S{40, 0}});
    triangles.push_back(
        {Vertex32S{10, 0}, Vertex32S{20, 10}, Vertex32S{30, 0}});

    polygons.push_back({{0, 0}, {20, 20}, {40, 0}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{20, 20}, Vertex32S{40, 0}});
    triangles.push_back({Vertex32S{0, 0}, Vertex32S{10, 10}, Vertex32S{20, 0}});

    polygons.push_back({{0, 0}, {20, 20}, {40, 0}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{20, 20}, Vertex32S{40, 0}});
    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{30, 10}, Vertex32S{40, 0}});

    polygons.push_back({{0, 0}, {20, 20}, {40, 0}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{20, 20}, Vertex32S{40, 0}});
    triangles.push_back(
        {Vertex32S{10, 10}, Vertex32S{20, 20}, Vertex32S{30, 10}});

    polygons.push_back({{0, 0}, {10, 10}, {30, 10}, {40, 0}});
    polygons.push_back({{10, 10}, {20, 20}, {30, 10}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{30, 30}, Vertex32S{60, 0}});
    triangles.push_back(
        {Vertex32S{20, 10}, Vertex32S{30, 20}, Vertex32S{40, 10}});

    polygons.push_back({{0, 0}, {30, 30}, {60, 0}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{40, 40}, Vertex32S{80, 0}});
    triangles.push_back(
        {Vertex32S{30, 20}, Vertex32S{50, 20}, Vertex32S{40, 10}});

    polygons.push_back({{0, 0}, {40, 40}, {80, 0}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{30, 30}, Vertex32S{60, 0}});
    triangles.push_back(
        {Vertex32S{10, 10}, Vertex32S{20, 20}, Vertex32S{30, 10}});

    polygons.push_back({{0, 0}, {30, 30}, {60, 0}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{30, 30}, Vertex32S{60, 0}});
    triangles.push_back(
        {Vertex32S{30, 10}, Vertex32S{40, 20}, Vertex32S{50, 10}});

    polygons.push_back({{0, 0}, {30, 30}, {60, 0}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{40, 40}, Vertex32S{80, 0}});
    triangles.push_back(
        {Vertex32S{20, 20}, Vertex32S{40, 20}, Vertex32S{30, 10}});

    polygons.push_back({{0, 0}, {40, 40}, {80, 0}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{40, 40}, Vertex32S{80, 0}});
    triangles.push_back(
        {Vertex32S{40, 20}, Vertex32S{60, 20}, Vertex32S{50, 10}});

    polygons.push_back({{0, 0}, {40, 40}, {80, 0}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{30, 30}, Vertex32S{60, 0}});
    triangles.push_back(
        {Vertex32S{20, 20}, Vertex32S{40, 20}, Vertex32S{30, 10}});

    polygons.push_back({{0, 0}, {20, 20}, {40, 20}, {60, 0}});
    polygons.push_back({{20, 20}, {30, 30}, {40, 20}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{30, 30}, Vertex32S{60, 0}});
    triangles.push_back(
        {Vertex32S{10, 10}, Vertex32S{30, 10}, Vertex32S{20, 0}});

    polygons.push_back({{0, 0}, {30, 30}, {60, 0}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{30, 30}, Vertex32S{60, 0}});
    triangles.push_back(
        {Vertex32S{20, 10}, Vertex32S{40, 10}, Vertex32S{30, 0}});

    polygons.push_back({{0, 0}, {30, 30}, {60, 0}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{30, 30}, Vertex32S{60, 0}});
    triangles.push_back(
        {Vertex32S{30, 10}, Vertex32S{50, 10}, Vertex32S{40, 0}});

    polygons.push_back({{0, 0}, {30, 30}, {60, 0}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back({Vertex32S{0, 0}, Vertex32S{20, 20}, Vertex32S{40, 0}});
    triangles.push_back(
        {Vertex32S{10, 10}, Vertex32S{30, 10}, Vertex32S{20, 0}});

    polygons.push_back({{0, 0}, {10, 10}, {30, 10}, {40, 0}});
    polygons.push_back({{10, 10}, {20, 20}, {30, 10}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{100, 0}, Vertex32S{100, 100}});
    triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{100, 100}, Vertex32S{0, 100}});
    triangles.push_back(
        {Vertex32S{50, 50}, Vertex32S{150, 50}, Vertex32S{150, 150}});
    triangles.push_back(
        {Vertex32S{50, 50}, Vertex32S{150, 150}, Vertex32S{50, 150}});

    polygons.push_back({{0, 0},
                        {0, 100},
                        {50, 100},
                        {50, 150},
                        {150, 150},
                        {150, 50},
                        {100, 50},
                        {100, 0}});
  }

  return tests;
}

INSTANTIATE_TEST_SUITE_P(
    HorizontalEdges, MergeTest,
    testing::ValuesIn(CreateParameters<OperationType::kMerge>()));

INSTANTIATE_TEST_SUITE_P(
    HorizontalEdges, UnionTest,
    testing::ValuesIn(CreateParameters<OperationType::kUnion>()));
}  // anonymous namespace
