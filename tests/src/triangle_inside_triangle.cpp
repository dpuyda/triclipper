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
        {Vertex32S{20, 0}, Vertex32S{40, 40}, Vertex32S{0, 20}});

    polygons.push_back({{20, 0}, {0, 20}, {40, 40}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 40}});
    triangles.push_back(
        {Vertex32S{20, 10}, Vertex32S{10, 20}, Vertex32S{30, 30}});

    polygons.push_back(
        {{20, 0}, {0, 20}, {40, 40}, {30, 30}, {10, 20}, {20, 10}});
    polygons.push_back({{20, 10}, {10, 20}, {30, 30}});
    polygons.push_back({{20, 0}, {20, 10}, {30, 30}, {40, 40}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 40}});
    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{10, 10}, Vertex32S{30, 20}});

    polygons.push_back({{20, 0}, {10, 10}, {30, 20}});
    polygons.push_back({{10, 10}, {0, 20}, {40, 40}, {30, 20}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 40}});
    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{25, 10}});

    polygons.push_back({{20, 0}, {0, 20}, {25, 10}});
    polygons.push_back({{25, 10}, {0, 20}, {40, 40}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 40}});
    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{10, 10}, Vertex32S{40, 40}});

    polygons.push_back({{20, 0}, {10, 10}, {40, 40}});
    polygons.push_back({{10, 10}, {0, 20}, {40, 40}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 40}});
    triangles.push_back(
        {Vertex32S{0, 20}, Vertex32S{20, 30}, Vertex32S{10, 10}});

    polygons.push_back({{20, 0}, {10, 10}, {20, 30}, {40, 40}});
    polygons.push_back({{10, 10}, {0, 20}, {20, 30}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 40}});
    triangles.push_back(
        {Vertex32S{0, 20}, Vertex32S{40, 40}, Vertex32S{10, 10}});

    polygons.push_back({{20, 0}, {10, 10}, {40, 40}});
    polygons.push_back({{10, 10}, {0, 20}, {40, 40}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 40}});
    triangles.push_back(
        {Vertex32S{0, 20}, Vertex32S{20, 30}, Vertex32S{20, 0}});

    polygons.push_back({{20, 0}, {0, 20}, {20, 30}});
    polygons.push_back({{20, 0}, {20, 30}, {40, 40}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 40}});
    triangles.push_back(
        {Vertex32S{20, 30}, Vertex32S{40, 40}, Vertex32S{30, 20}});

    polygons.push_back({{20, 0}, {0, 20}, {20, 30}, {30, 20}});
    polygons.push_back({{30, 20}, {20, 30}, {40, 40}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 40}});
    triangles.push_back(
        {Vertex32S{0, 20}, Vertex32S{40, 40}, Vertex32S{25, 10}});

    polygons.push_back({{20, 0}, {0, 20}, {25, 10}});
    polygons.push_back({{25, 10}, {0, 20}, {40, 40}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 40}});
    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{20, 30}, Vertex32S{40, 40}});

    polygons.push_back({{20, 0}, {0, 20}, {20, 30}});
    polygons.push_back({{20, 0}, {20, 30}, {40, 40}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 40}});
    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 40}});

    polygons.push_back({{20, 0}, {0, 20}, {40, 40}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 40}});
    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{20, 20}, Vertex32S{40, 40}});

    polygons.push_back({{20, 0}, {0, 20}, {40, 40}, {20, 20}});
    polygons.push_back({{20, 0}, {20, 20}, {40, 40}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 40}});
    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{30, 30}});

    polygons.push_back({{20, 0}, {0, 20}, {30, 30}});
    polygons.push_back({{20, 0}, {30, 30}, {40, 40}});
    polygons.push_back({{0, 20}, {40, 40}, {30, 30}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 40}});
    triangles.push_back(
        {Vertex32S{20, 10}, Vertex32S{0, 20}, Vertex32S{40, 40}});

    polygons.push_back({{20, 0}, {0, 20}, {20, 10}});
    polygons.push_back({{20, 10}, {0, 20}, {40, 40}});
    polygons.push_back({{20, 0}, {20, 10}, {40, 40}});
  }

  return tests;
}

template <>
std::vector<Parameters<OperationType::kUnion>> CreateParameters() {
  std::vector<Parameters<OperationType::kUnion>> tests;

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{40, 40}, Vertex32S{0, 20}});

    polygons.push_back({{20, 0}, {0, 20}, {40, 40}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 40}});
    triangles.push_back(
        {Vertex32S{20, 10}, Vertex32S{10, 20}, Vertex32S{30, 30}});

    polygons.push_back({{20, 0}, {0, 20}, {40, 40}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 40}});
    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{10, 10}, Vertex32S{30, 20}});

    polygons.push_back({{20, 0}, {0, 20}, {40, 40}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 40}});
    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{25, 10}});

    polygons.push_back({{20, 0}, {0, 20}, {40, 40}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 40}});
    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{10, 10}, Vertex32S{40, 40}});

    polygons.push_back({{20, 0}, {0, 20}, {40, 40}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 40}});
    triangles.push_back(
        {Vertex32S{0, 20}, Vertex32S{20, 30}, Vertex32S{10, 10}});

    polygons.push_back({{20, 0}, {0, 20}, {40, 40}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 40}});
    triangles.push_back(
        {Vertex32S{0, 20}, Vertex32S{40, 40}, Vertex32S{10, 10}});

    polygons.push_back({{20, 0}, {0, 20}, {40, 40}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 40}});
    triangles.push_back(
        {Vertex32S{0, 20}, Vertex32S{20, 30}, Vertex32S{20, 0}});

    polygons.push_back({{20, 0}, {0, 20}, {40, 40}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 40}});
    triangles.push_back(
        {Vertex32S{20, 30}, Vertex32S{40, 40}, Vertex32S{30, 20}});

    polygons.push_back({{20, 0}, {0, 20}, {40, 40}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 40}});
    triangles.push_back(
        {Vertex32S{0, 20}, Vertex32S{40, 40}, Vertex32S{25, 10}});

    polygons.push_back({{20, 0}, {0, 20}, {40, 40}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 40}});
    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{20, 30}, Vertex32S{40, 40}});

    polygons.push_back({{20, 0}, {0, 20}, {40, 40}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 40}});
    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 40}});

    polygons.push_back({{20, 0}, {0, 20}, {40, 40}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 40}});
    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{20, 20}, Vertex32S{40, 40}});

    polygons.push_back({{20, 0}, {0, 20}, {40, 40}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 40}});
    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{30, 30}});

    polygons.push_back({{20, 0}, {0, 20}, {40, 40}});
  }

  {
    auto& [triangles, polygons] = tests.emplace_back();

    triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 40}});
    triangles.push_back(
        {Vertex32S{20, 10}, Vertex32S{0, 20}, Vertex32S{40, 40}});

    polygons.push_back({{20, 0}, {0, 20}, {40, 40}});
  }

  return tests;
}

INSTANTIATE_TEST_SUITE_P(
    TriangleInsideTriangle, MergeTest,
    testing::ValuesIn(CreateParameters<OperationType::kMerge>()));

INSTANTIATE_TEST_SUITE_P(
    TriangleInsideTriangle, UnionTest,
    testing::ValuesIn(CreateParameters<OperationType::kUnion>()));
}  // anonymous namespace
