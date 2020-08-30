#include "polygons_test.hpp"

namespace {
std::vector<Parameters> CreateParameters() {
  std::vector<Parameters> tests;

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{20, 20}, Vertex32S{40, 0}});
    test.triangles.push_back(
        {Vertex32S{10, 0}, Vertex32S{20, 10}, Vertex32S{30, 0}});

    test.polygons.push_back({{0, 0}, {20, 20}, {20, 10}, {10, 0}});
    test.polygons.push_back({{10, 0}, {20, 10}, {30, 0}});
    test.polygons.push_back({{30, 0}, {20, 10}, {20, 20}, {40, 0}});
  }

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{20, 20}, Vertex32S{40, 0}});
    test.triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{10, 10}, Vertex32S{20, 0}});

    test.polygons.push_back({{0, 0}, {10, 10}, {20, 0}});
    test.polygons.push_back({{20, 0}, {10, 10}, {20, 20}, {40, 0}});
  }

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{20, 20}, Vertex32S{40, 0}});
    test.triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{30, 10}, Vertex32S{40, 0}});

    test.polygons.push_back({{0, 0}, {20, 20}, {30, 10}, {20, 0}});
    test.polygons.push_back({{20, 0}, {30, 10}, {40, 0}});
  }

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{20, 20}, Vertex32S{40, 0}});
    test.triangles.push_back(
        {Vertex32S{10, 10}, Vertex32S{20, 20}, Vertex32S{30, 10}});

    test.polygons.push_back({{0, 0}, {10, 10}, {30, 10}, {40, 0}});
    test.polygons.push_back({{10, 10}, {20, 20}, {30, 10}});
  }

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{30, 30}, Vertex32S{60, 0}});
    test.triangles.push_back(
        {Vertex32S{20, 10}, Vertex32S{30, 20}, Vertex32S{40, 10}});

    test.polygons.push_back({{0, 0}, {30, 30}, {30, 20}, {20, 10}, {60, 0}});
    test.polygons.push_back({{20, 10}, {30, 20}, {40, 10}});
    test.polygons.push_back({{60, 0}, {20, 10}, {40, 10}, {30, 20}, {30, 30}});
  }

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{40, 40}, Vertex32S{80, 0}});
    test.triangles.push_back(
        {Vertex32S{30, 20}, Vertex32S{50, 20}, Vertex32S{40, 10}});

    test.polygons.push_back(
        {{0, 0}, {40, 40}, {50, 20}, {30, 20}, {40, 10}, {80, 0}});
    test.polygons.push_back({{40, 10}, {30, 20}, {50, 20}});
    test.polygons.push_back({{80, 0}, {40, 10}, {50, 20}, {40, 40}});
  }

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{30, 30}, Vertex32S{60, 0}});
    test.triangles.push_back(
        {Vertex32S{10, 10}, Vertex32S{20, 20}, Vertex32S{30, 10}});

    test.polygons.push_back(
        {{0, 0}, {10, 10}, {30, 10}, {20, 20}, {30, 30}, {60, 0}});
    test.polygons.push_back({{10, 10}, {20, 20}, {30, 10}});
  }

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{30, 30}, Vertex32S{60, 0}});
    test.triangles.push_back(
        {Vertex32S{30, 10}, Vertex32S{40, 20}, Vertex32S{50, 10}});

    test.polygons.push_back(
        {{0, 0}, {30, 30}, {40, 20}, {30, 10}, {50, 10}, {60, 0}});
    test.polygons.push_back({{30, 10}, {40, 20}, {50, 10}});
  }

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{40, 40}, Vertex32S{80, 0}});
    test.triangles.push_back(
        {Vertex32S{20, 20}, Vertex32S{40, 20}, Vertex32S{30, 10}});

    test.polygons.push_back({{0, 0}, {20, 20}, {30, 10}, {80, 0}});
    test.polygons.push_back({{30, 10}, {20, 20}, {40, 20}});
    test.polygons.push_back({{80, 0}, {30, 10}, {40, 20}, {20, 20}, {40, 40}});
  }

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{40, 40}, Vertex32S{80, 0}});
    test.triangles.push_back(
        {Vertex32S{40, 20}, Vertex32S{60, 20}, Vertex32S{50, 10}});

    test.polygons.push_back(
        {{0, 0}, {40, 40}, {60, 20}, {40, 20}, {50, 10}, {80, 0}});
    test.polygons.push_back({{50, 10}, {40, 20}, {60, 20}});
    test.polygons.push_back({{80, 0}, {50, 10}, {60, 20}});
  }

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{30, 30}, Vertex32S{60, 0}});
    test.triangles.push_back(
        {Vertex32S{20, 20}, Vertex32S{40, 20}, Vertex32S{30, 10}});

    test.polygons.push_back({{0, 0}, {20, 20}, {30, 10}, {60, 0}});
    test.polygons.push_back({{30, 10}, {20, 20}, {40, 20}});
    test.polygons.push_back({{60, 0}, {30, 10}, {40, 20}});
    test.polygons.push_back({{20, 20}, {30, 30}, {40, 20}});
  }

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{30, 30}, Vertex32S{60, 0}});
    test.triangles.push_back(
        {Vertex32S{10, 10}, Vertex32S{30, 10}, Vertex32S{20, 0}});

    test.polygons.push_back({{0, 0}, {10, 10}, {20, 0}});
    test.polygons.push_back({{20, 0}, {10, 10}, {30, 10}});
    test.polygons.push_back({{20, 0}, {30, 10}, {10, 10}, {30, 30}, {60, 0}});
  }

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{30, 30}, Vertex32S{60, 0}});
    test.triangles.push_back(
        {Vertex32S{20, 10}, Vertex32S{40, 10}, Vertex32S{30, 0}});

    test.polygons.push_back({{0, 0}, {30, 30}, {40, 10}, {20, 10}, {30, 0}});
    test.polygons.push_back({{30, 0}, {20, 10}, {40, 10}});
    test.polygons.push_back({{30, 0}, {40, 10}, {30, 30}, {60, 0}});
  }

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{30, 30}, Vertex32S{60, 0}});
    test.triangles.push_back(
        {Vertex32S{30, 10}, Vertex32S{50, 10}, Vertex32S{40, 0}});

    test.polygons.push_back({{0, 0}, {30, 30}, {50, 10}, {30, 10}, {40, 0}});
    test.polygons.push_back({{40, 0}, {30, 10}, {50, 10}});
    test.polygons.push_back({{40, 0}, {50, 10}, {60, 0}});
  }

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{20, 20}, Vertex32S{40, 0}});
    test.triangles.push_back(
        {Vertex32S{10, 10}, Vertex32S{30, 10}, Vertex32S{20, 0}});

    test.polygons.push_back({{0, 0}, {10, 10}, {20, 0}});
    test.polygons.push_back({{20, 0}, {10, 10}, {30, 10}});
    test.polygons.push_back({{20, 0}, {30, 10}, {40, 0}});
    test.polygons.push_back({{10, 10}, {20, 20}, {30, 10}});
  }

  return tests;
}

INSTANTIATE_TEST_SUITE_P(HorizontalEdges, PolygonsTest,
                         testing::ValuesIn(CreateParameters()));
}  // anonymous namespace
