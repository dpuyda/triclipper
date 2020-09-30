#include "get_polygons_test.hpp"

namespace {
std::vector<Parameters> CreateParameters() {
  std::vector<Parameters> tests;

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{30, 0}, Vertex32S{0, 30}, Vertex32S{60, 60}});
    test.triangles.push_back(
        {Vertex32S{50, 10}, Vertex32S{30, 30}, Vertex32S{70, 50}});

    test.polygons.push_back(
        {{30, 0}, {0, 30}, {60, 60}, {50, 40}, {30, 30}, {40, 20}});
    test.polygons.push_back({{50, 10}, {40, 20}, {50, 40}, {70, 50}});
    test.polygons.push_back({{40, 20}, {30, 30}, {50, 40}});
  }

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 20}});
    test.triangles.push_back(
        {Vertex32S{40, 0}, Vertex32S{20, 20}, Vertex32S{60, 20}});

    test.polygons.push_back({{20, 0}, {0, 20}, {20, 20}, {30, 10}});
    test.polygons.push_back({{40, 0}, {30, 10}, {40, 20}, {60, 20}});
    test.polygons.push_back({{30, 10}, {20, 20}, {40, 20}});
  }

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{30, 0}, Vertex32S{10, 20}, Vertex32S{50, 20}});
    test.triangles.push_back(
        {Vertex32S{20, 10}, Vertex32S{0, 30}, Vertex32S{40, 30}});

    test.polygons.push_back({{30, 0}, {20, 10}, {30, 20}, {50, 20}});
    test.polygons.push_back({{20, 10}, {10, 20}, {30, 20}});
    test.polygons.push_back({{10, 20}, {0, 30}, {40, 30}, {30, 20}});
  }

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 20}});
    test.triangles.push_back(
        {Vertex32S{30, 10}, Vertex32S{10, 30}, Vertex32S{50, 30}});

    test.polygons.push_back({{20, 0}, {0, 20}, {20, 20}, {30, 10}});
    test.polygons.push_back({{30, 10}, {20, 20}, {40, 20}});
    test.polygons.push_back({{20, 20}, {10, 30}, {50, 30}, {40, 20}});
  }

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{30, 0}, Vertex32S{10, 20}, Vertex32S{50, 20}});
    test.triangles.push_back(
        {Vertex32S{20, 10}, Vertex32S{0, 30}, Vertex32S{40, 30}});
    test.triangles.push_back(
        {Vertex32S{40, 10}, Vertex32S{20, 30}, Vertex32S{60, 30}});

    test.polygons.push_back({{30, 0}, {20, 10}, {30, 20}, {40, 10}});
    test.polygons.push_back({{20, 10}, {10, 20}, {30, 20}});
    test.polygons.push_back({{40, 10}, {30, 20}, {50, 20}});
    test.polygons.push_back({{10, 20}, {0, 30}, {20, 30}, {30, 20}});
    test.polygons.push_back({{30, 20}, {20, 30}, {40, 30}});
    test.polygons.push_back({{30, 20}, {40, 30}, {60, 30}, {50, 20}});
  }

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{0, 20}, Vertex32S{40, 0}});
    test.triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{40, 20}, Vertex32S{40, 0}});
    test.triangles.push_back(
        {Vertex32S{10, 0}, Vertex32S{10, 20}, Vertex32S{30, 0}});

    test.polygons.push_back({{0, 0}, {0, 20}, {10, 15}, {10, 5}});
    test.polygons.push_back({{0, 0}, {10, 5}, {10, 0}});
    test.polygons.push_back({{10, 0}, {10, 5}, {20, 10}, {30, 0}});
    test.polygons.push_back({{30, 0}, {20, 10}, {40, 0}});
    test.polygons.push_back({{40, 0}, {20, 10}, {40, 20}});
    test.polygons.push_back({{10, 5}, {10, 15}, {20, 10}});
    test.polygons.push_back({{20, 10}, {10, 15}, {10, 20}});
  }

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{0, 20}, Vertex32S{40, 0}});
    test.triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{40, 20}, Vertex32S{40, 0}});
    test.triangles.push_back(
        {Vertex32S{10, 0}, Vertex32S{10, 20}, Vertex32S{30, 0}});
    test.triangles.push_back(
        {Vertex32S{10, 0}, Vertex32S{30, 20}, Vertex32S{30, 0}});

    test.polygons.push_back({{0, 0}, {0, 20}, {10, 15}, {10, 5}});
    test.polygons.push_back({{0, 0}, {10, 5}, {10, 0}});
    test.polygons.push_back({{10, 0}, {10, 5}, {20, 10}});
    test.polygons.push_back({{10, 0}, {20, 10}, {30, 0}});
    test.polygons.push_back({{30, 0}, {20, 10}, {30, 5}});
    test.polygons.push_back({{30, 0}, {30, 5}, {40, 0}});
    test.polygons.push_back({{40, 0}, {30, 5}, {30, 15}, {40, 20}});
    test.polygons.push_back({{10, 5}, {10, 15}, {20, 10}});
    test.polygons.push_back({{30, 5}, {20, 10}, {30, 15}});
    test.polygons.push_back({{20, 10}, {10, 15}, {10, 20}});
    test.polygons.push_back({{20, 10}, {30, 20}, {30, 15}});
  }

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{0, 20}, Vertex32S{40, 0}});
    test.triangles.push_back(
        {Vertex32S{0, 0}, Vertex32S{40, 20}, Vertex32S{40, 0}});
    test.triangles.push_back(
        {Vertex32S{10, 0}, Vertex32S{10, 20}, Vertex32S{30, 0}});
    test.triangles.push_back(
        {Vertex32S{10, 0}, Vertex32S{30, 20}, Vertex32S{30, 0}});
    test.triangles.push_back(
        {Vertex32S{0, 10}, Vertex32S{20, 30}, Vertex32S{40, 10}});

    test.polygons.push_back({{0, 0}, {0, 10}, {10, 10}, {10, 5}});
    test.polygons.push_back({{0, 0}, {10, 5}, {10, 0}});
    test.polygons.push_back({{10, 0}, {10, 5}, {20, 10}});
    test.polygons.push_back({{10, 0}, {20, 10}, {30, 0}});
    test.polygons.push_back({{30, 0}, {20, 10}, {30, 5}});
    test.polygons.push_back({{30, 0}, {30, 5}, {40, 0}});
    test.polygons.push_back({{40, 0}, {30, 5}, {30, 10}, {40, 10}});
    test.polygons.push_back({{10, 5}, {10, 10}, {20, 10}});
    test.polygons.push_back({{30, 5}, {20, 10}, {30, 10}});
    test.polygons.push_back({{0, 10}, {0, 20}, {7, 17}});
    test.polygons.push_back({{0, 10}, {7, 17}, {10, 15}, {10, 10}});
    test.polygons.push_back({{10, 10}, {10, 15}, {20, 10}});
    test.polygons.push_back({{20, 10}, {10, 15}, {10, 20}});
    test.polygons.push_back({{20, 10}, {10, 20}, {20, 30}, {30, 20}});
    test.polygons.push_back({{20, 10}, {30, 20}, {30, 15}});
    test.polygons.push_back({{20, 10}, {30, 15}, {30, 10}});
    test.polygons.push_back({{30, 10}, {30, 15}, {34, 17}, {40, 10}});
    test.polygons.push_back({{40, 10}, {34, 17}, {40, 20}});
    test.polygons.push_back({{10, 15}, {7, 17}, {10, 20}});
    test.polygons.push_back({{30, 15}, {30, 20}, {34, 17}});
  }

  return tests;
}

INSTANTIATE_TEST_SUITE_P(EdgeIntersections, GetPolygonsTest,
                         testing::ValuesIn(CreateParameters()));
}  // anonymous namespace
