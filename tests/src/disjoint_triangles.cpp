#include "get_polygons_test.hpp"

namespace {
std::vector<Parameters> CreateParameters() {
  std::vector<Parameters> tests;

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{0, 20}, Vertex32S{40, 40}});
    test.triangles.push_back(
        {Vertex32S{30, 10}, Vertex32S{40, 30}, Vertex32S{50, 20}});

    test.polygons.push_back({{20, 0}, {0, 20}, {40, 40}});
    test.polygons.push_back({{30, 10}, {40, 30}, {50, 20}});
  }

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{10, 0}, Vertex32S{0, 30}, Vertex32S{20, 30}});
    test.triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{30, 30}, Vertex32S{40, 0}});
    test.triangles.push_back(
        {Vertex32S{50, 0}, Vertex32S{40, 30}, Vertex32S{60, 30}});

    test.polygons.push_back({{10, 0}, {0, 30}, {20, 30}});
    test.polygons.push_back({{20, 0}, {30, 30}, {40, 0}});
    test.polygons.push_back({{50, 0}, {40, 30}, {60, 30}});
  }

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{10, 0}, Vertex32S{0, 10}, Vertex32S{20, 10}});
    test.triangles.push_back(
        {Vertex32S{10, 0}, Vertex32S{20, 10}, Vertex32S{30, 0}});

    test.polygons.push_back({{10, 0}, {0, 10}, {20, 10}});
    test.polygons.push_back({{10, 0}, {20, 10}, {30, 0}});
  }

  {
    auto& test = tests.emplace_back();

    test.triangles.push_back(
        {Vertex32S{20, 0}, Vertex32S{10, 10}, Vertex32S{30, 10}});
    test.triangles.push_back(
        {Vertex32S{10, 10}, Vertex32S{0, 20}, Vertex32S{20, 20}});
    test.triangles.push_back(
        {Vertex32S{30, 10}, Vertex32S{20, 20}, Vertex32S{40, 20}});

    test.polygons.push_back({{20, 0}, {10, 10}, {30, 10}});
    test.polygons.push_back({{10, 10}, {0, 20}, {20, 20}});
    test.polygons.push_back({{30, 10}, {20, 20}, {40, 20}});
  }

  return tests;
}

INSTANTIATE_TEST_SUITE_P(DisjointTriangles, GetPolygonsTest,
                         testing::ValuesIn(CreateParameters()));
}  // anonymous namespace
