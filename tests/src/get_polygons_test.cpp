#include "get_polygons_test.hpp"

#include <gtest/gtest.h>

std::ostream& operator<<(std::ostream& stream, const Parameters& parameters) {
  return stream << parameters.triangles;
}

TEST_P(GetPolygonsTest, Test) {
  const auto& parameters = GetParam();

  TriClipper32S clipper;
  const auto& triangles = parameters.triangles;
  for (const auto& triangle : triangles) {
    clipper.AddTriangle(triangle[0], triangle[1], triangle[2]);
  }

  clipper.Merge();

  std::vector<Vertex32S> vertices;
  std::vector<size_t> offsets;
  const auto polygons_count = clipper.GetPolygons(vertices, offsets);

  const auto& expected_polygons = parameters.polygons;
  ASSERT_EQ(polygons_count, expected_polygons.size())
      << "Incorrect number of polygons, " << polygons_count
      << " polygon(s) found, expected " << expected_polygons.size();

  for (size_t polygon_index = 0u; polygon_index < polygons_count;
       ++polygon_index) {
    const auto& expected_polygon = expected_polygons[polygon_index];

    const auto actual_vertices_count =
        offsets[polygon_index + 1u] - offsets[polygon_index];
    ASSERT_EQ(actual_vertices_count, expected_polygon.size())
        << "Incorrect number of polygon vertices, " << actual_vertices_count
        << " vertices found, expected " << expected_polygon.size()
        << "; polygon index is " << polygon_index;

    for (size_t actual_vertex_index = offsets[polygon_index],
                expected_vertex_index = 0u;
         actual_vertex_index < offsets[polygon_index + 1u];
         ++actual_vertex_index, ++expected_vertex_index) {
      ASSERT_EQ(vertices[actual_vertex_index],
                expected_polygon[expected_vertex_index])
          << "Incorrect polygon vertex " << vertices[actual_vertex_index]
          << ", expected " << expected_polygon[expected_vertex_index]
          << "; polygon index is " << polygon_index << "; vertex index is "
          << expected_vertex_index;
    }
  }
}
