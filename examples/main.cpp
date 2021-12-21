#include <triclipper/triclipper.h>

#include <cstdint>
#include <iostream>

struct Vertex {
  Vertex(const int32_t x_value, const int32_t y_value)
      : x(x_value), y(y_value) {}
  Vertex() : Vertex(0, 0) {}
  int32_t x;
  int32_t y;
};

bool operator==(const Vertex& lhs, const Vertex& rhs) {
  return lhs.x == rhs.x && lhs.y == rhs.y;
}

bool operator!=(const Vertex& lhs, const Vertex& rhs) { return !(lhs == rhs); }

int main() {
  triclipper::TriClipper<Vertex, int32_t> clipper;

  clipper.AddTriangle({20, 0}, {0, 20}, {40, 40});
  clipper.AddTriangle({20, 10}, {10, 20}, {30, 30});

  clipper.Execute();

  std::vector<Vertex> vertices;
  std::vector<size_t> offsets;
  const auto polygons_count = clipper.GetPolygons(vertices, offsets);

  std::cout << polygons_count << " polygon(s):" << std::endl;
  for (size_t polygon_index = 0u; polygon_index < polygons_count;
       ++polygon_index) {
    std::cout << std::endl << "Polygon " << polygon_index << ":" << std::endl;
    for (size_t vertex_index = offsets[polygon_index];
         vertex_index < offsets[polygon_index + 1u]; ++vertex_index) {
      std::cout << "(" << vertices[vertex_index].x << ", "
                << vertices[vertex_index].y << ")" << std::endl;
    }
  }

  return 0;
}
