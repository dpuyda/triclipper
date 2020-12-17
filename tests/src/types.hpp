#pragma once

#include <array>
#include <optional>
#include <triclipper/triclipper.hpp>

template <typename T>
struct Vertex {
  Vertex(const T x_value, const T y_value) : x(x_value), y(y_value) {}
  Vertex() : Vertex(T(0), T(0)) {}
  T x;
  T y;
};

template <typename T>
using Triangle = std::array<Vertex<T>, 3>;

template <typename T>
using Polygon = std::vector<Vertex<T>>;

using Vertex32S = Vertex<int32_t>;
using Triangle32S = Triangle<int32_t>;
using Polygon32S = Polygon<int32_t>;
using TriClipper32S = triclipper::TriClipper<Vertex32S, int32_t>;

template <typename T>
bool operator==(const Vertex<T>& lhs, const Vertex<T>& rhs) {
  return lhs.x == rhs.x && lhs.y == rhs.y;
}

template <typename T>
bool operator!=(const Vertex<T>& lhs, const Vertex<T>& rhs) {
  return !(lhs == rhs);
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const Vertex<T>& vertex) {
  return os << "(" << vertex.x << "," << vertex.y << ")";
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const Triangle<T>& triangle) {
  return os << "{" << triangle[0] << "," << triangle[1] << "," << triangle[2]
            << "}";
}

template <typename T>
std::ostream& operator<<(std::ostream& os,
                         const std::vector<Triangle<T>>& triangles) {
  os << triangles.size() << " triangle(s)";
  if (!triangles.empty()) {
    auto it = triangles.begin();
    os << ": " << *it;
    while (++it != triangles.end()) {
      os << ", " << *it;
    }
  }
  return os;
}
