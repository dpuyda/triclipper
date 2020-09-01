// Copyright 2020 by Dmytro Puyda <dpuyda@gmail.com>
// Licensed under the MIT License.

#pragma once

#include <algorithm>
#include <cassert>
#include <queue>
#include <vector>

#ifdef TRICLIPPER_DEBUG
#include <ostream>
#endif  // #ifdef TRICLIPPER_DEBUG

namespace triclipper {
template <typename VertexType, typename CoordinateType,
          typename SignedAreaType = long>
class TriClipper {
 public:
  /**
   * Creates a `TriClipper` instance.
   */
  TriClipper();

  /**
   * Adds a triangle to the clipper. The order of vertices does not matter.
   *
   * @param[in] v0 Triangle's first vertex.
   * @param[in] v1 Triangle's second vertex.
   * @param[in] v2 Triangle's third vertex.
   */
  void AddTriangle(const VertexType& v0, const VertexType& v1,
                   const VertexType& v2);

  /**
   * Merges added polygons.
   */
  void Merge();

  /**
   * Gets the monotone polygons that are obtained as the result of the merge
   * operation.
   *
   * @note The polygons are clockwise oriented.
   *
   * @param[out] vertices The vertices of the polygons.
   * @param[out] offsets For each polygon, stores the index of its first vertex
   * in `vertices`. The last element is the total number of vertices.
   *
   * @return The number of polygons.
   */
  size_t GetPolygons(std::vector<VertexType>& vertices,
                     std::vector<size_t>& offsets) const;

 private:
  /// A monotone polygon.
  struct Polygon {
    Polygon();

    /// Polygon's left-hand side vertices.
    std::vector<VertexType> left_vertices;

    /// Polygon's right-hand side vertices.
    std::vector<VertexType> right_vertices;

    /// The index of the polygon that is to the right from the current one and
    /// is not separated from the current one by an edge.
    size_t joined_index;
  };

  /// An edge of an added polygon. Edges are oriented from bottom to top or, if
  /// horizontal, from left to right.
  struct Edge {
    Edge(const std::vector<VertexType>& vertices, size_t start_index,
         size_t end_index, size_t index);

    /// The index of edge's bottom or, if horizontal, left endpoint in
    /// `vertices_`.
    size_t start_index;

    /// The index of edge's top or, if horizontal, right endpoint in
    /// `vertices_`.
    size_t end_index;

    /// The X coordinate of edge's intersection with the scan line (valid if
    /// intersects). Initialized with `start.x`.
    CoordinateType bx;

    /// 1 if `end.x > start.x`, -1 of `end.x < start.x` or 0 if
    /// `end.x == start.x`.
    int dx_sign;

    /// The absolute value of `end.x - start.x`.
    SignedAreaType dx;

    /// The value of `end.y - start.y`. The value is always positive.
    SignedAreaType dy;

    /// `(end.x - start.x) / (end.y - start.y)` or `kInfiniteSlope` if the edge
    /// is horizontal.
    double slope;

    /// The index of the current edge in `edges_`.
    size_t index;

    /// 1 for left-hand side edges, -1 for right-hand side edges.
    int parity;

    /// Indices of the connected edges `edges_`. An edge is connected to
    /// the current one if edge's start point coincides with the current edge's
    /// end point.
    std::vector<size_t> next_indices;

    /// The previous edge above the scan line.
    Edge* prev_above;

    /// The next edge above the scan line.
    Edge* next_above;

    /// The next edge below the scan line.
    Edge* next_below;

    /// The index of the polygon to the right from the edge in `polygons_`.
    size_t polygon_index;
  };

  /// A closed horizontal interval.
  struct Interval {
    Interval(CoordinateType left, CoordinateType right);

    /// The interval's left point.
    CoordinateType left;

    /// The interval's right point.
    CoordinateType right;
  };

  /// A local minimum of an added polygon.
  struct LocalMinimum {
    LocalMinimum(size_t root_index, size_t left_index, size_t right_index);

    /// The index of the local minimum's root in `vertices_`.
    size_t root_index;

    /// The index of the first left-hand side edge in `edges_`. This edge cannot
    /// be horizontal.
    size_t left_index;

    /// The index of the first right-hand side edge in `edges_`. This edge can
    /// be horizontal.
    size_t right_index;
  };

  /// A joint between two edges appears when there are two collinear edges of
  /// different length. Both such edges cannot be active at the same time, and
  /// thus one of them must be split and the part of it must be connected to
  /// the shorter edge.
  struct Joint {
    Joint(Edge* to, Edge* edge);

    /// The edge to join.
    Edge* edge;

    /// The edge to join to.
    Edge* to;
  };

  /// An intersection of two edges. Collinear edges do not intersect. An
  /// intersection is never edge's endpoint.
  struct Intersection {
    Intersection(const VertexType& p, Edge* left, Edge* right);

    /// The intersection point.
    VertexType p;

    /// The left-hand side edge below the intersection point.
    Edge* left;

    /// The right-hand side edge below the intersection point.
    Edge* right;
  };

  /// Checks if `lhs.y < rhs.y` or `lhs.y == rhs.y && lhs.x < rhs.x`.
  static bool IsLess(const VertexType& lhs, const VertexType& rhs);

  /// Checks if rotation from `lhs` to `rhs` is clockwise.
  static bool IsSlopeLess(const Edge* lhs, const Edge* rhs);

  /// Checks if `lhs` and `rhs` are collinear.
  static bool SlopeEqual(const Edge* lhs, const Edge* rhs);

  /// Checks if `lhs.bx < rhs.bx` or `lhs.bx == lhs.bx &&
  /// IsSlopeLess(lhs, rhs)`.
  bool IsLess(const Edge* lhs, const Edge* rhs, bool& collinear);

  /// Calculates the X coordinate of the intersection of an edge and
  /// a horizontal line with a given Y coordinate. The horizontal line must not
  /// be below the edge.
  CoordinateType Intersect(const Edge* edge, CoordinateType y);

  /// Sorts local minimums from bottom to top and sets `local_minimum_iterator_`
  /// to the first local minimum.
  void InitLocalMinimums();

  /// Adds a scan line at the root of every local minimum. This is needed to
  /// handle polygons that don't intersect other polygons.
  void InitScanLinesY();

  /// Gets the next scan line Y coordinate from `scan_lines_y_`.
  bool PopScanLineY(CoordinateType& scan_line_y);

  /// Gets the first local minimum that is not processed yet and starts on
  /// the scan line.
  bool PopLocalMinimum(CoordinateType scan_line_y,
                       const LocalMinimum*& local_minimum);

  /// Adds an edge that is collinear to an active edge above the scan line to
  /// active edges above the scan line.
  Edge* AddCollinearEdge(Edge* edge, Edge* active_edge);

  /// Adds an edge to active edges above the scan line.
  Edge* AddEdge(Edge* edge, Edge* add_after);

  /// Adds local minimums that start on the scan line to active edges above
  /// the scan line.
  void AddLocalMinimums(CoordinateType scan_line_y);

  /// Updates active edges above the scan line after moving the scan line
  /// upward. For every active edge, calculates the intersection of the edge and
  /// the scan line located at `scan_line_y`.
  void UpdateEdges(CoordinateType scan_line_y);

  /// Active edges that are above the scan line before moving the scan line
  /// upward become active edges below the scan line after moving the scan line
  /// upward.
  void CopyAboveToBelow();

  /// Removes an active edge that ends on the scan line from active edges above
  /// the scan line.
  void RemoveEdge(const Edge* edge);

  /// Removes active edges that end on the scan line from active edges above
  /// the scan line.
  void RemoveEdges(CoordinateType scan_line_y);

  /// Adds edges that do not start at a local minimum of an added polygon to
  /// active edges above the scan line.
  void AddIntermediateEdges(CoordinateType scan_line_y);

  /// Merges the horizontal edges located on the scan line into a collection of
  /// disjoint closed intervals.
  void MergeHorizontalEdges();

  /// Swaps two active edges above the scan line.
  void SwapEdges(Edge* e1, Edge* e2);

  /// Swaps active edges above the scan line that intersect on the scan line.
  void SwapEdges(CoordinateType scan_line_y);

  /// Sets the scan line at a given Y coordinate.
  void SetScanLine(CoordinateType y);

  /// Adds a right-hand side vertex to a polygon.
  void AddRight(size_t polygon_index, CoordinateType x, CoordinateType y);

  /// Starts a polygon to the right from a given edge.
  void StartPolygon(Edge* edge, CoordinateType left, CoordinateType right,
                    CoordinateType y);

  /// Finishes a polygon to the right from a given edge.
  void FinishPolygon(Edge* edge, CoordinateType left, CoordinateType right,
                     CoordinateType y);

  /// Extends a polygon below the scan line to the right from `below` upward.
  void ExtendPolygon(size_t polygon_index, Edge* below, Edge* above,
                     CoordinateType scan_line_y);

  /// Splits a polygon by an active edge above the scan line.
  void Split(size_t polygon_index, Edge* above, CoordinateType left,
             CoordinateType right, CoordinateType scan_line_y);

  /// Joins two polygons.
  void Join(size_t left_index, size_t right_index, CoordinateType left,
            CoordinateType right, CoordinateType scan_line_y);

  /// Checks if two X coordinates belong to an interval.
  bool Connected(
      CoordinateType x0, CoordinateType x1,
      const typename std::vector<Interval>::iterator& interval) const;

  /// Processes active edges.
  void ProcessEdges(CoordinateType scan_line_y);

  /// Joins the edges that are stored in `joints_`. See the documentation for
  /// `Joint` for details.
  void ProcessJoints();

  /// Adds an intersection of two edges to `intersections_`.
  void AddIntersection(Edge* left, Edge* right, CoordinateType bottom_y,
                       CoordinateType top_y);

  /// Adds an edge to `below_edges_`.
  void AddToBelow(Edge* edge, Edge* add_after);

  /// Finds edge intersections with Y coordinates in the interval
  /// `(bottom_y, top_y]` and adds them to `intersections_`.
  void AddIntersections(CoordinateType bottom_y, CoordinateType top_y);

  /// Processes edge intersections is the scan beam above the scan line.
  void ProcessIntersections(CoordinateType scan_line_y);

#ifdef TRICLIPPER_DEBUG
  /// Prints local minimums to a given stream.
  void PrintLocalMinimums(std::ostream& stream) const;

  /// Prints edges that intersect the scan line, start on the scan line or end
  /// on the scan line to a given stream.
  void PrintActiveEdges(CoordinateType scan_line_y, std::ostream& stream) const;

  /// Prints the monotone polygons to a given stream.
  void PrintPolygons(std::ostream& stream) const;
#endif  // #ifdef TRICLIPPER_DEBUG

  /// The infinite index value. Used to initialize index values.
  static const size_t kInfiniteIndex;

  /// The `slope` value of horizontal edges.
  static const double kInfiniteSlope;

  /// `side` value for left-hand side edges.
  static const int kLeft;

  /// `side` value for right-hand side edges.
  static const int kRight;

  /// Vertices of the added polygons.
  std::vector<VertexType> vertices_;

  /// Edges of the added polygons.
  std::vector<Edge> edges_;

  /// Edge joints, see the documentation for `Joint` for details.
  std::vector<Joint> joints_;

  /// Disjoint closed intervals consisting of the horizontal edges located on
  /// the scan line.
  std::vector<Interval> intervals_;

  /// Local minimums of the added polygons.
  std::vector<LocalMinimum> local_minimums_;

  /// Points at the first local minimum that is not processed yet.
  typename std::vector<LocalMinimum>::const_iterator local_minimum_iterator_;

  /// Scan line Y coordinates. May contain duplicates.
  std::priority_queue<CoordinateType, std::vector<CoordinateType>,
                      std::greater<> >
      scan_lines_y_;

  /// The first active edge above the scan line. Active edges above the scan
  /// line are edges that intersect the scan line or start on the scan line.
  /// Active edges cannot be horizontal.
  Edge* above_edges_;

  /// The first active edge below the scan line. Active edges below the scan
  /// line are edges that intersect the scan line or end on the scan line.
  /// Active edges cannot be horizontal.
  Edge* below_edges_;

  /// Edge intersections.
  std::vector<Intersection> intersections_;

  /// Points at the first intersection that is not processed yet.
  typename std::vector<Intersection>::const_iterator intersection_iterator_;

  /// The monotone polygons.
  std::vector<Polygon> polygons_;
};

//------------------------------------------------------------------------------
//                                  PRIVATE API
//------------------------------------------------------------------------------

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
const size_t
    TriClipper<VertexType, CoordinateType, SignedAreaType>::kInfiniteIndex =
        std::numeric_limits<size_t>::max();

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
const double
    TriClipper<VertexType, CoordinateType, SignedAreaType>::kInfiniteSlope =
        std::numeric_limits<double>::max();

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
const int TriClipper<VertexType, CoordinateType, SignedAreaType>::kLeft = 1;

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
const int TriClipper<VertexType, CoordinateType, SignedAreaType>::kRight = -1;

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
TriClipper<VertexType, CoordinateType, SignedAreaType>::Polygon::Polygon()
    : joined_index(kInfiniteIndex) {}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
TriClipper<VertexType, CoordinateType, SignedAreaType>::Edge::Edge(
    const std::vector<VertexType>& vertices, const size_t start_index,
    const size_t end_index, const size_t index)
    : start_index(start_index),
      end_index(end_index),
      index(index),
      parity(0),
      prev_above(nullptr),
      next_above(nullptr),
      next_below(nullptr),
      polygon_index(kInfiniteIndex) {
  const auto& start = vertices[start_index];
  const auto& end = vertices[end_index];
  assert(start.y < end.y || (start.y == end.y && start.x <= end.x));
  bx = start.x;
  dx_sign = (start.x < end.x) - (end.x < start.x);
  dx = dx_sign > 0 ? static_cast<SignedAreaType>(end.x - start.x)
                   : static_cast<SignedAreaType>(start.x - end.x);
  dy = static_cast<SignedAreaType>(end.y - start.y);
  slope = dy ? dx_sign * static_cast<double>(dx) / dy : kInfiniteSlope;
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
TriClipper<VertexType, CoordinateType, SignedAreaType>::Interval::Interval(
    const CoordinateType left, const CoordinateType right)
    : left(left), right(right) {
  assert(left <= right);
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
TriClipper<VertexType, CoordinateType,
           SignedAreaType>::LocalMinimum::LocalMinimum(const size_t root_index,
                                                       const size_t left_index,
                                                       const size_t right_index)
    : root_index(root_index),
      left_index(left_index),
      right_index(right_index) {}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
TriClipper<VertexType, CoordinateType, SignedAreaType>::Joint::Joint(Edge* to,
                                                                     Edge* edge)
    : edge(edge), to(to) {}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
TriClipper<VertexType, CoordinateType,
           SignedAreaType>::Intersection::Intersection(const VertexType& p,
                                                       Edge* left, Edge* right)
    : p(p), left(left), right(right) {}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
bool TriClipper<VertexType, CoordinateType, SignedAreaType>::IsLess(
    const VertexType& lhs, const VertexType& rhs) {
  return lhs.y < rhs.y || (lhs.y == rhs.y && lhs.x < rhs.x);
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
bool TriClipper<VertexType, CoordinateType, SignedAreaType>::IsSlopeLess(
    const Edge* lhs, const Edge* rhs) {
  if (lhs->dx_sign < rhs->dx_sign) {
    return true;
  }
  if (rhs->dx_sign < lhs->dx_sign) {
    return false;
  }
  const auto cross_1 = lhs->dx * rhs->dy;
  const auto cross_2 = rhs->dx * rhs->dy;
  return lhs->dx_sign > 0 ? cross_1 < cross_2 : cross_2 < cross_1;
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
bool TriClipper<VertexType, CoordinateType, SignedAreaType>::SlopeEqual(
    const Edge* lhs, const Edge* rhs) {
  return lhs->dx_sign == rhs->dx_sign && lhs->dx * rhs->dy == rhs->dx * lhs->dy;
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
bool TriClipper<VertexType, CoordinateType, SignedAreaType>::IsLess(
    const Edge* lhs, const Edge* rhs, bool& collinear) {
  if (lhs->bx != rhs->bx) {
    return lhs->bx < rhs->bx;
  }

  if (lhs->dx_sign < rhs->dx_sign) {
    return true;
  }
  if (lhs->dx_sign > rhs->dx_sign) {
    return false;
  }
  const auto cross_1 = lhs->dx * rhs->dy;
  const auto cross_2 = rhs->dx * lhs->dy;
  collinear = cross_1 == cross_2;
  return lhs->dx_sign > 0 ? cross_1 < cross_2 : cross_2 < cross_1;
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
CoordinateType TriClipper<VertexType, CoordinateType,
                          SignedAreaType>::Intersect(const Edge* edge,
                                                     const CoordinateType y) {
  const auto& start = vertices_[edge->start_index];
  const auto& end = vertices_[edge->end_index];
  assert(end.y >= y);
  return end.y == y
             ? end.x
             : round(start.x + edge->slope * (static_cast<double>(y) -
                                              static_cast<double>(start.y)));
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
void TriClipper<VertexType, CoordinateType,
                SignedAreaType>::InitLocalMinimums() {
  // Sort the local minimums from bottom to top. The minimums with roots with
  // the same Y coordinate must be sorted from left to right. The order of
  // minimums with the same root does not matter.
  std::sort(local_minimums_.begin(), local_minimums_.end(),
            [&](const LocalMinimum& lhs, const LocalMinimum& rhs) {
              return IsLess(vertices_[lhs.root_index],
                            vertices_[rhs.root_index]);
            });

  local_minimum_iterator_ = local_minimums_.begin();
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
void TriClipper<VertexType, CoordinateType, SignedAreaType>::InitScanLinesY() {
  for (const auto& minimum : local_minimums_) {
    scan_lines_y_.push(vertices_[minimum.root_index].y);
  }
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
bool TriClipper<VertexType, CoordinateType, SignedAreaType>::PopScanLineY(
    CoordinateType& scan_line_y) {
  if (scan_lines_y_.empty()) {
    return false;
  }
  // `scan_lines_y_` may contain duplicates.
  scan_line_y = scan_lines_y_.top();
  while (!scan_lines_y_.empty() && scan_lines_y_.top() == scan_line_y) {
    scan_lines_y_.pop();
  }
  return true;
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
bool TriClipper<VertexType, CoordinateType, SignedAreaType>::PopLocalMinimum(
    const CoordinateType scan_line_y, const LocalMinimum*& local_minimum) {
  if (local_minimum_iterator_ == local_minimums_.end() ||
      vertices_[local_minimum_iterator_->root_index].y != scan_line_y) {
    return false;
  }
  local_minimum = &*local_minimum_iterator_;
  ++local_minimum_iterator_;
  return true;
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
typename TriClipper<VertexType, CoordinateType, SignedAreaType>::Edge*
TriClipper<VertexType, CoordinateType, SignedAreaType>::AddCollinearEdge(
    Edge* edge, Edge* active_edge) {
  assert(SlopeEqual(edge, active_edge));
  const auto edge_y = vertices_[edge->end_index].y;
  const auto active_edge_y = vertices_[active_edge->end_index].y;

  // `edge` is shorter than `active_edge`, split `active_edge`
  if (edge_y < active_edge_y) {
    joints_.emplace_back(edge, active_edge);
    edge->next_above = active_edge->next_above;
    edge->prev_above = active_edge->prev_above;
    if (edge->next_above) {
      edge->next_above->prev_above = edge;
    }
    if (edge->prev_above) {
      edge->prev_above->next_above = edge;
    }
    edge->parity += active_edge->parity;
    return edge;
  }

  // `edge` is longer than `active_edge`, attach `edge` to `active_edge`
  if (edge_y > active_edge_y) {
    joints_.emplace_back(active_edge, edge);
    active_edge->parity += edge->parity;
    return active_edge;
  }

  // The edges are of the same length, attach `edge` continuation to
  // `active_edge`
  active_edge->next_indices.insert(active_edge->next_indices.begin(),
                                   edge->next_indices.begin(),
                                   edge->next_indices.end());
  active_edge->parity += edge->parity;
  return active_edge;
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
typename TriClipper<VertexType, CoordinateType, SignedAreaType>::Edge*
TriClipper<VertexType, CoordinateType, SignedAreaType>::AddEdge(
    Edge* edge, Edge* add_after) {
  if (!above_edges_) {
    above_edges_ = edge;
    edge->prev_above = nullptr;
    edge->next_above = nullptr;
    return above_edges_;
  }

  // Check if the edge must be added before `above_edges_`.
  bool collinear = false;
  if (IsLess(edge, above_edges_, collinear)) {
    edge->prev_above = nullptr;
    edge->next_above = above_edges_;
    above_edges_->prev_above = edge;
    above_edges_ = edge;
    return edge;
  }
  if (collinear) {
    above_edges_ = AddCollinearEdge(edge, above_edges_);
    return above_edges_;
  }

  // Add the new edge after `add_after`.
  if (!add_after) {
    add_after = above_edges_;
  }

  while (add_after->next_above &&
         IsLess(add_after->next_above, edge, collinear)) {
    add_after = add_after->next_above;
  }
  if (collinear) {
    return AddCollinearEdge(edge, add_after->next_above);
  }

  edge->next_above = add_after->next_above;
  edge->prev_above = add_after;

  if (add_after->next_above) {
    add_after->next_above->prev_above = edge;
  }

  add_after->next_above = edge;
  return edge;
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
void TriClipper<VertexType, CoordinateType, SignedAreaType>::AddLocalMinimums(
    const CoordinateType scan_line_y) {
  const LocalMinimum* local_minimum;
  while (PopLocalMinimum(scan_line_y, local_minimum)) {
    auto* left_edge = &edges_[local_minimum->left_index];
    auto* right_edge = &edges_[local_minimum->right_index];

    // The left-hand side edge cannot be horizontal, simply add it to active
    // edges.
    auto* insert_after = AddEdge(left_edge, nullptr);

    // The right-hand side edge can be horizontal, skip horizontal edges and
    // add the first non-horizontal edge to active edges.
    while (vertices_[right_edge->start_index].y ==
           vertices_[right_edge->end_index].y) {
      intervals_.emplace_back(vertices_[right_edge->start_index].x,
                              vertices_[right_edge->end_index].x);
      right_edge = &edges_[right_edge->next_indices[0]];
    }
    AddEdge(right_edge, insert_after);

    // Add scan lines at edge's endpoints.
    scan_lines_y_.push(vertices_[left_edge->end_index].y);
    scan_lines_y_.push(vertices_[right_edge->end_index].y);
  }
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
void TriClipper<VertexType, CoordinateType, SignedAreaType>::UpdateEdges(
    const CoordinateType scan_line_y) {
  auto* edge = above_edges_;
  while (edge) {
    assert(vertices_[edge->end_index].y >= scan_line_y);
    edge->bx = Intersect(edge, scan_line_y);
    edge = edge->next_above;
  }
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
void TriClipper<VertexType, CoordinateType,
                SignedAreaType>::CopyAboveToBelow() {
  below_edges_ = above_edges_;
  auto* edge = above_edges_;
  while (edge) {
    edge->next_below = edge->next_above;
    edge = edge->next_above;
  }
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
void TriClipper<VertexType, CoordinateType, SignedAreaType>::
    AddIntermediateEdges(const CoordinateType scan_line_y) {
  static std::queue<Edge*> intermediate_edges;
  auto* edge = below_edges_;
  while (edge) {
    if (vertices_[edge->end_index].y == scan_line_y) {
      assert(intermediate_edges.empty());
      auto intermediate_edge = edge;
      do {
        for (const auto index : intermediate_edge->next_indices) {
          auto* next_edge = &edges_[index];
          if (vertices_[next_edge->end_index].y == scan_line_y) {
            intervals_.emplace_back(vertices_[next_edge->start_index].x,
                                    vertices_[next_edge->end_index].x);
            intermediate_edges.push(next_edge);
          } else {
            AddEdge(next_edge, nullptr);
          }
        }
        if (intermediate_edges.empty()) {
          break;
        }
        intermediate_edge = intermediate_edges.front();
        intermediate_edges.pop();
      } while (true);
    }
    edge = edge->next_below;
  }
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
void TriClipper<VertexType, CoordinateType,
                SignedAreaType>::MergeHorizontalEdges() {
  if (intervals_.empty()) {
    return;
  }
  std::sort(intervals_.begin(), intervals_.end(),
            [](const Interval& lhs, const Interval& rhs) {
              return lhs.left < rhs.left;
            });
  auto interval = intervals_.begin();
  for (auto iterator = interval + 1; iterator != intervals_.end(); ++iterator) {
    if (iterator->left <= interval->right) {
      interval->right = std::max(interval->right, iterator->right);
    } else {
      ++interval;
    }
  }
  intervals_.erase(++interval, intervals_.end());
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
void TriClipper<VertexType, CoordinateType, SignedAreaType>::SwapEdges(
    Edge* e1, Edge* e2) {
  if (e1->next_above == e2) {
    // `e1` precedes `e2`
    const auto next = e2->next_above;
    if (next) {
      next->prev_above = e1;
    }
    const auto prev = e1->prev_above;
    if (prev) {
      prev->next_above = e2;
    }
    e2->prev_above = prev;
    e2->next_above = e1;
    e1->prev_above = e2;
    e1->next_above = next;
  } else if (e2->next_above == e1) {
    // `e2` precedes `e1`
    const auto next = e1->next_above;
    if (next) {
      next->prev_above = e2;
    }
    const auto prev = e2->prev_above;
    if (prev) {
      prev->next_above = e1;
    }
    e1->prev_above = prev;
    e1->next_above = e2;
    e2->prev_above = e1;
    e2->next_above = next;
  } else {
    // None of `e1` and `e2` precedes the other edge
    const auto next = e1->next_above;
    const auto prev = e1->prev_above;
    e1->next_above = e2->next_above;
    if (e1->next_above) {
      e1->next_above->prev_above = e1;
    }
    e1->prev_above = e2->prev_above;
    if (e1->prev_above) {
      e1->prev_above->next_above = e1;
    }
    e2->next_above = next;
    if (e2->next_above) {
      e2->next_above->prev_above = e2;
    }
    e2->prev_above = prev;
    if (e2->prev_above) {
      e2->prev_above->next_above = e2;
    }
  }

  if (!e1->prev_above) {
    above_edges_ = e1;
  } else if (!e2->prev_above) {
    above_edges_ = e2;
  }
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
void TriClipper<VertexType, CoordinateType, SignedAreaType>::SwapEdges(
    const CoordinateType scan_line_y) {
  if (intersections_.empty()) {
    return;
  }
  while (intersection_iterator_ != intersections_.end() &&
         intersection_iterator_->p.y == scan_line_y) {
    SwapEdges(intersection_iterator_->left, intersection_iterator_->right);
    intersection_iterator_->right->bx = intersection_iterator_->left->bx;
    ++intersection_iterator_;
  }
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
void TriClipper<VertexType, CoordinateType, SignedAreaType>::SetScanLine(
    const CoordinateType y) {
  intervals_.clear();
  joints_.clear();
  UpdateEdges(y);
  CopyAboveToBelow();
  RemoveEdges(y);
  SwapEdges(y);
  AddIntermediateEdges(y);
  AddLocalMinimums(y);
  MergeHorizontalEdges();
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
void TriClipper<VertexType, CoordinateType, SignedAreaType>::AddRight(
    const size_t polygon_index, const CoordinateType x,
    const CoordinateType y) {
  auto* polygon = &polygons_[polygon_index];
  while (polygon->joined_index != kInfiniteIndex) {
    polygon = &polygons_[polygon->joined_index];
  }
  polygon->right_vertices.emplace_back(x, y);
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
void TriClipper<VertexType, CoordinateType, SignedAreaType>::StartPolygon(
    Edge* edge, const CoordinateType left, const CoordinateType right,
    const CoordinateType y) {
  edge->polygon_index = polygons_.size();
  polygons_.emplace_back();
  auto& polygon = polygons_.back();
  polygon.left_vertices.emplace_back(left, y);
  polygon.right_vertices.emplace_back(right, y);
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
void TriClipper<VertexType, CoordinateType, SignedAreaType>::FinishPolygon(
    Edge* edge, const CoordinateType left, const CoordinateType right,
    const CoordinateType y) {
  if (edge->polygon_index == kInfiniteIndex) {
    return;
  }
  auto* polygon = &polygons_[edge->polygon_index];
  polygon->left_vertices.emplace_back(left, y);
  polygon->right_vertices.emplace_back(right, y);
  while (polygon->joined_index != kInfiniteIndex) {
    polygon = &polygons_[polygon->joined_index];
    polygon->left_vertices.emplace_back(right, y);
    polygon->right_vertices.emplace_back(right, y);
  }
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
void TriClipper<VertexType, CoordinateType, SignedAreaType>::ExtendPolygon(
    const size_t polygon_index, Edge* below, Edge* above,
    const CoordinateType scan_line_y) {
  above->polygon_index = polygon_index;
  auto& polygon = polygons_[polygon_index];
  if (above->bx == below->bx) {
    if (!SlopeEqual(above, below)) {
      polygon.left_vertices.emplace_back(below->bx, scan_line_y);
    }
  } else {
    polygon.left_vertices.emplace_back(below->bx, scan_line_y);
    polygon.left_vertices.emplace_back(above->bx, scan_line_y);
  }
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
void TriClipper<VertexType, CoordinateType, SignedAreaType>::Split(
    const size_t polygon_index, Edge* above, const CoordinateType left,
    const CoordinateType right, const CoordinateType scan_line_y) {
  if (polygons_[polygon_index].joined_index == kInfiniteIndex) {
    above->polygon_index = polygons_.size();
    polygons_.emplace_back();

    auto& left_polygon = polygons_[polygon_index];
    auto& right_polygon = polygons_.back();

    right_polygon.right_vertices.push_back(left_polygon.right_vertices.back());
    right_polygon.left_vertices.push_back(left_polygon.right_vertices.back());
    if (left != right) {
      right_polygon.left_vertices.emplace_back(left, scan_line_y);
    }
    right_polygon.left_vertices.emplace_back(right, scan_line_y);

    left_polygon.right_vertices.emplace_back(left, scan_line_y);
  } else {
    auto* left_polygon = &polygons_[polygon_index];
    left_polygon->right_vertices.emplace_back(left, scan_line_y);

    above->polygon_index = left_polygon->joined_index;
    auto* right_polygon = &polygons_[left_polygon->joined_index];

    while (right_polygon->joined_index != kInfiniteIndex) {
      right_polygon->left_vertices.emplace_back(left, scan_line_y);
      right_polygon->right_vertices.emplace_back(left, scan_line_y);
      above->polygon_index = right_polygon->joined_index;
      right_polygon = &polygons_[right_polygon->joined_index];
    }

    if (left != right) {
      right_polygon->left_vertices.emplace_back(left, scan_line_y);
    }
    right_polygon->left_vertices.emplace_back(right, scan_line_y);

    left_polygon->joined_index = kInfiniteIndex;
  }
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
void TriClipper<VertexType, CoordinateType, SignedAreaType>::Join(
    const size_t left_index, const size_t right_index,
    const CoordinateType left, const CoordinateType right,
    const CoordinateType scan_line_y) {
  auto* left_polygon = &polygons_[left_index];
  while (left_polygon->joined_index != kInfiniteIndex) {
    left_polygon = &polygons_[left_polygon->joined_index];
  }

  if (left != right) {
    left_polygon->right_vertices.emplace_back(left, scan_line_y);
  }
  left_polygon->right_vertices.emplace_back(right, scan_line_y);

  polygons_[right_index].left_vertices.emplace_back(right, scan_line_y);
  left_polygon->joined_index = right_index;
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
bool TriClipper<VertexType, CoordinateType, SignedAreaType>::Connected(
    const CoordinateType x0, const CoordinateType x1,
    const typename std::vector<Interval>::iterator& interval) const {
  return interval != intervals_.end() && x0 >= interval->left &&
         x0 <= interval->right && x1 >= interval->left && x1 <= interval->right;
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
void TriClipper<VertexType, CoordinateType, SignedAreaType>::ProcessEdges(
    const CoordinateType scan_line_y) {
  auto* above = above_edges_;
  auto* below = below_edges_;
  Edge* prev_above = nullptr;
  Edge* prev_below = nullptr;
  auto interval = intervals_.begin();
  auto polygon_index = kInfiniteIndex;
  int parity = 0;

  while (above || below) {
    const auto x = above && below ? std::min(above->bx, below->bx)
                                  : above ? above->bx : below->bx;

    while (interval != intervals_.end() && interval->right <= x) {
      ++interval;
    }

    // Add right-hand side vertices to the polygon that is to the left
    if (above && below && above != below && polygon_index != kInfiniteIndex) {
      if (above->bx == below->bx && !SlopeEqual(above, below)) {
        AddRight(polygon_index, x, scan_line_y);
      } else if (above->bx == x && below->bx > x &&
                 Connected(above->bx, below->bx, interval)) {
        AddRight(polygon_index, below->bx, scan_line_y);
        AddRight(polygon_index, x, scan_line_y);
      } else if (below->bx == x && above->bx > x &&
                 Connected(below->bx, above->bx, interval)) {
        AddRight(polygon_index, x, scan_line_y);
        AddRight(polygon_index, above->bx, scan_line_y);
      }
    }

    // Finish the polygons between `below` and `below->next_below`
    if (below && (below->bx == x || Connected(x, below->bx, interval))) {
      while (below && below->next_below &&
             (below->next_below->bx == below->bx ||
              Connected(below->bx, below->next_below->bx, interval))) {
        FinishPolygon(below, below->bx, below->next_below->bx, scan_line_y);
        prev_below = below;
        below = below->next_below;
      }
      polygon_index = below->polygon_index;
    }

    // Start the polygons between `above` and `above->next_above`
    if (above && (above->bx == x || Connected(x, above->bx, interval))) {
      parity += above->parity;
      while (above && above->next_above &&
             (above->next_above->bx == above->bx ||
              Connected(above->bx, above->next_above->bx, interval))) {
        if (parity != 0) {
          StartPolygon(above, above->bx, above->next_above->bx, scan_line_y);
        }
        prev_above = above;
        above = above->next_above;
        parity += above->parity;
      }
    }

    // Add left-hand side vertices to the polygon that is to the right
    if (polygon_index != kInfiniteIndex && above && below && above != below &&
        (above->bx == x || Connected(x, above->bx, interval)) &&
        (below->bx == x || Connected(x, below->bx, interval))) {
      ExtendPolygon(polygon_index, below, above, scan_line_y);
    } else if (above && prev_below && below && prev_below->bx < x &&
               below->bx > above->bx) {
      Split(polygon_index, above, x, above->bx, scan_line_y);
      polygon_index = above->polygon_index;
    } else if (below && prev_above && above && prev_above->bx < x &&
               above->bx > below->bx) {
      Join(prev_above->polygon_index, below->polygon_index, x, below->bx,
           scan_line_y);
    }

    if (below && (below->bx == x || Connected(x, below->bx, interval))) {
      prev_below = below;
      below = below->next_below;
    }

    if (above && (above->bx == x || Connected(x, above->bx, interval))) {
      prev_above = above;
      above = above->next_above;
    }
  }
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
void TriClipper<VertexType, CoordinateType, SignedAreaType>::ProcessJoints() {
  for (const auto& joint : joints_) {
    joint.edge->start_index = joint.to->end_index;
    joint.edge->bx = vertices_[joint.to->end_index].x;
    joint.to->next_indices.push_back(joint.edge->index);
  }
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
void TriClipper<VertexType, CoordinateType, SignedAreaType>::AddIntersection(
    Edge* left, Edge* right, const CoordinateType bottom_y,
    const CoordinateType top_y) {
  if (vertices_[left->start_index].y == vertices_[left->end_index].y) {
    // `left` is vertical
    assert(right->slope != 0.);
    const auto x = vertices_[left->start_index].x;
    const auto y =
        round(vertices_[right->start_index].y +
              (static_cast<double>(vertices_[left->start_index].x) -
               static_cast<double>(vertices_[right->start_index].x)) /
                  right->slope);
    intersections_.emplace_back(VertexType(x, y), left, right);
  } else if (vertices_[right->start_index].y == vertices_[right->end_index].y) {
    // `right` is vertical
    assert(left->slope != 0.);
    const auto x = vertices_[right->start_index].x;
    const auto y = round(vertices_[left->start_index].y +
                         (static_cast<double>(vertices_[right->start_index].x) -
                          static_cast<double>(vertices_[left->start_index].x)) /
                             left->slope);
    intersections_.emplace_back(VertexType(x, y), left, right);
  } else {
    // Both `left` and `right` are non-vertical
    const auto b1 = vertices_[left->start_index].x -
                    vertices_[left->start_index].y * left->slope;
    const auto b2 = vertices_[right->start_index].x -
                    vertices_[right->start_index].y * right->slope;
    const auto q = (b2 - b1) / (left->slope - right->slope);
    const auto x = std::abs(left->slope) < std::abs(right->slope)
                       ? left->slope * q + b1
                       : right->slope * q + b2;
    intersections_.emplace_back(VertexType(round(x), round(q)), left, right);
  }

  // The intersection point must be in the interval `(bottom_y, top_y]`
  auto& p = intersections_.back().p;
  if (p.y > top_y) {
    p.y = top_y;
    p.x = std::abs(left->slope) < std::abs(right->slope)
              ? Intersect(left, top_y)
              : Intersect(right, top_y);
  } else if (p.y <= bottom_y) {
    p.y = bottom_y + 1;
    p.x = std::abs(left->slope) < std::abs(right->slope)
              ? Intersect(left, bottom_y + 1)
              : Intersect(right, bottom_y + 1);
  }
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
void TriClipper<VertexType, CoordinateType, SignedAreaType>::AddToBelow(
    Edge* edge, Edge* add_after) {
  if (add_after) {
    edge->next_below = add_after->next_below;
    add_after->next_below = edge;
  } else {
    // Add the new edge before `below_edges_`
    edge->next_below = below_edges_;
    below_edges_ = edge;
  }
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
void TriClipper<VertexType, CoordinateType, SignedAreaType>::AddIntersections(
    const CoordinateType bottom_y, const CoordinateType top_y) {
  // We use an auxiliary sorted edge list (SEL), stored in `below_edges_`, to
  // find edge intersections in the scan beam between `scan_line_y` and `top_y`.
  // The edges in SEL are sorted from right to left by their intersections with
  // the horizontal line at `top_y`. We initialize SEL by the first active edge
  // above `scan_line_y`. Next, we iterate the active edges above `scan_line_y`
  // from left to right starting from the second edge and check for
  // the intersections with edges from SEL. For each active edge above
  // `scan_line_y`, all edges in SEL intersect the horizontal line at
  // `scan_line_y` to the left from this edge. We iterate the edges in SEL,
  // staring from the right-most edge, until they intersect the horizontal line
  // at `top_y` to the left from the active edge and add their intersections
  // with the active edge. When all intersections are added, the active edge is
  // added to SEL.
  below_edges_ = above_edges_;
  below_edges_->next_below = nullptr;
  auto above = above_edges_->next_above;
  while (above) {
    const auto top_x = Intersect(above, top_y);
    auto below = below_edges_;
    Edge* prev_below = nullptr;
    CoordinateType x;
    // Process edges that intersect the horizontal line at `top_y` to the right
    // from the active edge.
    while (below) {
      assert(below->bx <= above->bx);
      x = Intersect(below, top_y);
      if (x <= top_x) {
        break;
      }
      AddIntersection(below, above, bottom_y, top_y);
      prev_below = below;
      below = below->next_below;
    }
    // Process edges that intersect the horizontal line at `top_y` at the same
    // point where the active edge does.
    if (below && x == top_x) {
      do {
        assert(below->bx <= above->bx);
        if (vertices_[below->end_index].y > top_y &&
            vertices_[above->end_index].y > top_y) {
          intersections_.emplace_back(VertexType(top_x, top_y), below, above);
        }
        prev_below = below;
        below = below->next_below;
        if (!below) {
          break;
        }
        x = Intersect(below, top_y);
      } while (x == top_x);
    }
    AddToBelow(above, prev_below);
    above = above->next_above;
  }

  // Sort the intersections from bottom to top
  std::sort(intersections_.begin(), intersections_.end(),
            [](const Intersection& lhs, const Intersection& rhs) {
              return IsLess(lhs.p, rhs.p);
            });
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
void TriClipper<VertexType, CoordinateType, SignedAreaType>::
    ProcessIntersections(const CoordinateType scan_line_y) {
  intersections_.clear();
  if (scan_lines_y_.empty()) {
    return;
  }
  const auto top_y = scan_lines_y_.top();
  AddIntersections(scan_line_y, top_y);
  if (intersections_.empty()) {
    return;
  }

  intervals_.clear();
  intersection_iterator_ = intersections_.begin();
  while (intersection_iterator_ != intersections_.end() &&
         intersection_iterator_->p.y != top_y) {
    const auto y = intersection_iterator_->p.y;
    UpdateEdges(y);
    CopyAboveToBelow();
    SwapEdges(y);
#ifdef TRICLIPPER_DEBUG
    PrintActiveEdges(y, std::cout);
#endif  // #ifdef TRICLIPPER_DEBUG
    ProcessEdges(y);
#ifdef TRICLIPPER_DEBUG
    PrintPolygons(std::cout);
#endif  // #ifdef TRICLIPPER_DEBUG
  }
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
void TriClipper<VertexType, CoordinateType, SignedAreaType>::RemoveEdge(
    const Edge* edge) {
  if (edge == above_edges_) {
    above_edges_ = edge->next_above;
  }
  if (edge->prev_above) {
    edge->prev_above->next_above = edge->next_above;
  }
  if (edge->next_above) {
    edge->next_above->prev_above = edge->prev_above;
  }
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
void TriClipper<VertexType, CoordinateType, SignedAreaType>::RemoveEdges(
    const CoordinateType scan_line_y) {
  auto* edge = above_edges_;
  while (edge) {
    if (vertices_[edge->end_index].y == scan_line_y) {
      RemoveEdge(edge);
    }
    edge = edge->next_above;
  }
}

#ifdef TRICLIPPER_DEBUG
template <typename VertexType, typename CoordinateType, typename SignedAreaType>
void TriClipper<VertexType, CoordinateType, SignedAreaType>::PrintLocalMinimums(
    std::ostream& stream) const {
  stream << local_minimums_.size() << " local minimum(s)";
  if (!local_minimums_.empty()) {
    stream << ":";
    for (const auto& minimum : local_minimums_) {
      // Print the root
      stream << std::endl
             << "root:(" << vertices_[minimum.root_index].x << ","
             << vertices_[minimum.root_index].y << ")";
      // Print left-hand side edges
      auto edge = minimum.left_index;
      stream << " l:(" << vertices_[edges_[edge].start_index].x << ","
             << vertices_[edges_[edge].start_index].y << ")";
      do {
        stream << "-(" << vertices_[edges_[edge].end_index].x << ","
               << vertices_[edges_[edge].end_index].y << ")";
        if (edges_[edge].next_indices.empty()) {
          break;
        }
        edge = edges_[edge].next_indices.front();
      } while (true);
      // Print right-hand side edges
      edge = minimum.right_index;
      stream << " r:(" << vertices_[edges_[edge].start_index].x << ","
             << vertices_[edges_[edge].start_index].y << ")";
      do {
        stream << "-(" << vertices_[edges_[edge].end_index].x << ","
               << vertices_[edges_[edge].end_index].y << ")";
        if (edges_[edge].next_indices.empty()) {
          break;
        }
        edge = edges_[edge].next_indices.front();
      } while (true);
    }
  }
  stream << std::endl << std::endl;
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
void TriClipper<VertexType, CoordinateType, SignedAreaType>::PrintActiveEdges(
    const CoordinateType scan_line_y, std::ostream& stream) const {
  stream << "Scan line at " << scan_line_y << ":" << std::endl;
  size_t count = 0;
  // Print above
  auto edge = above_edges_;
  while (edge) {
    ++count;
    edge = edge->next_above;
  }
  stream << count << " edge(s) above";
  if (count) {
    stream << ":";
    edge = above_edges_;
    while (edge) {
      const auto& start = vertices_[edge->start_index];
      const auto& end = vertices_[edge->end_index];
      stream << std::endl
             << "(" << start.x << "," << start.y << ")-(" << end.x << ","
             << end.y << ") at " << edge->bx;
      edge = edge->next_above;
    }
  }
  stream << std::endl;
  // Print below
  count = 0;
  edge = below_edges_;
  while (edge) {
    ++count;
    edge = edge->next_below;
  }
  stream << count << " edge(s) below";
  if (count) {
    stream << ":";
    edge = below_edges_;
    while (edge) {
      const auto& start = vertices_[edge->start_index];
      const auto& end = vertices_[edge->end_index];
      stream << std::endl
             << "(" << start.x << "," << start.y << ")-(" << end.x << ","
             << end.y << ") at " << edge->bx;
      edge = edge->next_below;
    }
  }
  stream << std::endl << std::endl;
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
void TriClipper<VertexType, CoordinateType, SignedAreaType>::PrintPolygons(
    std::ostream& stream) const {
  stream << polygons_.size() << " monotone polygon(s)";
  if (!polygons_.empty()) {
    stream << ":";
    size_t polygon_index = 0;
    for (const auto& polygon : polygons_) {
      // Print left-hand side edges
      stream << std::endl << polygon_index++ << ": l:";
      const auto& left = polygon.left_vertices;
      if (!left.empty()) {
        stream << "(" << left[0].x << "," << left[0].y << ")";
      }
      for (size_t index = 1u; index < left.size(); ++index) {
        stream << "-(" << left[index].x << "," << left[index].y << ")";
      }
      // Print right-hand side edges
      stream << " r:";
      const auto& right = polygon.right_vertices;
      if (!right.empty()) {
        stream << "(" << right[0].x << "," << right[0].y << ")";
      }
      for (size_t index = 1u; index < right.size(); ++index) {
        stream << "-(" << right[index].x << "," << right[index].y << ")";
      }
    }
  }
  stream << std::endl << std::endl;
}
#endif  // #ifdef TRICLIPPER_DEBUG

//------------------------------------------------------------------------------
//                                  PUBLIC API
//------------------------------------------------------------------------------

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
TriClipper<VertexType, CoordinateType, SignedAreaType>::TriClipper()
    : above_edges_(nullptr), below_edges_(nullptr) {}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
void TriClipper<VertexType, CoordinateType, SignedAreaType>::AddTriangle(
    const VertexType& v0, const VertexType& v1, const VertexType& v2) {
  if (v0 == v1 || v0 == v2) {
    return;
  }
  auto i0 = vertices_.size();
  auto i1 = i0 + 1;
  auto i2 = i1 + 1;
  vertices_.emplace_back(v0);
  vertices_.emplace_back(v1);
  vertices_.emplace_back(v2);
  if (IsLess(vertices_[i2], vertices_[i0])) {
    std::swap(i0, i2);
  }
  if (IsLess(vertices_[i1], vertices_[i0])) {
    std::swap(i0, i1);
  }
  if (IsLess(vertices_[i2], vertices_[i1])) {
    std::swap(i1, i2);
  }
  const auto j0 = edges_.size();
  const auto j1 = j0 + 1;
  const auto j2 = j1 + 1;
  edges_.emplace_back(vertices_, i0, i1, j0);
  edges_.emplace_back(vertices_, i0, i2, j1);
  edges_.emplace_back(vertices_, i1, i2, j2);

  // Add a new local minimum
  auto* e0 = &edges_[j0];
  auto* e1 = &edges_[j1];
  auto* e2 = &edges_[j2];
  if (IsSlopeLess(e0, e1)) {
    e0->parity = kLeft;
    e1->parity = kRight;
    local_minimums_.emplace_back(i0, j0, j1);
  } else {
    e1->parity = kLeft;
    e0->parity = kRight;
    local_minimums_.emplace_back(i0, j1, j0);
  }
  if (e0->end_index == e2->start_index) {
    e2->parity = e0->parity;
    e0->next_indices.push_back(j2);
  } else {
    e2->parity = e1->parity;
    e1->next_indices.push_back(j2);
  }
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
void TriClipper<VertexType, CoordinateType, SignedAreaType>::Merge() {
  InitLocalMinimums();
#ifdef TRICLIPPER_DEBUG
  PrintLocalMinimums(std::cout);
#endif  // #ifdef TRICLIPPER_DEBUG
  InitScanLinesY();

  CoordinateType scan_line_y;
  while (PopScanLineY(scan_line_y)) {
    SetScanLine(scan_line_y);
#ifdef TRICLIPPER_DEBUG
    PrintActiveEdges(scan_line_y, std::cout);
#endif  // #ifdef TRICLIPPER_DEBUG
    ProcessEdges(scan_line_y);
    ProcessJoints();
    ProcessIntersections(scan_line_y);
#ifdef TRICLIPPER_DEBUG
    PrintPolygons(std::cout);
#endif  // #ifdef TRICLIPPER_DEBUG
  }
}

template <typename VertexType, typename CoordinateType, typename SignedAreaType>
size_t TriClipper<VertexType, CoordinateType, SignedAreaType>::GetPolygons(
    std::vector<VertexType>& vertices, std::vector<size_t>& offsets) const {
  offsets.reserve(polygons_.size() + 1u);
  offsets.push_back(0u);
  for (const auto& polygon : polygons_) {
    if (polygon.left_vertices.size() < 2u ||
        polygon.right_vertices.size() < 2u) {
      continue;
    }

    vertices.reserve(vertices.size() + polygon.left_vertices.size() +
                     polygon.right_vertices.size());

    // Insert left-hand side vertices
    std::transform(polygon.left_vertices.begin(), polygon.left_vertices.end(),
                   std::back_inserter(vertices),
                   [](const VertexType& vertex) { return vertex; });

    // Insert right-hand side vertices
    auto begin = polygon.right_vertices.rbegin();
    if (polygon.right_vertices.back() == polygon.left_vertices.back()) {
      ++begin;
    }
    std::transform(begin, polygon.right_vertices.rend() - 1,
                   std::back_inserter(vertices),
                   [](const VertexType& vertex) { return vertex; });
    if (polygon.right_vertices.front() != polygon.left_vertices.front()) {
      vertices.push_back(polygon.right_vertices.front());
    }

    offsets.push_back(vertices.size());
  }
  return offsets.size() - 1u;
}
}  // namespace triclipper
