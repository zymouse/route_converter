#include <lanelet/LaneletMap.h>
#include <lanelet/Geometry.h>

// Define the vertices of the polygon as a vector of Point2d objects
std::vector<lanelet::Point2d> vertices = {
  lanelet::Point2d(0, 0),
  lanelet::Point2d(1, 0),
  lanelet::Point2d(1, 1),
  lanelet::Point2d(0, 1)
};

// Create the polygon object using the vertices vector
auto polygon = lanelet::Polygon2d(vertices);

// Create the bounding box object from the polygon
auto bbox = lanelet::geometry::boundingBox2d(polygon);

