#include <Eigen/Dense>
#include <iostream>
#include <limits>
#include <random>
#include <vector>

// Some constants & typedefs.
namespace {
static constexpr size_t kDimension = 2;
typedef Eigen::Matrix<double, Eigen::Dynamic, kDimension> Path;
using Eigen::Vector2d;
}  // namespace

// Class defining a line segment.
struct Segment {
  Segment(const Vector2d& first, const Vector2d& second)
      : first(first), second(second), length((first - second).norm()) {}
  const Vector2d first;
  const Vector2d second;
  const double length;
};

// Make points along a straight-ish line, adding some random noise to each point. Returns
// points as a Nx2 matrix.
Path MakePath() {
  // Points in path.
  const size_t num_points = 5;
  // The magnitude of the randomness to give to each point's position.
  const double random_magnitude = 1.0;
  // The amount to deterministically increase each point coordinate.
  const double path_magnitude = 4;

  // Create the path.
  Path path = Path::Random(num_points, kDimension) * random_magnitude;
  path.resize(num_points, kDimension);
  for (size_t i = 0; i < num_points; ++i) {
    Vector2d p(i * path_magnitude, i * path_magnitude);
    // Add major magnitude + existing random initialization.
    path.row(i) = path.row(i) + p.transpose();
  }

  return path;
}

// Given a 2D line segment and point, compute the along-track and cross-track distances
// w.r.t. that segment, and compute the projection point itself.
void ProjectToSegment(const Segment& segment, const Vector2d& point,
                      double& cross_track_dist, double& along_track_dist,
                      Vector2d& projection_point) {
  const Vector2d AB = segment.second - segment.first;
  const Vector2d AC = point - segment.first;
  const double L_AB = AB.norm();
  const double L_AC = AC.norm();

  if (L_AB > 0.0 && L_AC > 0.0) {
    // Compute the projection to the segment for along-track distance.
    along_track_dist = AB.dot(AC) / L_AC;
    const Vector2d AB_normal(AB[1], -AB[0]);

    // Compute the projection to the segment normal for cross-track distance.
    cross_track_dist = AC.dot(AB_normal);

    // Compute the projection point.
    const double along_track_t = along_track_dist / L_AB;
    projection_point = segment.first + along_track_t * AB;
  } else {
    // If the segment has no length, return infinite cross-track/along-track distance.
    cross_track_dist = std::numeric_limits<double>::infinity();
    along_track_dist = std::numeric_limits<double>::infinity();
    projection_point = segment.first;
  }
  return;
}

// Find the closest point on a path from a given point. Returns (closest point, distance).
std::pair<Vector2d, double> FindNearest(const Path& path, const Vector2d& point) {
  // The number of points in the path.
  const size_t path_size = path.rows();
  // The best minimum distance found so far while searching through the path.
  double min_distance = std::numeric_limits<double>::infinity();
  // The point on the path corresponding to that minimum distance.
  Vector2d closest_point;

  // For each line segment in the path, see if the point in question is closest to the
  // segment itself or to one of the endpoints.
  for (size_t i = 0; i < path_size - 1; ++i) {
    // The segment in question.
    const Segment segment(path.row(i).transpose(), path.row(i + 1).transpose());

    // Compute the projection to the segment.
    // Note: Returns infinite distance if the projection cannot be made.
    double cross_track_dist, along_track_dist;
    Vector2d projected_point;
    ProjectToSegment(segment, point, cross_track_dist, along_track_dist, closest_point);

    // If it exists, compare the orthogonal projection to the segment against best known
    // distance.
    if (0.0 <= along_track_dist && along_track_dist <= segment.length &&
        cross_track_dist < min_distance) {
      min_distance = std::abs(cross_track_dist);
      closest_point = projected_point;
    } else {
      // If we can't project orthogonally onto the segment, check the endpoints.
      // Note: Only check the second point on the last iteration. (Each other point i will
      // be the first point in the segment on the ith iteration).
      const double distance_a = (segment.first - point).norm();
      if (distance_a < min_distance) {
        min_distance = distance_a;
        closest_point = segment.first;
      }
      if (i == (path_size - 2)) {
        const double distance_b = (segment.second - point).norm();
        if (distance_b < min_distance) {
          min_distance = distance_b;
          closest_point = segment.second;
        }
      }
    }
  }

  return std::make_pair(closest_point, min_distance);
}

int main() {
  Path path = MakePath();
  Vector2d point = {7, 8};
  Vector2d nearest_point;
  double distance;
  std::tie(nearest_point, distance) = FindNearest(path, point);
  std::cout << "nearest point is\n"
            << nearest_point << "\nat distance " << distance << std::endl;
  std::cout << "Path:\n" << path << std::endl;
  return 0;
}
