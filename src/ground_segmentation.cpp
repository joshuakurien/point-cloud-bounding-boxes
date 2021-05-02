#include <ground_segmentation.h>

// Constructor dynamically allocates memory for bins and ground points
GroundSegmentation::GroundSegmentation(pcl::PointCloud<PointT>::Ptr cloud) {
  point_bins = std::make_shared<BinContainer>();
  ground_points = std::make_shared<std::map<PointT, int>>();
  createPointBins(cloud);
}

// Organize points into bins along the same 'line' for main filtering algo
void GroundSegmentation::createPointBins(pcl::PointCloud<PointT>::Ptr cloud) {
  // Values based on HDL64 resolution - might be a free parameter for tuning later
  const double kAzimuthResolutionDeg = 0.08;
  const double kMaxBinNum = 360.0/kAzimuthResolutionDeg;
  const double kAzimuthResolutionRad = kAzimuthResolutionDeg*M_PI/180;

  // Sorts points based on azimuth angles to easily place into appropriate lines
  std::sort(cloud->points.begin(), cloud->points.end(),
  [](const PointT& p1, const PointT& p2) {
    return atan2(p1.y, p1.x) < atan2(p2.y, p2.x); 
  });

  // Main loop for placing each point into bin  
  double bin_max = -1*M_PI + kAzimuthResolutionRad;
  for (auto pt : *cloud) {
    auto yaw = atan2(pt.y, pt.x);
  }

  // Sorts each bin by euclidean distance to be able to compare points 
  // along the same line for core ground filtering algo 
  for (auto bin : *point_bins) {

  }
}

std::shared_ptr<std::map<PointT, int>> GroundSegmentation::getGroundPoints() {
  return ground_points;
}
