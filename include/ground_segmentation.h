#ifndef PC_OBJ_DETECT__GROUND_SEGMENTATION__H_
#define PC_OBJ_DETECT__GROUND_SEGMENTATION__H_

#include <queue>
#include <algorithm>
#include <set>
#include <memory>
#include <map>
#include <vector>
#include <pcl/common/common_headers.h>

enum ground_label {
  initial_ground_point,
  ground_point, 
  threshold_point,
  non_ground_point
};

using PointT = pcl::PointXYZI;
using Bin = std::vector<PointT>;
using BinContainer = std::vector<Bin>;

double euclideanDistance (PointT pt);
double euclideanDistanceDifference (PointT pt1, PointT pt2);
double flatDistance (PointT pt);
double azimuth(PointT pt);

// Implementation of ground segmentation algorithm
class GroundSegmentation {
public:
  GroundSegmentation(pcl::PointCloud<PointT>::Ptr cloud);
  std::shared_ptr<std::vector<PointT>> getGroundPoints();
  std::vector<pcl::PointCloud<PointT>::Ptr> getPointBins();
private:
  void createPointBins(pcl::PointCloud<PointT>::Ptr cloud);
  void determineGroundPoints();
  void segmentBinGroundPoints(const Bin& bin);
  bool compareConsecutivePoints(const PointT & prev, const PointT & cur, bool is_labelling_ground);
  double sensor_height = -1.9;
  std::shared_ptr<BinContainer> point_bins;
  std::shared_ptr<std::vector<PointT>> ground_points;
  const double kAzimuthResolutionDeg = 0.08;
  const double kMaxAngleDeg = 45.0;
  const double kMaxAngleRad = kMaxAngleDeg*M_PI/180;
  const double kMinHeight = 0.1; 
};

#endif // PC_OBJ_DETECT__GROUND_SEGMENTATION__H_