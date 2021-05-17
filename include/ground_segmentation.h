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
  std::vector<PointT> getGroundPoints();
  std::vector<PointT> getNonGroundPoints();
  std::vector<pcl::PointCloud<PointT>::Ptr> getPointBins();
private:
  void createPointBins(pcl::PointCloud<PointT>::Ptr cloud);
  void segmentGround();
  void separateBinPoints(const std::vector<ground_label>& labels, const Bin& bin);
  std::vector<ground_label> findKeyPoints(const Bin& bin);
  bool compareConsecutivePoints(const PointT & prev, const PointT & cur, bool is_labelling_ground);
  double sensor_height = -1.9;
  BinContainer point_bins;
  std::vector<PointT> ground_points;
  std::vector<PointT> non_ground_points;
  const double kAzimuthResolutionDeg = 0.08;
  const double kMaxAngleDeg = 45.0;
  const double kMaxAngleRad = kMaxAngleDeg*M_PI/180;
  const double kMinHeight = 0.1; 
};

#endif // PC_OBJ_DETECT__GROUND_SEGMENTATION__H_