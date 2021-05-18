#ifndef PC_OBJ_DETECT__GROUND_SEGMENTATION__H_
#define PC_OBJ_DETECT__GROUND_SEGMENTATION__H_

#include <queue>
#include <algorithm>
#include <set>
#include <memory>
#include <map>
#include <vector>
#include <pcl/common/common_headers.h>

using PointT = pcl::PointXYZI;
using Bin = std::vector<PointT>;
using BinContainer = std::vector<Bin>;

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
  void separateBinPoints(const std::vector<bool>& labels, const Bin& bin);
  bool thresholdCheck(const PointT & prev, const PointT & cur);
  bool newGroundCheck(const PointT & prev, const PointT & cur, const PointT & last_ground_point);
  std::vector<bool> labelBinPoints(const Bin& bin);
  Bin ground_points;
  Bin non_ground_points;
  BinContainer point_bins;
  const double kSensorHeight = -1.83;
  const double kMinHeight = 0.12; 
  const double kMaxAngleDeg = 45.0;
  const double kMaxAngleRad = kMaxAngleDeg*M_PI/180;
  const double kAzimuthResolutionDeg = 0.07;
  const double kAzimuthResolutionRad = kAzimuthResolutionDeg*M_PI/180;
  const double kMaxBinNum = 360.0/kAzimuthResolutionDeg;
};

// TODO: Put in own class
// Miscellaneous mathematical functions
double euclideanDistance (PointT pt);
double euclideanDistanceDifference (PointT pt1, PointT pt2);
double gradient(const PointT & pt1, const PointT & pt2);
double flatDistance (PointT pt);
double azimuth(PointT pt);

#endif // PC_OBJ_DETECT__GROUND_SEGMENTATION__H_