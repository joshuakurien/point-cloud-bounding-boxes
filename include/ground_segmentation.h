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
using Bin = std::shared_ptr<std::vector<PointT>>;
using BinContainer = std::vector<Bin>;

// Implementation of ground segmentation algorithm
class GroundSegmentation {
public:
  GroundSegmentation(pcl::PointCloud<PointT>::Ptr cloud);
  std::shared_ptr<std::map<PointT, int>> getGroundPoints();
private:
  void createPointBins(pcl::PointCloud<PointT>::Ptr cloud);
  std::shared_ptr<BinContainer> point_bins;
  std::shared_ptr<std::map<PointT, int>> ground_points;
};

#endif // PC_OBJ_DETECT__GROUND_SEGMENTATION__H_