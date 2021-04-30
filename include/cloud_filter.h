#ifndef PC_OBJ_DETECT__CLOUD_FILTER__H_
#define PC_OBJ_DETECT__CLOUD_FILTER__H_

#include <pcl/common/common_headers.h>

using PointT = pcl::PointXYZI;

class CloudFilter {
public:
  CloudFilter();
  // Default values for the CloudFilters included
  void distance (pcl::PointCloud<PointT>::Ptr cloud, int max_radius = 20);
  void voxel (pcl::PointCloud<PointT>::Ptr cloud, float voxel_size = 0.04f);
  void groundRansac (pcl::PointCloud<PointT>::Ptr cloud, int num_iterations = 2000, float plane_thickness = 0.5, float angle_threshold_deg = 2);
};

#endif // PC_OBJ_DETECT__CLOUD_FILTER__H_
