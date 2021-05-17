#ifndef PC_OBJ_DETECT__CLOUD_FILTER__H_
#define PC_OBJ_DETECT__CLOUD_FILTER__H_

#include <unordered_map>
#include <random>
#include <pcl/common/common_headers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

using PointT = pcl::PointXYZI;

// Class used for reducing the number points from 
// the point cloud through various filters
class CloudFilter {
public:
  CloudFilter();
  pcl::PointCloud<PointT>::Ptr distance (pcl::PointCloud<PointT>::Ptr cloud, double max_radius, double min_radius = 2);
  pcl::PointCloud<PointT>::Ptr voxel (pcl::PointCloud<PointT>::Ptr cloud, float leaf_size);
  pcl::PointCloud<PointT>::Ptr ground (pcl::PointCloud<PointT>::Ptr cloud);
private:
  pcl::PointCloud<PointT>::Ptr removePoints(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointIndices::Ptr inliers);
};

#endif // PC_OBJ_DETECT__CLOUD_FILTER__H_
