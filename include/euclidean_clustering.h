#include <vector>

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

using PointT = pcl::PointXYZI;

// Main implementation for algorithm that clusters point cloud points
class EuclideanClustering {
public:
  EuclideanClustering(pcl::PointCloud<PointT>::Ptr cloud);
  std::vector<pcl::PointCloud<PointT>::Ptr> getClusters();
private:
  double kMaxClusterPointDistance = 0.41;
  int kMinClusterSize = 100;
  int kMaxClusterSize = 27000;
  std::vector<pcl::PointCloud<PointT>::Ptr> clusters;
};