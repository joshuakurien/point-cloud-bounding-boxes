#include <pcl/visualization/pcl_visualizer.h>

// Main class used to visualize point clouds
using PointT = pcl::PointXYZI;
class CloudVisualizer {
public:
  CloudVisualizer();
  void addCloud(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, std::string cloud_name, bool get_random_colour);
  pcl::visualization::PCLVisualizer::Ptr viewer;
};