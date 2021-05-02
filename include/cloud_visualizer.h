#include <pcl/visualization/pcl_visualizer.h>

class CloudVisualizer {
public:
  CloudVisualizer();
  void addCloud(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, std::string cloud_name);
  pcl::visualization::PCLVisualizer::Ptr viewer;
};