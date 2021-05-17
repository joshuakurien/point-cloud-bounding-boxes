#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/moment_of_inertia_estimation.h>

// Main class used to visualize point clouds
using PointT = pcl::PointXYZI;
class CloudVisualizer {
public:
  CloudVisualizer();
  void addCloud(pcl::PointCloud<PointT>::ConstPtr cloud, std::string cloud_name, bool get_random_colour, bool visualize_box = false);
  void addBoundingBox(pcl::PointCloud<PointT>::ConstPtr cloud, std::string box_name, double r_colour, double g_colour, double b_colour);
  pcl::visualization::PCLVisualizer::Ptr viewer;
};