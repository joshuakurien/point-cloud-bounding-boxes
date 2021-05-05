#include <cloud_visualizer.h>

// Constructor creates viewer to visualize points
CloudVisualizer::CloudVisualizer() {
  viewer = std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
}

// Adds clouds to be visualized by the viewer
void CloudVisualizer::addCloud(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, std::string cloud_name, bool get_random_colour) {
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> point_cloud_color_handler(cloud, "intensity");
  viewer->addPointCloud<pcl::PointXYZI> (cloud, point_cloud_color_handler, cloud_name);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_name);
  if (!get_random_colour) {
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, cloud_name);
  } else {
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    double r = static_cast<double>(rand()) / RAND_MAX;
    double g = static_cast<double>(rand()) / RAND_MAX;
    double b = static_cast<double>(rand()) / RAND_MAX;
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1.0, 0, cloud_name);
  }
}