#include <cloud_visualizer.h>
#include <memory>
#include <pcl/common/common_headers.h>

CloudVisualizer::CloudVisualizer() {
  viewer = std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
}

void CloudVisualizer::addCloud(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, std::string cloud_name) {
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> point_cloud_color_handler(cloud, "intensity");
  viewer->addPointCloud<pcl::PointXYZI> (cloud, point_cloud_color_handler, cloud_name);
  // viewer->addPointCloud<pcl::PointXYZI> (cloud, cloud_name);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_name);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, cloud_name);
}