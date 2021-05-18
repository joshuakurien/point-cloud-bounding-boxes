#include <cloud_visualizer.h>

// Constructor creates viewer to visualize points
CloudVisualizer::CloudVisualizer() {
  viewer = std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
}

// Adds clouds to be visualized by the viewer
void CloudVisualizer::addCloud(pcl::PointCloud<PointT>::ConstPtr cloud, std::string cloud_name, bool get_random_colour, bool visualize_box) {
  pcl::visualization::PointCloudColorHandlerGenericField<PointT> point_cloud_color_handler(cloud, "intensity");
  viewer->addPointCloud<PointT> (cloud, point_cloud_color_handler, cloud_name);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_name);
  double r, g, b;
  if (get_random_colour) {
    r = static_cast<double>(rand()) / RAND_MAX;
    g = static_cast<double>(rand()) / RAND_MAX;
    b = static_cast<double>(rand()) / RAND_MAX;
  } else {
    r = 1.0, g = 1.0, b = 1.0;
  }
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, cloud_name);

  if (visualize_box) {
    addBoundingBox(cloud, cloud_name + "_box", r, g, b);
  }
}

// Visualizes a bounding box for a given point cloud
void CloudVisualizer::addBoundingBox(pcl::PointCloud<PointT>::ConstPtr cloud, std::string box_name, 
double r_colour, double g_colour, double b_colour) {
  pcl::MomentOfInertiaEstimation <PointT> feature_extractor;
  feature_extractor.setInputCloud (cloud);
  feature_extractor.compute ();
  PointT min_point, max_point;
  feature_extractor.getAABB (min_point, max_point);
  viewer->addCube (min_point.x, max_point.x, min_point.y, max_point.y, min_point.z, max_point.z, 1.0, 1.0, 0.0, box_name);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, box_name);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, box_name);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3.5, box_name);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r_colour, g_colour, b_colour, box_name);
 
 }
