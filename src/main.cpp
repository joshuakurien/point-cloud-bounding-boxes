#include <iostream>
#include <thread>
#include <math.h>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <cloud_filter.h>
#include <range_image.h>
#include <ground_segmentation.h>

using namespace std::chrono_literals;

// Simple viewer to visualize points clouds with intensity field
pcl::visualization::PCLVisualizer::Ptr visualizeClouds (pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud)
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  // pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> point_cloud_color_handler(cloud, "intensity");
  viewer->setBackgroundColor (0, 0, 0);
  // viewer->addPointCloud<pcl::PointXYZI> (cloud, point_cloud_color_handler, "sample cloud");
  viewer->addPointCloud<pcl::PointXYZI> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

int main (int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

  // Load pcd file into point cloud shared pointer
  if (pcl::io::loadPCDFile<pcl::PointXYZI> ("../data/test_cloud.pcd", *cloud) == -1)
  {
    PCL_ERROR ("Couldn't read the pcd file \n");
    return (-1);
  }

  // CloudFilter filter;
  // filter.distance(cloud);
  // filter.voxel(cloud);
  
  const double kFovUp = 2.0, kFovDown = 24.8, kImageWidth = 1024.0, kImageHeight = 64.0;
  RangeImage range_image(cloud, kFovUp, kFovDown, kImageWidth, kImageHeight);
  GroundSegmentation ground_segmentation(range_image);
  // pcl::visualization::PCLVisualizer::Ptr viewer = visualizeClouds(cloud);
  // // Main viewer loop
  // while (!viewer->wasStopped ())
  // {
  //   viewer->spinOnce (100);
  //   std::this_thread::sleep_for(100ms);
  // }
}