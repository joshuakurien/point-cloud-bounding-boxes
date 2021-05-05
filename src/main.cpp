#include <iostream>
#include <thread>
#include <math.h>
#include <vector>
#include <pcl/io/pcd_io.h>

#include <cloud_filter.h>
#include <ground_segmentation.h>
#include <cloud_visualizer.h>

using PointT = pcl::PointXYZI;
using namespace std::chrono_literals;

int main (int argc, char** argv) {
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_downsampled (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_ground_removed (new pcl::PointCloud<PointT>);
  
  // Load pcd file into point cloud shared pointer
  if (pcl::io::loadPCDFile<PointT> ("../data/test_data.pcd", *cloud) == -1)
  {
    PCL_ERROR ("Couldn't read the pcd file \n");
    return (-1);
  }

  auto filter = std::make_shared<CloudFilter>();
  cloud_filtered = filter->distance(cloud, 20);
  // cloud_downsampled = filter->voxel(cloud_filtered);

  cloud_ground_removed = filter->ground(cloud_filtered);

  CloudVisualizer vis;
  vis.addCloud(cloud_filtered, "filtered_cloud", false);
  vis.addCloud(cloud_ground_removed, "ground points", true);
  // Main viewer loop
  while (!vis.viewer->wasStopped ())
  {
    vis.viewer->spinOnce (100);
    // std::this_thread::sleep_for(100ms);
  }
}