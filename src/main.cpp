#include <iostream>
#include <thread>
#include <math.h>
#include <vector>
#include <pcl/io/pcd_io.h>

#include <cloud_filter.h>
#include <ground_segmentation.h>
#include <cloud_visualizer.h>
#include <euclidean_clustering.h>

using PointT = pcl::PointXYZI;
using namespace std::chrono_literals;

int main (int argc, char** argv) {
  // Used for random cloud color generator
  std::srand(static_cast<unsigned int>(std::time(nullptr)));
  
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_distance_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_ground_removed (new pcl::PointCloud<PointT>);
  if (pcl::io::loadPCDFile<PointT> ("../data/test_data.pcd", *cloud) == -1)
  {
    PCL_ERROR ("Couldn't read the pcd file \n");
    return (-1);
  }

  auto filter = std::make_shared<CloudFilter>();
  cloud_distance_filtered = filter->distance(cloud, 20);
  cloud_ground_removed = filter->ground(cloud_distance_filtered);
  EuclideanClustering clustering(cloud_ground_removed);
  std::vector<pcl::PointCloud<PointT>::Ptr> clusters = clustering.getClusters();
  
  CloudVisualizer vis;
  vis.addCloud(cloud_distance_filtered, "filtered_cloud", false);
  for (int i = 0; i < clusters.size(); i++) {
    vis.addCloud(clusters[i], "cluster " + std::to_string(i), true, true);
  }

  // Main viewer loop
  while (!vis.viewer->wasStopped ())
  {
    vis.viewer->spinOnce (100);
  }
}