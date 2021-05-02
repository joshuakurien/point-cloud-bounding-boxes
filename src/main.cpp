#include <iostream>
#include <thread>
#include <math.h>
#include <vector>
#include <map>
#include <pcl/io/pcd_io.h>

#include <cloud_filter.h>
#include <range_image.h>
#include <cloud_visualizer.h>

using namespace std::chrono_literals;

int main (int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ground_removed (new pcl::PointCloud<pcl::PointXYZI>);
  
  // Load pcd file into point cloud shared pointer
  if (pcl::io::loadPCDFile<pcl::PointXYZI> ("../data/test_data.pcd", *cloud) == -1)
  {
    PCL_ERROR ("Couldn't read the pcd file \n");
    return (-1);
  }

  auto filter = std::make_shared<CloudFilter>();
  cloud_filtered = filter->distance(cloud);

  std::map<double, int> map;
  for (auto pt : *cloud) {
    if (map.find(pt.z) == map.end())
      map[pt.z]++;
  }
  int count = 0;
  for (auto kv : map) {
    std::cout << "first: " << kv.first << "second: " << kv.second << std::endl;
    break;
  }
  std::cout << count << std::endl;
  cloud_downsampled = filter->voxel(cloud_filtered);

  const double kFovUp = 2.0, kFovDown = 24.9, kImageWidth = 4500.0, kImageHeight = 64.0;
  auto range_image = std::make_shared<RangeImage>(cloud_downsampled, kFovUp, kFovDown, kImageWidth, kImageHeight);
  
  cloud_ground_removed = filter->ground(cloud_downsampled, range_image);

  CloudVisualizer vis;
  vis.addCloud(cloud, "sample_cloud");
  // Main viewer loop
  while (!vis.viewer->wasStopped ())
  {
    vis.viewer->spinOnce (100);
    // std::this_thread::sleep_for(100ms);
  }
}