#include <cloud_filter.h>
#include <ground_segmentation.h>

CloudFilter::CloudFilter() {};

pcl::PointCloud<PointT>::Ptr CloudFilter::removePoints(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointIndices::Ptr inliers) {
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_filtered);
  return cloud_filtered;
}

pcl::PointCloud<PointT>::Ptr CloudFilter::distance(pcl::PointCloud<PointT>::Ptr cloud, int max_radius) {
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  for (int i = 0; i < (*cloud).size(); i++) {
    PointT point(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
    if (std::abs(point.x) > max_radius || std::abs(point.y) > max_radius) {
      inliers->indices.push_back(i);
    }
  }
  return removePoints(cloud, inliers);
}

// Downsamples point cloud based on voxel leaf size
pcl::PointCloud<PointT>::Ptr CloudFilter::voxel(pcl::PointCloud<PointT>::Ptr cloud, float leaf_size) {
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (leaf_size, leaf_size, leaf_size);
  sor.filter (*cloud_filtered);
  return cloud_filtered;
}


pcl::PointCloud<PointT>::Ptr CloudFilter::ground(pcl::PointCloud<PointT>::Ptr cloud) {
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<PointT> extract;
  GroundSegmentation seg(cloud);
  auto ground_points = seg.getGroundPoints();

  for (auto point : *ground_points) {
    cloud_filtered->push_back(point);
  }

  // TODO fix this issue
  // for (int i = 0; i < (*cloud).size(); i++) {
  //   std::map<PointT, int>::iterator it = ground_points->find(cloud->points[i]);
  //   if (it != ground_points->end()) {
  //     inliers->indices.push_back(i);
  //     ground_points->erase(it);
  //   }
  // }

  // extract.setInputCloud(cloud);
  // extract.setIndices(inliers);
  // extract.setNegative(true);
  // extract.filter(*cloud_filtered);
  return cloud_filtered;
}