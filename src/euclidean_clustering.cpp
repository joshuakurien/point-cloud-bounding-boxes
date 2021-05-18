#include <euclidean_clustering.h>

// TODO: Implement more optimized euclidean clustering from scratch without PCL
// Main euclidean clustering algorithm implemented with PCL
EuclideanClustering::EuclideanClustering(pcl::PointCloud<PointT>::Ptr cloud) {
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (cloud);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (kMaxClusterPointDistance);
  ec.setMinClusterSize (kMinClusterSize);
  ec.setMaxClusterSize (kMaxClusterSize);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
    for (const auto& idx : it->indices) {
      cloud_cluster->push_back ((*cloud)[idx]);
    }
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    clusters.push_back(cloud_cluster);
  }
}

// Returns output of euclidean clustering algorithm
std::vector<pcl::PointCloud<PointT>::Ptr> EuclideanClustering::getClusters() {
  return clusters;
}
