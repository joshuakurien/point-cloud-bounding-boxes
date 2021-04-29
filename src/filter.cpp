#include <unordered_map>
#include <random>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <filter.h>

Filter::Filter() {};

void Filter::distance (pcl::PointCloud<PointT>::Ptr cloud, int max_radius) {
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<PointT> extract;
  for (int i = 0; i < (*cloud).size(); i++) {
    PointT point(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
    if (std::abs(point.x) > max_radius || std::abs(point.y) > max_radius) {
      inliers->indices.push_back(i);
    }
  }
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud);  
}

void Filter::voxel (pcl::PointCloud<PointT>::Ptr cloud, float voxel_size) {
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (voxel_size, voxel_size, voxel_size);
  sor.filter (*cloud);
}

// Apply RANSAC to remove ground points
void Filter::groundRansac (pcl::PointCloud<PointT>::Ptr cloud, int num_iterations, float plane_thickness, float angle_threshold_deg) {
  std::random_device rd;
  std::default_random_engine gen(rd());
  std::uniform_int_distribution<> distrib(0, cloud->size()-1);
  
  const float kAngleThresholdRad = angle_threshold_deg*M_PI/180;
  int max_score = 0;
  std::unordered_map<char, float> max_plane_eqn;
  
  for (int i = 0; i < num_iterations; i++) {
    PointT rand_point_1 = *(cloud->begin() + distrib(gen));
    PointT rand_point_2 = *(cloud->begin() + distrib(gen));
    PointT rand_point_3 = *(cloud->begin() + distrib(gen));
  
    std::unordered_map<char, float> plane_vec_1({{'x',rand_point_1.x-rand_point_2.x}, 
                                                {'y',rand_point_1.y-rand_point_2.y}, 
                                                {'z',rand_point_1.z-rand_point_2.z}});

    std::unordered_map<char, float> plane_vec_2({{'x',rand_point_3.x-rand_point_2.x}, 
                                                {'y',rand_point_3.y-rand_point_2.y}, 
                                                {'z',rand_point_3.z-rand_point_2.z}});

    
    std::unordered_map<char, float> cur_plane_eqn({{'A', plane_vec_1['y']*plane_vec_2['z']-plane_vec_1['z']*plane_vec_2['y']}, 
                                              {'B', plane_vec_1['z']*plane_vec_2['x']-plane_vec_1['x']*plane_vec_2['z']}, 
                                              {'C', plane_vec_1['x']*plane_vec_2['y']-plane_vec_1['y']*plane_vec_2['x']}});
    
    int score = 0;
    float cur_plane_mag = std::sqrt(pow(cur_plane_eqn['A'],2) + pow(cur_plane_eqn['B'],2) + pow(cur_plane_eqn['C'],2));
    if (std::abs(std::acos(cur_plane_eqn['C']/cur_plane_mag)) < kAngleThresholdRad) {
      for (const auto& point: *cloud) {
        float point_distance_num = std::abs(cur_plane_eqn['A']*point.x + cur_plane_eqn['B']*point.y + cur_plane_eqn['C'] - point.z);
        float point_distance_denom = std::sqrt(pow(cur_plane_eqn['A'],2) + pow(cur_plane_eqn['B'],2) + 1);
        float point_distance = point_distance_num/point_distance_denom;
        if (point_distance < plane_thickness) {
          score++;
        }
      }
    }

    if (score > max_score) {
      max_score = score;
      max_plane_eqn = cur_plane_eqn;
    }
  }

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<PointT> extract;
  for (int i = 0; i < (*cloud).size(); i++) {
    PointT point(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
    float point_distance_num = std::abs(max_plane_eqn['A']*point.x + max_plane_eqn['B']*point.y + max_plane_eqn['C'] - point.z);
    float point_distance_denom = std::sqrt(pow(max_plane_eqn['A'],2) + pow(max_plane_eqn['B'],2) + 1);
    float point_distance = point_distance_num/point_distance_denom;
    if (point_distance < plane_thickness) {
      inliers->indices.push_back(i);
    }
  }
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud);    
}