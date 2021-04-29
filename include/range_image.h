#ifndef PC_OBJ_DETECT__RANGE_IMAGE__H_
#define PC_OBJ_DETECT__RANGE_IMAGE__H_

#include <vector>
#include <pcl/common/common_headers.h>

using PointT = pcl::PointXYZI;

class RangeImage {
public:
  RangeImage(pcl::PointCloud<PointT>::Ptr cloud_input, 
           float hor_res_input, float ver_res_input, 
           float image_width_input, float image_height_input);
  void convertToImage();
  void displayImage();

private:
  std::vector<std::vector<std::vector<double>>> image_data;
  pcl::PointCloud<PointT>::Ptr cloud_data;
  float max_hor_resolution, max_ver_resolution, image_width, image_height;
};

#endif // PC_OBJ_DETECT__RANGE_IMAGE__H_
