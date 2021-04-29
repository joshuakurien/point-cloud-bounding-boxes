#ifndef PC_OBJ_DETECT__RANGE_IMAGE__H_
#define PC_OBJ_DETECT__RANGE_IMAGE__H_

#include <vector>
#include <pcl/common/common_headers.h>

using PointT = pcl::PointXYZI;

class RangeImage {
public:
  RangeImage(pcl::PointCloud<PointT>::Ptr cloud_input, 
           double hor_res_input, double ver_res_input, 
           double image_width_input, double image_height_input);
  void displayImage();
protected:
  void convertToImage();
  int getPixelX(const double &yaw);
  int getPixelY(const double &pitch);
private:
  std::vector<std::vector<std::vector<double>>> image_data;
  pcl::PointCloud<PointT>::Ptr cloud_data;
  double fov_up, fov_down, image_width, image_height;
};

#endif // PC_OBJ_DETECT__RANGE_IMAGE__H_
