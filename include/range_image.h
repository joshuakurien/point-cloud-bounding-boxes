#ifndef PC_OBJ_DETECT__RANGE_IMAGE__H_
#define PC_OBJ_DETECT__RANGE_IMAGE__H_

#include <vector>
#include <pcl/common/common_headers.h>
#include <opencv2/opencv.hpp>

class RangePixelData {
public:
  int point_cloud_index;
  double vertical_angle;
};

using PointT = pcl::PointXYZI;

class RangeImage {
public:
  RangeImage(pcl::PointCloud<PointT>::Ptr cloud_input, 
           double hor_res_input, double ver_res_input, 
           double image_width_input, double image_height_input);
  void rangeImageConversion();
  void angleImageConversion();
  void smoothenAngleImage();
  void displayImage(bool display_range_image);
  int getRangeImageSize(bool is_num_rows);  
  int getAngleImageSize(bool is_num_rows);  
  double getRangeImageValue(int row, int col);  
  double getAngleImageValue(int row, int col);  
  int getImageIndexValue(int row, int col);
  void setAngleImageValue(int row, int col, double val);
  pcl::PointCloud<PointT>::Ptr getCloudData();
private:
  int rangePixelX(const double &yaw);
  int rangePixelY(const double &pitch);
  cv::Mat range_image;
  cv::Mat angle_image;
  std::vector<std::vector<RangePixelData>> range_image_pixel_data;
  pcl::PointCloud<PointT>::Ptr cloud_data;
  double fov_up, fov_down, image_width, image_height;
  double maxEuclideanDistance = 0, maxAlpha = 0;
};

#endif // PC_OBJ_DETECT__RANGE_IMAGE__H_
