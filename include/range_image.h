#ifndef PC_OBJ_DETECT__RANGE_IMAGE__H_
#define PC_OBJ_DETECT__RANGE_IMAGE__H_

#include <vector>
#include <pcl/common/common_headers.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

struct RangePixelData {
  int point_cloud_index;
  double vertical_angle;
};

using PointT = pcl::PointXYZI;

class RangeImage {
public:
  RangeImage(pcl::PointCloud<PointT>::Ptr cloud_input, 
           double hor_res_input, double ver_res_input, 
           double image_width_input, double image_height_input);
  void getAngleImage();
  void smoothenAngleImage();
  void displayImage(bool is_angle);
protected:
  void convertToImage();
  int getPixelX(const double &yaw);
  int getPixelY(const double &pitch);
private:
  cv::Mat range_image;
  cv::Mat angle_image;
  std::vector<std::vector<RangePixelData>> range_image_pixel_data;
  pcl::PointCloud<PointT>::Ptr cloud_data;
  double fov_up, fov_down, image_width, image_height;
  double maxEuclideanDistance = 0, maxAlpha = 0;
};

#endif // PC_OBJ_DETECT__RANGE_IMAGE__H_
