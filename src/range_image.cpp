#include <opencv2/opencv.hpp>
#include <range_image.h>

RangeImage::RangeImage (pcl::PointCloud<PointT>::Ptr cloud_input, 
           float hor_res_input, float ver_res_input, 
           float image_width_input, float image_height_input) :
           cloud_data{cloud_input}, max_hor_resolution{hor_res_input}, max_ver_resolution{ver_res_input}, 
           image_width{image_width_input}, image_height{image_height_input}
{
  image_data.assign(64, std::vector<std::vector<double>>(1024, std::vector<double>(5, 0.0)));
  convertToImage();
}

void RangeImage::convertToImage() {
  double fov_up_rad = (2 / 180) * M_PI;
  double fov_down_rad = (24.8 / 180) * M_PI;
  double fov_rad = std::abs(fov_up_rad) + std::abs(fov_down_rad);
  
  for (auto &point : *cloud_data) {
    int pixel_v = 0;
    int pixel_u = 0;
    double range = 0.0;
    range = sqrt(point.x* point.x + point.y * point.y + point.z * point.z);
    //  Getting the angle of all the Points
    auto yaw = atan2(point.y, point.x);
    auto pitch = asin(point.z / range);
    // Get projections in image coords and normalizing
    double v = 0.5 * (yaw / M_PI + 1.0);
    double u = 1.0 - (pitch + std::abs(fov_down_rad)) / fov_rad;
    // Scaling as per the lidar config given
    v *= 1024;
    u *= 64;
    // round and clamp for use as index
    v = floor(v);
    v = std::min(1024.0-1, v);
    v = std::max(0.0, v);
    pixel_v = int(v);

    u = floor(u);
    u = std::min(64.0-1, u);
    u = std::max(0.0, u);
    pixel_u = int(u);
    image_data.at(pixel_u).at(pixel_v) = std::vector<double>{point.x, point.y, point.z, range, point.intensity};
  }
}

void RangeImage::displayImage() {
  cv::Mat display_image(image_data.size(), image_data.at(0).size(), CV_64FC1);
  for (int i = 0; i < display_image.rows; ++i) {
      for (int j = 0; j < display_image.cols; ++j) {
          display_image.at<double>(i, j) = image_data.at(i).at(j).at(4);  // Intensity value
      }
  }
  cv::imshow("Intensity Image", display_image);
  cv::waitKey(0);
}