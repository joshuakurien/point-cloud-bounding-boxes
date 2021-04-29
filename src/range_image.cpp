#include <opencv2/opencv.hpp>
#include <range_image.h>

RangeImage::RangeImage (pcl::PointCloud<PointT>::Ptr cloud_input, 
           double fov_up_input, double fov_down_input, 
           double image_width_input, double image_height_input) :
           cloud_data{cloud_input}, fov_up{std::abs(fov_up_input/180*M_PI)}, fov_down{std::abs(fov_down_input/180*M_PI)}, 
           image_width{image_width_input}, image_height{image_height_input}
{
  image_data.assign(image_height, std::vector<std::vector<double>>(image_width, std::vector<double>(5, 0.0)));
  convertToImage();
}

void RangeImage::convertToImage() {
  for (auto &point : *cloud_data) {
    double euc_distance = sqrt(pow(point.x,2) + pow(point.y,2) + pow(point.z,2));
    auto yaw = atan2(point.y, point.x);
    auto pitch = asin(point.z / euc_distance);
    int pixel_x = getPixelX(yaw);
    int pixel_y = getPixelY(pitch);
    image_data.at(pixel_y).at(pixel_x) = std::vector<double>{point.x, point.y, point.z, euc_distance, point.intensity};
  }
}

void RangeImage::displayImage() {
  cv::Mat display_image(image_data.size(), image_data.at(0).size(), CV_64FC1);
  for (int i = 0; i < display_image.rows; ++i) {
      for (int j = 0; j < display_image.cols; ++j) {
          display_image.at<double>(i, j) = image_data.at(i).at(j).at(4);  // Intensity value
      }
  }
  cv::imshow("Range Image", display_image);
  cv::waitKey(0);
}

int RangeImage::getPixelX(const double &yaw) {
  double x = image_width*(0.5 * (yaw / M_PI + 1.0));
  x = floor(x);
  x = std::min(image_width-1, x);
  x = std::max(0.0, x);
  return static_cast<int>(x);
}

int RangeImage::getPixelY(const double &pitch) {
  double y = image_height*(1.0 - (pitch + std::abs(fov_down)) / (fov_down + fov_up));
  y = floor(y);
  y = std::min(image_height-1, y);
  y = std::max(0.0, y);
  return static_cast<int>(y);
}