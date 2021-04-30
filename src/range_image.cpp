#include <range_image.h>

RangeImage::RangeImage (pcl::PointCloud<PointT>::Ptr cloud_input, 
           double fov_up_input, double fov_down_input, 
           double image_width_input, double image_height_input) :
           cloud_data{cloud_input}, fov_up{std::abs(fov_up_input/180*M_PI)}, fov_down{std::abs(fov_down_input/180*M_PI)}, 
           image_width{image_width_input}, image_height{image_height_input}
{
  range_image = cv::Mat(image_height, image_width, CV_64FC1);
  range_image_pixel_data.assign(image_height, std::vector<RangePixelData>(image_width, RangePixelData()));
  convertToImage();
  getAngleImage();
  smoothenAngleImage();
}

void RangeImage::getAngleImage() {
  angle_image = cv::Mat(image_height - 1, image_width, CV_64FC1);
  for (int i = 0; i < image_height - 1; ++i) {
    for (int j = 0; j < image_width; ++j) {
      double range_upper = range_image.at<double>(i, j);
      double range_lower = range_image.at<double>(i+1, j);
      double vertical_angle_upper = range_image_pixel_data.at(i).at(j).vertical_angle;
      double vertical_angle_lower = range_image_pixel_data.at(i+1).at(j).vertical_angle;
      double delta_z = std::abs(range_lower*sin(vertical_angle_lower) - range_upper*sin(vertical_angle_upper));
      double delta_x = std::abs(range_lower*cos(vertical_angle_lower) - range_upper*cos(vertical_angle_upper));
      double alpha = atan2(delta_z, delta_x);
      if (alpha > maxAlpha) {
        maxAlpha = alpha;
      }
      angle_image.at<double>(i, j) = alpha;
    }
  }
}

void RangeImage::smoothenAngleImage() {
  int window_size = 11;
  if (window_size % 2 == 0) {
    throw std::logic_error("only odd window size allowed");
  }
  bool window_size_ok = window_size == 5 || window_size == 7 ||
                        window_size == 9 || window_size == 11;
  if (!window_size_ok) {
    throw std::logic_error("bad window size");
  }
  // below are no magic constants. See Savitsky-golay filter.
  cv::Mat kernel;
  switch (window_size) {
    case 5:
      kernel = cv::Mat::zeros(window_size, 1, CV_32F);
      kernel.at<float>(0, 0) = -3.0f;
      kernel.at<float>(0, 1) = 12.0f;
      kernel.at<float>(0, 2) = 17.0f;
      kernel.at<float>(0, 3) = 12.0f;
      kernel.at<float>(0, 4) = -3.0f;
      kernel /= 35.0f;
    case 7:
      kernel = cv::Mat::zeros(window_size, 1, CV_32F);
      kernel.at<float>(0, 0) = -2.0f;
      kernel.at<float>(0, 1) = 3.0f;
      kernel.at<float>(0, 2) = 6.0f;
      kernel.at<float>(0, 3) = 7.0f;
      kernel.at<float>(0, 4) = 6.0f;
      kernel.at<float>(0, 5) = 3.0f;
      kernel.at<float>(0, 6) = -2.0f;
      kernel /= 21.0f;
    case 9:
      kernel = cv::Mat::zeros(window_size, 1, CV_32F);
      kernel.at<float>(0, 0) = -21.0f;
      kernel.at<float>(0, 1) = 14.0f;
      kernel.at<float>(0, 2) = 39.0f;
      kernel.at<float>(0, 3) = 54.0f;
      kernel.at<float>(0, 4) = 59.0f;
      kernel.at<float>(0, 5) = 54.0f;
      kernel.at<float>(0, 6) = 39.0f;
      kernel.at<float>(0, 7) = 14.0f;
      kernel.at<float>(0, 8) = -21.0f;
      kernel /= 231.0f;
    case 11:
      kernel = cv::Mat::zeros(window_size, 1, CV_32F);
      kernel.at<float>(0, 0) = -36.0f;
      kernel.at<float>(0, 1) = 9.0f;
      kernel.at<float>(0, 2) = 44.0f;
      kernel.at<float>(0, 3) = 69.0f;
      kernel.at<float>(0, 4) = 84.0f;
      kernel.at<float>(0, 5) = 89.0f;
      kernel.at<float>(0, 6) = 84.0f;
      kernel.at<float>(0, 7) = 69.0f;
      kernel.at<float>(0, 8) = 44.0f;
      kernel.at<float>(0, 9) = 9.0f;
      kernel.at<float>(0, 10) = -36.0f;
      kernel /= 429.0f;
  }

  const cv::Point ANCHOR_CENTER = cv::Point(-1, -1);
  const int SAME_OUTPUT_TYPE = -1;
  cv::filter2D(angle_image, angle_image, SAME_OUTPUT_TYPE, kernel, ANCHOR_CENTER,
               0, cv::BORDER_REFLECT101);
}

void RangeImage::displayImage(bool is_angle) {
  if (!is_angle) {
    cv::imshow("Range Image", range_image/maxEuclideanDistance);
  } else {
    cv::imshow("Angle Image", angle_image/maxAlpha);
  }
  cv::waitKey(0);
}

void RangeImage::convertToImage() {
  for (auto &point : *cloud_data) {
    double euc_distance = sqrt(pow(point.x,2) + pow(point.y,2) + pow(point.z,2));
    if (euc_distance > maxEuclideanDistance) {
      maxEuclideanDistance = euc_distance;
    }
    auto yaw = atan2(point.y, point.x);
    auto pitch = asin(point.z / euc_distance);
    int pixel_x = getPixelX(yaw);
    int pixel_y = getPixelY(pitch);
    // Pitch stored for angle calculation
    range_image.at<double>(pixel_y, pixel_x) = euc_distance;
    auto& pixel_data = range_image_pixel_data.at(pixel_y).at(pixel_x);
    pixel_data.point_cloud_index = 1;
    pixel_data.vertical_angle = pitch;
  }
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