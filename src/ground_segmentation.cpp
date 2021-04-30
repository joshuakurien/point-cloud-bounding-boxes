#include <ground_segmentation.h>

GroundSegmentation::GroundSegmentation(RangeImage range_image_input) : 
  range_image{range_image_input} {
  range_image.rangeImageConversion();
  range_image.angleImageConversion();
  range_image.smoothenAngleImage();
  range_image.displayImage(false);
  ground_points = pcl::PointIndices::Ptr(new pcl::PointIndices());
}
