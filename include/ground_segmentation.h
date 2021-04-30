#ifndef PC_OBJ_DETECT__GROUND_SEGMENTATION__H_
#define PC_OBJ_DETECT__GROUND_SEGMENTATION__H_

#include <range_image.h>

class GroundSegmentation {
public:
  GroundSegmentation(RangeImage range_image_input); 
private:
  RangeImage range_image;
  pcl::PointIndices::Ptr ground_points;
};

#endif // PC_OBJ_DETECT__GROUND_SEGMENTATION__H_