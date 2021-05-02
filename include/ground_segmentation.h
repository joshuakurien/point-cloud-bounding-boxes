#ifndef PC_OBJ_DETECT__GROUND_SEGMENTATION__H_
#define PC_OBJ_DETECT__GROUND_SEGMENTATION__H_

#include <queue>
#include <algorithm>
#include <set>
#include <memory>
#include <range_image.h>

class SegmentationNode {
public:
  bool is_visited = false;
  bool is_ground = false;
  int row = 0;
  int col = 0;
};
class GroundSegmentation {
public:
private:
};

#endif // PC_OBJ_DETECT__GROUND_SEGMENTATION__H_