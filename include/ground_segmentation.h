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
  GroundSegmentation(std::shared_ptr<RangeImage> range_image_input); 
  void performSegmentation();
  std::set<int> getGroundIndices();
private:
  void labelLowestRow();
  void labelGroundPoints();
  void labelGroundPointsBFS(int row, int col);
  void extractGroundIndices();
  std::vector<SegmentationNode> findAvailableNeighbours(int row, int col);
  int graph_height, graph_width;
  std::vector<int> initial_ground_points;
  std::shared_ptr<RangeImage> range_image;
  std::set<int> ground_indices;
  // ground truth graph for ground and non-ground points
  std::vector<std::vector<SegmentationNode>> segmentation_graph;
};

#endif // PC_OBJ_DETECT__GROUND_SEGMENTATION__H_