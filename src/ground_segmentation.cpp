#include <ground_segmentation.h>

GroundSegmentation::GroundSegmentation(RangeImage range_image_input)
    : range_image{range_image_input} {
  ground_points = pcl::PointIndices::Ptr(new pcl::PointIndices());
  range_image.rangeImageConversion();
  range_image.angleImageConversion();
  graph_height = range_image.getAngleImageSize(true);
  graph_width = range_image.getAngleImageSize(false);
  // ground truth for which points are ground points
  segmentation_graph.assign(graph_height, std::vector<SegmentationNode>(
                                              graph_width, SegmentationNode()));
}

void GroundSegmentation::performSegmentation() {;
  range_image.smoothenAngleImage();
  range_image.displayImage(false);
  labelLowestRow();
  labelGroundPoints();
  extractGroundIndices();
  range_image.displayImage(false);
}

void GroundSegmentation::labelLowestRow() {
  double kAngleThreshold = 45.0 / 180.0 * M_PI;
  for (int j = 0; j < graph_width; j++) {
    if (range_image.getAngleImageValue(graph_height - 1, j) < kAngleThreshold) {
      initial_ground_points.push_back(j);
    }
  }
}

void GroundSegmentation::labelGroundPoints() {
  for (int ground_col : initial_ground_points) {
    if (!segmentation_graph.at(graph_height - 1).at(ground_col).is_visited) {
      labelGroundPointsBFS(graph_height - 1, ground_col);
    }
  }
}

void GroundSegmentation::labelGroundPointsBFS(int row, int col) {
  // initialization of node for the lowest point of the column
  SegmentationNode bottom_row_node;
  bottom_row_node.row = row;
  bottom_row_node.col = col;
  bottom_row_node.is_visited = true;
  std::queue<SegmentationNode> bfs_queue;
  bfs_queue.push(bottom_row_node);

  double kAngleThreshold = 5.0 / 180.0 * M_PI;
  while (!bfs_queue.empty()) {
    SegmentationNode cur_node = bfs_queue.front();
    // node is considered 'visited' and a ground point after its angle value has already been compared
    segmentation_graph.at(cur_node.row).at(cur_node.col).is_ground = true;

    std::vector<SegmentationNode> neighbours = findAvailableNeighbours(cur_node.row, cur_node.col);
    for (auto neighbour : neighbours) {
      double neighbour_angle = range_image.getAngleImageValue(neighbour.row, neighbour.col);
      double current_node_angle = range_image.getAngleImageValue(cur_node.row, cur_node.col);
      if (abs(neighbour_angle - current_node_angle) < kAngleThreshold) {
        bfs_queue.push(neighbour);
      }
      segmentation_graph.at(neighbour.row).at(neighbour.col).is_visited = true;
    }
    bfs_queue.pop();
  }
}

// Validity checked by bounds checking and if node already visited
std::vector<SegmentationNode> GroundSegmentation::findAvailableNeighbours(int row,
                                                                      int col) {
  std::vector<SegmentationNode> neighbours;
  if (row - 1 >= 0) {
    SegmentationNode upper_neighbour;
    upper_neighbour.row = row - 1;
    upper_neighbour.col = col;
    if (!segmentation_graph.at(upper_neighbour.row).at(upper_neighbour.col).is_visited) {
      neighbours.push_back(upper_neighbour);
    }
  }

  if (row + 1 <= graph_height - 1) {
    SegmentationNode lower_neighbour;
    lower_neighbour.row = row + 1;
    lower_neighbour.col = col;
    if (!segmentation_graph.at(lower_neighbour.row).at(lower_neighbour.col).is_visited) {
      neighbours.push_back(lower_neighbour);
    }
  }

  if (col - 1 >= 0) {
    SegmentationNode left_neighbour;
    left_neighbour.row = row;
    left_neighbour.col = col - 1;
    if (!segmentation_graph.at(left_neighbour.row).at(left_neighbour.col).is_visited) {
      neighbours.push_back(left_neighbour);
    }
  }

  if (col + 1 <= graph_width - 1) {
    SegmentationNode right_neighbour;
    right_neighbour.row = row;
    right_neighbour.col = col + 1;
    if (!segmentation_graph.at(right_neighbour.row).at(right_neighbour.col).is_visited) {
      neighbours.push_back(right_neighbour);
    }
  }

  return neighbours;
}

pcl::PointIndices::Ptr GroundSegmentation::getGroundIndices() {
  return ground_points;
}

void GroundSegmentation::extractGroundIndices() {
  for (int i = 0; i < graph_height; i++) {
    for (int j = 0; j < graph_width; j++) {
      if (segmentation_graph.at(i).at(j).is_ground) {
        int index = range_image.getImageIndexValue(i + 1, j);
        // range_image.setAngleImageValue(i, j, 1.0);
        ground_points->indices.push_back(i);
      }
    }
  }
}