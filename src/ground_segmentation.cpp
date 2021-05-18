#include <ground_segmentation.h>

// Constructor creates point bins and runs main segmentation algorithm
GroundSegmentation::GroundSegmentation(pcl::PointCloud<PointT>::Ptr cloud) {
  createPointBins(cloud);
  segmentGround();
}

// Organize points into bins along the same 'line' for main filtering algo
void GroundSegmentation::createPointBins(pcl::PointCloud<PointT>::Ptr cloud) {
  // Sorts points based on azimuth angles to easily place into appropriate bins
  std::sort(cloud->points.begin(), cloud->points.end(),
  [](const PointT& p1, const PointT& p2) {
    return azimuth(p1) < azimuth(p2); 
  });

  double bin_beginning = -1*M_PI;
  double bin_ending = bin_beginning + kAzimuthResolutionRad;
  auto cur_point = cloud->begin();
  Bin cur_bin;

  // Main loop for placing each point into a bin  
  while (cur_point != cloud->end()) {
    if (azimuth(*cur_point) >= bin_beginning && azimuth(*cur_point) < bin_ending) {
      cur_bin.push_back(*cur_point);
      cur_point++;
    } else {
      bin_beginning += kAzimuthResolutionRad;
      bin_ending += kAzimuthResolutionRad;
      point_bins.push_back(cur_bin);
      Bin next_bin;
      cur_bin = next_bin;
    }
  }
  point_bins.push_back(cur_bin);

  // Sorts each bin by flat distance (x and y components not considering z)
  for (auto& bin : point_bins) {
    std::sort(bin.begin(), bin.end(),
    [](const PointT& p1, const PointT& p2) {
      return flatDistance(p1) < flatDistance(p2); 
    });
  }
}

// Returns a vector with point clouds representing the separate bins/lines used for filterings
std::vector<pcl::PointCloud<PointT>::Ptr> GroundSegmentation::getPointBins() {
  std::vector<pcl::PointCloud<PointT>::Ptr> bin_clouds; 
  bin_clouds.resize(point_bins.size());
  for (int i = 0; i < point_bins.size(); i++) {
    auto& bin = point_bins[i]; 
    pcl::PointCloud<PointT>::Ptr cur_bin_cloud (new pcl::PointCloud<PointT>);
    cur_bin_cloud->insert(cur_bin_cloud->end(), bin.begin(), bin.end());
    bin_clouds[i] = cur_bin_cloud;
  }
  return bin_clouds;
}

// Main algorithm for segmenting bins into non-ground and ground points
void GroundSegmentation::segmentGround() {
  for (auto& bin : point_bins) {
    if (bin.size() == 1) {
      PointT prev_point(0, 0, kSensorHeight); 
      PointT cur_point = bin[0];
      if (thresholdCheck(prev_point, cur_point)) {
        non_ground_points.push_back(cur_point);
      } else {
        ground_points.push_back(cur_point);
      }
    } else {
      std::vector<bool> labels = labelBinPoints(bin);
      separateBinPoints(labels, bin);
    }
  }
}

// Determines ground and non-ground points within a bin
std::vector<bool> GroundSegmentation::labelBinPoints(const Bin& bin) {
  // boolean used to keep track of what we are labelling
  // true means we currently label ground points and check for next non-ground point
  // false means we currently label non-ground points and check for next ground point
  bool is_labelling_ground = true;
  PointT prev_point(0, 0, kSensorHeight);
  PointT last_ground_point;
  // Vector used to keep track of ground or non-ground point labels
  // true used to represent ground points, false for non-ground points
  std::vector<bool> labels;
  labels.resize(bin.size());
  for (int i = 0; i < bin.size(); i++) {
    PointT cur_point = bin[i];
    if (is_labelling_ground) {
      if (thresholdCheck(prev_point, cur_point)) {
        labels[i] = false;
        is_labelling_ground = false;
        last_ground_point = prev_point;
      } else {
        labels[i] = true;
      }
    } else {
      if (newGroundCheck(prev_point, cur_point, last_ground_point)) {
        labels[i] = true;
        is_labelling_ground = true;
      } else {
        labels[i] = false;
      }
    }
    prev_point = cur_point;
  }
  return labels;
}

// Returns true if the current point is a threshold point
bool GroundSegmentation::thresholdCheck(const PointT & prev, const PointT & cur) {
  bool gradient_check = gradient(prev, cur) <= kMaxAngleRad;
  bool euclidean_distance_check = euclideanDistance(prev) < euclideanDistance(cur);
  return !(gradient_check) || !(euclidean_distance_check);
}

// Returns true if current points is a new ground point
bool GroundSegmentation::newGroundCheck(const PointT & prev, const PointT & cur, const PointT & last_ground_point) {
  bool lower_point_check = cur.z < prev.z;
  bool height_check = abs(last_ground_point.z - cur.z) <= kMinHeight; 
  return lower_point_check && height_check;
}

// Used to place labelled bin points into ground and non-ground containers
void GroundSegmentation::separateBinPoints(const std::vector<bool>& labels, const Bin& bin) {
  for (int i = 0; i < labels.size(); i++) {
    // true label signifies ground point
    if (labels[i]) {
      ground_points.push_back(bin[i]);
    } else {
      non_ground_points.push_back(bin[i]);
    }
  }
}

std::vector<PointT> GroundSegmentation::getGroundPoints() {
  return ground_points;
}

std::vector<PointT> GroundSegmentation::getNonGroundPoints() {
  return non_ground_points;
}


double euclideanDistance (PointT pt) {
  sqrt(pow(pt.x, 2) + pow(pt.y, 2) + pow(pt.z, 2));
}

double euclideanDistanceDifference (PointT pt1, PointT pt2) {
  double x_difference = pt1.x - pt2.x;
  double y_difference = pt1.y - pt2.y;
  double z_difference = pt1.z - pt2.z;
  return sqrt(pow(x_difference, 2) + pow(y_difference, 2) + pow(z_difference, 2));
}

double gradient(const PointT & pt1, const PointT & pt2) {
  double height_diff = abs(pt1.z - pt2.z);
  double distance_diff = euclideanDistanceDifference(pt1, pt2);
  return abs(asin(height_diff/distance_diff));
}

double azimuth (PointT pt) {
  return atan2(pt.y, pt.x);
}

double flatDistance (PointT pt) {
  sqrt(pow(pt.x, 2) + pow(pt.y, 2));
}