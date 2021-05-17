#include <ground_segmentation.h>

// Constructor dynamically allocates memory for bins and ground points
GroundSegmentation::GroundSegmentation(pcl::PointCloud<PointT>::Ptr cloud) {
  point_bins = std::make_shared<BinContainer>();
  ground_points = std::make_shared<std::vector<PointT>>();
  createPointBins(cloud);
  determineGroundPoints();
}

// Organize points into bins along the same 'line' for main filtering algo
void GroundSegmentation::createPointBins(pcl::PointCloud<PointT>::Ptr cloud) {
  // Values based on HDL64 resolution - might be a free parameter for tuning later
  const double kMaxBinNum = 360.0/kAzimuthResolutionDeg;
  const double kAzimuthResolutionRad = kAzimuthResolutionDeg*M_PI/180;
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
  // Loop also used to determine the lowest point in the data in terms of z value
  // which is used as the initial ground point for further calculations
  while (cur_point != cloud->end()) {
    if (azimuth(*cur_point) >= bin_beginning && azimuth(*cur_point) < bin_ending) {
      cur_bin.push_back(*cur_point);
      cur_point++;
    } else {
      bin_beginning += kAzimuthResolutionRad;
      bin_ending += kAzimuthResolutionRad;
      point_bins->push_back(cur_bin);
      Bin next_bin;
      cur_bin = next_bin;
    }
  }
  // Following line needed to push the last_bin in
  point_bins->push_back(cur_bin);

  // Sorts each bin by flat distance (x and y components not considering z
  // to be able to compare points 
  // along the same line for core ground filtering algo 
  for (auto& bin : *point_bins) {
    std::sort(bin.begin(), bin.end(),
    [](const PointT& p1, const PointT& p2) {
      return flatDistance(p1) < flatDistance(p2); 
    });
  }
}

// Returns a vector with point clouds representing the separate bins/lines used for filterings
std::vector<pcl::PointCloud<PointT>::Ptr> GroundSegmentation::getPointBins() {
  std::vector<pcl::PointCloud<PointT>::Ptr> bin_clouds; 
  bin_clouds.resize(point_bins->size());
  for (int i = 0; i < point_bins->size(); i++) {
    auto& bin = (*point_bins)[i]; 
    pcl::PointCloud<PointT>::Ptr cur_bin_cloud (new pcl::PointCloud<PointT>);
    cur_bin_cloud->insert(cur_bin_cloud->end(), bin.begin(), bin.end());
    bin_clouds[i] = cur_bin_cloud;
  }
  return bin_clouds;
}

void GroundSegmentation::determineGroundPoints() {
  for (auto& bin : *point_bins) {
    std::cout << bin[0].z << std::endl;
    segmentBinGroundPoints(bin);
  }
}

void GroundSegmentation::segmentBinGroundPoints(const Bin& bin) {
  // Vector used to keep track of ground or non-ground point labels
  // 1 is ground, 0 is non-ground
  std::vector<ground_label> labels;
  labels.resize(bin.size());
  // boolean used to keep track of what we are labelling
  bool is_labelling_ground = true;
  // this initial previous point is the ground point
  PointT prev_point(0, 0, sensor_height);  
  
  if (bin.size() == 1) {
    PointT cur_point = bin[0];
    bool compare = compareConsecutivePoints(prev_point, cur_point, is_labelling_ground);
    if (compare) {
      ground_points->push_back(cur_point);
    }
  } else {
    // initial pass for threshold points/new ground points identification
    for (int i = 0; i < bin.size(); i++) {    
      PointT cur_point = bin[i];  
      bool compare = compareConsecutivePoints(prev_point, cur_point, is_labelling_ground);

      if (is_labelling_ground && compare) {
        labels[i] = ground_point;
      } else if (is_labelling_ground && !compare) {
        // note threshold point is considered to be the first point that is considered to be a ground point
        labels[i] = threshold_point;
        is_labelling_ground = false;
      } else if (!is_labelling_ground && compare) {
        labels[i] = non_ground_point;
      } else {
        labels[i] = initial_ground_point;
        is_labelling_ground = true;
      }
      prev_point = cur_point;
    }

    for (int i = 0; i < labels.size(); i++) {
      if (labels[i] == initial_ground_point || labels[i] == ground_point) {
        ground_points->push_back(bin[i]);
      }
    }
  }
}

// true means that it matches whatever the is_labelling_ground value is set to
bool GroundSegmentation::compareConsecutivePoints(const PointT & prev, const PointT & cur, bool is_labelling_ground) {
  if (is_labelling_ground) {
    double height_diff = abs(prev.z - cur.z);
    double distance_diff = euclideanDistanceDifference(prev, cur);
    double gradient = abs(asin(height_diff/distance_diff));
    // Gradient value based on case 1 described
    bool grad_check = gradient <= kMaxAngleRad;
    // // Height check based on case 2 described
    bool height_check = height_diff <= kMinHeight;
    // // Distance check based on case 3 described
    // bool dist_check = euclideanDistance(prev_point) < euclideanDistance(cur_point);
    // return grad_check && height_check && dist_check;
    return grad_check && height_check;

  } else {
    double height_diff = abs(prev.z - cur.z);
    double distance_diff = euclideanDistanceDifference(prev, cur);
    double gradient = abs(asin(height_diff/distance_diff));
    bool grad_check = gradient < kMaxAngleRad;
    return grad_check;
  }

}


std::shared_ptr<std::vector<PointT>> GroundSegmentation::getGroundPoints() {
  return ground_points;
}

double azimuth (PointT pt) {
  return atan2(pt.y, pt.x);
}

double flatDistance (PointT pt) {
  sqrt(pow(pt.x, 2) + pow(pt.y, 2));
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