#include "PointCloud.hpp"

PointCloud::PointCloud(double maxSensorRange) :
      maxSensorRange2_(maxSensorRange * maxSensorRange) {
}

PointCloud::PointCloud(double subsampleRadius, double maxSensorRange, double minSensorRange, bool kitti) :
      subsampleRadius2_(subsampleRadius * subsampleRadius),
      maxSensorRange2_(maxSensorRange * maxSensorRange),
      minSensorRange2_(minSensorRange * minSensorRange),
      kitti_(kitti) {
}

const std::vector<Eigen::Vector4d>& PointCloud::getPtCloud() const {
    return ptCloud_;
}
        
const NanoflannPointsContainer<double>& PointCloud::getPcForKdTree() const {
    return pcForKdTree_;
}

void PointCloud::addPoint(double x, double y, double z) {
    double normSquared = pow(x, 2) + pow(y, 2) + pow(z, 2);
    if ((normSquared > minSensorRange2_) && (normSquared < maxSensorRange2_)) {
      allPoints_.insert(ptCloud_.size());
      ptCloud_.push_back({x, y, z, 1});
    }
}

void PointCloud::readScan(std::string fileName) {
    ptCloud_.clear();
    allPoints_.clear();
    std::ifstream file(fileName, std::ios::in | std::ios::binary);

    float item;
    std::vector<double> ptsFromFile;
    int counter = 0;
    
    while (file.read((char*)&item, sizeof(item))) {
        ptsFromFile.push_back(item);
    }

    unsigned int numPts = ptsFromFile.size() / NUM_COLUMNS_BIN; // .bin format
 
    for (unsigned int i = 0; i < ptsFromFile.size(); i+=NUM_COLUMNS_BIN) {
        // Save the pt if it is within the maximum and mininmum sensor ranges.
        double normSquared = pow(ptsFromFile[i], 2) + pow(ptsFromFile[i+1], 2) + pow(ptsFromFile[i+2], 2);
        if ((normSquared > minSensorRange2_) && (normSquared < maxSensorRange2_)) {
            ptCloud_.push_back({ptsFromFile[i], ptsFromFile[i+1], ptsFromFile[i+2], 1});
            
            // Save the pt index for subsampling.
            allPoints_.insert(counter); 
            counter++;
        }
    }

    // Process the PointCloud.
    processPointCloud();
}

void PointCloud::processPointCloud() {
    // Check if the PointCloud needs to be corrected.
    // Apply the calibration factor as explained in IMLS-SLAM, CT-ICP, and KISS-ICP.
    if (kitti_) {
        correctKittiScan_();
    }

    // Subsample the point cloud.
    subsample_(ptCloud_, subsampleRadius2_);
}

void PointCloud::correctKittiScan_() {
    // Adapted from KISS-ICP opensource code to correct the KITTI scans.
    constexpr double VERTICAL_ANGLE_OFFSET = (0.205 * M_PI) / 180.0;
    
    tbb::parallel_for(
    tbb::blocked_range<int>(0, ptCloud_.size()),
    [&](tbb::blocked_range<int> r) {
        for (unsigned int i = r.begin(); i < r.end(); i++) {
            Eigen::Vector3d pt;
            Eigen::Vector3d ptCorrected;
            pt << ptCloud_[i](0), ptCloud_[i][1], ptCloud_[i][2];
            const Eigen::Vector3d rotationVector = pt.cross(Eigen::Vector3d(0.0, 0.0, 1.0));
            ptCorrected = Eigen::AngleAxisd(VERTICAL_ANGLE_OFFSET, rotationVector.normalized()) * pt;
            ptCloud_[i] = {ptCorrected(0), ptCorrected(1), ptCorrected(2), 1};
        }
    });
}

void PointCloud::subsample_(std::vector<Eigen::Vector4d> &pts, double subsampleRadius2) {
    std::vector<Eigen::Vector4d> ptsSubsampled;
    
    // Nanoflann uses the squared radius.
    convertToPointCloudKdTree_(pts);

    // Create a Kd tree (dimension, PointCloud, max leaf).
    my_kd_tree_t *scanKdTree = new my_kd_tree_t(3, pcForKdTree_,{10});
    unsigned int counter = 0;

    // Subsample radially.
    for (unsigned int i : allPoints_) {
        std::vector<nanoflann::ResultItem<uint32_t, double>> ret_matches;
        const double query_pt[3] = {pts[i][0], pts[i][1], pts[i][2]};
        const size_t nMatches = scanKdTree->radiusSearch(&query_pt[0], subsampleRadius2, ret_matches);
        for (unsigned int j = 0; j < nMatches; j++) {
            if (i != ret_matches[j].first) {
                allPoints_.erase(ret_matches[j].first);
            }
        }
        ptsSubsampled.push_back({pts[i][0], pts[i][1], pts[i][2], 1});
    }
    delete scanKdTree; // free memory

    // Overwrite the PointCloud with the subsampled points.
    pts = ptsSubsampled;
}

void PointCloud::convertToPointCloudKdTree_(std::vector<Eigen::Vector4d> &pts) {
    size_t pcLength = pts.size();
    pcForKdTree_.pts.clear();
    pcForKdTree_.pts.resize(pcLength);

    tbb::parallel_for(
    tbb::blocked_range<int>(0, pcLength),
    [&](tbb::blocked_range<int> r) { 
        for (size_t i = r.begin(); i < r.end(); i++) {
            pcForKdTree_.pts[i].x = pts[i](0);
            pcForKdTree_.pts[i].y = pts[i](1);
            pcForKdTree_.pts[i].z = pts[i](2);        
        } 
    });
}
