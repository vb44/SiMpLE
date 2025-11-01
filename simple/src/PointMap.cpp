#include "PointMap.hpp"

PointMap::PointMap(double voxelSize, double maxSensorRange)
    : PointCloud(maxSensorRange) {
    voxelSize_ = voxelSize;
}

void PointMap::updateMap(const std::vector<Eigen::Vector4d> &pts, const Eigen::Matrix4d &pose) {
    // Add transformed scan to the existing map.
    for (auto &pt : pts) {
        Eigen::Vector4d ptTf = pose * pt;
        ptCloud_.push_back(ptTf);
    }

    // Subsample and save the new map.
    std::vector<Eigen::Vector4d> tempMap;
    subsample_(ptCloud_, voxelSize_);
    tempMap = ptCloud_;

    // Remove points outside rMax to limit the size of the map.
    ptCloud_.clear();
    for (unsigned int k = 0; k < tempMap.size(); k++) {
        if ((pow(tempMap[k](0) - pose(0,3),2) + 
             pow(tempMap[k](1) - pose(1,3),2) + 
             pow(tempMap[k](2) - pose(2,3),2)) < maxSensorRange2_) {
            ptCloud_.push_back(tempMap[k]);
        }
    }

    // Update the Kd-tree point cloud.
    convertToPointCloudKdTree_(ptCloud_);
}