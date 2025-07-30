#include "PointMap.hpp"

PointMap::PointMap(double mapSubsampleRadius, double maxSensorRange)
    : PointCloud(maxSensorRange) {
    
    subsampleRadius_ = mapSubsampleRadius;
    subsampleRadius2_ = mapSubsampleRadius * mapSubsampleRadius;

    // Compute the dimension of point cloud (pc) grid.
    dim_ = std::round(maxSensorRange/mapSubsampleRadius) * 2 + 1;
    offset_ = std::round(maxSensorRange/mapSubsampleRadius);

    gridOccupied_.resize(dim_, std::vector<std::vector<bool> >(dim_, std::vector<bool>(dim_, false)));
}

void PointMap::updateMap(const std::vector<Eigen::Vector4d> &pts, const Eigen::Matrix4d &pose) {
    // Add transformed scan to the existing map.
    for (auto &pt : pts) {
        Eigen::Vector4d ptTf = pose * pt;
        ptCloud_.push_back(ptTf);
    }

    // Subsample and save the new map.
    std::vector<Eigen::Vector4d> tempMap;
    for (auto& plane : gridOccupied_)
    {
        for (auto& row : plane)
        {
            std::fill(row.begin(), row.end(), false);
        }
    }

    int x, y, z;
    Eigen::Vector3d pt;
    for (auto& plane : gridOccupied_)
    {
        for (auto& row : plane)
        {
            std::fill(row.begin(), row.end(), false);
        }
    }
    int sX = std::floor(pose(0,3)/subsampleRadius_);
    int sY = std::floor(pose(1,3)/subsampleRadius_);
    int sZ = std::floor(pose(2,3)/subsampleRadius_);
    for (unsigned int i = 0; i < ptCloud_.size(); i++) {

        // Save the pt if it is within the maximum and mininmum sensor ranges.
        double normSquared = pow(ptCloud_[i][0]-pose(0,3), 2) + pow(ptCloud_[i][1]-pose(1,3), 2) + pow(ptCloud_[i][2]-pose(2,3), 2);
        if ((normSquared < maxSensorRange2_)) {
            
            pt = ptCloud_[i].head(3);
            auto voxel = (pt / subsampleRadius_).array().floor().cast<int>();
            x = voxel(0) + offset_ - sX;
            y = voxel(1) + offset_ - sY;
            z = voxel(2) + offset_ - sZ;

            if (x > -1 && x < dim_ &&
                y > -1 && y < dim_ &&
                z > -1 && z < dim_ &&
                !gridOccupied_[x][y][z])
            {
                gridOccupied_[x][y][z] = true;
                tempMap.push_back({ptCloud_[i][0], ptCloud_[i][1], ptCloud_[i][2], 1});
            }            
        }
    }
    ptCloud_ = tempMap;

    // Update the Kd-tree point cloud.
    convertToPointCloudKdTree_(ptCloud_);
}