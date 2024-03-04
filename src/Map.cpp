#include "Map.hpp"

Map::Map(ConfigParser &config)
    : Scan(config)
{
    subsampleRadius_ = config.rMap;
}

Map::~Map()
{
}

void Map::updateMap(std::vector<Eigen::Vector4d> &pts, Eigen::Matrix4d &pose)
{
    // Add transformed scan to the existing map.
    for (auto &pt : pts)
    {
        Eigen::Vector4d ptTf = pose * pt;
        ptCloud.push_back(ptTf);
    }

    // Subsample and save the new map.
    std::vector<Eigen::Vector4d> tempMap;
    allPoints_.clear();
    for (unsigned int i = 0; i < ptCloud.size(); i++)
        allPoints_.insert(i);        
    subsample_(ptCloud, subsampleRadius_);
    tempMap = ptCloud;

    // Remove points outside rMax to limit the size of the map.
    ptCloud.clear();
    for (unsigned int k = 0; k < tempMap.size(); k++)
    {
        if ((pow(tempMap[k](0) - pose(0,3),2) + 
             pow(tempMap[k](1) - pose(1,3),2) + 
             pow(tempMap[k](2) - pose(2,3),2)) < pow(maxSensorRange_,2))
        {
            ptCloud.push_back(tempMap[k]);
        }
    }

    // Update the Kd-tree point cloud.
    convertToPointCloudKdTree_(ptCloud);
}