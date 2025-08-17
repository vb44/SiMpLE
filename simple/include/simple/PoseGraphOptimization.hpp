/**
 * @file PoseGraphOptimization.hpp
 * @brief This class is an implementation from https://github.com/gisbi-kim/SC-A-LOAM/blob/main/src/laserPosegraphOptimization.cpp.
 * The implmentation is being used as a starting point for developing SiMpLE into a SLAM algorithm, simple-slam.
 */

#pragma once

#ifndef POSE_GRAPH_OPTIMIZATION_H
#define POSE_GRAPH_OPTIMIZATION_H

#include <filesystem>
#include <fstream>
#include <iostream>
#include <math.h>
#include <optional>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <ceres/ceres.h>

#include <eigen3/Eigen/Dense>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h> 
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>
#include <pcl/search/impl/search.hpp>

#include "Scancontext.hpp"

using namespace gtsam;

typedef pcl::PointXYZI PointType;

struct Pose6D {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

class PoseGraphOptimization {
    
    public:

        // PoseGraphOptimization();
        PoseGraphOptimization(std::ofstream& logFile, double keyframeMeterGap, double keyframeDegGap,
                              double scDistThres, double scMaximumRadius,
                              double relinearizeThreshold, double relinearizeSkip,
                              double filter_size, double mapVizFilterSize, double loopFitnessScoreThreshold, std::string saveDirectory);

        ~PoseGraphOptimization() = default;

        double rad2deg(double radians);

        double deg2rad(double degrees);
        

        std::string padZeros(int val, int num_digits = 6);

        std::string getVertexStr(const int _node_idx, const gtsam::Pose3& _Pose);

        void writeEdge(const std::pair<int, int> _node_idx_pair, const gtsam::Pose3& _relPose, std::vector<std::string>& edges_str);
        
        void saveSCD(std::string fileName, Eigen::MatrixXd matrix, std::string delimiter = " ");

        gtsam::Pose3 Pose6DtoGTSAMPose3(const Pose6D& p);

        // void saveGTSAMgraphG2oFormat(const gtsam::Values& _estimates);

        void saveOdometryVerticesKITTIformat(std::string _filename);

        void saveOptimizedVerticesKITTIformat(gtsam::Values _estimates, std::string _filename);

        void laserOdometryHandler(const Pose6D laserOdometry);

        void laserCloudFullResHandler(const pcl::PointCloud<PointType>::Ptr laserCloudFullRes);

        void initNoises( void );

        Pose6D diffTransformation(const Pose6D& _p1, const Pose6D& _p2);

        pcl::PointCloud<PointType>::Ptr local2global(const pcl::PointCloud<PointType>::Ptr &cloudIn, const Pose6D& tf);

        void updatePoses(void);

        void runISAM2opt(void);

        pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, gtsam::Pose3 transformIn);

        void loopFindNearKeyframesCloud( pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& submap_size, const int& root_idx);

        // TODO: Replace this with SiMpLE later?
        std::optional<gtsam::Pose3> doICPVirtualRelative( int _loop_kf_idx, int _curr_kf_idx );

        void process_pg();

        void performSCLoopClosure(void);

        void process_lcd(void);

        void process_icp(void);

        void process_isam(void);

        // template <typename PointType>
        void eigenToPCL(const std::vector<Eigen::Vector4d> &ptCloud);


    // private:
        std::ofstream &logFile_;
        
        double keyframeMeterGap_;
        double keyframeRadGap_;
        double translationAccumulated;
        double rotaionAccumulated;
        double loopFitnessScoreThreshold_;

        bool isNowKeyFrame = false; 

        Pose6D odom_pose_prev {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init 
        Pose6D odom_pose_curr {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init pose is zero 

        // Inputs from the front-end LiDAR odometry
        std::queue<Pose6D> odometryBuf; // Odometry estimates buffer // TODO: Don't think I need a buffer for sequential usage
        std::queue<pcl::PointCloud<PointType>::Ptr> fullResBuf; // Point cloud buffer
        std::queue<std::pair<int, int> > scLoopICPBuf;
        std::queue<double> odometryBufTime; // TODO: Don't think I need a buffer for sequential usage
        std::queue<double> fullResBufTime;
        double timeLaserOdometry;
        double timeLaser;

        pcl::PointCloud<PointType>::Ptr laserCloudFullRes; //(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr laserCloudMapAfterPGO; //(new pcl::PointCloud<PointType>());

        std::vector<pcl::PointCloud<PointType>::Ptr> keyframeLaserClouds; 
        std::vector<Pose6D> keyframePoses;
        std::vector<Pose6D> keyframePosesUpdated;
        std::vector<double> keyframeTimes;
        int recentIdxUpdated;

        gtsam::NonlinearFactorGraph gtSAMgraph;
        bool gtSAMgraphMade;
        gtsam::Values initialEstimate;
        gtsam::ISAM2 *isam;
        gtsam::Values isamCurrentEstimate;

        noiseModel::Diagonal::shared_ptr priorNoise;
        noiseModel::Diagonal::shared_ptr odomNoise;
        noiseModel::Base::shared_ptr robustLoopNoise;
        noiseModel::Base::shared_ptr robustGPSNoise;

        pcl::VoxelGrid<PointType> downSizeFilterScancontext;
        SCManager scManager;
        double scDistThres, scMaximumRadius;

        pcl::VoxelGrid<PointType> downSizeFilterICP;

        pcl::PointCloud<PointType>::Ptr laserCloudMapPGO; //(new pcl::PointCloud<PointType>());
        pcl::VoxelGrid<PointType> downSizeFilterMapPGO;
        bool laserCloudMapPGORedraw;

        double recentOptimizedX;
        double recentOptimizedY;

        // std::string save_directory;
        // std::string pgScansDirectory, pgSCDsDirectory;
        std::string odomKITTIformat_, pgKITTIformat_;
        // std::fstream pgG2oSaveStream, pgTimeSaveStream;

        std::vector<std::string> edges_str; // used in writeEdge      
};

#endif // POSE_GRAPH_OPTIMIZATION_H