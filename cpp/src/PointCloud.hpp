#pragma once

#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <nanoflann.hpp>
#include <set>
#include <tbb/parallel_for.h>

#include "ConfigParser.hpp"
#include "nanoflannUtils.hpp"
#include "utils.hpp"

/**
 * @brief A generic class for performing operations on a pointcloud.
 * 
 */
class PointCloud
{
    public:
        /**
         * @brief Construct a new Scan object.
         * 
         * @param config The algorithm configuration parameters.
         */
        PointCloud(const ConfigParser &config);
        
        /**
         * @brief Destroy the Scan object.
         * 
         */
        ~PointCloud() = default;

        /**
         * @brief Read a new .bin scan file.
         * 
         * @param fileName Name of the file to read.
         */
        void readScan(const std::string &fileName);

        /**
         * @brief Get the point cloud.
         * 
         * @return const std::vector<Eigen::Vector4d>& The point cloud.
         */
        const std::vector<Eigen::Vector4d> &getPtCloud() const;

        /**
         * @brief Get the point cloud in a nanoflann-friendly container.
         * 
         * @return const NanoflannPointsContainer<double>& The point cloud in
         *         a nanoflann-friendly container.
         */
        const NanoflannPointsContainer<double> &getPcForKdTree() const;

    protected:
        static constexpr int NUM_COLUMNS_BIN = 4;

        // Point cloud subsample radius.
        double subsampleRadius_;
        
        // Maximum range of the points in the scan.
        double maxSensorRange_;

        // Minimum range of the points in the scan.
        double minSensorRange_;

        // A container used for radially subsampling the points.
        std::set<int> allPoints_;

        // Container to store the point cloud.
        std::vector<Eigen::Vector4d> ptCloud_;

        // Container for a nanoflann-friendly point cloud.
        NanoflannPointsContainer<double> pcForKdTree_;

        /**
         * @brief Radially subsample the point cloud.
         * 
         * @param pts The points to subsample.
         * @param subsampleRadius The subsample radius in meters.
         */
        void subsample_(std::vector<Eigen::Vector4d> &pts,
                        double subsampleRadius);

        /**
         * @brief Convert the points to a nanoflann-friendly container.
         * 
         * @param pts The points to convert to a nanoflann-friendly container.
         */
        void convertToPointCloudKdTree_(const std::vector<Eigen::Vector4d> &pts);

    private:
        // Boolean to apply the correcttion factor to the KITTI scans.
        bool kitti_;
        
        /**
         * @brief Apply a correcttion factor to fix the KITTI scans.
         * 
         */
        void correctKittiScan();

        /**
         * @brief Apply the scan correction factor if required and subsample
         *        the scan.
         * 
         */
        void processPointCloud();
};

#endif