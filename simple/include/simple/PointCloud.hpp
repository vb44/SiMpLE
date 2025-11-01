#pragma once

#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <set>
#include <tbb/parallel_for.h>
#include <unordered_map>

#include "nanoflannUtils.hpp"
#include "utils.hpp"

/**
 * @brief A generic class for performing operations on a scan.
 * 
 */
class PointCloud {

    public:
        /**
         * @brief Construct a new Scan object.
         * 
         * @param maxSensorRange Maximum sensor range.
         */
        PointCloud(double maxSensorRange);
        
        /**
         * @brief Construct a new Scan object.
         * 
         * @param voxelSize Point cloud voxel size.
         * @param maxSensorRange Maximum sensor range.
         * @param minSensorRange Minimum sensor range.
         * @param kitti Flag when using kitti scans as these need to be corrected.
         */
        PointCloud(double voxelSize, double maxSensorRange,
                   double minSensorRange, bool kitti);
        
        /**
         * @brief Destroy the Scan object.
         * 
         */
        ~PointCloud() = default;

        /**
         * @brief Add point to pointcloud if it is within the maximum and mininmum sensor ranges.
         *
         * @param x point x coordinate.
         * @param y point y coordinate.
         * @param z point z coordinate.
         */
        void addPoint(double x, double y, double z);

        /**
         * @brief Read a new .bin scan file.
         * 
         * @param fileName Name of the file to read.
         */
        void readScan(std::string fileName);

        /**
         * @brief Apply the scan correction factor if required and subsample
         *        the scan.
         *
         */
        void processPointCloud();

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

        // Point cloud voxel size.
        double voxelSize_; // meters
        
        // Maximum range of the points in the scan.
        double maxSensorRange2_; // squared (^2)

        // Minimum range of the points in the scan.
        double minSensorRange2_; // squared (^2)

        // Container to store the point cloud.
        std::vector<Eigen::Vector4d> ptCloud_;

        // Container for a nanoflann-friendly point cloud.
        NanoflannPointsContainer<double> pcForKdTree_;

        /**
         * @brief Subsample the point cloud using a voxel-based filter (original point is retained).
         * 
         * @param pts       The points to subsample.
         * @param voxelSize The voxel size in meters.
         */
        void subsample_(std::vector<Eigen::Vector4d> &pts, double voxelSize_);

        /**
         * @brief Convert the points to a nanoflann-friendly container.
         * 
         * @param pts The points to convert to a nanoflann-friendly container.
         */
        void convertToPointCloudKdTree_(std::vector<Eigen::Vector4d> &pts);

    private:
        // Boolean to apply the correcttion factor to the KITTI scans.
        bool kitti_;

        /**
         * @brief Apply a correcttion factor to fix the KITTI scans.
         * 
         */
        void correctKittiScan_();
};

#endif // POINTCLOUD_H
