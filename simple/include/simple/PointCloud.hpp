#pragma once

#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <set>
#include <tbb/parallel_for.h>

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
         * @param subsampleRadius Point cloud subsampling radius.
         * @param maxSensorRange Maximum sensor range.
         * @param minSensorRange Minimum sensor range.
         * @param kitti Flag when using kitti scans as these need to be corrected.
         */
        PointCloud(double subsampleRadius, double maxSensorRange,
                   double minSensorRange, bool kitti);
        
        /**
         * @brief Destroy the Scan object.
         * 
         */
        ~PointCloud() = default;

        /**
         * @brief Add point to pointcloud if it is within the maximum and mininmum sensor ranges
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

        // Point cloud subsample radius.
        double subsampleRadius2_; // squared (^2)
        
        // Maximum range of the points in the scan.
        double maxSensorRange2_; // squared (^2)

        // Minimum range of the points in the scan.
        double minSensorRange2_; // squared (^2)

        // A container used for radially subsampling the points.
        std::set<int> allPoints_;

        // Container to store the point cloud.
        std::vector<Eigen::Vector4d> ptCloud_;

        // Container for a nanoflann-friendly point cloud.
        NanoflannPointsContainer<double> pcForKdTree_;

        /**
         * @brief Radially subsample the point cloud.
         * 
         * @param pts               The points to subsample.
         * @param subsampleRadius2   The subsample radius in meters^2.
         */
        void subsample_(std::vector<Eigen::Vector4d> &pts, double subsampleRadius2);

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
