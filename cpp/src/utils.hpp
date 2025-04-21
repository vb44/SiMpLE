#pragma once

#include <chrono>
#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <vector>

#include "dlib/optimization.h"

#include "ConfigParser.hpp"

/**
 * @brief A column vector used to interface with the Dlib library.
 */
typedef dlib::matrix<double, 0, 1> column_vector;

namespace utils {

    /**
     * @brief Construct a homogeneous (4x4) matrix.
     * 
     * @param roll              Roll angle in radians.
     * @param pitch             Pitch angle in radians.
     * @param yaw               Yaw angle in radians.
     * @param x                 X position in metres.
     * @param y                 Y position in metres.
     * @param z                 Z position in metres.
     * @return Eigen::Matrix4d  The homogeneous (4x4) matrix constructed. 
     */
    Eigen::Matrix4d homogeneous(double roll, double pitch, double yaw, 
                                double x, double y, double z);

    /**
     * @brief Convert from a homogeneous (4x4) matrix to a homogeneous (6x1) vector.
     * 
     * @param T                     Input homogeneous (4x4) matrix.
     * @return std::vector<double>  The constructed homogeneous (6x1) vector.
     */
    std::vector<double> hom2rpyxyz(Eigen::Matrix4d T);

    /**
     * @brief Read strings and convert to numbers to correctly order the input
     *        files. No two files should have the same name.
     * 
     * @param a         The first string to compare.
     * @param b         The second string to compare.
     * @return true     If file a's name is smaller than file b's name.
     * @return false    If file b's name is smaller than file a's name.
     */
    bool compareStrings(std::string a, std::string b);

    /**
     * @brief Write the results in the Kitti format. Each row contains 12 values
     *        that can be reshaped to a (4x3) matrix representing the first three
     *        rows of a homogenous matrix. 
     * 
     * @param config            Configuration parameters used to print the MATLAB
     *                          readable config file.
     * @param poseEstimates     Pose estimates to write to file.
     * @param outputFileName    Output file name.
     * @param avgTimePerScan    Average time for a single scan's registration result.
     */
    void writeResults(ConfigParser &config,
                    std::vector<std::vector<double> > poseEstimates,
                    std::string outputFileName, double avgTimePerScan);

    /**
     * @brief Print the progress to the terminal as a percentage out of 100%.
     * 
     * @param percentage The percentage of total scans registered.
     */
    void printProgress(double percentage);
} // namespace utils