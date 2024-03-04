// This file is modeified from the nanoflann library: 
// https://github.com/jlblancoc/nanoflann/tree/master/examples.

/***********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright 2011-2022 Jose Luis Blanco (joseluisblancoc@gmail.com).
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *************************************************************************/
#pragma once

#include <chrono>
#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <vector>

#include "dlib/optimization.h"
#include "nanoflann.hpp"

#include "ConfigParser.hpp"

// From the nanoflann library.
template <typename T>
struct PointCloud
{
    struct Point
    {
        T x, y, z;
    };

    using coord_t = T;  //!< The type of each coordinate

    std::vector<Point> pts;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate
    // value, the
    //  "if/else's" are actually solved at compile time.
    inline T kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        if (dim == 0)
            return pts[idx].x;
        else if (dim == 1)
            return pts[idx].y;
        else
            return pts[idx].z;
    }

    // Optional bounding-box computation: return false to default to a standard
    // bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned
    //   in "bb" so it can be avoided to redo it again. Look at bb.size() to
    //   find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const
    {
        return false;
    }
};

/**
 * @brief Kd tree type used directly from the nanoflann library.
 */
using my_kd_tree_t = const nanoflann::KDTreeSingleIndexAdaptor<
                     nanoflann::L2_Simple_Adaptor<double, PointCloud<double>>,
                     PointCloud<double>, 3
                     >;

/**
 * @brief A column vector used to interface with the Dlib library.
 */
typedef dlib::matrix<double, 0, 1> column_vector;

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