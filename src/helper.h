#ifndef HELPER_H
#define HELEPR_H

// include libraries
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <dlib/optimization.h>
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <nanoflann.hpp>
#include <sstream>
#include <string>
#include <vector>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>

// nanoflann helper file
#include "utils.h"

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
 * @brief A container to store all the configuration parameters and settings.
 */
struct params {
    std::string path;           // scan path 
    double sigma;               // config:   reward standard deviation
    double rMap;                // config:   local map subsampling radius
    double rNew;                // config:   new scan subsampling radius
    double convergenceTol;      // config:   optimisation solver convergence tolerance
    double maxSensorRange;      // hardware: maximum laser range
    double minSensorRange;      // optional: for computational benfit only, set to zero otherwise
    std::string outputFileName; // output file name
    bool verbose = false;       // print the test settings
    bool kitti = false;         // the KITTI scans need to be corrected
};

/**
 * @brief Parse the commandline arguments are store the configuration
 *        parameters and the test settings. 
 * 
 * @param parameters    An params type to store the configuration parameters. 
 * @param argc          The number of arguments entered at the commandline.
 * @param argv          The commandline input.
 * @return int          Returns 0 if arguments are successfully passed, or 1
 *                      if there is an error in parsing the arguments.
 */
int parseArgs(params* parameters, int argc, char* argv[]);

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
Eigen::Matrix4d homogeneous(double roll, double pitch, double yaw, double x, double y, double z);

/**
 * @brief Convert from a homogeneous (4x4) matrix to a homogeneous (6x1) vector.
 * 
 * @param T                     Input homogeneous (4x4) matrix.
 * @return std::vector<double>  The constructed homogeneous (6x1) vector.
 */
std::vector<double> hom2rpyxyz(Eigen::Matrix4d T);

/**
 * @brief Convert from a scan stored as a matrix to a point cloud readable by
 *        the nanoflann library to construct the Kd-tree.
 * 
 * @param pc    The converted PointCloud type.  
 * @param scan  Scan to be converted.
 */
void convertToPointCloud3D(PointCloud<double>& pc, Eigen::MatrixXd scan);

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
 * @brief Radially subsample the input scan at the subsampling radius.
 * 
 * @param subsampleRadius   Radial subsampling radius.
 * @param allPoints         A set of the scan indicies used to subsample.
 * @param scan              Scan to be subsampled.
 * @return Eigen::MatrixXd  Subsampled scan.
 */
Eigen::MatrixXd subsample(double subsampleRadius, std::set<int> allPoints, Eigen::MatrixXd scan);

/**
 * @brief Correct the Kitti scans by applying a calibration parameter
 *        originally introduced by IMLS-SLAM and used by CT-ICP and
 *        KISS-ICP.
 * 
 * @param scan              Scan to be corrected.
 * @return Eigen::MatrixXd  Corrected scan.
 */
Eigen::MatrixXd correctKittiScan(Eigen::MatrixXd scan);

/**
 * @brief Read the scan from a file.
 * 
 * @param fileName          Filename of the scan. The scan must be in the
 *                          Kitti .bin format.
 * @param config            Configuration parameters to check the rMin and
 *                          rMax for each scan.
 * @param allPoints         The function generates the set of scan indicies 
 *                          used for subsampling.
 * @return Eigen::MatrixXd  Scan read from the file.
 */
Eigen::MatrixXd readScan(std::string fileName, params &config, std::set<int> &allPoints);

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
void writeResults(params* config, std::vector<std::vector<double> > poseEstimates, std::string outputFileName, double avgTimePerScan);

/**
 * @brief Print the progress to the terminal as a percentage out of 100%.
 * 
 * @param percentage The percentage of total scans registered.
 */
void printProgress(double percentage);

#endif