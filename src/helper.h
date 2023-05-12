#ifndef HELPER_H
#define HELEPR_H

// include libraries
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <dlib/optimization.h>
#include <dlib/global_optimization.h>
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

using my_kd_tree_t = const nanoflann::KDTreeSingleIndexAdaptor<
                     nanoflann::L2_Simple_Adaptor<double, PointCloud<double>>,
                     PointCloud<double>, 3
                     >;

// dlib example 
typedef dlib::matrix<double,0,1> column_vector;

// scan type
struct scan {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
};

// parameters
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
};

int parseArgs(params* parameters, int argc, char* argv[]);
Eigen::Matrix4d homogeneous(double roll, double pitch, double yaw, 
                            double x, double y, double z);
std::vector<double> hom2rpyxyz(Eigen::Matrix4d T);
void convertToPointCloud3D(PointCloud<double>& pc, scan &pointcloudRead);
bool compareStrings(std::string a, std::string b);
scan subsample(double subsampleRadius, std::set<int> allPoints, scan pts);
void writeResults(params* config, std::vector<std::vector<double> > poseEstimates, std::string outputFileName, double avgTimePerScan);
void printProgress(double percentage);
#endif