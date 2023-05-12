#ifndef OBJECTIVE_FUNCTION_H
#define OBJECTIVE_FUNCTION_H

#include "helper.h"

class ObjectiveFunction
{
    private:
        Eigen::Matrix3d uncertainty_;           // registration parameter
        scan targetScanInput_;                  // target scan
        Eigen::MatrixXd targetScanMultiply_;
        int targetScanSize_;                    // length of the output scan
        PointCloud<double> sourceScan_;         // create a point cloud object for the kd tree
        my_kd_tree_t *subMapKdTree;             // submap Kd tree

    public:        
        // constructor to initialise member variables
        ObjectiveFunction(Eigen::Matrix3d uncertainty, scan targetScanInput,
                          PointCloud<double> subMap,  my_kd_tree_t *sourceScanKdTree_);

        // overload function call operator
        double operator()(const column_vector& m) const;
};

#endif