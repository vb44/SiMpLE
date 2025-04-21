#pragma once

#ifndef OBJECTIVE_FUNCTION_H
#define OBJECTIVE_FUNCTION_H

#include "nanoflannUtils.hpp"
#include "utils.hpp"

/**
 * @brief The objective function used by the optimisation solver to find the
 *        best homogeneous transform that aligns the new scan with the submap.
 *        The only configuration parameter used here is sigma to score each
 *        point in the new scan.
 */
class ObjectiveFunction 
{
    public:
        /**
         * @brief Constructor to initialise member variables.
         * 
         * @param uncertainty Calculated using sigma.
         * @param scan The scan to register to subMap.
         * @param sourceScanKdTree A KdTree of the subMap.
         */
        ObjectiveFunction(double sigma, const std::vector<Eigen::Vector4d> &scan, my_kd_tree_t* subMapKdTree);

        /**
         * @brief Destroy the Objective Function object.
         * 
         */
        ~ObjectiveFunction() = default;

        /**
         * @brief Overload the function call operator.
         * 
         * @param m         Hypothesis to register new scan to exsiting map.
         * @return double   The registration score.
         */
        double operator()(const column_vector& m) const;

    private:
        // Registration parameter, this is 1/(2*sig^2).
        double rewardParam_;

        // New scan to register to the existing map.
        std::vector<Eigen::Vector4d> scan_;

        // Size of the new scan.
        int scanSize_;

        // KdTree of the existing map.
        my_kd_tree_t *subMapKdTree_;
};

#endif // ObJECTIVE_FUNCTION_H