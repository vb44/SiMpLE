#pragma once

#ifndef REGISTER_H
#define REGISTER_H

#include <dlib/optimization.h>
#include <eigen3/Eigen/Dense>
#include <nanoflann.hpp>

#include "ConfigParser.hpp"
#include "ObjectiveFunction.hpp"

class Register
{
    public:
        /**
         * @brief Construct a new Register object.
         * 
         * @param config Algorithm configuration parameters.
         */
        Register(ConfigParser &config);

        /**
         * @brief Destroy the Register object.
         * 
         */
        ~Register();

        /**
         * @brief Register the new scan with the map.
         * 
         * @param scan      The new scan to register with the exisitng map.
         * @param kdTreePts The local map in a nanoflann-friendly container.
         */
        void registerScan(std::vector<Eigen::Vector4d> &scan,
                          PointCloud<double> &kdTreePts);

    public:
        // Registration result score.
        double registrationScore;

        // Registration result using a dlib-friendly container.
        column_vector regResult = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    private:
        // Initial seed for the optimisation solver - start at the origin.
        std::vector<double> seedConstVel_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        // Previous registration result.
        column_vector prevResult_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    private:
        // Optimizer convergence tolerance for the exit condition.
        double convTol_;

        // Scoring parameter for the objective function.
        double rewardParam_;
};

#endif