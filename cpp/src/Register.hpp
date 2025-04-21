#pragma once

#ifndef REGISTER_H
#define REGISTER_H

#include <dlib/optimization.h>
#include <eigen3/Eigen/Dense>

#include "ObjectiveFunction.hpp"

class Register {
    public:
        /**
         * @brief Construct a new Register object.
         * 
         * @param convergenceTol The convergence tolerance for the optimisation.alignas
         * @param sigma The scoring parameter for the objective function.
         */
        Register(double convergenceTol, double sigma);

        /**
         * @brief Destroy the Register object.
         * 
         */
        ~Register() = default;

        /**
         * @brief Get the registration score.
         * 
         * @return double The registration score.
         */
        double getRegistrationScore() const;

        /**
         * @brief Get the registration result.
         * 
         * @return column_vector The registration result.
         */
        column_vector getRegResult() const;

        /**
         * @brief Register the new scan with the map.
         * 
         * @param scan      The new scan to register with the exisitng map.
         * @param kdTreePts The local map in a nanoflann-friendly container.
         */
        void registerScan(const std::vector<Eigen::Vector4d> &scan, const NanoflannPointsContainer<double> &kdTreePts);

    private:
        // Optimizer convergence tolerance for the exit condition.
        double convTol_;

         // Registration result score.
        double registrationScore_;

        // Scoring parameter for the objective function.
        double rewardParam_;

        // Initial seed for the optimisation solver - start at the origin.
        std::vector<double> seedConstVel_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        // Registration result using a dlib-friendly container.
        column_vector regResult_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        // Previous registration result.
        column_vector prevResult_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

#endif // REIGISTER_H