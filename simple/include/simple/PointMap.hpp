#pragma once

#ifndef POINTMAP_H
#define POINTMAP_H

#include "PointCloud.hpp"

/**
 * @brief A class to maintain an exisitng map of the environment used for
 *        registration with a new scan.
 * 
 */
class PointMap : public PointCloud {
    
    public:
        /**
         * @brief Construct a new Map object.
         * 
         * @param mapSubsampleRadius2 The spatial separation used to subsample the local map, squared (meters^2).
         * @param maxSensorRange     The sensor's maximum range.
         */
        PointMap(double mapSubsampleRadius, double maxSensorRange);

        /**
         * @brief Destroy the Map object.
         * 
         */
        ~PointMap() = default;

        /**
         * @brief Update the existing map with the new registration result.
         * 
         * @param pts   The new scan points to update the exisitng map.
         * @param pose  The current pose of the sensor.
         */
        void updateMap(const std::vector<Eigen::Vector4d> &pts, const Eigen::Matrix4d &pose);
};

#endif // POINTMAP_H