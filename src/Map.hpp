#pragma once

#ifndef MAP_H
#define MAP_H

#include "Scan.hpp"

/**
 * @brief A class to maintain an exisitng map of the environment used for
 *        registration with a new scan.
 * 
 */
class Map : public Scan
{
    public:
        /**
         * @brief Construct a new Map object.
         * 
         * @param config The algorithm configuration parameters.
         */
        Map(ConfigParser &config);

        /**
         * @brief Destroy the Map object.
         * 
         */
        ~Map();

        /**
         * @brief Update the existing map with the new registration result.
         * 
         * @param pts   The new scan points to update the exisitng map.
         * @param pose  The current pose of the sensor.
         */
        void updateMap(std::vector<Eigen::Vector4d> &pts,
                       Eigen::Matrix4d &pose);
};

#endif