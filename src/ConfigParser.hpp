#pragma once

#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>

/**
 * @brief A class to handle the algorithm configuration parsing.
 * 
 */
class ConfigParser
{
    public:
        /**
         * @brief Construct a new Config Parser object.
         * 
         * @param argc          Number of commandline arguments.
         * @param yamlFilePath  Commandline arguments.
         */
        ConfigParser(int argc, char** yamlFilePath);

        /**
         * @brief Destroy the Config Parser object.
         * 
         */
        ~ConfigParser();

        /**
         * @brief Parse the algrithm configuration path.
         * 
         * @return int Returns 0 if algorithm parsing was successful and 1
         *             otherwise.
         */
        int parseConfig();

    public:
        // Print the algorithm configuration parameters.
        bool verbose;

        // Correct the KITTI scans.
        bool kitti;

        // The standard deviation used to calculate proximity-based reward.
        double sigma;

        // The spatial separation used to subsample the existing map.
        double rMap;

        // The spatial separation used to subsample new point cloud data.
        double rNew;

        // The optimization solver’s exit condition is described as a minimum
        // reward improvement between iterations.
        double convergenceTol;

        // The sensor's maximum range.
        double maxSensorRange;
        
        // The sensor's minimum range.
        double minSensorRange;

        // Path to the scan files.
        std::string scanPath;

        // Name of the output file.
        std::string outputFileName;

    private:
        // Path to the algorithm configuration file.
        std::string yamlFilePath_;
};