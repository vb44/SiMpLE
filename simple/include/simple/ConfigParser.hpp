#pragma once

#ifndef CONFIGPARSER_H
#define CONFIGPARSER_H

#include <iostream>
#include <string>

#include <yaml-cpp/yaml.h>

/**
 * @brief Handle the algorithm configuration parsing.
 * 
 */
class ConfigParser {
    
    public:
        /**
         * @brief Construct a new ConfigParser object.
         * 
         * @param argc Number of commandline arguments.
         * @param argv Commandline arguments.
         */
        ConfigParser(int argc, char** argv);

        /**
         * @brief Destroy the ConfigParser object.
         * 
         */
        ~ConfigParser() = default;

        /**
         * @brief Parse the algorithm configuration path.
         * 
         * @return int Returns 0 if the algorithm configuration parsing was
         *             successful, 1 if there was an error.
         */
        int parseConfig();

        /**
         * @brief Get the KITTI flag.
         * 
         * @return true If the KITTI flag is set.
         * @return false If the KITTI flag is not set.
         */
        bool getKitti() const;

        /**
         * @brief Get the sigma value.
         * 
         * @return double The sigma value.
         */
        double getSigma() const;

        /**
         * @brief Get the voxel resolution used to subsample the existing
         *        map.
         * 
         * @return double The voxel resolution for the map.
         */
        double getVoxelSizeMap() const;

        /**
         * @brief Get the voxel resolution used to subsample new point cloud
         *        data.
         * 
         * @return double The voxel resolution for the scan.
         */
        double getVoxelSizeScan() const;

        /**
         * @brief Get the optimization solver's exit condition.
         * 
         * @return double The convergence tolerance.
         */
        double getConvergenceTol() const;

        /**
         * @brief Get the sensor's maximum range.
         * 
         * @return double The sensor's maximum range.
         */
        double getMaxSensorRange() const;

        /**
         * @brief Get the sensor's minimum range.
         * 
         * @return double The sensor's minimum range.
         */
        double getMinSensorRange() const;

        /**
         * @brief Get the path to the scan files.
         * 
         * @return std::string The path to the scan files.
         */
        const std::string getScanPath() const;

        /**
         * @brief Get the output file name.
         * 
         * @return std::string The output file name.
         */
        const std::string getOutputFileName() const;

        // PGO
        int getScanIntervalLoopClosure() const;
        int getScanIntervalPGO() const;
        double getKeyframeMeterGap() const;
        double getKeyframeDegGap() const;
        double getRelinearizeThreshold() const;
        double getRelinearizeSkip() const;
        double getScDistThres() const;
        double getScMaximumRadius() const;
        double getFilterSize() const;
        double getMapVizFilterSize() const;
        double getLoopFitnessScoreThreshold() const;
        
    private:
        // The expected number of commandline arguments.
        static constexpr int EXPECTED_ARGUMENT_COUNT = 2;

        // Correct the KITTI scans.
        bool kitti_;

        // The standard deviation used to calculate proximity-based reward.
        double sigma_;

        // The voxel resolution used to subsample the existing map.
        double voxelSizeMap_;

        // The voxel resolution used to subsample new point cloud data.
        double voxelSizeScan_;

        // The optimization solver’s exit condition is described as a minimum
        // reward improvement between iterations.
        double convergenceTol_;

        // The sensor's maximum range.
        double maxSensorRange_;
        
        // The sensor's minimum range.
        double minSensorRange_;

        // Path to the scan files.
        std::string scanPath_;

        // Name of the output file.
        std::string outputFileName_;

        // Path to the algorithm configuration file.
        std::string yamlFilePath_;

        // PGO
        int scanIntervalLoopClosure_;
        int scanIntervalPGO_;
        double keyframeMeterGap_; // pose assignment every k m move 
        double keyframeDegGap_; // pose assignment every k deg rot
        double relinearizeThreshold_;
        double relinearizeSkip_;
        double scDistThres_;  
        double scMaximumRadius_; // 80 is recommended for outdoor, and lower (ex, 20, 40) values are recommended for indoor 
        double filterSize_;
        double mapVizFilterSize_; // pose assignment every k frames 
        double loopFitnessScoreThreshold_; // user parameter but fixed low value is safe.
};

#endif // CONFIGPARSER_H