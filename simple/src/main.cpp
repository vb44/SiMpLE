#include <chrono>
#include <filesystem>

#include "ConfigParser.hpp"
#include "PointCloud.hpp"
#include "PointMap.hpp"
#include "Register.hpp"
#include "utils.hpp"

int main(int argc, char* argv[]) {
    // Argument parsing.
    ConfigParser config(argc, argv);
    int configStatus = config.parseConfig();
    if (configStatus) exit(1);

    // Load the scan paths.
    std::vector<std::string> scanFiles;
    for (auto const& dir_entry : std::filesystem::directory_iterator(config.getScanPath())) {
        scanFiles.push_back(dir_entry.path());
    }

    // Sort the scans in order of the file name.
    std::sort(scanFiles.begin(), scanFiles.end(), utils::compareStrings);

    // Number of scans.
    unsigned int numScans = scanFiles.size();

    // Estimate the point cloud registration results.
    // Store the pose estimates in (roll,pitch,yaw,x,y,z,registrationScore) format.
    std::vector<std::vector<double> > poseEstimates(numScans, std::vector<double>(7));
    
    // Start the timer.
    auto startReg = std::chrono::high_resolution_clock::now();

    // Container for a new scan.
    PointCloud newScan(config.getRNew(), config.getMaxSensorRange(), config.getMinSensorRange(), config.getKitti());
    PointMap subMap(config.getRMap(), config.getMaxSensorRange());
    Register scanToMapRegister(config.getConvergenceTol(), config.getSigma());

    // Loop over all input scans, update the submap, and save the registration
    // result.
    for (unsigned int scanNum = 0; scanNum < numScans; scanNum++) {
        // Step 1: Read the scan and subsample the scan at rNew.
        newScan.readScan(scanFiles[scanNum]);

         // Step 2: Input point cloud to local map registration.
        if (scanNum > 0) {
            scanToMapRegister.registerScan(newScan.getPtCloud(), subMap.getPcForKdTree());

            column_vector res = scanToMapRegister.getRegResult();

            // Save the results.
            poseEstimates[scanNum] = {res(0), res(1), res(2), res(3), res(4), res(5),
                                      scanToMapRegister.getRegistrationScore()};
        }
        
        // Step 3: Update the local map.
        // Transform the current scan to the current pose estimate.
        Eigen::Matrix4d hypothesis = utils::homogeneous(poseEstimates[scanNum][0], poseEstimates[scanNum][1],
                                                        poseEstimates[scanNum][2], poseEstimates[scanNum][3],
                                                        poseEstimates[scanNum][4], poseEstimates[scanNum][5]);

        subMap.updateMap(newScan.getPtCloud(), hypothesis);

        // Print the progress to the terminal.
        // Comment printProgress to remove this. 
        utils::printProgress((double(scanNum) / numScans));
    }
    // End with a new line character for the progress bar.
    printf("\n");

    // Calculate the average time per registration result.
    auto stopReg = std::chrono::high_resolution_clock::now();
    auto durationReg = std::chrono::duration_cast<std::chrono::milliseconds>(stopReg - startReg);
    double avgTimePerScan = durationReg.count() / numScans;

    // Write results.
    // Output a file with the results in the KITTI format, and a file with the configuration parameters.
    utils::writeResults(config, poseEstimates, config.getOutputFileName(), avgTimePerScan);
    std::cout << "avgTimePerScan [ms] = " << avgTimePerScan << ";" << std::endl;
    
    return 0;
}