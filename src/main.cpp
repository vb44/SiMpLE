#include <chrono>
#include <filesystem>

#include "ConfigParser.hpp"
#include "Scan.hpp"
#include "Map.hpp"
#include "Register.hpp"
#include "utils.hpp"

int main(int argc, char* argv[])
{
    // ------------------------------------------------------------------------
    // ARGUMENT PARSING
    // ------------------------------------------------------------------------
    ConfigParser config(argc, argv);
    int configStatus = config.parseConfig();
    if (configStatus) exit(1);

    // ------------------------------------------------------------------------
    // LOAD THE SCAN PATHS
    // ------------------------------------------------------------------------
    std::vector<std::string> scanFiles;
    for (auto const& dir_entry : 
            std::filesystem::directory_iterator(config.scanPath)) 
        scanFiles.push_back(dir_entry.path());

    // Sort the scans in order of the file name.
    std::sort(scanFiles.begin(), scanFiles.end(), compareStrings);

    // Number of scans.
    unsigned int numScans = scanFiles.size();

    // ------------------------------------------------------------------------
    // DETERMINE POINT CLOUD REGISTRATION RESULTS
    // ------------------------------------------------------------------------     
    // Store the pose estimates in (roll,pitch,yaw,x,y,z,registrationScore)
    // format.
    std::vector<std::vector<double> > poseEstimates(numScans, 
                                                    std::vector<double>(7));
    
    // Start the timer.
    auto startReg = std::chrono::high_resolution_clock::now();

    // Container for a new scan.
    Scan newScan(config);
    Map subMap(config);
    Register scanToMapRegister(config);

    // Loop over all input scans, update the submap, and save the registration
    // result.
    for (unsigned int scanNum = 0; scanNum < numScans; scanNum++)
    {
        // Step 1: Read the scan and subsample the scan at rNew.
        newScan.readScan(scanFiles[scanNum]);

        // --------------------------------------------------------------------
        // STEP 2: INPUT POINT CLOUD TO LOCAL MAP REGISTRATION
        // --------------------------------------------------------------------
        if (scanNum > 0)
        {
            scanToMapRegister.registerScan(newScan.ptCloud, 
                                           subMap.pcForKdTree_);

            // Save the results.
            poseEstimates[scanNum] = {scanToMapRegister.regResult(0),
                                      scanToMapRegister.regResult(1),
                                      scanToMapRegister.regResult(2),
                                      scanToMapRegister.regResult(3),
                                      scanToMapRegister.regResult(4),
                                      scanToMapRegister.regResult(5),
                                      scanToMapRegister.registrationScore};
        }
        
        // --------------------------------------------------------------------
        // STEP 3: UPDATE THE LOCAL MAP
        // --------------------------------------------------------------------
        // Transform the current scan to the current pose estimate.
        Eigen::Matrix4d hypothesis = homogeneous(poseEstimates[scanNum][0],
                                                 poseEstimates[scanNum][1],
                                                 poseEstimates[scanNum][2], 
                                                 poseEstimates[scanNum][3],
                                                 poseEstimates[scanNum][4],
                                                 poseEstimates[scanNum][5]);

        subMap.updateMap(newScan.ptCloud, hypothesis);

        // Print the progress to the terminal.
        // Comment printProgress to remove this. 
        printProgress((double(scanNum) / numScans));
    }
    // End with a new line character for the progress bar.
    printf("\n");

    // Calculate the average time per registration result.
    auto stopReg = std::chrono::high_resolution_clock::now();
    auto durationReg = std::chrono::duration_cast<std::chrono::milliseconds>
                                                        (stopReg - startReg);
    double avgTimePerScan = durationReg.count() / numScans;

    // ------------------------------------------------------------------------
    // OUTPUT RESULTS
    // ------------------------------------------------------------------------
    if (config.verbose)
        std::cout << "avgTimePerScan [ms] = "
                  << avgTimePerScan << ";" << std::endl;

    // Output a file with the results in the KITTI format, and a file with the
    // configuration parameters.
    writeResults(config, poseEstimates, config.outputFileName, avgTimePerScan);
    
    return 0;
}