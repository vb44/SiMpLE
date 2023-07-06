#include "objectiveFunction.h"

int main(int argc, char* argv[])
{
    // ------------------------------------------------------------------------
    // ARGUMENT PARSING
    // ------------------------------------------------------------------------
    params config;
    int err = parseArgs(&config, argc, argv);
    if (err) exit(1);
    
    // construct the scoring matrix used in the objective function 
    Eigen::Matrix3d uncertainty;
    uncertainty << pow(config.sigma,-2),0,0,
                   0,pow(config.sigma,-2),0,
                   0,0,pow(config.sigma,-2);

    // ------------------------------------------------------------------------
    // LOAD THE SCANS PATHS
    // ------------------------------------------------------------------------
    std::vector<std::string> scanFiles;
    for (auto const& dir_entry : std::filesystem::directory_iterator(config.path)) 
        scanFiles.push_back(dir_entry.path());

    // sort the scans in order of the file name
    std::sort(scanFiles.begin(),scanFiles.end(),compareStrings);

    // number of scans
    unsigned int numScans = scanFiles.size();

    // ------------------------------------------------------------------------
    // DETERMINE POINT CLOUD REGISTRATION RESULTS
    // ------------------------------------------------------------------------     
    Eigen::MatrixXd subMap(0,4);           // container to store the submap
    PointCloud<double> subMapNanoflann;    // a nanoflann-friendly submap

    // store the pose estimates in (roll,pitch,yaw,x,y,z,registrationScore) format
    std::vector<std::vector<double> > poseEstimates(numScans,std::vector<double>(7));
    
    // seed for the optimisation solver
    std::vector<double> seedConstVel = {0.0,0.0,0.0,0.0,0.0,0.0};

    // start the timer
    auto startReg = std::chrono::high_resolution_clock::now();

    // loop over all input scans, update the submap, and save the registration result
    for (unsigned int scanNum = 0; scanNum < numScans; scanNum++)
    {
        // --------------------------------------------------------------------
        // READ THE POINT CLOUD FROM THE .bin FILES (KITTI FORMAT)
        // --------------------------------------------------------------------
        // a set with the indicies used for subsampling in STEP 1
        std::set<int> allPoints;

        // read the input scan
        Eigen::MatrixXd ptsInRange = readScan(scanFiles[scanNum], config, allPoints);

        // --------------------------------------------------------------------
        // CORRECT THE SCAN - KITTI ONLY
        // --------------------------------------------------------------------
        // apply the calibration factor as explained in IMLS-SLAM, CT-ICP, and KISS-ICP
        // comment the next line if it is not the Kitti dataset and uncomment
        // the following line
        Eigen::MatrixXd ptsCorrected = correctKittiScan(ptsInRange);    // comment out for not Kitti
        // Eigen::MatrixXd ptsCorrected = ptsInRange;                   // uncomment for not Kitti       
        
        // --------------------------------------------------------------------
        // STEP 1: SUBSAMPLE THE INPUT POINT CLOUD AT rNew
        // --------------------------------------------------------------------
        Eigen::MatrixXd ptsSubsampled = subsample(config.rNew, allPoints, ptsCorrected);

        // --------------------------------------------------------------------
        // STEP 2: INPUT POINT CLOUD TO LOCAL MAP REGISTRATION
        // --------------------------------------------------------------------
        if (scanNum > 0)
        {
            // construct a kd tree for the registration process
            // (dimension, scan, max leaf)
            convertToPointCloud3D(subMapNanoflann,subMap);
            my_kd_tree_t *subMapKdTree = new my_kd_tree_t(3,subMapNanoflann,{10});

            // instantiate the objective function
            ObjectiveFunction objFuncFine = ObjectiveFunction(uncertainty,ptsSubsampled,
                                                              subMapNanoflann, subMapKdTree);

            // use the previous pose estimate as the seed
            column_vector regResult = {seedConstVel[0],
                                       seedConstVel[1],
                                       seedConstVel[2],
                                       seedConstVel[3],
                                       seedConstVel[4],
                                       seedConstVel[5]};

            // find the best solution to the objective function
            double registrationScore = dlib::find_min_using_approximate_derivatives(
                                       dlib::bfgs_search_strategy(),
                                       dlib::objective_delta_stop_strategy(config.convergenceTol),
                                       objFuncFine, regResult, -10000000);
            delete subMapKdTree; // free memory
            
            // save the results
            poseEstimates[scanNum] = {regResult(0),
                                      regResult(1),
                                      regResult(2),
                                      regResult(3),
                                      regResult(4),
                                      regResult(5),
                                      registrationScore};

            // calculate the seed for the next pose estimate using the constant velocity mdoel
            seedConstVel = hom2rpyxyz(homogeneous(poseEstimates[scanNum][0],
                                                  poseEstimates[scanNum][1],
                                                  poseEstimates[scanNum][2],
                                                  poseEstimates[scanNum][3],
                                                  poseEstimates[scanNum][4],
                                                  poseEstimates[scanNum][5])*
                                     (homogeneous(poseEstimates[scanNum-1][0],
                                                  poseEstimates[scanNum-1][1],
                                                  poseEstimates[scanNum-1][2],
                                                  poseEstimates[scanNum-1][3],
                                                  poseEstimates[scanNum-1][4],
                                                  poseEstimates[scanNum-1][5]).inverse() *
                                      homogeneous(poseEstimates[scanNum][0],
                                                  poseEstimates[scanNum][1],
                                                  poseEstimates[scanNum][2],
                                                  poseEstimates[scanNum][3],
                                                  poseEstimates[scanNum][4],
                                                  poseEstimates[scanNum][5])));
        }
        
        // --------------------------------------------------------------------
        // STEP 3: UPDATE THE LOCAL MAP
        // --------------------------------------------------------------------
        // transform the current scan to the current pose estimate
        int currentScanSize = ptsSubsampled.rows();
        Eigen::Matrix4d hypothesis = homogeneous(poseEstimates[scanNum][0],
                                                 poseEstimates[scanNum][1],
                                                 poseEstimates[scanNum][2],
                                                 poseEstimates[scanNum][3],
                                                 poseEstimates[scanNum][4],
                                                 poseEstimates[scanNum][5]);
        Eigen::MatrixXd currentScanTransformed = hypothesis * ptsSubsampled.transpose();

        // add the transformed scan to the previous submap
        Eigen::MatrixXd subMapToUpdate(subMap.rows()+ptsSubsampled.rows(),4);
        subMapToUpdate << subMap,currentScanTransformed.transpose();

        std::set<int> allPointsSubMap;
        int counter = 0;
        for (unsigned int i = 0; i < subMapToUpdate.rows(); i++)
        {
            // CAN WE DO THE MAX RANGE HERE?
            allPointsSubMap.insert(counter);
            counter++;
        } 
        Eigen::MatrixXd subMapUpdated = subsample(config.rMap, allPointsSubMap, subMapToUpdate);

        // remove points outside rMax
        Eigen::MatrixXd subMapInMaxRange(subMapUpdated.rows(),subMapUpdated.cols());
        counter = 0;
        for (unsigned int k = 0; k < subMapUpdated.rows(); k++)
        {
            if ((pow(subMapUpdated(k,0)-poseEstimates[scanNum][3],2) + 
                 pow(subMapUpdated(k,1)-poseEstimates[scanNum][4],2) + 
                 pow(subMapUpdated(k,2)-poseEstimates[scanNum][5],2)) < pow(config.maxSensorRange,2))
                {
                    subMapInMaxRange.row(counter) << subMapUpdated(k,0),subMapUpdated(k,1),subMapUpdated(k,2),1;
                    counter++;
                }
        }

        // overwrite the submap for the next registration result
        subMap.resize(counter,4);
        subMap = subMapInMaxRange.topRows(counter);
        // print the progress to the terminal
        // comment printProgress to remove this 
        printProgress((double(scanNum)/numScans));
        
    }
    printf("\n"); // end with a new line character for the progress bar

    // calculate the average time per registration result
    auto stopReg = std::chrono::high_resolution_clock::now();
    auto durationReg = std::chrono::duration_cast<std::chrono::milliseconds>(stopReg-startReg);
    double avgTimePerScan = durationReg.count()/numScans;

    // ------------------------------------------------------------------------
    // OUTPUT RESULTS
    // ------------------------------------------------------------------------
    if (config.verbose)
        std::cout << "avgTimePerScan [ms] = " << avgTimePerScan << ";" << std::endl;

    // output a file with the results in the KITTI format, and a file with the configuration parameters
    writeResults(&config, poseEstimates, config.outputFileName, avgTimePerScan);

    return 0;
}