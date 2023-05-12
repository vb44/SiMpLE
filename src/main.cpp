#include "objectiveFunction.h"

int main(int argc, char* argv[])
{
    // ------------------------------------------------------------------------
    // ARGUMENT PARSING
    // ------------------------------------------------------------------------
    params config;
    int err = parseArgs(&config, argc, argv);
    if (err)
        exit(1);
    
    Eigen::Matrix3d uncertainty;
    uncertainty << pow(config.sigma,-2),0,0,
                   0,pow(config.sigma,-2),0,
                   0,0,pow(config.sigma,-2);

    // ------------------------------------------------------------------------
    // LOAD THE SCANS
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
    bool addPoint = true;
    scan subMap;
    scan targetScan;    
    PointCloud<double> subMapNF;

    std::vector<std::vector<double> > poseEstimates(numScans,std::vector<double>(7));
    std::vector<double> startingPointPrev = {0.0,0.0,0.0,0.0,0.0,0.0};

    // start the timer
    auto startReg = std::chrono::high_resolution_clock::now();

    // loop over all scans and save the point clouds
    for (unsigned int scanNum = 0; scanNum < numScans; scanNum++)
    {
        // --------------------------------------------------------------------
        // READ THE POINT CLOUD
        // --------------------------------------------------------------------
        std::ifstream file(scanFiles[scanNum], std::ios::in | std::ios::binary);
        if (!file) return EXIT_FAILURE;

        float item;
        scan pts;
        std::vector<double> ptsFromFile;
        std::set<int> allPoints;
        int counter = 0;
        
        while (file.read((char*)&item, sizeof(item)))
            ptsFromFile.push_back(item);

        for (unsigned int i = 0; i < ptsFromFile.size(); i+=4)
        {
            if (pow(ptsFromFile[i],2)+pow(ptsFromFile[i+1],2) > pow(config.minSensorRange,2))
            {
                pts.x.push_back(ptsFromFile[i]);
                pts.y.push_back(ptsFromFile[i+1]);
                pts.z.push_back(ptsFromFile[i+2]);

                allPoints.insert(counter);
                counter++;
            }
        }

        // --------------------------------------------------------------------
        // STEP 1: SUBSAMPLE THE INPUT POINT CLOUD
        // --------------------------------------------------------------------
        targetScan = subsample(config.rNew, allPoints, pts);
        
        // --------------------------------------------------------------------
        // STEP 2: INPUT POINT CLOUD TO LOCAL MAP REGISTRATION
        // --------------------------------------------------------------------
        if (scanNum > 0)
        {
            
            // construct a kd tree for the registration process
            // (dimension, scan, max leaf)
            convertToPointCloud3D(subMapNF,subMap);
            my_kd_tree_t *subMapKdTree = new my_kd_tree_t(3,subMapNF,{10});

            // find the best solution to the objective function
            ObjectiveFunction objFuncFine = ObjectiveFunction(uncertainty,targetScan,
                                                              subMapNF, subMapKdTree);

            // use the previous pose estimate as the seed
            column_vector startingPoint = {startingPointPrev[0],
                                           startingPointPrev[1],
                                           startingPointPrev[2],
                                           startingPointPrev[3],
                                           startingPointPrev[4],
                                           startingPointPrev[5]};
        
            double regScoreFine = dlib::find_min_using_approximate_derivatives(
                                        dlib::bfgs_search_strategy(),
                                        dlib::objective_delta_stop_strategy(config.convergenceTol),
                                        objFuncFine, startingPoint, -10000000);
            delete subMapKdTree; // free memory

            // save the results
            std::vector<double> res = {startingPoint(0),
                                       startingPoint(1),
                                       startingPoint(2),
                                       startingPoint(3),
                                       startingPoint(4),
                                       startingPoint(5),
                                       regScoreFine,
                                       double(subMap.x.size())};
            
            // save the results
            poseEstimates[scanNum] = res;

            // calculate the seed for the next pose estimate using the constant velocity mdoel
            startingPointPrev = hom2rpyxyz(homogeneous(poseEstimates[scanNum][0],
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
        // transform the current scan to the pose estimates
        auto startLocalMapUpdate = std::chrono::high_resolution_clock::now();
        
        Eigen::MatrixXd currentScanMultiply;
        int currentScanSize = targetScan.x.size();

        currentScanMultiply.resize(4,currentScanSize);
        currentScanMultiply.row(0) = Eigen::Map<Eigen::RowVectorXd>(targetScan.x.data(),currentScanSize);
        currentScanMultiply.row(1) = Eigen::Map<Eigen::RowVectorXd>(targetScan.y.data(),currentScanSize);
        currentScanMultiply.row(2) = Eigen::Map<Eigen::RowVectorXd>(targetScan.z.data(),currentScanSize);
        currentScanMultiply.row(3) = Eigen::VectorXd::Ones(currentScanSize);

        Eigen::Matrix4d hypothesis = homogeneous(poseEstimates[scanNum][0],
                                                 poseEstimates[scanNum][1],
                                                 poseEstimates[scanNum][2],
                                                 poseEstimates[scanNum][3],
                                                 poseEstimates[scanNum][4],
                                                 poseEstimates[scanNum][5]);

        scan currentScanTransformedScan;
        currentScanTransformedScan.x.resize(currentScanSize);
        currentScanTransformedScan.y.resize(currentScanSize);
        currentScanTransformedScan.z.resize(currentScanSize);
        Eigen::MatrixXd currentScanTransformed = hypothesis * currentScanMultiply;
        Eigen::VectorXd::Map(&currentScanTransformedScan.x[0],currentScanSize) = currentScanTransformed.row(0);
        Eigen::VectorXd::Map(&currentScanTransformedScan.y[0],currentScanSize) = currentScanTransformed.row(1);
        Eigen::VectorXd::Map(&currentScanTransformedScan.z[0],currentScanSize) = currentScanTransformed.row(2);

        // add the scan to the map
        // loop through every point in the input scan
        for (unsigned int j = 0; j < currentScanSize; j++)
        {
            std::vector<double> point = {currentScanTransformedScan.x[j],
                                         currentScanTransformedScan.y[j],
                                         currentScanTransformedScan.z[j]};
            for (unsigned int k = 0; k < subMap.x.size(); k++)
            {
                addPoint = true;
                if (sqrt(pow(point[0]-subMap.x[k],2) + 
                     pow(point[1]-subMap.y[k],2) + 
                     pow(point[2]-subMap.z[k],2)) < config.rMap)
                    {        
                        addPoint = false;
                        break;
                    }
            }

            if (addPoint)
            {
                subMap.x.push_back(point[0]);
                subMap.y.push_back(point[1]);
                subMap.z.push_back(point[2]);
            }
        }

        // remove points outside the maximum laser range
        scan subMapNew;
        
        for (unsigned int k = 0; k < subMap.x.size(); k++)
        {
            if (pow(subMap.x[k]-poseEstimates[scanNum][3],2) + 
                pow(subMap.y[k]-poseEstimates[scanNum][4],2) < pow(config.maxSensorRange,2))
                {
                    subMapNew.x.push_back(subMap.x[k]);
                    subMapNew.y.push_back(subMap.y[k]);
                    subMapNew.z.push_back(subMap.z[k]);
                }
        }
 
        subMap = subMapNew;
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