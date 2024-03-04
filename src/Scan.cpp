#include "Scan.hpp"

Scan::Scan(ConfigParser &config)
{
    subsampleRadius_ = config.rNew;
    maxSensorRange_ = config.maxSensorRange;
    minSensorRange_ = config.minSensorRange;
    kitti_ = config.kitti;
}

Scan::~Scan()
{
}

void Scan::readScan(std::string fileName)
{
    ptCloud.clear();
    allPoints_.clear();
    std::ifstream file(fileName, std::ios::in | std::ios::binary);
    // if (!file) return EXIT_FAILURE;

    float item;
    std::vector<double> ptsFromFile;
    int counter = 0;
    
    while (file.read((char*)&item, sizeof(item)))
        ptsFromFile.push_back(item);

    unsigned int numPts = ptsFromFile.size() / 4; // .bin format
 
    for (unsigned int i = 0; i < ptsFromFile.size(); i+=4)
    {
        // Save the pt if it is within the maximum and mininmum sensor ranges.
        double normSquared = pow(ptsFromFile[i], 2) + 
                             pow(ptsFromFile[i+1], 2) + 
                             pow(ptsFromFile[i+2], 2); 
        if ((normSquared > pow(minSensorRange_, 2)) &&
            (normSquared < pow(maxSensorRange_, 2)))
        {
            ptCloud.push_back({ptsFromFile[i],
                               ptsFromFile[i+1],
                               ptsFromFile[i+2],
                               1});
            
            // Save the pt index for subsampling.
            allPoints_.insert(counter); 
            counter++;
        }
    }

    // Process the scan.
    processPointCloud_();
}

void Scan::processPointCloud_()
{
    // Check if the scan needs to be corrected.
    // Apply the calibration factor as explained in IMLS-SLAM, CT-ICP,
    // and KISS-ICP.
    if (kitti_) correctKittiScan_();

    // Subsample the point cloud.
    subsample_(ptCloud, subsampleRadius_);
}

void Scan::correctKittiScan_() {

    // Adapted from KISS-ICP opensource code to correct the KITTI scans.
    constexpr double VERTICAL_ANGLE_OFFSET = (0.205 * M_PI) / 180.0;
    
    tbb::parallel_for(
    tbb::blocked_range<int>(0, ptCloud.size()),
    [&](tbb::blocked_range<int> r)
    {
        for (unsigned int i = r.begin(); i < r.end(); i++)
        {
            Eigen::Vector3d pt;
            Eigen::Vector3d ptCorrected;
            pt << ptCloud[i](0), ptCloud[i][1], ptCloud[i][2];
            const Eigen::Vector3d rotationVector = 
                                  pt.cross(Eigen::Vector3d(0.0, 0.0, 1.0));
            ptCorrected = Eigen::AngleAxisd(VERTICAL_ANGLE_OFFSET,
                                            rotationVector.normalized()) * pt;
            ptCloud[i] = {ptCorrected(0), ptCorrected(1), ptCorrected(2), 1};
        }
    });
}

void Scan::subsample_(std::vector<Eigen::Vector4d> &pts,
                      double subsampleRadius)
{
    std::vector<Eigen::Vector4d> ptsSubsampled;
    
    // Nanoflann uses the squared radius.
    subsampleRadius = pow(subsampleRadius, 2);
    convertToPointCloudKdTree_(pts);

    // Create a Kd tree (dimension, scan, max leaf).
    my_kd_tree_t *scanKdTree = new my_kd_tree_t(3, pcForKdTree_,{10});
    unsigned int counter = 0;

    // Subsample radially.
    for (unsigned int i : allPoints_)
    {
        std::vector<nanoflann::ResultItem<uint32_t, double>> ret_matches;
        const double query_pt[3] = {pts[i][0], pts[i][1], pts[i][2]};
        const size_t nMatches = scanKdTree->radiusSearch(&query_pt[0],
                                subsampleRadius, ret_matches);
        for (unsigned int j = 0; j < nMatches; j++)
        {
            if (i != ret_matches[j].first)
                allPoints_.erase(ret_matches[j].first);
        }
        ptsSubsampled.push_back({pts[i][0], pts[i][1], pts[i][2], 1});
    }
    delete scanKdTree; // free memory

    // Overwrite the scan with the subsampled points.
    pts = ptsSubsampled;
}

void Scan::convertToPointCloudKdTree_(std::vector<Eigen::Vector4d> &pts)
{
    size_t pcLength = pts.size();
    pcForKdTree_.pts.clear();
    pcForKdTree_.pts.resize(pcLength);

    tbb::parallel_for(
    tbb::blocked_range<int>(0, pcLength),
    [&](tbb::blocked_range<int> r)
    { 
        for (size_t i = r.begin(); i < r.end(); i++)
        {
            pcForKdTree_.pts[i].x = pts[i](0);
            pcForKdTree_.pts[i].y = pts[i](1);
            pcForKdTree_.pts[i].z = pts[i](2);        
        } 
    });
}