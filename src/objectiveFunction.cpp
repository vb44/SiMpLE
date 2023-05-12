#include "objectiveFunction.h"

// constructor to initialise the member variables
ObjectiveFunction::ObjectiveFunction(Eigen::Matrix3d uncertainty, scan targetScanInput,
                    PointCloud<double> subMap,  my_kd_tree_t *sourceScanKdTree_) 
{
    uncertainty_ = uncertainty;
    targetScanInput_ = targetScanInput;
    sourceScan_ = subMap;
    targetScanSize_ = targetScanInput.x.size();
    subMapKdTree = sourceScanKdTree_;

    targetScanMultiply_.resize(4,targetScanSize_);
    targetScanMultiply_.row(0) = Eigen::Map<Eigen::RowVectorXd>(targetScanInput.x.data(),targetScanSize_);
    targetScanMultiply_.row(1) = Eigen::Map<Eigen::RowVectorXd>(targetScanInput.y.data(),targetScanSize_);
    targetScanMultiply_.row(2) = Eigen::Map<Eigen::RowVectorXd>(targetScanInput.z.data(),targetScanSize_);
    targetScanMultiply_.row(3) = Eigen::VectorXd::Ones(targetScanSize_);
}

// overload function call operator
double ObjectiveFunction::operator()(const column_vector& m) const
{

    // transformation hypothesis from the new scan to the local map
    Eigen::Matrix4d hypothesis = homogeneous(m(0),m(1),m(2),m(3),m(4),m(5));
    
    // transform the new scan using the transformation hypothesis            
    scan targetScanTransformedInput_;
    targetScanTransformedInput_.x.resize(targetScanSize_);
    targetScanTransformedInput_.y.resize(targetScanSize_);
    targetScanTransformedInput_.z.resize(targetScanSize_);
    Eigen::MatrixXd targetScanTransformed_ = hypothesis * targetScanMultiply_;
    Eigen::VectorXd::Map(&targetScanTransformedInput_.x[0],targetScanSize_) = targetScanTransformed_.row(0);
    Eigen::VectorXd::Map(&targetScanTransformedInput_.y[0],targetScanSize_) = targetScanTransformed_.row(1);
    Eigen::VectorXd::Map(&targetScanTransformedInput_.z[0],targetScanSize_) = targetScanTransformed_.row(2);
    
    double score = 0;
    std::vector<double> scores(targetScanTransformedInput_.x.size(),0.0);
    
    // calculate the hypothesis reward 
    // loop through each transformed target scan and find the closest point in the source scan
    tbb::parallel_for(
    tbb::blocked_range<int>(0,targetScanTransformedInput_.x.size()),
    [&](tbb::blocked_range<int> r)
    {
        for (unsigned int i = r.begin(); i < r.end(); ++i)
        {
            size_t numResults = 1;
            uint32_t retIndex;
            double outDistSqr;
            double query_pt[3];
            Eigen::RowVector3d pointTaregtScan;
            Eigen::RowVector3d nearestPointSourceScan;
            std::vector<double> nearestPoints;

            query_pt[0] = targetScanTransformedInput_.x[i];
            query_pt[1] = targetScanTransformedInput_.y[i];
            query_pt[2] = targetScanTransformedInput_.z[i];
            
            subMapKdTree->knnSearch(&query_pt[0],numResults,&retIndex,&outDistSqr);

            // score the points
            pointTaregtScan << query_pt[0], query_pt[1], query_pt[2];
            nearestPointSourceScan << sourceScan_.pts[retIndex].x,
                                        sourceScan_.pts[retIndex].y,
                                        sourceScan_.pts[retIndex].z;

            scores[i] = exp((-((pointTaregtScan-nearestPointSourceScan)) * 
                                        uncertainty_ * 
                                    ((pointTaregtScan-nearestPointSourceScan)).transpose() / 2)(0,0));
        }
    });

    for (auto& n: scores)
        score -= n;

    // for (int i = 0; i < targetScanTransformedInput_.x.size(); i++)
    // {
    //     score -= scores[i];
    // }


    return score;
}