#include "objectiveFunction.h"

ObjectiveFunction::ObjectiveFunction(Eigen::Matrix3d uncertainty, Eigen::MatrixXd scan,
                    PointCloud<double> subMap,  my_kd_tree_t *sourceScanKdTree) 
{
    uncertainty_ = uncertainty;
    scan_ = scan;
    sourceScan_ = subMap;
    scanSize_ = scan.rows();
    subMapKdTree_ = sourceScanKdTree;
}

double ObjectiveFunction::operator()(const column_vector& m) const
{
    // transformation hypothesis from the new scan to the local map
    Eigen::Matrix4d hypothesis = homogeneous(m(0),m(1),m(2),m(3),m(4),m(5));

    
    // transform the new scan using the transformation hypothesis            
    Eigen::MatrixXd scanTransformed_ = hypothesis * scan_.transpose();
    
    // set the score to zero
    double score = 0;
    std::vector<double> scores(scanTransformed_.cols(),0.0);
    
    // calculate the hypothesis reward 
    // loop through each transformed target scan and find the closest point in the source scan
    tbb::parallel_for(
    tbb::blocked_range<int>(0,scanTransformed_.cols()),
    [&](tbb::blocked_range<int> r)
    {
        for (unsigned int i = r.begin(); i < r.end(); i++)
        {

            size_t numResults = 1;
            uint32_t retIndex;
            double outDistSqr;
            double query_pt[3];
            Eigen::RowVector3d e;
            query_pt[0] = scanTransformed_.coeffRef(0,i); 
            query_pt[1] = scanTransformed_.coeffRef(1,i);
            query_pt[2] = scanTransformed_.coeffRef(2,i);
            
            // search for the closest point
            subMapKdTree_->knnSearch(&query_pt[0],numResults,&retIndex,&outDistSqr);

            // score the difference between the current and nearest point
            e << query_pt[0]-sourceScan_.pts[retIndex].x,
                 query_pt[1]-sourceScan_.pts[retIndex].y,
                 query_pt[2]-sourceScan_.pts[retIndex].z;
            scores[i] = exp((-(e)*uncertainty_*(e).transpose() / 2)(0,0));
        }
    });

    // sum the scores
    for (auto& n: scores)
        score -= n;
        
    return score;
}