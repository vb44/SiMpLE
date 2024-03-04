#include "ObjectiveFunction.hpp"

ObjectiveFunction::ObjectiveFunction(double sigma,
                                     std::vector<Eigen::Vector4d> &scan,
                                     my_kd_tree_t *sourceScanKdTree) 
{
    sigma_ = sigma;
    scan_ = scan;
    scanSize_ = scan.size();
    subMapKdTree_ = sourceScanKdTree;
}

ObjectiveFunction::~ObjectiveFunction()
{
}

double ObjectiveFunction::operator()(const column_vector& m) const
{
    // Transformation hypothesis from the new scan to the existing map.
    Eigen::Matrix4d hypothesis = homogeneous(m(0), m(1), m(2),
                                             m(3), m(4), m(5));
 
    // Transform the new scan using the transformation hypothesis.
    std::vector<Eigen::Vector4d> scanTf_(scan_.size());
    
    // Set the registration score to zero.
    double score = 0;
    std::vector<double> scores(scanTf_.size(), 0.0);
    size_t numResults = 1;

    // Calculate the hypothesis reward.
    // Loop through each transformed target scan and find the closest point
    // in the source scan.
    tbb::parallel_for(
    tbb::blocked_range<int>(0, scan_.size()),
    [&](tbb::blocked_range<int> r)
    {
        for (unsigned int i = r.begin(); i < r.end(); i++)
        {
            uint32_t retIndex;
            double outDistSqr;
            double query_pt[3];
            Eigen::Vector4d ptTf = hypothesis*scan_[i];
            query_pt[0] = ptTf(0); 
            query_pt[1] = ptTf(1);
            query_pt[2] = ptTf(2);
            
            // Search for the closest point.
            subMapKdTree_->knnSearch(&query_pt[0], numResults, &retIndex,
                                     &outDistSqr);

            // Score the difference between the current and nearest point,
            // Equation 5.
            scores[i] = exp(-outDistSqr * sigma_);
        }
    });

    // Deterministic results, sum the scores.
    for (auto& n: scores)
        score -= n;
        
    return score;
}