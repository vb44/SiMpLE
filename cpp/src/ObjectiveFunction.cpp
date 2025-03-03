#include "ObjectiveFunction.hpp"

ObjectiveFunction::ObjectiveFunction(double rewardParam_,
                                     const std::vector<Eigen::Vector4d> &scan,
                                     my_kd_tree_t* subMapKdTree) :
                    rewardParam_(rewardParam_),
                    scan_(scan),
                    scanSize_(scan.size()),
                    subMapKdTree_(subMapKdTree)
{
}

double ObjectiveFunction::operator()(const column_vector& m) const
{
    // Transformation hypothesis from the new scan to the existing map.
    Eigen::Matrix4d hypothesis = utils::homogeneous(m(0), m(1), m(2),
                                                    m(3), m(4), m(5));
    
    // Set the registration scores to zero.
    std::vector<double> scores(scan_.size(), 0.0);
    size_t numResults = 1;

    // Calculate the hypothesis reward.
    // Loop through each transformed target scan and find the closest point
    // in the source scan.
    tbb::parallel_for(
    tbb::blocked_range<int>(0, scan_.size()),
    [&](tbb::blocked_range<int> r)
    {
        for (auto i = r.begin(); i < r.end(); i++)
        {
            // Transform the point by the hypothesis.
            Eigen::Vector4d ptTf = hypothesis*scan_[i];
            
            // Search for the closest point.
            uint32_t retIndex;
            double outDistSqr;
            double query_pt[] = {ptTf(0), ptTf(1), ptTf(2)}; 
            subMapKdTree_->knnSearch(&query_pt[0], numResults, &retIndex,
                                     &outDistSqr);

            // Score the difference between the current and nearest point,
            // Equation 5.
            scores[i] = exp(-outDistSqr * rewardParam_);
        }
    });

    // Deterministic results, sum the scores.
    double score = 0;
    for (auto& n: scores)
    {
        score -= n;
    }
    return  score;
}