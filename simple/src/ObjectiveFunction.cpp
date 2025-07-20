#include "ObjectiveFunction.hpp"

ObjectiveFunction::ObjectiveFunction(double rewardParam_, const std::vector<Eigen::Vector4d> &scan,
                                     my_kd_tree_t* subMapKdTree) :
    rewardParam_(rewardParam_),
    scan_(scan),
    scanSize_(scan.size()),
    subMapKdTree_(subMapKdTree) {
}

double ObjectiveFunction::operator()(const column_vector& m) const {
    Eigen::Matrix4d hypothesis = utils::homogeneous(m(0), m(1), m(2), m(3), m(4), m(5));
    size_t n = scan_.size();

    double score = tbb::parallel_reduce(
        tbb::blocked_range<size_t>(0, n),
        0.0,
        [&](const tbb::blocked_range<size_t>& r, double local_sum) -> double {
            for (size_t i = r.begin(); i < r.end(); ++i) {
                // Transform point as 3D vector
                // Eigen::Vector4d ptTf = hypothesis*scan_[i];
                Eigen::Vector3d pt = scan_[i].head<3>();
                Eigen::Vector3d ptTf = hypothesis.block<3, 3>(0, 0) * pt + hypothesis.block<3, 1>(0, 3);
                double query_pt[3] = {ptTf(0), ptTf(1), ptTf(2)};
                uint32_t retIndex;
                double outDistSqr;
                subMapKdTree_->knnSearch(query_pt, 1, &retIndex, &outDistSqr);

                local_sum -= std::exp(-outDistSqr * rewardParam_);
            }
            return local_sum;
        },
        std::plus<double>()
    );
    return score;
}