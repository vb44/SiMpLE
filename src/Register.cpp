#include "Register.hpp"

Register::Register(ConfigParser &config)
{
    convTol_ = config.convergenceTol;
    rewardParam_ = pow(config.sigma,-2)/2;
}

Register::~Register()
{
}

void Register::registerScan(std::vector<Eigen::Vector4d> &scan,
                            PointCloud<double> &kdTreePts)
{
    my_kd_tree_t *subMapKdTree = new my_kd_tree_t(3, kdTreePts, {10});
    ObjectiveFunction objFunc = ObjectiveFunction(rewardParam_, scan,
                                                  subMapKdTree);

    // Use the previous estimate and a constant velocity model to set the seed
    // for the registration.
    regResult = {seedConstVel_[0], seedConstVel_[1], seedConstVel_[2],
                 seedConstVel_[3], seedConstVel_[4], seedConstVel_[5]};

    // The parameter regResult is replaced with the current pose estimate.
    // Find the best solution to the objective function (Equation 2).
    // The large number (last parameter) is the maximum negative reward
    // required by dlib (never achieved or used).
    registrationScore = dlib::find_min_using_approximate_derivatives(
                            dlib::bfgs_search_strategy(),
                            dlib::objective_delta_stop_strategy(convTol_),
                            objFunc, regResult, -10000000);
    
    // calculate the seed for the next pose estimate using the constant
    // velocity model, Equation 6.
    seedConstVel_ = hom2rpyxyz(homogeneous(regResult(0), regResult(1),
                                           regResult(2), regResult(3),
                                           regResult(4), regResult(5)) *
                    (homogeneous(prevResult_(0), prevResult_(1),
                                 prevResult_(2), prevResult_(3),
                                 prevResult_(4), prevResult_(5)).inverse() *
                     homogeneous(regResult(0), regResult(1),
                                 regResult(2), regResult(3),
                                 regResult(4), regResult(5))));
    
    // Save the current registration result for the next constant velocity
    // model calculation.
    prevResult_ = regResult;
    
    delete subMapKdTree;
}