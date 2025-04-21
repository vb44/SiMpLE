#include "Register.hpp"

Register::Register(double convergenceTol, double sigma)
    : convTol_(convergenceTol),
      rewardParam_(pow(sigma,-2)/2),
      registrationScore_(0.0) {
}

double Register::getRegistrationScore() const {
    return registrationScore_;
}

column_vector Register::getRegResult() const {
    return regResult_;
}

void Register::registerScan(const std::vector<Eigen::Vector4d> &scan,
                            const NanoflannPointsContainer<double> &kdTreePts) {
    my_kd_tree_t *subMapKdTree = new my_kd_tree_t(3, kdTreePts, {10});
    ObjectiveFunction objFunc = ObjectiveFunction(rewardParam_, scan,
                                                  subMapKdTree);

    // Use the previous estimate and a constant velocity model to set the seed
    // for the registration.
    regResult_ = {seedConstVel_[0], seedConstVel_[1], seedConstVel_[2],
                  seedConstVel_[3], seedConstVel_[4], seedConstVel_[5]};

    // The parameter regResult is replaced with the current pose estimate.
    // Find the best solution to the objective function (Equation 2).
    // The large number (last parameter) is the maximum negative reward
    // required by dlib (never achieved or used).
    registrationScore_ = dlib::find_min_using_approximate_derivatives(
                            dlib::bfgs_search_strategy(),
                            dlib::objective_delta_stop_strategy(convTol_),
                            objFunc, regResult_, -10000000);
    
    // calculate the seed for the next pose estimate using the constant
    // velocity model, Equation 6.
    seedConstVel_ = utils::hom2rpyxyz(utils::homogeneous(regResult_(0), regResult_(1), regResult_(2),
                                                         regResult_(3), regResult_(4), regResult_(5)) *
                                     (utils::homogeneous(prevResult_(0), prevResult_(1), prevResult_(2),
                                                        prevResult_(3), prevResult_(4), prevResult_(5)).inverse() *
                                      utils::homogeneous(regResult_(0), regResult_(1), regResult_(2),
                                                         regResult_(3), regResult_(4), regResult_(5))));
    
    // Save the current registration result for the next constant velocity model calculation.
    prevResult_ = regResult_;
    
    delete subMapKdTree;
}