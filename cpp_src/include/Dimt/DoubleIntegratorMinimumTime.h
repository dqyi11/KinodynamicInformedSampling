#ifndef DOUBLE_INTEGRATOR_MINIMUM_TIME_H_
#define DOUBLE_INTEGRATOR_MINIMUM_TIME_H_

// Standard libarary
#include <iostream>
#include <utility>    // std::pair
#include <limits>     // std::numeric_limits::infinity()
#include <algorithm>  // std::max and std::min
#include <vector>     // std::vector
#include <chrono>
#include <memory>
#include <Eigen/Dense>
#include "Dimt/Params.h"

class DoubleIntegratorImpl
{
public:
    virtual double getMinTime(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2) = 0;

    virtual std::tuple<double, double, double>
    getMinTimeAndIntervals1Dof(const double x1, const double v1,
                               const double x2, const double v2,
                               int dof_index = 0) = 0;

    virtual void interpolate(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2,
                             double t, Eigen::VectorXd& x) = 0;
};

class DoubleIntegratorMinimumTime
{
private:
    std::shared_ptr<DoubleIntegratorImpl> doubleIntegratorImpl_;
public:
    //
    // Constructor
    //
    // @param maxAccelerations Max accelerations of all DOFs
    // @param maxVelocities Velocity limits of all DOFs
    //
    DoubleIntegratorMinimumTime(std::vector<double>& maxAccelerations,
                                std::vector<double>& maxVelocities);

    // This function calculates the maximum time given a set number of joints
    // x = [x_1, x_1_dot,...,x_n,x_n_dot]
    // @param x1 Initial state
    // @param x2 Final state
    // @return T Maximum time
    double getMinTime(const Eigen::VectorXd x1, const Eigen::VectorXd x2) const
    {
        return doubleIntegratorImpl_->getMinTime(x1, x2);
    }


    std::tuple<double, double, double>
    getMinTimeAndIntervals1Dof(const double x1, const double v1,
                               const double x2, const double v2,
                               int dof_index = 0) const
    {
        return doubleIntegratorImpl_->getMinTimeAndIntervals1Dof(x1,v1,
                                                                 x2,v2,
                                                                 dof_index);
    }

    void interpolate(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2,
                     double t, Eigen::VectorXd& x) const
    {
        return doubleIntegratorImpl_->interpolate(x1, x2, t, x);
    }

};

using DIMT = DoubleIntegratorMinimumTime;
using DIMTPtr = std::shared_ptr<DoubleIntegratorMinimumTime>;

#endif // DOUBLE_INTEGRATOR_MINIMUM_TIME_H_
