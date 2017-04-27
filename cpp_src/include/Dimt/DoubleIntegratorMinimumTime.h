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
#include "Dimt/Interval.h"
#include "Dimt/DoubleIntegrator.h"

class DoubleIntegratorMinimumTime
{
public:
    unsigned int numDOF_;
    std::vector<double> maxAccelerations_;
    std::vector<double> maxVelocities_;
    DoubleIntegratorPtr<int> doubleIntegrator_;
    ///
    /// Constructor
    ///
    /// @param numDOF Number of DOF
    /// @param maxAccelerations Max accelerations of all DOFs
    /// @param maxVelocities Velocity limits of all DOFs
    ///
    DoubleIntegratorMinimumTime(unsigned int numDOF_,
                                std::vector<double>& maxAccelerations,
                                std::vector<double> maxVelocities);

    bool isValidStates(const Eigen::VectorXd x1, const Eigen::VectorXd x2,
                       const Eigen::VectorXd xi) const;

    // This function calculates the maximum time given a set number of joints
    // x = [x_1, x_1_dot,...,x_n,x_n_dot]
    // @param x1 Initial state
    // @param x2 Final state
    // @param xi Intermediate state
    // @return T Maximum time
    double getMinTime(const Eigen::VectorXd x1, const Eigen::VectorXd x2,
                      const Eigen::VectorXd xi) const;

    // This function calculates the maximum time given a set number of joints
    // x = [x_1, x_1_dot,...,x_n,x_n_dot]
    // @param x1 Initial state
    // @param x2 Final state
    // @return T Maximum time
    double getMinTime(const Eigen::VectorXd x1, const Eigen::VectorXd x2) const;

     // returns the minimum time between 3 points ([x1,y1] -> [xi,yi] -> [x2,y2])
    // in the state space
    // returns -1 if the solution does not exist
    double getMinTime1Dof(const double x1, const double v1,
                          const double x2, const double v2,
                          const double xi, const double vi) const;

    // returns the minimum time between 2 points ([x1,y1] and [x2,y2]) in the
    // state space
    // returns -1 if the solution does not exist
    double getMinTime1Dof(const double x1, const double v1,
                          const double x2, const double v2) const;

    ///
    /// Function to get the infeasible time_interval for one DOF
    ///
    /// @param x1 start position
    /// @param v1 start velocity
    /// @param x2 end position
    /// @param v2 end velocity
    /// return A pair of values (min_time, max_time) is infinity if not in region
    ///
    std::pair<double, double>
    getInfeasibleTimeInterval(const double x1, const double v1,
                              const double x2, const double v2) const;

};

#endif // DOUBLE_INTEGRATOR_MINIMUM_TIME_H_
