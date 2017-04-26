#include "Dimt/DoubleIntegrator.h"
#include "Dimt/DoubleIntegratorMinimumTime.h"

DoubleIntegratorMinimumTime::DoubleIntegratorMinimumTime(unsigned int numDOF,
                                                         std::vector<double>& maxAccelerations,
                                                         std::vector<double> maxVelocities)
    : numDOF_(numDOF), maxAccelerations_(maxAccelerations), maxVelocities_(maxVelocities)
{
    DoubleIntegrator::Vector maxA;
    DoubleIntegrator::Vector maxV;
    for(unsigned int i=0; i<numDOF_;i++)
    {
        maxA[i] = maxAccelerations_[i];
        maxV[i] = maxVelocities_[i];
    }
    doubleIntegrator_ = std::make_shared<DoubleIntegrator<numDOF>>(maxA, maxV);
}

bool DoubleIntegratorMinimumTime::isValidStates(const Eigen::VectorXd x1,
                                                const Eigen::VectorXd x2,
                                                const Eigen::VectorXd xi) const
{
    return (x1.size() == x2.size() and x1.size() == xi.size()) and
           (x1.size() != 0 and x2.size() != 0 and xi.size() != 0) and (x1.size() % 2 == 0);
}

double DoubleIntegratorMinimumTime::getMinTime(const Eigen::VectorXd x1,
                                               const Eigen::VectorXd x2,
                                               const Eigen::VectorXd xi) const
{
    // Assert that the start, goal, and x1 are valid size (same and != 0)
    if (!isValidStates(x1, x2, xi))
    {
        std::string errorMsg = "Invalid Arguments to get_min_time";
        throw std::invalid_argument(errorMsg);
    }

    return getMinTime(x1, xi) + getMinTime(xi, x2);
}

double DoubleIntegratorMinimumTime::getMinTime(const Eigen::VectorXd x1,
                                               const Eigen::VectorXd x2) const
{

}

 // returns the minimum time between 3 points ([x1,y1] -> [xi,yi] -> [x2,y2])
// in the state space
// returns -1 if the solution does not exist
double DoubleIntegratorMinimumTime::getMinTime1Dof(const double x1, const double v1,
                                                   const double x2, const double v2,
                                                   const double xi, const double vi) const
{
    double T1 = getMinTime1Dof(x1, v1, xi, vi);
    double T2 = getMinTime1Dof(xi, vi, x2, v2);
    if (T1 < 0 || T2 < 0)
        return -1;
    return T1 + T2;
}

// returns the minimum time between 2 points ([x1,y1] and [x2,y2]) in the
// state space
// returns -1 if the solution does not exist
double DoubleIntegratorMinimumTime::getMinTime1Dof(const double x1, const double v1,
                                                   const double x2, const double v2) const
{

}

std::pair<double, double>
DoubleIntegratorMinimumTime::getInfeasibleTimeInterval(const double x1, const double v1,
                                                       const double x2, const double v2) const
{

}
