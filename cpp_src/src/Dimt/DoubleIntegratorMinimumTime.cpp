#include "DoubleIntegrator.h"
#include "Dimt/DoubleIntegratorMinimumTime.h"


class DoubleIntegratorTobiasImpl : public DoubleIntegratorImpl
{
    using DI = DoubleIntegrator<param.dof>;
public:
    DoubleIntegratorTobiasImpl(std::vector<double>& maxAccelerations,
                               std::vector<double>& maxVelocities)
        : maxAccelerations_(maxAccelerations), maxVelocities_(maxVelocities)
    {
        DI::Vector maxA;
        DI::Vector maxV;
        for(int i=0; i<param.dof; ++i)
        {
            maxA[i] = maxAccelerations_[i];
            maxV[i] = maxVelocities_[i];
        }
        doubleIntegrator_ = std::make_shared< DI > (maxA, maxV);
    }

    Eigen::VectorXd convertToTobiasFormat(const Eigen::VectorXd& x)
    {
        // convert [x1, v1, x2, v2] to [x1, x2, v1, v2]
        Eigen::VectorXd newX(param.dimensions);
        for(int i=0;i<param.dof;i++)
        {
            newX[i] = x[2*i];
            newX[param.dof+i] = x[2*i+1];
        }
        return newX;
    }

    Eigen::VectorXd convertFromTobiasFormat(const Eigen::VectorXd& x)
    {
        // convert [x1, x2, v1, v2] to [x1, v1, x2, v2]
        Eigen::VectorXd newX(param.dimensions);
        for(int i=0;i<param.dof;i++)
        {
            newX[2*i] = x[i];
            newX[2*i+1] = x[param.dof+i];
        }
        return newX;
    }

    virtual double getMinTime(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2)
    {
        Eigen::VectorXd newX1 = convertToTobiasFormat(x1);
        Eigen::VectorXd newX2 = convertToTobiasFormat(x2);
        return doubleIntegrator_->getMinTime(newX1, newX2);
    }


    virtual double getMinTimeIfSmallerThan(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2,
                                    double timeThreshold)
    {
        Eigen::VectorXd newX1 = convertToTobiasFormat(x1);
        Eigen::VectorXd newX2 = convertToTobiasFormat(x2);
        return doubleIntegrator_->getMinTimeIfSmallerThan(newX1, newX2, timeThreshold);
    }

    virtual std::tuple<double, double, double>
    getMinTimeAndIntervals1Dof(const double x1, const double v1,
                               const double x2, const double v2,
                               int dof_index = 0)

    {
        double a1(0);
        double minTime =  doubleIntegrator_->getMinTime1D(v1, v2, x2-x1,
                                               maxAccelerations_[dof_index],
                                               maxVelocities_[dof_index],
                                               std::numeric_limits<double>::max(),
                                               a1);

        std::pair<double, double> infeasibleInterval =
                doubleIntegrator_->getInfeasibleInterval(v1, v2, x2-x1, a1,
                                                         maxAccelerations_[dof_index],
                                                         maxVelocities_[dof_index]);

        return std::make_tuple(minTime, infeasibleInterval.first, infeasibleInterval.second);
    }

    virtual void interpolate(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2,
                             double t, Eigen::VectorXd& x)
    {
        Eigen::VectorXd newX1 = convertToTobiasFormat(x1);
        Eigen::VectorXd newX2 = convertToTobiasFormat(x2);
        typename DI::Trajectory traj = doubleIntegrator_->getTrajectory(newX1, newX2);
        Eigen::VectorXd newX = traj.getState(t);
        x = convertFromTobiasFormat(newX);
        return;
    }
protected:
    std::vector<double> maxAccelerations_;
    std::vector<double> maxVelocities_;
    std::shared_ptr< DI > doubleIntegrator_;
};


DoubleIntegratorMinimumTime::DoubleIntegratorMinimumTime(std::vector<double>& maxAccelerations,
                                                         std::vector<double>& maxVelocities)
{
    doubleIntegratorImpl_ = std::make_shared<DoubleIntegratorTobiasImpl>(maxAccelerations,
                                                                         maxVelocities);
}













