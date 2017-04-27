#include "DoubleIntegrator.h"
#include "Dimt/DoubleIntegratorMinimumTime.h"


class DoubleIntegratorTubiasImpl : public DoubleIntegratorImpl
{
    using DI = DoubleIntegrator<param.dof>;
public:
    DoubleIntegratorTubiasImpl(std::vector<double>& maxAccelerations,
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

    virtual double getMinTime(const Eigen::VectorXd x1, const Eigen::VectorXd x2)
    {
        return doubleIntegrator_->getMinTime(x1, x2);
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
                     double t, Eigen::VectorXd& x)     {
        typename DI::Trajectory traj = doubleIntegrator_->getTrajectory(x1, x2);
        x = traj.getState(t);
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
    doubleIntegratorImpl_ = std::make_shared<DoubleIntegratorTubiasImpl>(maxAccelerations,
                                                                         maxVelocities);
}













