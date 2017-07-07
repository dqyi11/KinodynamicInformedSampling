#include "DoubleIntegrator.h"
#include "Dimt/DoubleIntegratorMinimumTime.h"


class DoubleIntegratorTobiasImpl : public DoubleIntegratorImpl
{
    using DI = DoubleIntegrator<param.dof>;
public:
    Eigen::VectorXd newX1;
    Eigen::VectorXd newX2;
    Eigen::VectorXd newX;

    DoubleIntegratorTobiasImpl(std::vector<double>& maxAccelerations,
                               std::vector<double>& maxVelocities)
        : maxAccelerations_(maxAccelerations), maxVelocities_(maxVelocities),
          newX1(param.dimensions), newX2(param.dimensions), newX(param.dimensions)
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

    void convertToTobiasFormat(const Eigen::VectorXd& x, Eigen::VectorXd& newX)
    {
        // convert [x1, v1, x2, v2] to [x1, x2, v1, v2]
        for(int i=0;i<param.dof;i++)
        {
            newX[i] = x[2*i];
            newX[param.dof+i] = x[2*i+1];
        }
        return ;
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

    void convertFromTobiasFormat(const Eigen::VectorXd& x, Eigen::VectorXd& newX)
    {
        // convert [x1, x2, v1, v2] to [x1, v1, x2, v2]
        for(int i=0;i<param.dof;i++)
        {
            newX[2*i] = x[i];
            newX[2*i+1] = x[param.dof+i];
        }
        return;
    }


    virtual double getMinTime(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2)
    {
        convertToTobiasFormat(x1, newX1);
        convertToTobiasFormat(x2, newX2);
        return doubleIntegrator_->getMinTime(newX1, newX2);
    }


    virtual double getMinTimeIfSmallerThan(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2,
                                    double timeThreshold)
    {     
        convertToTobiasFormat(x1, newX1);
        convertToTobiasFormat(x2, newX2);

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
        convertToTobiasFormat(x1, newX1);
        convertToTobiasFormat(x2, newX2);
        typename DI::Trajectory traj = doubleIntegrator_->getTrajectory(newX1, newX2);
        newX = traj.getState(t);
        convertFromTobiasFormat(newX, x);
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













