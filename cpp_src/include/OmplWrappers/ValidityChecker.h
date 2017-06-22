#ifndef VALIDITYCHECK_H_
#define VALIDITYCHECK_H_

#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <ompl/base/StateValidityChecker.h>
#include "OmplWrappers/OmplHelpers.h"

class Obstacle
{
public:
    Obstacle() {};
    virtual bool isValid(const ompl::base::State * state) const = 0;
};

class NSphere : public Obstacle
{
public:
    NSphere(unsigned int dimension, Eigen::VectorXd center, double radius)
        : dimension_(dimension), center_(center), radius_(radius)
    {
    }

    bool isValid(const ompl::base::State *state) const;

    unsigned int dimension_;
    Eigen::VectorXd center_;
    double radius_;
    ompl::base::SpaceInformationPtr si_;
};

class Hypercube : public Obstacle
{
public:
    Hypercube(unsigned int dimension, Eigen::VectorXd center, Eigen::VectorXd radius)
          : dimension_(dimension), center_(center), radius_(radius)
    {
    }

    bool isValid(const ompl::base::State *state) const;

    unsigned int dimension_;
    Eigen::VectorXd center_;
    Eigen::VectorXd radius_;
    ompl::base::SpaceInformationPtr si_;
};

class ValidityChecker : public ompl::base::StateValidityChecker
{
public:
    ValidityChecker(const ompl::base::SpaceInformationPtr &si) : ompl::base::StateValidityChecker(si)
    {
    }
    virtual ~ValidityChecker();

    bool addNSphereObstacle(Eigen::VectorXd center, double radius);
    bool addHypercubeObstacle(Eigen::VectorXd center, Eigen::VectorXd radius);
    bool isValid(const ompl::base::State *state) const;

    std::vector<Obstacle*> obstacles_;
};


#endif // VALIDITYCHECK_H_
