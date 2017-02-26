#include <iostream>
#include <fstream>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>

#include <Sampler/Sampler.h>
#include <ProblemDefinition/ProblemDefinition.h>
#include <Sampler/RejectionSampler.h>
#include <Sampler/MonteCarloSamplers.h>
#include <OmplWrappers/OmplSamplers.h>
#include <OmplWrappers/MyOptimizationObjective.h>
#include <OmplWrappers/MyInformedRRTstar.h>

#include "DimtStateSpace.h"
#include "Dimt/DoubleIntegrator.h"
#include "Dimt/Dimt.h"
#include "Dimt/Params.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

#include <limits>

class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr &si)
        : ob::StateValidityChecker(si)
    { }

    // rectangle obstacles
    bool isValid(const ob::State* state) const
    {
        // const ob::RealVectorStateSpace::StateType* state_rv =
        //     state->as<ob::RealVectorStateSpace::StateType>();
        // for (int i=0; i<param.dimensions; i=i+2)
        // {
        //     if(state_rv->values[i]<-1 || state_rv->values[i]>1)
        //     {
        //         return true;
        //     }
        // }
        // return false;

        return true;
    }
};

ProblemDefinition create_prob_definition(const VectorXd& start_state,
                     const VectorXd& goal_state,
                     const int& dimension,
                     const double& minval,
                     const double& maxval,
                     const double& level_set,
                     const CostFxn& costfxn)
{
    VectorXd state_min(dimension);
    state_min << VectorXd::Constant(dimension, minval);

    VectorXd state_max(dimension);
    state_max << VectorXd::Constant(dimension, maxval);

    return ProblemDefinition(start_state,
                             goal_state,
                             state_min,
                             state_max,
                             level_set,
                             costfxn);
}

ob::OptimizationObjectivePtr get_geom_opt_obj(const ob::SpaceInformationPtr& si,
                        const VectorXd& start_state,
                        const VectorXd& goal_state,
                        const std::shared_ptr<Sampler> sampler,
                        const double& batch_size)
{
    return ob::OptimizationObjectivePtr(new ompl::base::MyOptimizationObjective(si, sampler, batch_size,
        [start_state, goal_state](const VectorXd& state)
        {
            return (start_state - state).norm() + (goal_state - state).norm();
        },
        [](const VectorXd& s1, const VectorXd& s2)
        {
            return (s2 - s1).norm();
        }));
}

template <int dof>
ob::OptimizationObjectivePtr get_dimt_opt_ob(const ob::SpaceInformationPtr &si,
                                             const VectorXd& start_state,
                                             const VectorXd& goal_state,
                                             const std::shared_ptr<Sampler> sampler,
                                             const double& batch_size,
                                             const DoubleIntegrator<dof> &di)
{
    return ob::OptimizationObjectivePtr(new ompl::base::MyOptimizationObjective(si, sampler, batch_size,
        [start_state, goal_state, di](const VectorXd& state)
        {
            // std::cout << "Start state size: " << start_state.size() << std::endl;
            // std::cout << "Goal state size: " << goal_state.size() << std::endl;
            // std::cout << "State size: " << state.size() << std::endl;
            return di.getMinTime(start_state, state) + di.getMinTime(state, goal_state);
        },
        [di](const VectorXd& s1, const VectorXd& s2)
        {
            // std::cout << "Got min time: " << std::endl << std::endl;
            // std::cout << "Start: " << s1 << std::endl;
            // std::cout << "Goal: " << s2 << std::endl;
            return di.getMinTime(s1, s2);
        }));
}

bool MAIN_VERBOSE = true;

void planWithSimpleSetup(void)
{
    ///
    /// Set up the DIMT space
    ///
    Dimt dimt(param.a_max);
    DoubleIntegrator<param.dof>::Vector maxAccelerations, maxVelocities;
    for (unsigned int i = 0; i < param.dof; ++i)
    {
        maxVelocities[i] = 10;
        maxAccelerations[i] = param.a_max;
    }

    DoubleIntegrator<param.dof> double_integrator(maxAccelerations, maxVelocities);

    if(MAIN_VERBOSE) std::cout << "Created the double integrator model!" << std::endl;

    ///
    /// Set up the problem definition
    ///
    const int dimension = param.dimensions; const double minval = -10; const double maxval = 10;
    VectorXd start_state(dimension);
    VectorXd goal_state(dimension);
    start_state << 0.0, 0.0;
    goal_state << 5.0, 5.0;

    if(MAIN_VERBOSE) std::cout << "Start: " << std::endl << start_state << std::endl;
    if(MAIN_VERBOSE) std::cout << "Goal: " << std::endl << goal_state << std::endl;

    ///
    /// Construct the problem definition
    ///
    const double level_set = std::numeric_limits<double>::infinity();
    auto prob = create_prob_definition(start_state, goal_state, dimension, minval, maxval, level_set,
        [start_state, goal_state, double_integrator](const VectorXd& state)
        {
            return double_integrator.getMinTime(start_state, state) +
                   double_integrator.getMinTime(state, goal_state);
        });

    std::cout << "Level set: " << level_set << std::endl;

    ///
    /// Construct the DIMT state space and space information
    ///
    ompl::base::StateSpacePtr
        space(new ompl::base::DimtStateSpace(dimt, double_integrator, param.dimensions));
    ob::RealVectorBounds bounds(param.dimensions);
    bounds.setLow(minval);
    bounds.setHigh(maxval);
    space->as<ompl::base::DimtStateSpace>()->setBounds(bounds);

    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
    si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));
    si->setStateValidityCheckingResolution(0.03); // 3%
    si->setup();

    if(MAIN_VERBOSE) std::cout << "Set up the state space!" << std::endl;

    ///
    /// Get the OMPL start and goal
    ///
    ompl::base::State *start_s = space->allocState();
    ompl::base::State *goal_s = space->allocState();
    for (int i=0; i<param.dimensions; i++)
    {
        start_s->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = start_state[i];
        goal_s->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = goal_state[i];
    }

    ob::ScopedState<ompl::base::RealVectorStateSpace> start(space, start_s);
    ob::ScopedState<ompl::base::RealVectorStateSpace> goal(space, goal_s);

    auto start_state_t = start_s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    auto goal_state_t = goal_s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    std::vector<double> start_state_vector(start_state_t, start_state_t +
                                           sizeof start_state_t / sizeof start_state_t[0]);
    std::vector<double> goal_state_vector(goal_state_t, goal_state_t +
                                           sizeof goal_state_t / sizeof goal_state_t[0]);

    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
    pdef->setStartAndGoalStates(start, goal);

    if(MAIN_VERBOSE) std::cout << "Set up the OMPL problem definition!" << std::endl;

    ///
    /// Construct Sampler
    ///
    double sigma = 5; int max_steps = 20; double alpha = 0.1; double batch_size = 20;
    auto mcmc_s = std::make_shared<MCMCSampler>(prob, alpha, sigma, max_steps);

    if(MAIN_VERBOSE) std::cout << "Set up the MCMC sampler!" << std::endl;

    ///
    /// Get Optimization Objective
    ///
    auto opt = get_dimt_opt_ob(si, start_state, goal_state, mcmc_s, batch_size, double_integrator);
    opt->setCostThreshold(ob::Cost(1.00));
    pdef->setOptimizationObjective(opt);

    if(MAIN_VERBOSE) std::cout << "Created the optimization objection!" << std::endl;

    ///
    /// Set up the planner
    ///
    // ob::PlannerPtr planner(new MyInformedRRTstar(si));
    ob::PlannerPtr planner(new og::InformedRRTstar(si));
    // Set the problem instance for our planner to solve
    planner->setProblemDefinition(pdef);
    planner->setup();

    if(MAIN_VERBOSE) std::cout << "Set up Informed RRT* planner!" << std::endl;

    ///
    /// Run the planner and print
    ///
    ob::PlannerStatus solved = planner->solve(15.0);

    if(MAIN_VERBOSE) std::cout << "Planner solved!" << std::endl;

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        ob::PathPtr path = pdef->getSolutionPath();
        path->print(std::cout);
    }
}

int main(int argc, char const *argv[])
{
    planWithSimpleSetup();
    return 0;
}
