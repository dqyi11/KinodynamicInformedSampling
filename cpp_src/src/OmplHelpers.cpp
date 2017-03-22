#include <OmplWrappers/OmplHelpers.h>

ompl::base::OptimizationObjectivePtr get_geom_opt_obj(const ompl::base::SpaceInformationPtr& si,
                                                      const VectorXd& start_state,
                                                      const VectorXd& goal_state,
                                                      const std::shared_ptr<Sampler> sampler,
                                                      const double& batch_size)
{
  return
  ompl::base::OptimizationObjectivePtr(new ompl::base::MyOptimizationObjective(si, sampler, batch_size,
    [start_state, goal_state](const VectorXd& state)
    {
      return (start_state - state).norm() + (goal_state - state).norm();
    },
    [](const VectorXd& s1, const VectorXd& s2)
    {
      return (s2 - s1).norm();
    }));
}

ompl::base::OptimizationObjectivePtr get_geom_opt_obj(const ompl::base::SpaceInformationPtr& si,
                                                      const VectorXd& start_state,
                                                      const VectorXd& goal_state,
                                                      const double& batch_size)
{
  return
  ompl::base::OptimizationObjectivePtr(new ompl::base::MyOptimizationObjective(si, batch_size,
    [start_state, goal_state](const VectorXd& state)
    {
      return (start_state - state).norm() + (goal_state - state).norm();
    },
    [](const VectorXd& s1, const VectorXd& s2)
    {
      return (s2 - s1).norm();
    }));
}
