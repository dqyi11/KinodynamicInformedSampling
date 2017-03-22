#pragma once

#include <OmplWrappers/MyOptimizationObjective.h>

// Our libs
#include <Dimt/DoubleIntegrator.h>

template <int dof>
ompl::base::OptimizationObjectivePtr get_dimt_opt_ob(const ompl::base::SpaceInformationPtr &si,
                                                     const VectorXd& start_state,
                                                     const VectorXd& goal_state,
                                                     const double& batch_size,
                                                     const DoubleIntegrator<dof> &di)
{
  return
  ompl::base::OptimizationObjectivePtr(new ompl::base::MyOptimizationObjective(si, batch_size,
    [start_state, goal_state, di](const VectorXd& state)
    {
      std::cout << "Start state size: " << start_state.size() << std::endl;
      std::cout << "Goal state size: " << goal_state.size() << std::endl;
      std::cout << "State size: " << state.size() << std::endl;
      return di.getMinTime(start_state, state) + di.getMinTime(state, goal_state);
    },
    [di](const VectorXd& s1, const VectorXd& s2)
    {
      // std::cout << "Start: " << s1 << std::endl;
      // std::cout << "Goal: " << s2 << std::endl;
      return di.getMinTime(s1, s2);
    }));
}

ompl::base::OptimizationObjectivePtr get_geom_opt_obj(const ompl::base::SpaceInformationPtr& si,
                                                      const VectorXd& start_state,
                                                      const VectorXd& goal_state,
                                                      const std::shared_ptr<Sampler> sampler,
                                                      const double& batch_size);

ompl::base::OptimizationObjectivePtr get_geom_opt_obj(const ompl::base::SpaceInformationPtr& si,
                                                      const VectorXd& start_state,
                                                      const VectorXd& goal_state,
                                                      const double& batch_size);

template <int dof>
ompl::base::OptimizationObjectivePtr get_dimt_opt_ob(const ompl::base::SpaceInformationPtr &si,
                                                     const VectorXd& start_state,
                                                     const VectorXd& goal_state,
                                                     const std::shared_ptr<Sampler> sampler,
                                                     const double& batch_size,
                                                     const DoubleIntegrator<dof> &di)
{
  return
  ompl::base::OptimizationObjectivePtr(new ompl::base::MyOptimizationObjective(si, sampler, batch_size,
    [start_state, goal_state, di](const VectorXd& state)
    {
      std::cout << "Start state size: " << start_state.size() << std::endl;
      std::cout << "Goal state size: " << goal_state.size() << std::endl;
      std::cout << "State size: " << state.size() << std::endl;
      return di.getMinTime(start_state, state) + di.getMinTime(state, goal_state);
    },
    [di](const VectorXd& s1, const VectorXd& s2)
    {
      // std::cout << "Start: " << s1 << std::endl;
      // std::cout << "Goal: " << s2 << std::endl;
      return di.getMinTime(s1, s2);
    }));
}
