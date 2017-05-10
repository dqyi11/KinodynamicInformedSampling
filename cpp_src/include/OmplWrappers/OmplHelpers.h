#pragma once

#include <OmplWrappers/MyOptimizationObjective.h>

#include <ompl/base/OptimizationObjective.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;

// template <int dof>
// ompl::base::OptimizationObjectivePtr get_dimt_opt_ob(const
// ompl::base::SpaceInformationPtr &si,
//                                                      const
//                                                      ompl::base::ProblemDefinitionPtr
//                                                      &problem,
//                                                      const double levelSet,
//                                                      const unsigned int
//                                                      maxNumberCalls,
//                                                      const int
//                                                      sampleBatchSize,
//                                                      const VectorXd&
//                                                      start_state,
//                                                      const VectorXd&
//                                                      goal_state,
//                                                      const
//                                                      DoubleIntegrator<dof>
//                                                      &di)
// {
//   return
//   ompl::base::OptimizationObjectivePtr(new
//   ompl::base::MyOptimizationObjective(si, problem,
//     levelSet, maxNumberCalls, sampleBatchSize,
//     [start_state, goal_state, di](const VectorXd& state)
//     {
//       return di.getMinTime(start_state, state) + di.getMinTime(state,
//       goal_state);
//     },
//     [di](const VectorXd& s1, const VectorXd& s2)
//     {
//       return di.getMinTime(s1, s2);
//     }));
// }

// ompl::base::OptimizationObjectivePtr get_geom_opt_obj(const
// ompl::base::SpaceInformationPtr& si,
//                                                       const VectorXd&
//                                                       start_state,
//                                                       const VectorXd&
//                                                       goal_state,
//                                                       const
//                                                       ompl::base::InformedSamplerPtr
//                                                       sampler,
//                                                       const double&
//                                                       batch_size);

// ompl::base::OptimizationObjectivePtr get_geom_opt_obj(const
// ompl::base::SpaceInformationPtr& si,
//                                                       const VectorXd&
//                                                       start_state,
//                                                       const VectorXd&
//                                                       goal_state,
//                                                       const double&
//                                                       batch_size);

// template <int dof>
// ompl::base::OptimizationObjectivePtr get_dimt_opt_ob(const
// ompl::base::SpaceInformationPtr &si,
//                                                      const VectorXd&
//                                                      start_state,
//                                                      const VectorXd&
//                                                      goal_state,
//                                                      const
//                                                      ompl::base::InformedSamplerPtr
//                                                      sampler,
//                                                      const double&
//                                                      batch_size,
//                                                      const
//                                                      DoubleIntegrator<dof>
//                                                      &di)
// {
//   return
//   ompl::base::OptimizationObjectivePtr(new
//   ompl::base::MyOptimizationObjective(si, sampler, batch_size,
//     [start_state, goal_state, di](const VectorXd& state)
//     {
//       return di.getMinTime(start_state, state) + di.getMinTime(state,
//       goal_state);
//     },
//     [di](const VectorXd& s1, const VectorXd& s2)
//     {
//       return di.getMinTime(s1, s2);
//     }));
// }

//
// Function to convert a State to a VectorXd
//
// @param s Ompl State
// @return Eigen VectorXd
//
Eigen::VectorXd get_eigen_vector(const ompl::base::State *s);

//
// Function to convert a std::vector to a VectorXd
//
// @param vec Standard vector
// @return A VectorXd with the state information
//
template <typename T>
Eigen::VectorXd get_eigen_vector(const std::vector<T> &vec)
{
    Eigen::VectorXd v(param.dimensions);

    for (uint i = 0; i < param.dimensions; i++)
    {
        v[i] = vec[i];
    }

    return v;
}

//
// Convert an Eigen::VectorXd to an ompl State pointer
//
// @param vec VectorXd representing the state
// @return Pointer to an ompl state
//
bool get_ompl_state(const Eigen::VectorXd &vec, ompl::base::State* state);
