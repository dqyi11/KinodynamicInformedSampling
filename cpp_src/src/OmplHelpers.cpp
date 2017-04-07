#include <OmplWrappers/OmplHelpers.h>

// ompl::base::OptimizationObjectivePtr get_geom_opt_obj(const ompl::base::SpaceInformationPtr& si,
//                                                       const VectorXd& start_state,
//                                                       const VectorXd& goal_state,
//                                                       const ompl::base::InformedSamplerPtr sampler,
//                                                       const double& batch_size)
// {
//   return
//   ompl::base::OptimizationObjectivePtr(new ompl::base::MyOptimizationObjective(si, sampler, batch_size,
//     [start_state, goal_state](const VectorXd& state)
//     {
//       return (start_state - state).norm() + (goal_state - state).norm();
//     },
//     [](const VectorXd& s1, const VectorXd& s2)
//     {
//       return (s2 - s1).norm();
//     }));
// }

// ompl::base::OptimizationObjectivePtr get_geom_opt_obj(const ompl::base::SpaceInformationPtr &si,
//                                                       const ompl::base::ProblemDefinitionPtr &problem,
//                                                       const double levelSet,
//                                                       const unsigned int maxNumberCalls,
//                                                       const int sampleBatchSize,
//                                                       const StateCostFxn &stateCostFn,
//                                                       const MotionCostFxn &motionCostFn,
//                                                       const VectorXd& start_state,
//                                                       const VectorXd& goal_state)
// {
//   return
//   ompl::base::OptimizationObjectivePtr(new ompl::base::MyOptimizationObjective(si, problem,
//     levelSet, maxNumberCalls, sampleBatchSize,
//     [start_state, goal_state](const VectorXd& state)
//     {
//       return (start_state - state).norm() + (goal_state - state).norm();
//     },
//     [](const VectorXd& s1, const VectorXd& s2)
//     {
//       return (s2 - s1).norm();
//     }));
// }

///
/// Function to convert a State to a VectorXd
///
/// @param s Ompl State
/// @return Eigen VectorXd
///
Eigen::VectorXd get_eigen_vector(const ompl::base::State* s)
{
    double * val = static_cast<const ompl::base::RealVectorStateSpace::StateType*>(s)->values;

    Eigen::VectorXd v(param.dimensions);

    for(uint i = 0; i < param.dimensions; i++)
    {
        v[i] = val[i];
    }

    return v;
}

///
/// Convert an Eigen::VectorXd to an ompl State pointer
///
/// @param vec VectorXd representing the state
/// @return Pointer to an ompl state
///
ompl::base::State* get_ompl_state(const Eigen::VectorXd &vec,
                                  const ompl::base::RealVectorStateSpace* space)
{
    ompl::base::State *state = space->allocState();

    for(uint i = 0; i < vec.size(); i++)
    {
        state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = vec[i];
    }

    return state;
}
