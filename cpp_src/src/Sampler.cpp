#include <Sampler/Sampler.h>

// Our libraries
#include <Dimt/Params.h>
#include <OmplWrappers/MyOptimizationObjective.h>

// OMPL
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/goals/GoalState.h>

// stdlib
#include <vector>
#include <memory>
#include <stdexcept>

namespace
{

///
/// Function to convert a State to a VectorXd
///
/// @param s Ompl State
/// @return Eigen VectorXd
///
Eigen::VectorXd get_eigen_vector(const ompl::base::State* s)
{
    double * val = static_cast<const ompl::base::RealVectorStateSpace::StateType*>(s)->values;

    VectorXd v(param.dimensions);

    for(uint i = 0; i < param.dimensions; i++)
    {
        v[i] = val[i];
    }

    return v;
}

///
/// Function to convert a std::vector to a VectorXd
///
/// @param vec Standard vector
/// @return A VectorXd with the state information
///
template <typename T>
VectorXd get_eigen_vector(const std::vector<T>& vec)
{
    VectorXd v(param.dimensions);

    for(uint i = 0; i < param.dimensions; i++)
    {
        v[i] = vec[i];
    }

    return v;
}

///
/// Convert an Eigen::VectorXd to an ompl State pointer
///
/// @param vec VectorXd representing the state
/// @return Pointer to an ompl state
///
ompl::base::State* get_ompl_state(const VectorXd &vec,
                                  const ompl::base::RealVectorStateSpace* space)
{
    ompl::base::State *state = space->allocState();

    for(uint i = 0; i < vec.size(); i++)
    {
        state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = vec[i];
    }

    return state;
}

} // namespace

///
/// Get the state limits of the space
///
/// @return Tuple(state_max, state_min)
///
std::tuple<Eigen::VectorXd, Eigen::VectorXd> Sampler::state_limits() const
{
    // Get the bounds from the vector
    const auto space = si_->getStateSpace()->as<ompl::base::RealVectorStateSpace>();
    VectorXd high = get_eigen_vector(space->getBounds().high);
    VectorXd low = get_eigen_vector(space->getBounds().low);

    return std::make_tuple(low, high);
}

///
/// Get the start state
///
/// @return Start state defined in the constructor
///
VectorXd Sampler::start_state() const
{
    return get_eigen_vector(problem_->getStartState(0));
}

///
/// Get the goal state
///
/// @return Start state defined in the constructor
///
VectorXd Sampler::goal_state() const
{
    const auto goal_state = problem_->getGoal()->as<ompl::base::GoalState>();
    return get_eigen_vector(goal_state->getState());
}

VectorXd Sampler::get_grad(const VectorXd& curr_state) const
{
    // Assert that the matrix is not empty
    assert(curr_state.size() != 0 or curr_state.size() != 0);

    // Derivative constant
    double h = 0.001;

    // Loop through and calculate the gradients
    VectorXd grad(curr_state.size());
    for(int dim = 0; dim < curr_state.size(); dim++)
    {
        VectorXd state_plus = curr_state; VectorXd state_min = curr_state;
        state_plus(dim) = curr_state(dim) + h;
        state_min(dim) = curr_state(dim) - h;
        grad(dim) = (get_cost(state_plus) - get_cost(state_min)) / (2*h);
    }

    return grad;
}

VectorXd Sampler::get_inv_jacobian(const VectorXd& curr_state) const
{
    // Assert that the matrix is not empty
    assert(curr_state.size() != 0 or curr_state.size() != 0);

    // Derivative constant
    double h = 0.001;

    // Loop through and calculate the gradients
    VectorXd inv_jacobian(curr_state.size());
    for(int dim = 0; dim < curr_state.size(); dim++)
    {
        VectorXd state_plus = curr_state; VectorXd state_min = curr_state;
        state_plus(dim) = curr_state(dim) + h;
        state_min(dim) = curr_state(dim) - h;
        double delta = get_cost(state_plus) - get_cost(state_min);
        //std::cout << "DELTA " << delta << std::endl;
        if(delta == 0.0)
        {
            inv_jacobian(dim) = 0.0;
        }
        else
        {
            inv_jacobian(dim) = (2*h) / (delta);
        }
    }

    return inv_jacobian;
}

///
/// Get the cost for a specific state
///
/// @param curr_state Current state to get the cost for
/// @return Cost at that state
///
double Sampler::get_cost(const VectorXd& curr_state) const
{
    // Assert that the problem definition has an optimization objective defined
    assert(problem_->hasOptimizationObjective());

    const ompl::base::State* start_state = problem_->getStartState(0);
    const ompl::base::State* goal_state = problem_->getGoal()->as<ompl::base::GoalState>()->getState();

    const auto space = si_->getStateSpace()->as<ompl::base::RealVectorStateSpace>();

    const ompl::base::State* state = get_ompl_state(curr_state, space);

    const ompl::base::OptimizationObjectivePtr optim_obj = problem_->getOptimizationObjective();

    return optim_obj->motionCost(start_state, state).value() +
           optim_obj->motionCost(state, goal_state).value();
}
