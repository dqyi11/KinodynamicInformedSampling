#include <OmplWrappers/OmplHelpers.h>

//
// Function to convert a State to a VectorXd
//
// @param s Ompl State
// @return Eigen VectorXd
//
bool get_eigen_vector(const ompl::base::State *s, Eigen::VectorXd& vec)
{
    /*
    if(vec.SizeAtCompileTime != param.dimensions)
    {
        return false;
    }*/

    auto tmp = s->as<ompl::base::RealVectorStateSpace::StateType>();
    for (uint i = 0; i < param.dimensions; i++)
    {
        //vec[i] = s->as<ompl::base::RealVectorStateSpace::StateType>()->values[i];
        vec[i] = tmp->values[i];
    }
    return true;
}

//
// Convert an Eigen::VectorXd to an ompl State pointer
//
// @param vec VectorXd representing the state
// @return Pointer to an ompl state
//
bool get_ompl_state(const Eigen::VectorXd &vec, ompl::base::State* state)
{
    for (uint i = 0; i <  param.dimensions; i++)
    {
        state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = vec[i];
    }

    return true;
}
