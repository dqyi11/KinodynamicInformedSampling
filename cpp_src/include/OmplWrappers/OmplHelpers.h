#pragma once

#include <OmplWrappers/MyOptimizationObjective.h>

#include <ompl/base/OptimizationObjective.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;

///
/// Function to convert a State to a VectorXd
///
/// @param s Ompl State
/// @return Eigen VectorXd
///
Eigen::VectorXd get_eigen_vector(const ompl::base::State *s);

///
/// Function to convert a std::vector to a VectorXd
///
/// @param vec Standard vector
/// @return A VectorXd with the state information
///
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

///
/// Convert an Eigen::VectorXd to an ompl State pointer
///
/// @param vec VectorXd representing the state
/// @return Pointer to an ompl state
///
bool get_ompl_state(const Eigen::VectorXd &vec, ompl::base::State* state);
