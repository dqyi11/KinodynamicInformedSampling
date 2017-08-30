#include "MultiLinkDIValidityChecker.hpp"
#include "OmplWrappers/OmplHelpers.h"

MultiLinkDIValidityChecker::~MultiLinkDIValidityChecker()
{
}

bool MultiLinkDIValidityChecker::isValid(const ompl::base::State *state) const
{
    if(di_)
    {
        Eigen::VectorXd vec(param.dimensions);
        get_eigen_vector(state, vec);

        if(di_->isCollided(vec)==true)
        {
            return false;
        }
    }
    return true;
}
