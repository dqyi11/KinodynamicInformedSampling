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
            //std::cout << "COLLIDED " << vec << std::endl;
            return false;
        }

        //std::cout << "NOT COLLIDED " << vec << std::endl;
    }
    return true;
}
