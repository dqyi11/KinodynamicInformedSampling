#include "HerbValidityChecker.hpp"


HerbValidityChecker::~HerbValidityChecker()
{
}

bool HerbValidityChecker::isValid(const ompl::base::State *state) const
{
    if(herb_)
    {
        //Eigen::VectorXd vec(param.dimensions);
        //get_eigen_vector(state, vec);

        //if(herb_->isCollided(vec)==true)
        {
            return true;
        }
    }
    return true;
}
