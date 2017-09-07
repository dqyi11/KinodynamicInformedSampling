#ifndef HERB_VALIDITY_CHECKER_H_
#define HERB_VALIDITY_CHECKER_H_

#include <ompl/base/StateValidityChecker.h>
#include <libherb/herb.hpp>

class HerbValidityChecker : public ompl::base::StateValidityChecker
{
public:
    HerbValidityChecker(const ompl::base::SpaceInformationPtr &si,
                               std::shared_ptr<herb::Herb> herb)
        : ompl::base::StateValidityChecker(si), herb_(herb)
    {
    }
    virtual ~HerbValidityChecker();

    bool isValid(const ompl::base::State *state) const;
protected:
    std::shared_ptr<herb::Herb> herb_;
};

#endif // HERB_VALIDITY_CHECKER_H_
