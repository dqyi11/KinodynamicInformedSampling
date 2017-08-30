#ifndef MULTI_LINKDI_VALIDITY_CHECKER_H_
#define MULTI_LINKDI_VALIDITY_CHECKER_H_

#include <ompl/base/StateValidityChecker.h>
#include "../External/multiLinkDI-dart/include/MultiLinkDI.hpp"

class MultiLinkDIValidityChecker : public ompl::base::StateValidityChecker
{
public:
    MultiLinkDIValidityChecker(const ompl::base::SpaceInformationPtr &si, MultiLinkDI* di)
        : ompl::base::StateValidityChecker(si), di_(di)
    {
    }
    virtual ~MultiLinkDIValidityChecker();

    bool isValid(const ompl::base::State *state) const;
protected:
    MultiLinkDI* di_;
};

#endif // MULTI_LINKDI_VALIDITY_CHECKER_H_
