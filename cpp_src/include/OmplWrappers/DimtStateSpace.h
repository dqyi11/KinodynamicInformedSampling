#ifndef OMPL_BASE_SPACES_DIMT_STATE_SPACE_
#define OMPL_BASE_SPACES_DIMT_STATE_SPACE_
#include <Dimt/Params.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <Dimt/DoubleIntegratorMinimumTime.h>
#include <Eigen/Dense>

namespace ompl
{
    namespace base
    {
        class DimtStateSpace : public RealVectorStateSpace
        {
        private:
            std::shared_ptr<DIMT> dimt_;

        public:
            DimtStateSpace(const std::shared_ptr<DIMT> dimt)
              : RealVectorStateSpace(param.dimensions)
            {
                dimt_ = dimt;
            }

            virtual double distance(const State *state1, const State *state2) const
            {
                const double *s1 = static_cast<const StateType *>(state1)->values;
                const double *s2 = static_cast<const StateType *>(state2)->values;

                Eigen::VectorXd eig_from(param.dimensions), eig_to(param.dimensions);
                for (unsigned int i = 0; i < dimension_; ++i)
                {
                    eig_from[i] = s1[i];
                    eig_to[i] = s2[i];
                }
                double time = dimt_->getMinTime(eig_from, eig_to);
                return time;
            }

            virtual void interpolate(const State *from, const State *to,
                                     const double t, State *state) const
            {
                const StateType *rfrom = static_cast<const StateType *>(from);
                const StateType *rto = static_cast<const StateType *>(to);
                const StateType *rstate = static_cast<StateType *>(state);
                /*
                Eigen::VectorXd eig_from(param.dimensions), eig_to(param.dimensions), eig_state(param.dimensions);

                for (unsigned int i = 0; i < dimension_; ++i)
                {
                    eig_from[i] = rfrom->values[i];
                    eig_to[i] = rto->values[i];
                }
                dimt_->interpolate(eig_from, eig_to, t, eig_state);
                for (unsigned int i = 0; i < dimension_; ++i)
                {
                    rstate->values[i] = eig_state[i];
                }
                */
                // TODO: Fix assert fail in steering function (using straight line for now)
                for (unsigned int i = 0; i < dimension_; ++i)
                    rstate->values[i] = rfrom->values[i] + (rto->values[i] - rfrom->values[i]) * t;
            }



            virtual bool isMetricSpace() const
            {
                return false;
            }
        };

        using DimtStateSpacePtr = std::shared_ptr<DimtStateSpace>;
    }
}
#endif
