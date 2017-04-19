#ifndef OMPL_BASE_SPACES_DIMT_STATE_SPACE_
#define OMPL_BASE_SPACES_DIMT_STATE_SPACE_
#include <Dimt/Params.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <Dimt/Dimt.h>
#include <Dimt/DoubleIntegrator.h>
#include <Eigen/Dense>

namespace ompl
{
    namespace base
    {
        class DimtStateSpace : public RealVectorStateSpace
        {
        private:
            Dimt dimt_;
            DoubleIntegrator<param.dof> double_integrator_;

        public:
            DimtStateSpace(Dimt dimt, DoubleIntegrator<param.dof> double_integrator, unsigned int dim = 0)
              : dimt_(dimt), double_integrator_(double_integrator), RealVectorStateSpace(dim)
            {
            }

            virtual double distance(const State *state1, const State *state2) const
            {
                double dist = 0.0;
                const double *s1 = static_cast<const StateType *>(state1)->values;
                const double *s2 = static_cast<const StateType *>(state2)->values;
                // Eigen::VectorXd eig_s1(dimension_);
                // Eigen::VectorXd eig_s2(dimension_);
                // for (unsigned int i = 0; i < dimension_; ++i)
                // {
                // 	eig_s1[i] = s1[i];
                // 	eig_s2[i] = s2[i];
                // }
                // return dimt_.get_min_time(eig_s1, eig_s2);
                DoubleIntegrator<param.dof>::StateVector eig_from, eig_to;
                for (unsigned int i = 0; i < dimension_; ++i)
                {
                    eig_from[i] = s1[i];
                    eig_to[i] = s2[i];
                }
                double time = double_integrator_.getMinTime(eig_from, eig_to);
                return time;
            }

            virtual void interpolate(const State *from, const State *to, const double t, State *state) const
            {
                const StateType *rfrom = static_cast<const StateType *>(from);
                const StateType *rto = static_cast<const StateType *>(to);
                const StateType *rstate = static_cast<StateType *>(state);
                DoubleIntegrator<param.dof>::StateVector eig_from, eig_to, eig_state;
                // for (unsigned int i = 0 ; i < dimension_ ; ++i)
                // 	rstate->values[i] = rfrom->values[i] + (rto->values[i] -
                // rfrom->values[i]) * t;
                for (unsigned int i = 0; i < dimension_; ++i)
                {
                    eig_from[i] = rfrom->values[i];
                    eig_to[i] = rto->values[i];
                }
                DoubleIntegrator<param.dof>::Trajectory traj = double_integrator_.getTrajectory(eig_from, eig_to);
                eig_state = traj.getState(t);
                for (unsigned int i = 0; i < dimension_; ++i)
                {
                    rstate->values[i] = eig_state[i];
                }
                // TODO: Fix assert fail in steering function (using straight line for now)
                for (unsigned int i = 0; i < dimension_; ++i)
                    rstate->values[i] = rfrom->values[i] + (rto->values[i] - rfrom->values[i]) * t;
            }

            virtual bool isMetricSpace() const
            {
                return false;
            }
        };
    }
}
#endif
