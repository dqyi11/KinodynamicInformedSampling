#ifndef OMPL_BASE_SPACES_DIMT_STATE_SPACE_
#define OMPL_BASE_SPACES_DIMT_STATE_SPACE_
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <Dimt/Dimt.h>
#include <Eigen/Dense>

namespace ompl
{
	namespace base
	{
		class DimtStateSpace : public RealVectorStateSpace
		{
		private:
			Dimt dimt_;	
		public:
			DimtStateSpace(Dimt dimt, unsigned int dim = 0) : dimt_(dimt), RealVectorStateSpace(dim){}

			virtual double distance(const State *state1, const State *state2) const
			{
				double dist = 0.0;
			  const double *s1 = static_cast<const StateType*>(state1)->values;
			  const double *s2 = static_cast<const StateType*>(state2)->values;
  			Eigen::VectorXd eig_s1(dimension_);
  			Eigen::VectorXd eig_s2(dimension_);
			  for (unsigned int i = 0; i < dimension_; ++i)
			  {
			  	eig_s1[i] = s1[i];
			  	eig_s2[i] = s2[i];
			  }
			  return dimt_.get_min_time(eig_s1, eig_s2);
			}

		  virtual void interpolate(const State *from, const State *to, const double t, State *state) const
		  {
				const StateType *rfrom = static_cast<const StateType*>(from);
			  const StateType *rto = static_cast<const StateType*>(to);
			  const StateType *rstate = static_cast<StateType*>(state);
			  for (unsigned int i = 0 ; i < dimension_ ; ++i)
			  rstate->values[i] = rfrom->values[i] + (rto->values[i] - rfrom->values[i]) * t;
		  }

		  virtual bool isMetricSpace() const {return false;}
		};
	}	
}
#endif