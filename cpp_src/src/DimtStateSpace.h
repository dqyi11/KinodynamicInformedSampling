#ifndef OMPL_BASE_SPACES_DIMT_STATE_SPACE_
#define OMPL_BASE_SPACES_DIMT_STATE_SPACE_
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ompl
{
	namespace base
	{
		class DimtStateSpace : public RealVectorStateSpace
		{
		public:
			DimtStateSpace(unsigned int dim = 0) : RealVectorStateSpace(dim) {}

			virtual double distance(const State *state1, const State *state2) const
			{
				double dist = 0.0;
			  const double *s1 = static_cast<const StateType*>(state1)->values;
			  const double *s2 = static_cast<const StateType*>(state2)->values;
			  for (unsigned int i = 0 ; i < dimension_ ; ++i)
			  {
			  	double diff = (*s1++) - (*s2++);
			  	dist += diff * diff;
			  }
			  return sqrt(dist);
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