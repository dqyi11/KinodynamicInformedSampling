#pragma once

// Standard Library Functions
#include <iostream>
#include <chrono>
using namespace std::chrono;

// Eigen namespace
#include <Eigen/Dense>
using Eigen::MatrixXd;

// Include Sampler
#include <Sampler/Sampler.h>

// Example for how to inherit and create your own sampler
class RejectionSampler: public Sampler
{
public:
	RejectionSampler(ProblemDefinition problem)
		: Sampler(problem)
	{ }

	// Only function that you must implement
	virtual MatrixXd sample(const int& no_samples, const bool& time) const override;

	// Can implement as many private functions as you want to help do the sampling
	virtual VectorXd get_random_sample(const double& max, const double& min, const int& size) const;
};

///
/// Heirarchical Rejection Sampler
///
template <int dof>
class HierarchicalRejectionSampler: public RejectionSampler
{
typedef Eigen::Matrix<double, 2 * dof, 1> StateVector;
typedef Eigen::Matrix<double, dof, 1> Vector;
public:
	///
	/// Constructor
	///
	/// @param problem Problem Definition
	/// 

	///
	/// Get a series of samples for the problem space
	///
	/// @param no_samples Number of samples to get
	/// @param time Boolean that determines if the time to run the proccess is displayed 
	/// @return A series of samples of shape (number of samples, sample dimension)
	///
	virtual MatrixXd sample(const int& no_samples, const bool& time) const override
	{
		
	}

private:

	// 
	// Code from Tobias Kunz Github: https://github.com/tobiaskunz/dimt/blob/master/DoubleIntegrator.h
	//	
	static double squared(double d) 
	{
		return d * d;
	}

	static std::pair<double, double> getInfeasibleInterval(double startVelocity, double goalVelocity, double distance,
		double acceleration1, double maxAcceleration, double maxVelocity)
	{
		std::pair<double, double> infeasibleInterval;

		if(startVelocity * goalVelocity <= 0.0 || acceleration1 * startVelocity < 0.0) {
			// If minimum-time solution goes through zero-velocity, there is no infeasible time interval, because we can stop and wait at zero-velocity
			infeasibleInterval.first = std::numeric_limits<double>::infinity();
			infeasibleInterval.second = std::numeric_limits<double>::infinity();
		}
		// for all cases below: sign(startVelocity) == sign(goalVelocity) == sign(distance) == sign(additionalDistance) == sign(acceleration1) == -sign(acceleration2)
		else {
			double zeroTime1 = std::abs(startVelocity) / maxAcceleration;
			double zeroTime2 = std::abs(goalVelocity) / maxAcceleration;
			double zeroDistance = zeroTime1 * startVelocity / 2.0 + zeroTime2 * goalVelocity / 2.0;
			if(std::abs(zeroDistance) < std::abs(distance)) {
				infeasibleInterval.first = std::numeric_limits<double>::infinity();
				infeasibleInterval.second = std::numeric_limits<double>::infinity();
			}
			else {
				double timeLow1, timeLow2, timeHigh1, timeHigh2;
				getPPTime(startVelocity, goalVelocity, distance, -acceleration1, &timeLow1, &timeLow2, &timeHigh1, &timeHigh2);
				infeasibleInterval.first = timeLow1 + timeLow2;
				infeasibleInterval.second = timeHigh1 + timeHigh2;
				if(infeasibleInterval.second == std::numeric_limits<double>::infinity()) {
					std::cout << "infinity 1" << std::endl;
					infeasibleInterval.first = 0.0;
					infeasibleInterval.second = std::numeric_limits<double>::infinity();
					return infeasibleInterval;
					assert(false);
				}

				if(std::abs(startVelocity + timeHigh1 * -acceleration1) >= maxVelocity) {
					infeasibleInterval.second = getPLPTime(startVelocity, goalVelocity, distance, maxVelocity, -acceleration1);
					if(infeasibleInterval.second == std::numeric_limits<double>::infinity()) {
						std::cout << "infinity 2" << std::endl;
						assert(false);
					}
				}
			}
		}

		return infeasibleInterval;
	}

	static double getPLPTime(double startVelocity, double goalVelocity, double distance, double maxVelocity, double acceleration1)
	{
		assert(std::abs(startVelocity) <= maxVelocity && std::abs(goalVelocity) <= maxVelocity);

		const double boundaryVelocity = sign(acceleration1) * maxVelocity;
		const double timeToBoundary1 = (boundaryVelocity - startVelocity) / acceleration1;
		const double timeToBoundary2 = (goalVelocity - boundaryVelocity) / -acceleration1;
		const double distanceToBoundary = 0.5 * (2.0 * squared(boundaryVelocity) - squared(startVelocity) - squared(goalVelocity)) / acceleration1;
		const double boundaryTime = (distance - distanceToBoundary) / boundaryVelocity;

		if(boundaryTime < 0.0)
			return std::numeric_limits<double>::infinity();
		else {
			return timeToBoundary1 + boundaryTime + timeToBoundary2;
		}
	}

	static void getPPTime(double startVelocity, double goalVelocity, double distance, double acceleration1,
	                      double* tMin1, double* tMin2, double* tMax1, double* tMax2)
	{
		if(startVelocity == goalVelocity && distance == 0.0) {
			if(tMin1) {
				*tMin1 = 0.0;
				*tMin2 = 0.0;
			}
			if(tMax1) {
				*tMax1 = 0.0;
				*tMax2 = 0.0;
			}
			return;
		}

		const double a = acceleration1;
		const double b = 2.0 * startVelocity;
		//const double c = (squared(goalVelocity) - squared(startVelocity)) / 2.0 / acceleration2 - distance;
		const double c = 0.5 * (startVelocity + goalVelocity) * ((goalVelocity - startVelocity) / -acceleration1) - distance;

		const double radicand = b*b - 4.0*a*c;
		assert(radicand >= 0.0);
		
		// numerically stable solution to the quadratic equation
		const double q = -0.5 * (b + sign(b) * sqrt(radicand));

		if(a * sign(b) >= 0.0) {
			if(tMin1) *tMin1 = q / a;
			if(tMax1) *tMax1 = c / q;
		}
		else {
			if(tMin1) *tMin1 = c / q;
			if(tMax1) *tMax1 = q / a;
		}

		if(tMin1) {
			*tMin2 = *tMin1 - (goalVelocity - startVelocity) / acceleration1;
			if(*tMin1 < 0.0 || *tMin2 < 0.0) {
				*tMin1 = std::numeric_limits<double>::infinity();
				*tMin2 = std::numeric_limits<double>::infinity();
			}
		}

		if(tMax1) {
			*tMax2 = *tMax1 - (goalVelocity - startVelocity) / acceleration1;
			if(*tMax1 < 0.0 || *tMax2 < 0.0) {
				*tMax1 = std::numeric_limits<double>::infinity();
				*tMax2 = std::numeric_limits<double>::infinity();
			}
		}
	}

	static double getMinTime1D(double startVelocity, double goalVelocity, double distance,
	                           double maxAcceleration, double maxVelocity, double maxTime,
	                           double& acceleration1)
	{
			// determine distance travelled while accelerating from start velocity to goal velocity
			const double accelerationTime = std::abs(goalVelocity - startVelocity) / maxAcceleration;
			if(accelerationTime >= maxTime)
				return accelerationTime;
			const double accelerationDistance = 0.5 * (startVelocity + goalVelocity) * accelerationTime;
			
			// determine whether to accelerate at the upper or lower limit first
			const double additionalDistance = distance - accelerationDistance;

			acceleration1 = sign(additionalDistance) * maxAcceleration;

			double time1, time2;
			getPPTime(startVelocity, goalVelocity, distance, acceleration1, NULL, NULL, &time1, &time2);
			double time = time1 + time2;

			assert(time >= 0.0);
			assert(time != std::numeric_limits<double>::infinity());

			if(time < maxTime && std::abs(startVelocity + acceleration1 * time1) >= maxVelocity) {
				time = getPLPTime(startVelocity, goalVelocity, distance, maxVelocity, acceleration1);
			}

			return time;
	}

	static double getMinTime(const StateVector &state1, const StateVector &state2,
	                         const Vector &maxAccelerations,
	                         const Vector &maxVelocities =  std::numeric_limits<double>::infinity() * Vector::Ones(),
	                         double maxTime = std::numeric_limits<double>::infinity())
	{
			const Vector distances = state2.template head<dof>() - state1.template head<dof>();
			double minTime = 0.0;
			int limitDof = -1; // DOF for which the min time but not the infeasible interval has been calculated yet
			std::pair<double, double> infeasibleIntervals[dof];
			Vector firstAccelerations;
			
			for(unsigned int i = 0; i < dof && minTime < maxTime; ++i) {
				const double time = getMinTime1D(state1[dof+i], state2[dof+i], distances[i], maxAccelerations[i], maxVelocities[i], maxTime, firstAccelerations[i]);
				adjustForInfeasibleIntervals(i, time, maxTime, state1.template tail<dof>(), state2.template tail<dof>(), distances, firstAccelerations, maxAccelerations, maxVelocities,
				                             minTime, infeasibleIntervals, limitDof);
			}

			assert(minTime < std::numeric_limits<double>::infinity());
			return minTime;
	}
	// 
	// Code from Tobias Kunz Github: https://github.com/tobiaskunz/dimt/blob/master/DoubleIntegrator.h
	//

	template<typename T>
	static inline T sign(const T& a)
	{
		return a >= T(0) ? T(1):T(-1);
	}
};