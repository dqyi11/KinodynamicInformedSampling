#pragma once
#include <iostream>
#include <Eigen/Dense>
using Eigen::VectorXd;

class Dimt
{
public:
	double a_max_;
	///
	/// Constructor 
	///
	/// @param a_max Max Acceleration
	/// @param prob Problem Definition
	/// 
	Dimt(double a_max) : a_max_(a_max)
	{
		// // Assert that the start, goal, and limits are the same size
		// if(!this->is_valid_constructor())
		// {
		// 	std::string error_msg = "Invalid Arguments of Shapes into Constructor.";
		// 	throw std::invalid_argument(error_msg);
		// }
	}

	bool is_valid_states(const VectorXd x1, const VectorXd x2, const VectorXd xi) const
	{
		return (x1.size() == x2.size() and x1.size() == xi.size())
		   and (x1.size() != 0 and x2.size() != 0 and xi.size() != 0) and (x1.size() % 2 == 0); 
	}

	// This function calculates the maximum time given a set number of joints
	// x = [x_1, x_1_dot,...,x_n,x_n_dot]
	// @param x1 Initial state
	// @param x2 Final state
	// @param xi Intermediate state
	// @return T Maximum time
	double get_min_time(const VectorXd x1, const VectorXd x2, const VectorXd xi) const 
	{	
		// Assert that the start, goal, and x1 are valid size (same and != 0)
		if(!is_valid_states(x1, x2, xi))
		{
			std::string error_msg = "Invalid Arguments to get_min_time";
			throw std::invalid_argument(error_msg);
		}
	    VectorXd Ts(int(x1.size()/2));
	    for(int i = 0; i<x1.size(); i=i+2)
	    {
	      Ts(i/2) = get_min_time_1dof(x1(i), x1(i+1), x2(i), x2(i+1), xi(i), xi(i+1));
	    }
	    return Ts.maxCoeff();
	}

	// This function calculates the maximum time given a set number of joints
	// x = [x_1, x_1_dot,...,x_n,x_n_dot]
	// @param x1 Initial state
	// @param x2 Final state
	// @return T Maximum time
	double get_min_time(const VectorXd x1, const VectorXd x2) const 
	{	
		// Assert that the start, goal, and x1 are valid size (same and != 0)
		// TODO: Implement this check !!
		// if(!is_valid_states(x1, x2))
		// {
		// 	std::string error_msg = "Invalid Arguments to get_min_time";
		// 	throw std::invalid_argument(error_msg);
		// }
	    VectorXd Ts(int(x1.size()/2));
	    for(int i = 0; i<x1.size(); i=i+2)
	    {
	      Ts(i/2) = get_min_time_1dof(x1(i), x1(i+1), x2(i), x2(i+1));
	    }
	    return Ts.maxCoeff();
	}

	// returns the minimum time between 3 points ([x1,y1] -> [xi,yi] -> [x2,y2]) in the state space
	// returns -1 if the solution does not exist
	double get_min_time_1dof(const double x1, const double v1, const double x2,
			const double v2, const double xi, const double vi) const
	{
    double T1 = get_min_time_1dof(x1, v1, xi, vi);
    double T2 = get_min_time_1dof(xi, vi, x2, v2);
    if(T1 < 0 || T2 < 0) return -1;
    return T1 + T2;
	}

	// returns the minimum time between 2 points ([x1,y1] and [x2,y2]) in the state space
	// returns -1 if the solution does not exist
	double get_min_time_1dof(const double x1, const double v1, const double x2,
			const double v2) const
	{
		double ta1;
    double dp_acc = 0.5*(v1+v2)*std::abs(v2-v1)/a_max_;
    double sigma = sign(x2 - x1 - dp_acc);
    double a2 = -sigma*a_max_;  double a1 = -a2;
    double a = a1; double b = 2*v1; double c = (v2*v2-v1*v1)/(2*a2) - (x2-x1);
    double q = -0.5*(b + sign(b)*std::sqrt(b*b-4*a*c));
    double ta1_a = q/a; double ta1_b = c/q;
    if (ta1_a > 0) ta1 = ta1_a;
    else if (ta1_b > 0) ta1 = ta1_b;
    else if (ta1_a == 0 || ta1_b == 0) return 0;
    else return -1;
    return (v2-v1)/a2 + 2*ta1;
  }

private:
	template<typename T>
	inline T sign(const T& a) const
	{
		return a >= T(0) ? T(1):T(-1);
	}
};