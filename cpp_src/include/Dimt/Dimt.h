#pragma once

// Standard libarary
#include <iostream>
#include <utility>    // std::pair
#include <limits>     // std::numeric_limits::infinity()
#include <algorithm>  // std::max and std::min
#include <vector>     // std::vector

// Eigen
#include <Eigen/Dense>
using Eigen::VectorXd;

// VERBOSE constant
const bool VERBOSE = false;

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
        return (x1.size() == x2.size() and x1.size() == xi.size()) and
               (x1.size() != 0 and x2.size() != 0 and xi.size() != 0) and (x1.size() % 2 == 0);
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
        if (!is_valid_states(x1, x2, xi))
        {
            std::string error_msg = "Invalid Arguments to get_min_time";
            throw std::invalid_argument(error_msg);
        }

        return get_min_time(x1, xi) + get_min_time(xi, x2);
    }

    // This function calculates the maximum time given a set number of joints
    // x = [x_1, x_1_dot,...,x_n,x_n_dot]
    // @param x1 Initial state
    // @param x2 Final state
    // @return T Maximum time
    double get_min_time(const VectorXd x1, const VectorXd x2) const
    {
        int joint = 0;

        double min_time = -1;
        double slow_dof = -1;

        // Create the vectors to hold the infeasible times and
        // allocate space
        std::vector<std::pair<double, double>> infeasible;
        infeasible.reserve(x1.size() * 2);

        if (VERBOSE)
            std::cout << "Getting min time." << std::endl;
        while (joint < x1.size())
        {
            const double time = get_min_time_1dof(x1(joint), x1(joint + 1), x2(joint), x2(joint + 1));

            if (time < min_time)
            {
                infeasible[joint] = get_infeasible_time_interval(x1(joint), x1(joint + 1), x2(joint), x2(joint + 1));
            }
            else
            {
                min_time = time;
                if (slow_dof != -1)
                {
                    infeasible[slow_dof] =
                        get_infeasible_time_interval(x1(slow_dof), x1(slow_dof + 1), x2(slow_dof), x2(slow_dof + 1));
                }
                slow_dof = joint;
            }

            if (VERBOSE)
                std::cout << "Checking previous joints" << std::endl;
            int j = 0;
            while (j <= joint)
            {
                if (j != slow_dof)
                {
                    if (min_time >= infeasible[j].first and min_time <= infeasible[j].second)
                    {
                        min_time = infeasible[j].second;
                        slow_dof = j;

                        j = 0;
                        if (slow_dof != -1)
                        {
                            infeasible[slow_dof] = get_infeasible_time_interval(x1(slow_dof), x1(slow_dof + 1),
                                                                                x2(slow_dof), x2(slow_dof + 1));
                            slow_dof = -1;
                        }
                    }
                }
                j++;
            }
            if (VERBOSE)
                std::cout << "Checked previous joints" << std::endl;
            joint += 2;
        }

        if (VERBOSE)
            std::cout << "Finished getting min time" << std::endl;

        return min_time;
    }

    // returns the minimum time between 3 points ([x1,y1] -> [xi,yi] -> [x2,y2])
    // in the state space
    // returns -1 if the solution does not exist
    double get_min_time_1dof(const double x1, const double v1, const double x2, const double v2, const double xi,
                             const double vi) const
    {
        double T1 = get_min_time_1dof(x1, v1, xi, vi);
        double T2 = get_min_time_1dof(xi, vi, x2, v2);
        if (T1 < 0 || T2 < 0)
            return -1;
        return T1 + T2;
    }

    // returns the minimum time between 2 points ([x1,y1] and [x2,y2]) in the
    // state space
    // returns -1 if the solution does not exist
    double get_min_time_1dof(const double x1, const double v1, const double x2, const double v2) const
    {
        double ta1;
        const double dp_acc = 0.5 * (v1 + v2) * std::abs(v2 - v1) / a_max_;
        const double sigma = sign(x2 - x1 - dp_acc);
        const double a2 = -sigma * a_max_;
        const double a1 = -a2;
        const double a = a1;
        const double b = 2 * v1;
        const double c = (v2 * v2 - v1 * v1) / (2 * a2) - (x2 - x1);
        const double q = -0.5 * (b + sign(b) * std::sqrt(b * b - 4 * a * c));
        const double ta1_a = q / a;
        const double ta1_b = c / q;
        if (ta1_a > 0)
            ta1 = ta1_a;
        else if (ta1_b > 0)
            ta1 = ta1_b;
        else if (ta1_a == 0 || ta1_b == 0)
            return 0;
        else
            return -1;
        return (v2 - v1) / a2 + 2 * ta1;
    }

    ///
    /// Function to get the infeasible time_interval for one DOF
    ///
    /// @param x1 start position
    /// @param v1 start velocity
    /// @param x2 end position
    /// @param v2 end velocity
    /// return A pair of values (min_time, max_time) is infinity if not in region
    ///
    std::pair<double, double> get_infeasible_time_interval(const double x1, const double v1, const double x2,
                                                           const double v2) const
    {
        // Calculate a1 and a2 with reverse signs as descussed in the paper:
        // http://www.tobiaskunz.net/pubs/KunzIROS14-DimtRrt.pdf
        double ta1;
        const double dp_acc = 0.5 * (v1 + v2) * std::abs(v2 - v1) / a_max_;
        const double distance = x2 - x1 - dp_acc;
        const double sigma = sign(distance);
        const double a2 = sigma * a_max_;
        const double a1 = -a2;

        // Check to see if there is no infeasible time interval because it
        // is not in the correct region
        if (v1 * v2 <= 0.0 or a1 * v1 < 0.0)
        {
            return std::make_pair(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
        }

        // Again determine if there is an infeasible time interval
        const double zeroTime1 = std::abs(v1) / a_max_;
        const double zeroTime2 = std::abs(v2) / a_max_;
        const double zeroDistance = zeroTime1 * v1 / 2.0 + zeroTime2 * v2 / 2.0;
        if (std::abs(zeroDistance) < std::abs(distance))
        {
            // No infeasible time interval, because it is not in the correct region
            return std::make_pair(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
        }

        // If it is in the right region, return the infesible time interval
        const double a = a1;
        const double b = 2 * v1;
        const double c = (v2 * v2 - v1 * v1) / (2 * a2) - (x2 - x1);
        const double q = -0.5 * (b + sign(b) * std::sqrt(b * b - 4 * a * c));
        const double ta1_a = q / a;
        const double ta1_b = c / q;
        return std::make_pair(std::min(ta1_a, ta1_b), std::max(ta1_a, ta1_b));
    }

private:
    template <typename T>
    inline T sign(const T &a) const
    {
        return a >= T(0) ? T(1) : T(-1);
    }
};