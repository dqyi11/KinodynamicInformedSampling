/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, University of Toronto
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the University of Toronto nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Authors: Cole Gulino, Daqing Yi, Oren Salzman, and Rohan Thakker */

#include <Sampler/RejectionSampler.h>

// Standard library
#include <limits>
#include <math.h>

namespace ompl
{
    namespace base
    {
        ///
        /// Get a series of samples for the problem space
        ///
        /// @param no_samples Number of samples to get
        /// @param time Boolean that determines if the time to run the proccess is
        /// displayed
        /// @return A series of samples of shape (number of samples, sample dimension)
        ///
        Eigen::MatrixXd RejectionSampler::sample(const int numSamples,
                                                 std::chrono::high_resolution_clock::duration &duration)
        {
            // Reset rejection rate
            rejectionRatio_ = 1.0;
            // Get the limits of the space
            Eigen::VectorXd max_vals, min_vals;
            std::tie(max_vals, min_vals) = getStateLimits();
            double max = max_vals(0);
            double min = min_vals(0);

            // Run until you get the correct number of samples
            int numAcceptedSamples = 0;
            int numRejectedSamples = 0;
            Eigen::MatrixXd samples(numSamples, getStartState().size() + 1);

            // If you want to time the sampling
            std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

            while (numAcceptedSamples < numSamples)
            {
                Eigen::VectorXd sample = getRandomSample(max, min, getStartState().size());

                if (isInLevelSet(sample))
                {
                    Eigen::VectorXd newsample(getStartState().size() + 1);
                    newsample << sample, getCost(sample);
                    samples.row(numAcceptedSamples) = newsample;
                    numAcceptedSamples++;

                    //debugging purposes
                    for (int i(0); i < 2; ++i)
                    {
                        double cost1, cost2, infeasible_min, infeasible_max;
                        std::tie(cost1, infeasible_min, infeasible_max) =
                            doubleIntegrator1Dof_.getMinTimeAndIntervals(getStartState().segment(i, 2), newsample.segment(i, 2));
                        std::tie(cost2, infeasible_min, infeasible_max) =
                            doubleIntegrator1Dof_.getMinTimeAndIntervals(newsample.segment(i, 2), getGoalState().segment(i, 2));
                        double cost = cost1 + cost2;

                        assert (cost < levelSet_);
                    }
                }
                else
                {
                    numRejectedSamples++;
                }
            }

            std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
            duration = t2 - t1;

            rejectionRatio_ = static_cast<double>(numAcceptedSamples) /
                              static_cast<double>(numAcceptedSamples+numRejectedSamples);
            return samples;
        }

        // Can implement as many private functions as you want to help do the sampling
        Eigen::VectorXd RejectionSampler::getRandomSample(const double max, const double min, const int size) const
        {
            // Set up the random number generator
            std::mt19937 gen( std::random_device{}());
            std::uniform_real_distribution<> dis(min, max);

            Eigen::VectorXd sample(size);

            for (int i = 0; i < size; i++)
            {
                sample(i) = dis(gen);
            }

            return sample;
        }

        ///
        /// HierarchicalRejectionSampler
        ///

        ///
        /// Get a series of samples for the problem space
        ///
        /// @param no_samples Number of samples to get
        /// @param time Boolean that determines if the time to run the proccess is
        /// displayed
        /// @return A series of samples of shape (number of samples, sample dimension)
        ///
        Eigen::MatrixXd HierarchicalRejectionSampler::sample(const int numSamples,
                                                             std::chrono::high_resolution_clock::duration &duration)
        {
            // Get the limits of the space
            Eigen::VectorXd max_vals, min_vals;
            std::tie(max_vals, min_vals) = getStateLimits();
            double max = max_vals(0);
            double min = min_vals(0);

            // Run until you get the correct number of samples
            int currNumSamples = 0;
            Eigen::MatrixXd samples(numSamples, getStartState().size() + 1);

            // If you want to time the sampling
            std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

            while (currNumSamples < numSamples)
            {
                Eigen::VectorXd sample(getStartState().size());
                HRS(0, dimension_ - 1, sample);

                if (isInLevelSet(sample))
                {
                    Eigen::VectorXd newsample(getStartState().size() + 1);
                    newsample << sample, getCost(sample);
                    samples.row(currNumSamples) = newsample;
                    currNumSamples++;
                }
            }

            // If you want to time the sampling and display it
            std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
            duration = t2 - t1;

            return samples;
        }

        ///
        /// Get one sample using a recursive algorithm of heirarchical rejection
        /// sampling
        ///
        /// @param start_index Start index of the hierarchical sample
        /// @param end_index End index of the hierarchical sample
        /// @param sample Reference to a sample that gets changed in place
        /// @return (c_start, c_goal)
        ///
        std::tuple<double, double> HierarchicalRejectionSampler::HRS(const int startIndex, const int endIndex,
                                                                     Eigen::VectorXd &sample)
        {
            // Initialize the costs
            double cStart = std::numeric_limits<double>::infinity();
            double cGoal = std::numeric_limits<double>::infinity();

            // std::cout << " [ " << sample[0] << ", " << sample[1] <<  ", " <<
            //     sample[2] << ", " << sample[3] << " ]" << std::endl;

            if (startIndex == endIndex)
            {
                while (Cost(cStart) + Cost(cGoal) > getLevelSet())
                {
                    sampleLeaf(sample, startIndex);

                    cStart = calculateLeaf(getStartState(), sample, startIndex);
                    cGoal = calculateLeaf(sample, getGoalState(), startIndex);
                }
            }
            else
            {
                int mid_index = std::floor(startIndex + endIndex) / 2;
                double c_dash_start = std::numeric_limits<double>::infinity();
                double c_dash_goal = std::numeric_limits<double>::infinity();

                while (Cost(cStart) + Cost(cGoal) > getLevelSet())
                {
                    std::tie(cStart, cGoal) = HRS(startIndex, mid_index, sample);
                    std::tie(c_dash_start, c_dash_goal) = HRS(mid_index + 1, endIndex, sample);
                    cStart =
                        combineCosts(getStartState(), sample, startIndex, mid_index, endIndex, cStart, c_dash_start);
                    cGoal =
                        combineCosts(sample, getGoalState(), startIndex, mid_index, endIndex, cGoal, c_dash_goal);
                }
            }

            return std::make_tuple(cStart, cGoal);
        }

        ///
        /// GeometricHierarchicalRejectionSampler
        ///

        ///
        /// Calculates the cost of a leaf node
        ///
        /// @param x1 First state
        /// @param x2 Second state
        /// @param i Index of the degree of freedom
        /// @return Cost to go from x1 to x2
        ///
        double GeometricHierarchicalRejectionSampler::calculateLeaf(const Eigen::VectorXd &x1,
                                                                    const Eigen::VectorXd &x2, const int i)
        {
            return std::pow(x1[i] - x2[i], 2);
        }

        ///
        /// Combines the cost of two states
        ///
        /// @param x1 First state
        /// @param x2 Second state
        /// @param i Index of the degree of freedom for first state
        /// @param m Mid degree of freedom
        /// @param j Index of  the degree of freedom of the second state
        /// @param c1 Cost one
        /// @param c2 Cost two
        /// @return Combination of the costs
        ///
        double GeometricHierarchicalRejectionSampler::combineCosts(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2,
                                                                   const int i, const int m, const int j,
                                                                   const double c1, const double c2) const
        {
            return c1 + c2;
        }

        ///
        /// How to sample a leaf (ex: geometric is one dimension and kino is 2)
        ///
        /// @param sample A vector to the sample
        /// @param dof An index to the degree of freedom to sample
        /// @return A random vector in the space
        ///
        void GeometricHierarchicalRejectionSampler::sampleLeaf(Eigen::VectorXd &sample, const int dof)
        {
            std::uniform_real_distribution<double> dis(min_[dof], max_[dof]);

            sample[dof] = dis(gen_);
        }

        ///
        /// Dimt Hierarchical Rejection Sampler
        ///

        ///
        /// Calculates the cost of a leaf node
        ///
        /// @param x1 First state
        /// @param x2 Second state
        /// @param i Index of the degree of freedom
        /// @return Cost to go from x1 to x2
        ///
        double DimtHierarchicalRejectionSampler::calculateLeaf(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2,
                                                               const int i)
        {
            const auto state1 = x1.segment(i, 2);
            const auto state2 = x2.segment(i, 2);
            const Eigen::Matrix<double, 1, 1> distances = state2.template head<1>() - state1.template head<1>();
            Eigen::Matrix<double, 1, 1> firstAccelerations;

            double cost, infeasible_min, infeasible_max;
            std::tie(cost, infeasible_min, infeasible_max) =
                doubleIntegrator1Dof_.getMinTimeAndIntervals(x1.segment(i, 2), x2.segment(i, 2));

            infeasibleIntervals_[i] = std::make_pair(infeasible_min, infeasible_max);

            costs_[i] = cost;

            return costs_[i];
        }

        std::pair<size_t, double> max_in_range(const std::vector<double> &costs, const size_t start, const size_t end)
        {
            double max = -1;
            size_t index = -1;
            for (size_t i = start; i <= end; i++)
            {
                if (costs[i] > max)
                {
                    max = costs[i];
                    index = i;
                }
            }

            return std::make_pair(index, max);
        }

        std::pair<bool, double> find_infeasible_intervals(const std::vector<std::pair<double, double>> &intervals,
                                                          const double max_val, const size_t start, const size_t end)
        {
            for (size_t i = start; i <= end; i++)
            {
                if (max_val > std::get<0>(intervals[i]) and max_val < std::get<1>(intervals[i]))
                {
                    return std::make_pair(true, std::get<1>(intervals[i]));
                }
            }

            return std::make_pair(false, max_val);
        }

        ///
        /// Combines the cost of two states
        ///
        /// @param x1 First state
        /// @param x2 Second state
        /// @param i Index of the degree of freedom for first state
        /// @param m Mid degree of freedom
        /// @param j Index of  the degree of freedom of the second state
        /// @param c1 Cost one
        /// @param c2 Cost two
        /// @return Combination of the costs
        ///
        double DimtHierarchicalRejectionSampler::combineCosts(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2,
                                                              const int i, const int m, const int j, const double c1,
                                                              const double c2) const
        {
            // Find the index and the max value of cost from the costs in the range
            size_t index;
            double max_val;
            std::tie(index, max_val) = max_in_range(costs_, i, j);

            bool is_invalid = true;
            while (is_invalid)
            {
                std::tie(is_invalid, max_val) = find_infeasible_intervals(infeasibleIntervals_, max_val, i, j);
            }
            return max_val;
        }

        ///
        /// How to sample a leaf (ex: geometric is one dimension and kino is 2)
        ///
        /// @param sample A vector to the sample
        /// @param dof An index to the degree of freedom to sample
        /// @return A random vector in the space
        ///
        void DimtHierarchicalRejectionSampler::sampleLeaf(Eigen::VectorXd &sample, const int dof)
        {
            std::uniform_real_distribution<double> dis1(min_[dof * 2], max_[dof * 2]);
            std::uniform_real_distribution<double> dis2(min_[dof * 2 + 1], max_[dof * 2 + 1]);

            sample[dof * 2] = dis1(gen_);
            sample[dof * 2 + 1] = dis2(gen_);

            // std::cout << "DOF: " << dof << " [ " << sample[dof * 2] << ", " <<
            // sample[dof * 2 + 1] << " ]" << std::endl;
        }
    }  // namespace ompl
}  // namespace base
