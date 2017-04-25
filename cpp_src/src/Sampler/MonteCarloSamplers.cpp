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

#include "Sampler/MonteCarloSamplers.h"

// Standard library functions
#include <math.h> /* exp, tanh, log */
#include <limits>
#include <algorithm>

namespace
{
    void print_out_states3(const Eigen::VectorXd &state)
    {
        std::cout << "[ ";
        for (uint i = 0; i < state.size(); i++)
        {
            std::cout << state[i] << " ";
        }
        std::cout << " ]" << std::endl;
    }

    // Verbose constant
    const bool VERBOSE = false;

    ///
    /// Sigmoid function
    ///
    /// @param x Input
    /// @param a Controls shape of sigmoid
    /// @param c Controls shape of sigmoid
    /// @return Output of sigmoid function
    ///
    inline double sigmoid(const double &x, const double &a = 200, const double &c = 0)
    {
        return 1 / (1 + exp(-a * (x - c)));
    }

    ///
    /// Function to get some value between 0 and 1
    ///
    /// @return A uniform value between 0 and 1
    ///
    inline double rand_uni()
    {
        // Set up the random number generator
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> dis(0.0, 1.0);

        return dis(gen);
    }

}  // namespace

namespace ompl
{
    namespace base
    {
        ///
        /// Function to determine if any of the joint limits are violated
        /// @param sample Sample to check
        /// @return Boolean that is true if any are in violation
        ///
        bool MonteCarloSampler::anyDimensionInViolation(const Eigen::VectorXd &sample) const
        {
            const std::tuple<Eigen::VectorXd, Eigen::VectorXd> limits = getStateLimits();
            const auto min_vals = std::get<0>(limits);
            const auto max_vals = std::get<1>(limits);

            for (uint i = 0; i < sample.size(); i++)
            {
                if (sample[i] > max_vals[i] or sample[i] < min_vals[i])
                {
                    return true;
                }
            }

            return false;
        }

        ///
        /// Get the energy of the state from the cost function
        ///
        /// @param curr_state Current state to get the energy for
        /// @return Energy of the function
        ///
        double MonteCarloSampler::getEnergy(const Eigen::VectorXd &curr_state) const
        {
            const double cost = getCost(curr_state);
            // double E_grad = tanh(cost);
            const double E_grad = log(1 + log(1 + cost));
            const double E_informed = 100 * sigmoid(cost - getLevelSet());
            double E_region = 0;
            std::tuple<Eigen::VectorXd, Eigen::VectorXd> limits = getStateLimits();
            for (int i = 0; i < curr_state.size(); i++)
            {
                E_region += 100 * sigmoid(std::get<0>(limits)(i) - curr_state(i));  // Lower Limits
                E_region += 100 * sigmoid(curr_state(i) - std::get<1>(limits)(i));  // Higher Limits
            }
            return E_region + E_grad + E_informed;
        }

        ///
        /// Get the probability of the state from the cost function
        ///
        /// @param energy Energy of the state
        /// @return Probability of the state
        ///
        double MonteCarloSampler::getProb(const double energy) const
        {
            return exp(-energy);
        }

        ///
        /// Get the probability of the state from the cost function
        ///
        /// @param curr_state Current state to get the energy for
        /// @return Probability of the state
        ///
        double MonteCarloSampler::getProb(const Eigen::VectorXd &curr_state) const
        {
            return exp(-MonteCarloSampler::getEnergy(curr_state));
        }

        ///
        /// Get one random uniform sample from the space
        ///
        /// @return Random uniform vector of length size
        ///
        Eigen::VectorXd MonteCarloSampler::getRandomSample() const
        {
            // Set up the random number generator
            std::random_device rd;
            std::mt19937 gen(rd());
            // Get the limits of the space
            Eigen::VectorXd max_vals, min_vals;
            std::tie(max_vals, min_vals) = getStateLimits();

            int size = getSpaceDimension();
            Eigen::VectorXd sample(size);
            for (int i = 0; i < size; i++)
            {
                // Get a random distribution between the values of the joint
                double min = min_vals(i);
                double max = max_vals(i);
                std::uniform_real_distribution<> dis(min, max);

                sample(i) = dis(gen);
            }

            return sample;
        }

        ///
        /// Function to concatenate a vector to back of matrix
        ///
        /// @param matrix Matrix to concatenate to bottom of
        /// @param vector Vector to contatenate to bottom of matrix
        /// @return A new matrix with the concatenation
        ///
        Eigen::MatrixXd concatenate_matrix_and_vector(const Eigen::MatrixXd &matrix, const Eigen::VectorXd &vector)
        {
            // Concatenate to results matrix
            Eigen::MatrixXd new_results(matrix.rows() + 1, matrix.cols());
            new_results << matrix, vector.transpose();
            return new_results;
        }

        // ///
        // /// Surf down the cost function to get to the levelset
        // ///
        // /// @param alpha Learning rate
        // /// @return Path to the level set
        // ///
        // VectorXd MonteCarloSampler::grad_descent(const double& alpha) const
        // {
        //  VectorXd start = MonteCarloSampler::get_random_sample();
        //  double cost = problem().get_cost(start);

        //  int steps = 0;
        //     while(cost > problem().level_set())
        //     {
        //         double last_cost = cost;
        //         VectorXd inv_jacobian = problem().get_inv_jacobian(start);
        //         //std::cout << "inv jacobian " << inv_jacobian << std::endl;
        //         start = start - inv_jacobian * cost;
        //      cost = problem().get_cost(start);
        //      steps++;

        //      // If the number of steps reaches some threshold, start over
        //      const double thresh = 50;
        //      if( abs(last_cost - cost) < 0.0001 )
        //         //if(steps > thresh)
        //      {
        //          //std::cout << "RESTART GRAD DESCENT" << std::endl;
        //          //return grad_descent();
        //          start = MonteCarloSampler::get_random_sample();
        //          cost = problem().get_cost(start);
        //      }
        //  }

        //  return start;
        // }

        ///
        /// Surf down the cost function to get to the levelset
        ///
        /// @param alpha Learning rate
        /// @return Path to the level set
        ///
        Eigen::VectorXd MonteCarloSampler::gradDescent(const double alpha) const
        {
            Eigen::VectorXd start = MonteCarloSampler::getRandomSample();
            double cost = getCost(start);
            double prev_cost = cost;

            int steps = 0;
            while (cost > getLevelSet())
            {
                Eigen::VectorXd grad = getGradient(start);
                start = start - alpha * grad;

                cost = getCost(start);

                if (VERBOSE)
                    std::cout << cost << std::endl;

                steps++;

                // If the number of steps reaches some threshold, start over
                const double thresh = 20;
                // if(steps > thresh || cost > prev_cost)
                if (steps > thresh || cost > 10 * getLevelSet() || grad.norm() < 0.001)
                {
                    if (VERBOSE)
                        std::cout << "Restarting!" << std::endl;
                    // recursing gives segfaults so just change the start position instead
                    // return grad_descent(alpha);
                    start = MonteCarloSampler::getRandomSample();
                    steps = 0;
                }
                prev_cost = cost;
            }

            return start;
        }

        ///
        /// Surf down the cost function to get to the levelset
        ///
        /// @param start Vector to start
        /// @return A state in the level set
        ///
        Eigen::VectorXd MonteCarloSampler::newtonRaphson(const Eigen::VectorXd &start) const
        {
            Eigen::VectorXd end = start;
            double cost = getCost(end);

            int steps = 0;
            while (cost > getLevelSet())
            {
                double last_cost = cost;
                Eigen::VectorXd inv_jacobian = getInvJacobian(end);
                end = end - inv_jacobian * cost;
                cost = getCost(end);
                steps++;

                // If the number of steps reaches some threshold, start over
                const double trap_threshold = 0.0001;
                if (last_cost - cost < trap_threshold)
                {
                    end = MonteCarloSampler::getRandomSample();
                    cost = getCost(end);
                }
            }

            return end;
        }

        ///
        /// Get a normal random vector for the momentum
        ///
        /// @return A vector of momentum sampled from a random distribution
        ///
        Eigen::VectorXd MonteCarloSampler::sampleNormal(const double mean, const double sigma) const
        {
            // Set up the random number generator
            std::random_device rd;
            std::mt19937 gen(rd());
            std::normal_distribution<double> dis(mean, sigma);

            int size = getSpaceDimension();
            Eigen::VectorXd sample(size);
            for (int i = 0; i < size; i++)
            {
                sample(i) = dis(gen);
            }

            return sample;
        }

        Eigen::MatrixXd HMCSampler::sampleBatchMemorized(const int numSamples,
                                                         std::chrono::high_resolution_clock::duration &duration)
        {
            Eigen::MatrixXd samples(1, getSpaceDimension() + 1);
            // If you want to time the sampling
            std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

            for (int i = 0; i < numSamples; i++)
            {
                Eigen::VectorXd newsample = sampleMemorized();
                samples = concatenate_matrix_and_vector(samples, newsample);
            }

            std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
            duration = t2 - t1;

            return samples;
        }

        Eigen::VectorXd HMCSampler::sampleMemorized()
        {
            // last sample
            Eigen::VectorXd q = Eigen::VectorXd(getStartState().size());
            for (unsigned int i = 0; i < getStartState().size(); i++)
            {
                q[i] = lastSample_[i];
            }

            if (getCurrentStep() < 0)
            {
                Eigen::VectorXd start = MonteCarloSampler::getRandomSample();
                // q = newton_raphson(start);
                q = gradDescent(getAlpha());
            }
            updateCurrentStep();

            // Sample the momentum and set up the past and current state and momentum
            Eigen::VectorXd q_last = q;
            Eigen::VectorXd p = MonteCarloSampler::sampleNormal(0, getSigma());
            Eigen::VectorXd p_last = p;

            if (VERBOSE)
                std::cout << "Sampled the momentum" << std::endl;

            // Make a half step for momentum at the beginning
            Eigen::VectorXd grad = getGradient(q);
            if (VERBOSE)
                std::cout << "Got the gradient" << std::endl;

            // Ensure that the gradient isn't two large
            while (grad.maxCoeff() > 1e2)
            {
                if (VERBOSE)
                    std::cout << "WARNING: Gradient too high" << std::endl;

                Eigen::VectorXd start = MonteCarloSampler::getRandomSample();
                q = newtonRaphson(start);
                grad = getGradient(q);
            }

            p = p - getEpsilon() * grad / 2;

            // Alternate Full steps for q and p
            for (int i = 0; i < getL(); i++)
            {
                q = q + getEpsilon() * p;
                if (i != getL())
                    p = p - getEpsilon() * grad;
            }

            if (VERBOSE)
                std::cout << "Integrated Along momentum" << std::endl;

            // Make a half step for momentum at the end
            p = p - getEpsilon() * grad / 2;

            // Negate the momentum at the end of the traj to make proposal
            // symmetric
            p = -p;

            // Evaluate potential and kinetic energies at start and end of traj
            double U_last = getEnergy(q_last);
            double K_last = p_last.norm() / 2;
            double U_proposed = getEnergy(q);
            double K_proposed = p_last.norm() / 2;

            if (VERBOSE)
                std::cout << "Got energies" << std::endl;

            // Accept or reject the state at the end of trajectory
            double alpha = std::min(1.0, std::exp(U_last - U_proposed + K_last - K_proposed));
            if (rand_uni() > alpha)
            {
                q = q_last;
            }

            Eigen::VectorXd newsample(getStartState().size() + 1);
            newsample << q, getCost(q);

            lastSample_ = newsample;
            if (getCurrentStep() >= getSteps())
            {
                updateCurrentStep(-1);
            }

            return newsample;
        }

        //
        // HMCSampler
        //

        ///
        /// Get a series of samples for the problem space
        ///
        /// @param no_samples Number of samples to get
        /// @param time Boolean that determines if the time to run the proccess is
        /// displayed
        /// @return A series of samples of shape (number of samples, sample dimension)
        ///
        Eigen::MatrixXd HMCSampler::sample(const int numSamples, std::chrono::high_resolution_clock::duration &duration)
        {
            if (VERBOSE)
                std::cout << "Number of samples: " << numSamples << std::endl;
            if (VERBOSE)
                std::cout << "Surfing" << std::endl;
            Eigen::VectorXd q = HMCSampler::gradDescent(getAlpha());
            if (VERBOSE)
                std::cout << "Got Through Gradient Descent" << std::endl;

            // Store the samples
            Eigen::MatrixXd samples(1, getSpaceDimension() + 1);
            samples << q.transpose(), getCost(q);

            int accepted = 0;
            int rejected = 0;
            // If you want to time the sampling
            std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
            while (accepted < numSamples)
            {
                if (VERBOSE)
                    std::cout << "New start!" << std::endl;
                Eigen::VectorXd q = HMCSampler::gradDescent(getAlpha());
                if (VERBOSE)
                    std::cout << "Got Through Gradient Descent in loop" << std::endl;

                int curr_rejections = 0;
                int curr_step = 0;
                while (curr_rejections < 10 and accepted < numSamples and curr_step < getSteps())
                {
                    // Sample the momentum and set up the past and current state and momentum
                    Eigen::VectorXd q_last = q;
                    Eigen::VectorXd p = MonteCarloSampler::sampleNormal(0, getSigma());
                    Eigen::VectorXd p_last = p;

                    if (VERBOSE)
                        std::cout << "Sampled the momentum" << std::endl;

                    // Make a half step for momentum at the beginning
                    Eigen::VectorXd grad = getGradient(q);
                    if (VERBOSE)
                        std::cout << "Got the gradient" << std::endl;

                    // Ensure that the gradient isn't two large
                    if (grad.maxCoeff() > 1e2)
                    {
                        if (VERBOSE)
                            std::cout << "WARNING: Gradient too high" << std::endl;
                        break;
                    }

                    p = p - getEpsilon() * grad / 2;

                    // Alternate Full steps for q and p
                    for (int i = 0; i < getL(); i++)
                    {
                        q = q + getEpsilon() * p;
                        if (i != getL())
                            p = p - getEpsilon() * grad;
                    }

                    if (VERBOSE)
                        std::cout << "Integrated Along momentum" << std::endl;

                    // Make a half step for momentum at the end
                    p = p - getEpsilon() * grad / 2;

                    // Negate the momentum at the end of the traj to make proposal
                    // symmetric
                    p = -p;

                    // Evaluate potential and kinetic energies at start and end of traj
                    double U_last = getEnergy(q_last);
                    double K_last = p_last.norm() / 2;
                    double U_proposed = getEnergy(q);
                    double K_proposed = p_last.norm() / 2;

                    if (VERBOSE)
                        std::cout << "Got energies" << std::endl;

                    // Accept or reject the state at the end of trajectory
                    double alpha = std::min(1.0, std::exp(U_last - U_proposed + K_last - K_proposed));
                    if (rand_uni() <= alpha)
                    {
                        Eigen::VectorXd newsample(getStartState().size() + 1);
                        newsample << q, getCost(q);
                        samples = concatenate_matrix_and_vector(samples, newsample);
                        accepted++;
                    }
                    else
                    {
                        q = q_last;
                        rejected++;
                        curr_rejections++;
                        // std::cout << "Current Rejections: " << curr_rejections << std::endl;
                    }

                    curr_step++;
                    if (VERBOSE)
                        std::cout << "Decided on rejection / acceptance" << std::endl;
                    if (VERBOSE)
                        std::cout << "Number Accepted: " << accepted << std::endl;
                    if (VERBOSE)
                        std::cout << "Number Accepted: " << accepted << std::endl;
                }
            }

            std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
            duration = t2 - t1;

            if (VERBOSE)
                std::cout << "Percentage Accepted: " << (accepted + 0.0) / (rejected + accepted) << std::endl;

            return samples;
        }

        ///
        /// MCMC Sampler
        ///

        ///
        /// Get a series of samples for the problem space
        ///
        /// @param no_samples Number of samples to get
        /// @param time Boolean that determines if the time to run the proccess is
        /// displayed
        /// @return A series of samples of shape (number of samples, sample dimension)
        ///
        Eigen::MatrixXd MCMCSampler::sample(const int no_samples,
                                            std::chrono::high_resolution_clock::duration &duration)
        {
            if (VERBOSE)
                std::cout << "Number of samples: " << no_samples << std::endl;
            if (VERBOSE)
                std::cout << "Surfing" << std::endl;
            Eigen::VectorXd q = MCMCSampler::gradDescent(getAlpha());
            if (VERBOSE)
                std::cout << "Got Through Gradient Descent" << std::endl;

            // Store the samples
            Eigen::MatrixXd samples(1, getSpaceDimension() + 1);
            samples << q.transpose(), getCost(q);

            int accepted = 0;
            int rejected = 0;
            // If you want to time the sampling
            std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
            while (accepted < no_samples)
            {
                if (samples.rows() > 1)
                {
                    if (VERBOSE)
                        std::cout << "New start!" << std::endl;
                    Eigen::VectorXd q = MCMCSampler::gradDescent(getAlpha());
                    if (VERBOSE)
                        std::cout << "Got Through Gradient Descent in loop" << std::endl;
                }

                int curr_rejections = 0;
                int curr_step = 0;
                while (curr_rejections < 10 and accepted < no_samples and curr_step < getSteps())
                {
                    Eigen::VectorXd q_proposed = q + sampleNormal(0, getSigma());
                    double prob_proposed = getProb(q_proposed);
                    double prob_before = getProb(q);

                    // if(prob_proposed / prob_before >= rand_uni() and
                    //    // !any_dimensions_in_violation(q_proposed))
                    if (prob_proposed / prob_before >= rand_uni())
                    {
                        Eigen::VectorXd newsample(getStartState().size() + 1);
                        newsample << q_proposed, getCost(q_proposed);
                        samples = concatenate_matrix_and_vector(samples, newsample);
                        accepted++;
                        q = q_proposed;
                    }
                    else
                    {
                        rejected++;
                        curr_rejections++;
                        if (VERBOSE)
                            std::cout << "Rejected!" << std::endl;
                    }
                    curr_step++;
                }

                if (VERBOSE)
                    std::cout << "Number of accepted: " << accepted << std::endl;
            }

            if (VERBOSE)
                std::cout << "Number of accepted: " << accepted << std::endl;

            std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
            duration = t2 - t1;

            if (VERBOSE)
                std::cout << "Percentage Accepted: " << (accepted + 0.0) / (rejected + accepted) << std::endl;

            return samples;
        }

    }  // base
}  // ompl
