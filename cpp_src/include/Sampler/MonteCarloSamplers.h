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

/* Authors: Cole Gulino and Rohan Thakker */

// #ifndef OMPL_BASE_SAMPLERS_INFORMED_MONTE_CARLO_SAMPLERS_
// #define OMPL_BASE_SAMPLERS_INFORMED_MONTE_CARLO_SAMPLERS_

#pragma once

// Our Libs
#include <Sampler/Sampler.h>

// Eigen
#include <Eigen/Dense>

// stdlib
#include <iostream>

namespace ompl
{
    namespace base
    {
        class MonteCarloSampler: public MyInformedSampler
        {
        public:
            ///
            /// Constructor for HMC Sampler (calls super class constructor)
            ///
            /// @param si Space information pointer
            /// @param problem OMPL's problem definition
            /// @param levelSet Initial level set of the problem
            /// @param maxNumberCalls Max number of calls to the sampler
            /// @param sampler Sampler that inherits from Sampler.h
            /// @param sample_batch_size How many samples to get each time a new
            /// batch of samples is gotten
            /// @param alpha Step size of gradient descent
            ///
            MonteCarloSampler(const SpaceInformationPtr &si,
                              const ProblemDefinitionPtr &problem,
                              const double levelSet,
                              const unsigned int maxNumberCalls,
                              const int sampleBatchSize,
                              const double alpha)
                : MyInformedSampler(si, problem, levelSet, maxNumberCalls, sampleBatchSize), alpha_(alpha)
            { }

            ///
            /// Get alpha - learning rate for gradient descent
            ///
            double getAlpha() const { return alpha_; }

            ///
            /// Get a series of samples for the problem space
            ///
            /// @param no_samples Number of samples to get
            /// @param time Boolean that determines if the time to run the proccess is displayed
            /// @return A series of samples of shape (number of samples, sample dimension)
            ///
            virtual Eigen::MatrixXd sample(const int no_samples,
                                           std::chrono::high_resolution_clock::duration &duration) = 0;

            ///
            /// Surf down the cost function to get to the levelset
            ///
            /// @param alpha Learning rate
            /// @return Path to the level set
            ///
            virtual Eigen::VectorXd gradDescent(const double alpha = 0.01) const;

            ///
            /// Surf down the cost function to get to the levelset
            ///
            /// @param start Starting state
            /// @return Path to the level set
            ///
            virtual Eigen::VectorXd newtonRaphson(const Eigen::VectorXd &start) const;

            ///
            /// Get the energy of the state from the cost function
            ///
            /// @param curr_state Current state to get the energy for
            /// @return Energy of the state
            ///
            virtual double getEnergy(const Eigen::VectorXd &curr_state) const;

            ///
            /// Get the probability of the state from the cost function
            ///
            /// @param energy Energy of the state
            /// @return Probability of the state
            ///
            virtual double getProb(const double energy) const;

            ///
            /// Get the probability of the state from the cost function
            ///
            /// @param curr_state Current state to get the energy for
            /// @return Probability of the state
            ///
            virtual double getProb(const Eigen::VectorXd &curr_state) const;

            ///
            /// Get one random uniform sample from the space
            ///
            /// @return Random uniform vector from the space
            ///
            virtual Eigen::VectorXd getRandomSample() const;

            ///
            /// Get a normal random vector for the momentum
            ///
            /// @param mean Mean of the normal distribution
            /// @paramm sigma Sigma of the normal distribution
            /// @return A vector of momentum sampled from a random distribution
            ///
            Eigen::VectorXd sampleNormal(const double mean, const double sigma) const;

        private:
            // Learning rate for gradient descent
            double alpha_;

        protected:
            ///
            /// Function to determine if any of the joint limits are violated
            /// @param sample Sample to check
            /// @return Boolean that is true if any are in violation
            ///
            bool anyDimensionInViolation(const Eigen::VectorXd &sample) const;
        }; // MonteCarloSampler

        class HMCSampler: public MonteCarloSampler
        {
        public:
            ///
            /// Constructor for HMC Sampler (calls super class constructor)
            ///
            /// @param si Space information pointer
            /// @param problem OMPL's problem definition
            /// @param levelSet Initial level set of the problem
            /// @param maxNumberCalls Max number of calls to the sampler
            /// @param sampler Sampler that inherits from Sampler.h
            /// @param sample_batch_size How many samples to get each time a new
            /// batch of samples is gotten
            /// @param alpha Learning rate for gradient descent
            /// @param L Distance of integration for HMC step
            /// @param epsilon Integration constant for HMC
            /// @param sigma Sampling the momentum as a normal distribution
            /// @param steps Number of steps to run HMC for each chain
            ///
            HMCSampler(const SpaceInformationPtr &si,
                       const ProblemDefinitionPtr &problem,
                       const double levelSet,
                       const unsigned int maxNumberCalls,
                       const int sampleBatchSize,
                       const double& alpha,
                       const double& L,
                       const double& epsilon,
                       const double& sigma,
                       const int& steps)
                : MonteCarloSampler(si, problem, levelSet, maxNumberCalls, sampleBatchSize, alpha),
                  L_(L), epsilon_(epsilon), sigma_(sigma),
                  steps_(steps), currentStep_(-1)
                {
                    lastSample_ = Eigen::VectorXd(getStartState().size()+1);
                }

            ///
            /// Get L - Distance of integration for HMC step
            ///
            double getL() const { return L_; }

            ///
            /// Get epsilon - Integration constant for HMC
            ///
            double getEpsilon() const { return epsilon_; }

            ///
            /// Get sigma - Sampling the momentum as a normal distribution
            ///
            double getSigma() const { return sigma_; }

            ///
            /// Get steps - Number of steps to run HMC for each chain
            ///
            double getSteps() const { return steps_; }

            ///
            /// Get current step
            ///
            int getCurrentStep() const { return currentStep_; }

            ///
            /// Get last sample
            ///
            Eigen::VectorXd getLastSample() const { return lastSample_; }

            ///
            /// Update current step
            ///
            void updateCurrentStep(int currentStep) { currentStep_ = currentStep; }

            ///
            /// Update current step
            ///
            void updateCurrentStep() { currentStep_++; }

            ///
            /// Get a series of samples for the problem space
            ///
            /// @param noSamples Number of samples to get
            /// @param time Boolean that determines if the time to run the proccess is displayed
            /// @return A series of samples of shape (number of samples, sample dimension)
            ///
            virtual Eigen::MatrixXd sample(const int noSamples,
                                           std::chrono::high_resolution_clock::duration &duration);

            virtual Eigen::MatrixXd sampleBatchMemorized(const int no_samples,
                                                         std::chrono::high_resolution_clock::duration &duration);
            virtual Eigen::VectorXd sampleMemorized();

        private:
            // Distance of integration for HMC step
            double L_;

            // Integration constant for HMC
            double epsilon_;

            // Sigma for sampling the momentum as a normal distribution
            double sigma_;

            // Number of steps to run HMC
            double steps_;

            int currentStep_;

            // Last sample
            Eigen::VectorXd lastSample_;
        };

        class MCMCSampler: public MonteCarloSampler
        {
        public:
            ///
            /// Constructor for HMC Sampler (calls super class constructor)
            ///
            /// @param si Space information pointer
            /// @param problem OMPL's problem definition
            /// @param levelSet Initial level set of the problem
            /// @param maxNumberCalls Max number of calls to the sampler
            /// @param sampler Sampler that inherits from Sampler.h
            /// @param sample_batch_size How many samples to get each time a new
            /// batch of samples is gotten
            /// @param alpha Learning rate for gradient descent
            /// @param sigma Sigma for sampling the step
            /// @param steps Number of steps to run MCMC for each chain
            ///
            MCMCSampler(const SpaceInformationPtr &si,
                        const ProblemDefinitionPtr &problem,
                        const double levelSet,
                        const unsigned int maxNumberCalls,
                        const int sampleBatchSize,
                        const double alpha,
                        const double sigma,
                        const double steps)
            : MonteCarloSampler(si, problem, levelSet, maxNumberCalls, sampleBatchSize, alpha),
              sigma_(sigma), steps_(steps)
            { }

            ///
            /// Get a series of samples for the problem space
            ///
            /// @param no_samples Number of samples to get
            /// @param time Boolean that determines if the time to run the proccess is displayed
            /// @return A series of samples of shape (number of samples, sample dimension)
            ///
            virtual Eigen::MatrixXd sample(const int no_samples,
                                           std::chrono::high_resolution_clock::duration &duration);

            ///
            /// Get Sigma for sampling the step
            ///
            double getSigma() const { return sigma_; }

            ///
            /// Get number of steps to run for each MCMC chain
            ///
            double getSteps() const { return steps_; }

        private:
            // Sigma for sampling the step
            double sigma_;

            // Number of steps to run MCMC for each chain
            double steps_;
        };
    } // base
} // ompl
