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

// #ifndef OMPL_BASE_SAMPLERS_HIT_AND_RUN_
// #define OMPL_BASE_SAMPLERS_HIT_AND_RUN_

#pragma once

// Standard Library Functions
#include <iostream>
#include <chrono>
#include <tuple>
#include <math.h>

// Eigen namespace
#include <Eigen/Dense>

// Include Sampler
#include <Sampler/Sampler.h>

namespace ompl
{
    namespace base
    {
        class GibbsSampler : public MyInformedSampler
        {
        public:
            //
            // Constructor for Gibbs Sampler
            //
            // @param si Space information pointer
            // @param problem OMPL's problem definition
            // @param levelSet Initial level set of the problem
            // @param maxNumberCalls Max number of calls to the sampler
            // @param sampler Sampler that inherits from Sampler.h
            // @param sample_batch_size How many samples to get each time a new
            // batch of samples is gotten
            //
            GibbsSampler(const SpaceInformationPtr &si, const ProblemDefinitionPtr &problem, const double levelSet,
                         const unsigned int maxNumberCalls, const int sampleBatchSize)
              : MyInformedSampler(si, problem, levelSet, maxNumberCalls, sampleBatchSize)
            {
                // Set up the random number generator
                gen_ = std::mt19937(std::random_device{}());

                // Use the start positions as starting point of the algorithm
                // (This will always be inside the informed subspace)
                prev_sample_ = getStartState();
            }

            //
            // Get a series of samples for the problem space
            //
            // @param no_samples Number of samples to get
            // @param time Boolean that determines if the time to run the proccess is
            // displayed
            // @return A series of samples of shape (number of samples, sample dimension)
            //
            virtual Eigen::MatrixXd sample(const int no_samples,
                                           std::chrono::high_resolution_clock::duration &duration) override;

            virtual void updateLevelSet(const double level_set) override;

            std::mt19937 gen_;
            Eigen::VectorXd prev_sample_;

        private:
            //
            // Get one random uniform sample from the space
            //
            // @return Random uniform vector from the space
            //
            virtual Eigen::VectorXd getRandomSample(double min, double max, const int dim);
        };

        class HitAndRun : GibbsSampler
        {
        public:
            HitAndRun(const SpaceInformationPtr &si, const ProblemDefinitionPtr &problem, const double levelSet,
                      const unsigned int maxNumberCalls, const int sampleBatchSize, const int numOfTries=10000)
              : GibbsSampler(si, problem, levelSet, maxNumberCalls, sampleBatchSize),
                numOfTries_(numOfTries)
            {
            }

            //
            // Get a series of samples for the problem space
            //
            // @param no_samples Number of samples to get
            // @param time Boolean that determines if the time to run the proccess is
            // displayed
            // @return A series of samples of shape (number of samples, sample dimension)
            //
            virtual Eigen::MatrixXd sample(const int no_samples,
                                           std::chrono::high_resolution_clock::duration &duration) override;
        protected:
            int numOfTries_;
        };
    }  // namespace base
}  // namespace ompl

// #endif // OMPL_BASE_SAMPLERS_INFORMED_SAMPLER_
