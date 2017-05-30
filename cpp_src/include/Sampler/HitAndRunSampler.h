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
            ///
            /// Constructor for Gibbs Sampler
            ///
            /// @param si Space information pointer
            /// @param problem OMPL's problem definition
            /// @param levelSet Initial level set of the problem
            /// @param maxNumberCalls Max number of calls to the sampler
            /// @param sampler Sampler that inherits from Sampler.h
            /// @param sample_batch_size How many samples to get each time a new
            /// batch of samples is gotten
            ///
            GibbsSampler(const SpaceInformationPtr &si, const ProblemDefinitionPtr &problem, const double levelSet,
                         const unsigned int maxNumberCalls, const int sampleBatchSize)
              : MyInformedSampler(si, problem, levelSet, maxNumberCalls, sampleBatchSize)
            {
                // Use the start positions as starting point of the algorithm
                // (This will always be inside the informed subspace)
                prev_sample_ = getStartState();
            }

            ///
            /// Get a series of samples for the problem space
            ///
            /// @param numSamples Number of samples to get
            /// @param time Boolean that determines if the time to run the proccess is
            /// displayed
            /// @return A series of samples of shape (number of samples, sample dimension)
            ///
            virtual Eigen::MatrixXd sample(const uint numSamples,
                                           std::chrono::high_resolution_clock::duration &duration) override;

            virtual bool sampleInLevelSet(Eigen::VectorXd& sample) override { return false; }

            virtual void updateLevelSet(const double level_set) override;

            Eigen::VectorXd prev_sample_;

        private:
            ///
            /// Get one random uniform sample from the space
            ///
            /// @return Random uniform vector from the space
            ///
            virtual Eigen::VectorXd getRandomSample(double min, double max, const int dim);
        };

        class HitAndRunSampler : public GibbsSampler
        {
        public:
            HitAndRunSampler(const SpaceInformationPtr &si, const ProblemDefinitionPtr &problem, const double levelSet,
                      const unsigned int maxNumberCalls, const int sampleBatchSize, const int numOfTries=10000)
              : GibbsSampler(si, problem, levelSet, maxNumberCalls, sampleBatchSize),
                numOfTries_(numOfTries)
            {
                diagonalLength_ = 0.0;
                for (uint i = 0; i < getSpaceDimension(); i++)
                    diagonalLength_ = diagonalLength_ + (stateMax_[i] - stateMin_[i]) * (stateMax_[i] - stateMin_[i]);
                diagonalLength_ = std::sqrt(diagonalLength_);

                numOfPrevSamples_ = 6;
                prevSamples_.reserve(numOfPrevSamples_);
                headOfPrevSamples_ = -1;
            }

            ///
            /// Get a series of samples for the problem space
            ///
            /// @param numSamples Number of samples to get
            /// @param time Boolean that determines if the time to run the proccess is
            /// displayed
            /// @return A series of samples of shape (number of samples, sample dimension)
            ///
            virtual Eigen::MatrixXd sample(const uint numSamples,
                                           std::chrono::high_resolution_clock::duration &duration) override;

            virtual bool sampleInLevelSet(Eigen::VectorXd& sample) override;

            void pushPrevSamples(Eigen::VectorXd& sample)
            {
                if(prevSamples_.size() < numOfPrevSamples_)
                {
                    prevSamples_.push_back(sample);
                }
                else {
                    prevSamples_[headOfPrevSamples_] = sample;
                }
                headOfPrevSamples_ = (headOfPrevSamples_ +1) % numOfPrevSamples_;
                selectPrevSample();
            }

            void selectPrevSample()
            {
                if (prevSamples_.size() < numOfPrevSamples_)
                {
                    prev_sample_ = prevSamples_.back();
                    return;
                }

                double p = uniRndGnr_.sample();
                int index = headOfPrevSamples_;
                if (p < 0.5)
                {
                    if (p > 0.25)
                        index = index-1;
                    else if (p > 0.125)
                        index = index-2;
                    else if (p > 0.06125)
                        index = index-3;
                    else if (p > 0.0306125)
                        index = index-4;
                    else index = index-5;

                    index = (index + numOfPrevSamples_) % numOfPrevSamples_;
                }
                prev_sample_ = prevSamples_[index];
//                prev_sample_ = samples.row(index).head(getSpaceDimension());
            }

        protected:
            int numOfTries_;
            NormalRealRandomGenerator normRndGnr_;
            double diagonalLength_;
            int numOfPrevSamples_;
            std::vector<Eigen::VectorXd> prevSamples_;
            int headOfPrevSamples_;

        };
    }  // namespace base
}  // namespace ompl

// #endif // OMPL_BASE_SAMPLERS_INFORMED_SAMPLER_
