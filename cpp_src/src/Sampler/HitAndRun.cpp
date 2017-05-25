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

#include "Sampler/HitAndRun.h"

const bool VERBOSE = false;

namespace ompl
{
    namespace base
    {
        Eigen::VectorXd GibbsSampler::getRandomSample(double min, double max, const int dim)
        {
            // Updates the member variable of the class as well
            prev_sample_(dim) = uniRndGnr_.sample(min, max);
            return prev_sample_;
        }

        Eigen::MatrixXd GibbsSampler::sample(const uint numSamples,
                                             std::chrono::high_resolution_clock::duration &duration)
        {
            // Get the limits of the space
            Eigen::VectorXd max_vals, min_vals;
            std::tie(max_vals, min_vals) = getStateLimits();

            // Run until you get the correct number of samples
            Eigen::MatrixXd samples(numSamples, getSpaceDimension() + 1);

            // If you want to time the sampling
            std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

            Eigen::VectorXd sample;
            double sampleCost = std::numeric_limits<double>::infinity();
            unsigned int skip = 0, trys = 0;
            for (uint i = 0; i < numSamples; i++)
            {
                trys = 0;
                do
                {
                    if (trys > 10000)
                    {
                        skip++;
                        trys = 0;
                    }
                    sample = getRandomSample(min_vals[(i + skip) % getSpaceDimension()], max_vals[(i + skip) % getSpaceDimension()], (i + skip) % getSpaceDimension());
                    trys++;
                } while (!isInLevelSet(sample, sampleCost));

                Eigen::VectorXd newsample(getSpaceDimension() + 1);
                prev_sample_ = sample;
                newsample << sample, sampleCost;
                samples.row(i) = newsample;
            }

            std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
            duration = t2 - t1;
            return samples;
        }

        void GibbsSampler::updateLevelSet(const double level_set)
        {
            prev_sample_ = getStartState();
            MyInformedSampler::updateLevelSet(level_set);
            // std::cout << "Updated Level Set" << std::endl;
        }

        Eigen::MatrixXd HitAndRun::sample(const uint numSamples, std::chrono::high_resolution_clock::duration &duration)
        {
            // Run until you get the correct number of samples
            Eigen::MatrixXd samples(numSamples, getSpaceDimension() + 1);

            // If you want to time the sampling
            std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

            for (uint i = 0; i < numSamples; i++)
            {
                Eigen::VectorXd newSample(getSpaceDimension() + 1);

                sampleInLevelSet(newSample);
                samples.row(i) = newSample;
            }

            std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
            duration = t2 - t1;
            return samples;
        }


        bool HitAndRun::sampleInLevelSet(Eigen::VectorXd& sample)
        {
            double lamda_lower_bound = 0.0, lamda_upper_bound = 1.0;
            Eigen::VectorXd dir(getSpaceDimension());
            double newSampleCost = std::numeric_limits<double>::infinity();
            Eigen::VectorXd newSample;

            int trys = -1;
            bool retry = false;

            do
            {
                retry = false;
                if (trys > numOfTries_ || trys == -1)
                {
                    // Sample random direction in S^dim
                    double sum = 0;
                    for (int i = 0; i < getSpaceDimension(); i++)
                    {
                        dir[i] = normRndGnr_.sample();
                        sum = sum + dir[i] * dir[i];
                    }
                    dir = dir / sum;
                    lamda_upper_bound = diagonalLength_;
                    lamda_lower_bound = -diagonalLength_;
                   // skip++;
                    trys = 0;
                }
                // Generate random sample along dir
                double lamda = uniRndGnr_.sample(lamda_lower_bound, lamda_upper_bound);

                newSample = prev_sample_ + lamda * dir;
                if (!isInBound(newSample))
                {
                    if (lamda > 0)
                        lamda_upper_bound = lamda;
                    else
                        lamda_lower_bound = lamda;
                    retry = true;
                }
                else if (!isInLevelSet(newSample, newSampleCost))
                    retry = true;
                trys++;

            } while (retry);

            sample << newSample, newSampleCost;
            pushPrevSamples(newSample);
            return true;
        }
    }  // namespace base
}  // namespace ompl
