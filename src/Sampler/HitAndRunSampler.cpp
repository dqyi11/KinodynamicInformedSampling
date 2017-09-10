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

#include "Sampler/HitAndRunSampler.h"

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

        bool GibbsSampler::sampleInLevelSet(Eigen::VectorXd &sample)
        {
            double sampleCost = std::numeric_limits<double>::infinity();
            int trys = 0;
            bool inLevelset = true;

            std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
            std::chrono::high_resolution_clock::time_point t2 = t1;
            std::chrono::high_resolution_clock::duration timeElapsed;
            double timeElapsedDouble = 0.0;
            int skip = 0;
            Eigen::VectorXd q;
            do
            {
                if (timeElapsedDouble >= timelimit_)
                {
                    inLevelset = false;
                    break;
                }
                if (trys > 10000)
                {
                    skip++;
                    trys = 0;
                }
                q = getRandomSample(stateMin_[(numAcceptedSamples_ + skip) % getSpaceDimension()],
                                    stateMax_[(numAcceptedSamples_ + skip) % getSpaceDimension()],
                                    (numAcceptedSamples_ + skip) % getSpaceDimension());
                trys++;

                t2 = std::chrono::high_resolution_clock::now();
                timeElapsed = t2 - t1;
                timeElapsedDouble = std::chrono::duration_cast<std::chrono::seconds>(timeElapsed).count();

            } while (!isInLevelSet(q, sampleCost));
            prev_sample_ = q;

            sample << q, sampleCost;
            if (inLevelset)
            {
                numAcceptedSamples_++;
            }
            else
            {
                numRejectedSamples_++;
            }

            return inLevelset;
        }

        void GibbsSampler::updateLevelSet(const double level_set)
        {
            prev_sample_ = getStartState();
            MyInformedSampler::updateLevelSet(level_set);
            // std::cout << "Updated Level Set" << std::endl;
        }

        bool HitAndRunSampler::sampleInLevelSet(Eigen::VectorXd &sample)
        {
            double lamda_lower_bound = 0.0, lamda_upper_bound = 1.0;
            Eigen::VectorXd dir(getSpaceDimension());
            double newSampleCost = std::numeric_limits<double>::infinity();
            Eigen::VectorXd newSample;

            int trys = -1;
            bool retry = false;

            std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
            std::chrono::high_resolution_clock::time_point t2 = t1;
            std::chrono::high_resolution_clock::duration timeElapsed;
            double timeElapsedDouble = 0.0;

            bool inLevelset = true;
            do
            {
                if (timeElapsedDouble > timelimit_)
                {
                    inLevelset = false;
                    break;
                }
                retry = false;
                if (trys > numOfTries_ || trys == -1)
                {
                    // Sample random direction in S^dim
                    double sum = 0;
                    for (uint i = 0; i < getSpaceDimension(); i++)
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
                double lamda;
                double power = 4;
                if (lamda_lower_bound < 0 && lamda_upper_bound > 0)
                {
                    double rand_no1 = uniRndGnr_.sample(0, 1);
                    double rand_no2 = uniRndGnr_.sample(0, 1);
                    lamda = rand_no1 < 0.5 ? lamda_lower_bound * pow(rand_no2,1/power) : lamda_upper_bound * pow(rand_no2,1/power);
                }
                else
                {
                    double rand_no = uniRndGnr_.sample(0, 1);
                    if (lamda_lower_bound > 0)
                        lamda = pow(pow(lamda_lower_bound, power) +
                                     rand_no * (pow(lamda_upper_bound, power) - pow(lamda_lower_bound, power)),1/power);
                    else if (lamda_upper_bound < 0)
                      lamda = -pow(pow(lamda_upper_bound,power) +
                                        rand_no * (pow(lamda_lower_bound,power) -
                                                   pow(lamda_upper_bound,power)),1/power);
                }

                // lamda = uniRndGnr_.sample(lamda_lower_bound, lamda_upper_bound);
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

                t2 = std::chrono::high_resolution_clock::now();
                timeElapsed = t2 - t1;
                timeElapsedDouble = std::chrono::duration_cast<std::chrono::seconds>(timeElapsed).count();

            } while (retry);

            sample << newSample, newSampleCost;
            pushPrevSamples(newSample);

            if (inLevelset)
            {
                numAcceptedSamples_++;
            }
            else
            {
                numRejectedSamples_++;
            }
            return inLevelset;
        }
    }  // namespace base
}  // namespace ompl
