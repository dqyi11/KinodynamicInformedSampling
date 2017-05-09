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
            std::uniform_real_distribution<> dis(min, max);
            // Updates the member variable of the class as well
            prev_sample_(dim) = dis(gen_);
            return prev_sample_;
        }

        Eigen::MatrixXd GibbsSampler::sample(const int no_samples,
                                             std::chrono::high_resolution_clock::duration &duration)
        {
            // Get the limits of the space
            Eigen::VectorXd max_vals, min_vals;
            const int dim = getStartState().size();
            std::tie(max_vals, min_vals) = getStateLimits();

            // Run until you get the correct number of samples
            Eigen::MatrixXd samples(no_samples, dim + 1);

            // If you want to time the sampling
            std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

            Eigen::VectorXd sample;
            unsigned int skip = 0, trys = 0;
            for (int i = 0; i < no_samples; i++)
            {
                trys = 0;
                do
                {
                    if (trys > 10000)
                    {
                        skip++;
                        trys = 0;
                    }
                    sample = getRandomSample(min_vals[(i + skip) % dim], max_vals[(i + skip) % dim], (i + skip) % dim);
                    trys++;
                } while (!isInLevelSet(sample));

                Eigen::VectorXd newsample(getStartState().size() + 1);
                prev_sample_ = sample;
                newsample << sample, getCost(sample);
                samples.row(i) = newsample;
            }

            std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
            duration = t2 - t1;
            return samples;
        }

        void GibbsSampler::updateLevelSet(const double level_set)
        {
            prev_sample_ = getStartState();
            updateLevelSet(level_set);
            // std::cout << "Updated Level Set" << std::endl;
        }

        Eigen::MatrixXd HitAndRun::sample(const int no_samples, std::chrono::high_resolution_clock::duration &duration)
        {
            // Get the limits of the space
            Eigen::VectorXd max_vals, min_vals;
            const int dim = getStartState().size();
            std::tie(max_vals, min_vals) = getStateLimits();
            double diag = 0;
            for (int i = 0; i < dim; i++)
                diag = diag + (max_vals[i] - min_vals[i]) * (max_vals[i] - min_vals[i]);
            diag = std::sqrt(diag);

            // Run until you get the correct number of samples
            Eigen::MatrixXd samples(no_samples, dim + 1);

            // Set up RGN
            std::normal_distribution<> norm_dis(0, 1);
            double lamda_upper_bound, lamda_lower_bound;
            Eigen::VectorXd dir(dim);

            // If you want to time the sampling
            std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

            //cleamn up later
            std::uniform_real_distribution<> uni_dis01(0, 1);

            Eigen::VectorXd sample;
            int skip = 0, trys = 0;
            bool retry;

            for (int i = 0; i < no_samples; i++)
            {
                trys = -1;
                do
                {
                    retry = false;
                    if (trys > numOfTries_ || trys == -1)
                    {
                        // Sample random direction in S^dim
                        double sum = 0;
                        for (int i = 0; i < dim; i++)
                        {
                            dir[i] = norm_dis(gen_);
                            sum = sum + dir[i] * dir[i];
                        }
                        dir = dir / sum;
                        lamda_upper_bound = diag;
                        lamda_lower_bound = -diag;
                        skip++;
                        trys = 0;
                    }
                    // Generate random sample along dir
                    //std::uniform_real_distribution<> uni_dis(lamda_lower_bound, lamda_upper_bound);
                    //double lamda = uni_dis(gen_);
                    double lamda = uni_dis01(gen_);
                    lamda = lamda * (lamda_upper_bound - lamda_lower_bound) + lamda_lower_bound;

                    sample = prev_sample_ + lamda * dir;
                    if (!isInBound(sample))
                    {
                        if (lamda > 0)
                            lamda_upper_bound = lamda;
                        else
                            lamda_lower_bound = lamda;
                        retry = true;
                    }
                    else if (!isInLevelSet(sample))
                        retry = true;
                    trys++;
                    if (VERBOSE)
                        std::cout << "Current_sample: " << i << " Trys: " << trys << " Skip: " << skip
                                  << " UB:" << lamda_upper_bound << " LB:" << lamda_lower_bound << std::endl;
                    // if (!problem().is_in_level_set(prev_sample_))
                    //   std::cout << "\nstart_FALSE!!!!!!!!!!!!!!!!!!"  << std::endl;
                    // else
                    //   std::cout << "\nstart_TRUE!!!!!!!" << std::endl;
                } while (retry);

                Eigen::VectorXd newsample(getStartState().size() + 1);
                prev_sample_ = sample;

                newsample << sample, getCost(sample);
                samples.row(i) = newsample;

                //hack - use previos samples and not only last one

                double p = uni_dis01(gen_);
                int index = i;
                if (i > 5 && p < 0.5)
                {
                    if (p > 0.25)
                        index = i-2;
                    else if (p > 0.125)
                        index = i-3;
                    else if (p > 0.06125)
                        index = i-4;
                    else if (p > 0.0306125)
                        index = i-5;
                    else index = i-6;
                }
                prev_sample_ = samples.row(index).head(dim);

            }

            std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
            duration = t2 - t1;
            return samples;
        }
    }  // namespace base
}  // namespace ompl
