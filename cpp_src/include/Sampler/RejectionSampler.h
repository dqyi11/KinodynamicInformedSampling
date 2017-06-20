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

// #ifndef OMPL_BASE_SAMPLERS_INFORMED_REJECTION_SAMPLER_
// #define OMPL_BASE_SAMPLERS_INFORMED_REJECTION_SAMPLER_

#pragma once

// Standard Library Functions
#include <iostream>
#include <chrono>
#include <tuple>
#include <math.h>
#include <utility>

// Eigen namespace
#include <Eigen/Dense>

// Include Sampler
#include <Sampler/Sampler.h>

// Get the Double Integrator Time Model
#include <Dimt/Params.h>
#include <Dimt/DoubleIntegratorMinimumTime.h>

// Example for how to inherit and create your own sampler
namespace ompl
{
    namespace base
    {
        class RejectionSampler : public MyInformedSampler
        {
        public:
            ///
            /// Constructor
            ///
            /// @param si Space information pointer
            /// @param problem OMPL's problem definition
            /// @param levelSet Initial level set of the problem
            /// @param maxNumberCalls Max number of calls to the sampler
            /// @param sampler Sampler that inherits from Sampler.h
            /// @param sample_batch_size How many samples to get each time a new
            /// batch of samples is gotten
            ///
            RejectionSampler(const SpaceInformationPtr &si, const ProblemDefinitionPtr &problem, const double levelSet,
                             const unsigned int maxNumberCalls, const int sampleBatchSize)
              : MyInformedSampler(si, problem, levelSet, maxNumberCalls, sampleBatchSize),
                rejectionRatio_(1.0)
            {
            }

            /// Only function that you must implement
            virtual Eigen::MatrixXd sample(const uint numSamples,
                                           std::chrono::high_resolution_clock::duration &duration) override;

            virtual bool sampleInLevelSet(Eigen::VectorXd& sample) override;

            /// Can implement as many private functions as you want to help do the sampling
            virtual Eigen::VectorXd getRandomSample(const Eigen::VectorXd min, const Eigen::VectorXd max, const int size);

            /// Get rejection ratio
            double getRejectionRatio() { return rejectionRatio_; }

        protected:
            double rejectionRatio_;

        };

        ///
        /// Heirarchical Rejection Sampler
        ///
        class HierarchicalRejectionSampler : public RejectionSampler
        {
        public:
            ///
            /// Constructor
            ///
            /// @param si Space information pointer
            /// @param problem OMPL's problem definition
            /// @param levelSet Initial level set of the problem
            /// @param maxNumberCalls Max number of calls to the sampler
            /// @param sampler Sampler that inherits from Sampler.h
            /// @param sample_batch_size How many samples to get each time a new
            /// batch of samples is gotten
            ///
            HierarchicalRejectionSampler(const SpaceInformationPtr &si, const ProblemDefinitionPtr &problem,
                                         const double levelSet, const unsigned int maxNumberCalls,
                                         const int sampleBatchSize)
              : RejectionSampler(si, problem, levelSet, maxNumberCalls, sampleBatchSize)
            {
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
        private:
            ///
            /// Get one sample using a recursive algorithm of heirarchical rejection
            /// sampling
            ///
            /// @param start_index Start index of the hierarchical sample
            /// @param end_index End index of the hierarchical sample
            /// @param sample Reference to a sample that gets changed in place
            /// @return (c_start, c_goal)
            ///
            virtual std::tuple<double, double> HRS(const int start_index, const int end_index, Eigen::VectorXd &sample,
                                                   double timeLeft);

            ///
            /// Calculates the cost of a leaf node
            ///
            /// @param x1 First state
            /// @param x2 Second state
            /// @param i Index of the degree of freedom
            /// @return Cost to go from x1 to x2
            ///
            virtual double calculateLeaf(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2, const int i) = 0;

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
            virtual double combineCosts(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2, const int i, const int m,
                                        const int j, const double c1, const double c2) const = 0;

            ///
            /// How to sample a leaf (ex: geometric is one dimension and kino is 2)
            ///
            /// @param sample A vector to the sample
            /// @param dof An index to the degree of freedom to sample
            /// @return A random vector in the space
            ///
            virtual void sampleLeaf(Eigen::VectorXd &sample, const int dof) = 0;

            ///
            /// Function to get the final cost from all of the dimensions
            ///
            /// @param cost Cost to get final form of
            /// @return Final cost of the aggregation
            ///
            virtual double Cost(const double cost) const
            {
                return cost;
            }

        protected:
            /// Dimension of the space
            uint dimension_;
        };

        ///
        /// Geometric Heirarchical Rejection Sampler
        ///
        class GeometricHierarchicalRejectionSampler : public HierarchicalRejectionSampler
        {
        public:
            ///
            /// Constructor
            ///
            /// @param si Space information pointer
            /// @param problem OMPL's problem definition
            /// @param levelSet Initial level set of the problem
            /// @param maxNumberCalls Max number of calls to the sampler
            /// @param sampler Sampler that inherits from Sampler.h
            /// @param sample_batch_size How many samples to get each time a new
            /// batch of samples is gotten
            ///
            GeometricHierarchicalRejectionSampler(const SpaceInformationPtr &si, const ProblemDefinitionPtr &problem,
                                                  const double levelSet, const unsigned int maxNumberCalls,
                                                  const int sampleBatchSize)
              : HierarchicalRejectionSampler(si, problem, levelSet, maxNumberCalls, sampleBatchSize)
            {
                // Get the limits of the state
                std::tie(min_, max_) = getStateLimits();
            }

        private:
            ///
            /// Calculates the cost of a leaf node
            ///
            /// @param x1 First state
            /// @param x2 Second state
            /// @param i Index of the degree of freedom
            /// @return Cost to go from x1 to x2
            ///
            virtual double calculateLeaf(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2, const int i) override;

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
            virtual double combineCosts(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2, const int i, const int m,
                                        const int j, const double c1, const double c2) const override;

            ///
            /// How to sample a leaf (ex: geometric is one dimension and kino is 2)
            ///
            /// @param sample A vector to the sample
            /// @param dof An index to the degree of freedom to sample
            /// @return A random vector in the space
            ///
            virtual void sampleLeaf(Eigen::VectorXd &sample, const int dof) override;

            ///
            /// Function to get the final cost from all of the dimensions
            ///
            /// @param cost Cost to get final form of
            /// @return Final cost of the aggregation
            ///
            virtual double Cost(const double cost) const override
            {
                return std::sqrt(cost);
            }

            // Limits
            Eigen::VectorXd max_;
            Eigen::VectorXd min_;
        };

        ///
        /// Dimt Hierarchical Rejection Sampler
        ///
        class DimtHierarchicalRejectionSampler : public HierarchicalRejectionSampler
        {
        public:
            ///
            /// Constructor
            ///
            /// @param si Space information pointer
            /// @param problem OMPL's problem definition
            /// @param levelSet Initial level set of the problem
            /// @param maxNumberCalls Max number of calls to the sampler
            /// @param sampler Sampler that inherits from Sampler.h
            /// @param sample_batch_size How many samples to get each time a new
            /// batch of samples is gotten
            /// @param double_integrator_1dof A 1DOF double integrator model
            ///
            DimtHierarchicalRejectionSampler(const SpaceInformationPtr &si, const ProblemDefinitionPtr &problem,
                                             const DIMTPtr dimt, const double levelSet,
                                             const unsigned int maxNumberCalls, const int sampleBatchSize)
              : HierarchicalRejectionSampler(si, problem, levelSet, maxNumberCalls, sampleBatchSize)
              , dimt_(dimt)
              , infeasibleIntervals_(param.dof, std::make_pair(0.0, 0.0))
              , costs_(param.dof, 0.0)
            {
                // Get the limits of the state
                std::tie(min_, max_) = getStateLimits();

                // Figure out the dimension. For DIMT it is the number of joints.
                dimension_ = param.dof;
            }

        private:
            ///
            /// Calculates the cost of a leaf node
            ///
            /// @param x1 First state
            /// @param x2 Second state
            /// @param i Index of the degree of freedom
            /// @return Cost to go from x1 to x2
            ///
            virtual double calculateLeaf(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2, const int i) override;

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
            virtual double combineCosts(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2, const int i, const int m,
                                        const int j, const double c1, const double c2) const override;

            ///
            /// How to sample a leaf (ex: geometric is one dimension and kino is 2)
            ///
            /// @param sample A vector to the sample
            /// @param dof An index to the degree of freedom to sample
            /// @return A random vector in the space
            ///
            virtual void sampleLeaf(Eigen::VectorXd &sample, const int dof) override;

            ///
            /// Function to get the final cost from all of the dimensions
            ///
            /// @param cost Cost to get final form of
            /// @return Final cost of the aggregation
            ///
            virtual double Cost(const double cost) const override
            {
                return cost;
            }

            DIMTPtr dimt_;

            /// Limits
            Eigen::VectorXd max_;
            Eigen::VectorXd min_;

            /// Infeasible time intervals
            std::vector<std::pair<double, double>> infeasibleIntervals_;

            /// Costs for the dofs
            std::vector<double> costs_;

        };
    }  // namespace base
}  // namespace ompl

// #endif // OMPL_BASE_SAMPLERS_INFORMED_REJECTION_SAMPLER_
