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

#include <Sampler/Sampler.h>

// Our libraries
#include <Dimt/Params.h>
#include <OmplWrappers/MyOptimizationObjective.h>

// OMPL
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/goals/GoalState.h>

// stdlib
#include <vector>
#include <memory>
#include <stdexcept>

namespace
{
    bool same_cost(double a, double b)
    {
        return std::fabs(a - b) < std::numeric_limits<double>::epsilon();
    }

    void print_out_states2(ompl::base::State *statePtr)
    {
        double *val = static_cast<ompl::base::RealVectorStateSpace::StateType *>(statePtr)->values;

        std::cout << "Printing sample of size: " << param.dimensions << " | Vec: [ ";
        for (uint i = 0; i < param.dimensions; i++)
        {
            std::cout << val[i] << " ";
        }
        std::cout << "]" << std::endl;
    }

    void print_out_states2(const Eigen::VectorXd &state)
    {
        std::cout << "[ ";
        for (uint i = 0; i < state.size(); i++)
        {
            std::cout << state[i] << " ";
        }
        std::cout << " ]" << std::endl;
    }

}  // namespace

namespace ompl
{
    namespace base
    {
        ///
        /// Get the state limits of the space
        ///
        /// @return Tuple(state_max, state_min)
        ///
        std::tuple<Eigen::VectorXd, Eigen::VectorXd> MyInformedSampler::getStateLimits() const
        {
            // Get the bounds from the vector
            const auto space = si_->getStateSpace()->as<ompl::base::RealVectorStateSpace>();
            Eigen::VectorXd high = get_eigen_vector(space->getBounds().high);
            Eigen::VectorXd low = get_eigen_vector(space->getBounds().low);

            return std::make_tuple(low, high);
        }

        ///
        /// Get the start state
        ///
        /// @return Start state defined in the constructor
        ///
        Eigen::VectorXd MyInformedSampler::getStartState() const
        {
            return get_eigen_vector(problem_->getStartState(0));
        }

        ///
        /// Get the goal state
        ///
        /// @return Start state defined in the constructor
        ///
        Eigen::VectorXd MyInformedSampler::getGoalState() const
        {
            const auto goal_state = problem_->getGoal()->as<ompl::base::GoalState>();
            return get_eigen_vector(goal_state->getState());
        }

        Eigen::VectorXd MyInformedSampler::getGradient(const Eigen::VectorXd &curr_state)
        {
            // Assert that the matrix is not empty
            assert(curr_state.size() != 0 or curr_state.size() != 0);

            // Derivative constant
            double h = 0.001;

            // Loop through and calculate the gradients
            VectorXd grad(curr_state.size());
            for (int dim = 0; dim < curr_state.size(); dim++)
            {
                VectorXd state_plus = curr_state;
                VectorXd state_min = curr_state;
                state_plus(dim) = curr_state(dim) + h;
                state_min(dim) = curr_state(dim) - h;
                grad(dim) = (getCost(state_plus) - getCost(state_min)) / (2 * h);
            }

            return grad;
        }

        Eigen::VectorXd MyInformedSampler::getInvJacobian(const Eigen::VectorXd &curr_state) const
        {
            // Assert that the matrix is not empty
            assert(curr_state.size() != 0 or curr_state.size() != 0);

            // Derivative constant
            double h = 0.001;

            // Loop through and calculate the gradients
            Eigen::VectorXd inv_jacobian(curr_state.size());
            for (int dim = 0; dim < curr_state.size(); dim++)
            {
                VectorXd state_plus = curr_state;
                VectorXd state_min = curr_state;
                state_plus(dim) = curr_state(dim) + h;
                state_min(dim) = curr_state(dim) - h;
                double delta = getCost(state_plus) - getCost(state_min);
                // std::cout << "DELTA " << delta << std::endl;
                if (delta == 0.0)
                {
                    inv_jacobian(dim) = 0.0;
                }
                else
                {
                    inv_jacobian(dim) = (2 * h) / (delta);
                }
            }

            return inv_jacobian;
        }

        ///
        /// Get the cost for a specific state
        ///
        /// @param curr_state Current state to get the cost for
        /// @return Cost at that state
        ///
        double MyInformedSampler::getCost(const Eigen::VectorXd &curr_state) const
        {

            // Assert that the problem definition has an optimization objective defined
            assert(problem_->hasOptimizationObjective());

            const ompl::base::State *start_state = problem_->getStartState(0);
            const ompl::base::State *goal_state = problem_->getGoal()->as<ompl::base::GoalState>()->getState();

            ompl::base::State* state = si_->allocState();
            get_ompl_state(curr_state, state);

            const ompl::base::OptimizationObjectivePtr optim_obj = problem_->getOptimizationObjective();

            double cost =  optim_obj->motionCost(start_state, state).value() +
                    optim_obj->motionCost(state, goal_state).value();

            si_->freeState(state);
            return cost;
        }

        bool MyInformedSampler::isInLevelSet(const Eigen::VectorXd &curr_state, double thresholdCost, double& stateCost) const
        {
            // Assert that the problem definition has an optimization objective defined
            assert(problem_->hasOptimizationObjective());
            const ompl::base::OptimizationObjectivePtr optim_obj = problem_->getOptimizationObjective();
            const ompl::base::DimtObjective* dimt_obj = dynamic_cast<ompl::base::DimtObjective*>(optim_obj.get());
            assert(dimt_obj!=nullptr);

            const ompl::base::State *start_state = problem_->getStartState(0);
            const ompl::base::State *goal_state = problem_->getGoal()->as<ompl::base::GoalState>()->getState();

            ompl::base::State* state = si_->allocState();
            get_ompl_state(curr_state, state);

            double costToCome = dimt_obj->getCostIfSmallerThan(start_state, state, Cost(thresholdCost)).value();
            if (costToCome == std::numeric_limits<double>::infinity() )
            {
                si_->freeState(state);
                return false;
            }
            double costToGo = dimt_obj->getCostIfSmallerThan(state, goal_state, Cost(thresholdCost-costToCome)).value();
            if (costToGo == std::numeric_limits<double>::infinity() )
            {
                si_->freeState(state);
                return false;
            }
            stateCost = costToCome + costToGo;
            si_->freeState(state);
            return true;
        }

        /// Can implement as many private functions as you want to help do the sampling
        double MyInformedSampler::getRandomDimension(const double max, const double min)
        {
            return uniRndGnr_.sample(min, max);
        }

        ///
        /// Function to sample a state uniformly from the entire space before you have
        /// a solution
        ///
        /// @param statePtr Pointer to the state to sample
        ///
        bool MyInformedSampler::sampleFullSpace(State *statePtr)
        {
            // Get the limits of the space
            Eigen::VectorXd max_vals, min_vals;
            std::tie(max_vals, min_vals) = getStateLimits();

            double *val = static_cast<ompl::base::RealVectorStateSpace::StateType *>(statePtr)->values;
            for (int i = 0; i < param.dimensions; i++)
            {
                val[i] = getRandomDimension(max_vals[i], min_vals[i]);
            }

            return true;
        }

        ///
        /// Function to sample uniformly from the informed subset
        ///
        /// @param statePtr Pointer to the state to sample
        /// @param maxCost Best cost found so far
        ///
        bool MyInformedSampler::sampleInformedSpace(State *statePtr, const Cost maxCost)
        {
            if (started_ == false)  // Set the timer when the function is called for the first time
            {
                started_ = true;
                // set time
                tStart_ = std::chrono::high_resolution_clock::now();
                // push back zero time and initial cost
                durationVec_.push_back(tStart_ - tStart_);
                costVec_.push_back(maxCost.value());
            }
            else
            {
                // Measure time and save it with cost
                tNow_ = std::chrono::high_resolution_clock::now();
                durationVec_.push_back(tNow_ - tStart_);
                costVec_.push_back(maxCost.value());
            }

            if (!same_cost(maxCost.value(), prevCost_) or sampleIndex_ >= sampleBatchSize_)
            {
                if (maxCost.value() != prevCost_)
                    updateLevelSet(maxCost.value());

                std::chrono::high_resolution_clock::duration duration;
                batchSamples_ = sample(sampleBatchSize_, duration);

                prevCost_ = maxCost.value();
                sampleIndex_ = 0;
            }

            auto sample = batchSamples_.row(sampleIndex_);

            double *val = static_cast<ompl::base::RealVectorStateSpace::StateType *>(statePtr)->values;
            for (int i = 0; i < sample.size() - 1; i++)
            {
                val[i] = sample(i);
            }

            sampleIndex_++;

            return true;
        }

        ///
        /// Sample uniformly from the informed space
        ///
        /// @param statePtr Pointer of the state you're sampling
        /// @param maxCost Max cost of the informed subspace
        /// @return true if a sample is gotten false, if not
        ///
        bool MyInformedSampler::sampleUniform(State *statePtr, const Cost &maxCost)
        {
            if (maxCost.value() == std::numeric_limits<double>::infinity())
            {
                return sampleFullSpace(statePtr);
            }
            else
            {
                return sampleInformedSpace(statePtr, maxCost);
            }
        }

        ///
        /// Just call sampleUniform(statePtr, maxCost) - there is no mincost
        ///
        /// @param statePtr Pointer of the state you're sampling
        /// @param maxCost Max cost of the informed subspace
        /// @param minCost Minimum cost of the informed subspace
        /// @return true if a sample is gotten false, if not
        ///
        bool MyInformedSampler::sampleUniform(State *statePtr, const Cost &minCost, const Cost &maxCost)
        {
            return sampleUniform(statePtr, maxCost);
        }

        ///
        /// Function that lets the planner know if we have an informed measure
        ///
        /// @return True if we have implemented an informed measure, false if not
        ///
        bool MyInformedSampler::hasInformedMeasure() const
        {
            return false;
        }

        ///
        /// Function to return the measure of the informed space
        ///
        /// @param currentCost - Current cost of the best path
        /// @return Measure of the informed space
        ///
        double MyInformedSampler::getInformedMeasure(const Cost &currentCost) const
        {
            return InformedSampler::space_->getMeasure();
        }

        ///
        /// Determines if a sample is within the boundaries of the space
        ///
        /// @param state State to test
        /// @return Boolean that is true if it is in the boundaries of the space
        ///
        bool MyInformedSampler::isInBound(const Eigen::VectorXd &state) const
        {

            for (unsigned int i = 0; i < state.size(); i++)
                if (state(i) > stateMax_(i) || state(i) < stateMin_(i))
                    return false;
            return true;
        }

    }  // base
}  // oml
