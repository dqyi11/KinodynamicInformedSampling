#pragma once

// Standard Library Functions
#include <iostream>
#include <chrono>
using namespace std::chrono;
#include <tuple>
#include <math.h>
#include <utility>

// Eigen namespace
#include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Include Sampler
#include <Sampler/Sampler.h>

// Get the Double Integrator Time Model
#include <Dimt/Params.h>
#include <Dimt/DoubleIntegrator.h>

// Example for how to inherit and create your own sampler
class RejectionSampler: public Sampler
{
public:
    ///
    /// Constructor
    ///
    /// @param si Pointer to the space information
    /// @param problem Problem deficition pointer
    /// @param level_set Level set of the problem
    ///
	RejectionSampler(const ompl::base::SpaceInformationPtr& si,
                     const ompl::base::ProblemDefinitionPtr& problem,
                     const double level_set)
		: Sampler(si, problem, level_set)
	{ }

	// Only function that you must implement
	virtual MatrixXd sample(const int& no_samples, high_resolution_clock::duration& duration);

	// Can implement as many private functions as you want to help do the sampling
	virtual VectorXd get_random_sample(const double& max, const double& min, const int& size) const;
};

///
/// Heirarchical Rejection Sampler
///
class HierarchicalRejectionSampler: public RejectionSampler
{
public:
    ///
    /// Constructor
    ///
    /// @param si Pointer to the space information
    /// @param problem Problem deficition pointer
    /// @param level_set Level set of the problem
    ///
	HierarchicalRejectionSampler(const ompl::base::SpaceInformationPtr& si,
                                 const ompl::base::ProblemDefinitionPtr& problem,
                                 const double level_set)
		: RejectionSampler(si, problem, level_set)
	{ }

	///
	/// Get a series of samples for the problem space
	///
	/// @param no_samples Number of samples to get
	/// @param time Boolean that determines if the time to run the proccess is displayed
	/// @return A series of samples of shape (number of samples, sample dimension)
	///
	virtual MatrixXd sample(const int& no_samples, high_resolution_clock::duration& duration);

private:
	///
	/// Get one sample using a recursive algorithm of heirarchical rejection sampling
	///
	/// @param start_index Start index of the hierarchical sample
	/// @param end_index End index of the hierarchical sample
	/// @param sample Reference to a sample that gets changed in place
	/// @return (c_start, c_goal)
	///
	virtual std::tuple<double, double>
		HRS(const int &start_index, const int &end_index, VectorXd &sample);

	///
	/// Calculates the cost of a leaf node
	///
	/// @param x1 First state
	/// @param x2 Second state
	/// @param i Index of the degree of freedom
	/// @return Cost to go from x1 to x2
	///
	virtual double calculate_leaf(const VectorXd &x1, const VectorXd &x2, const int &i) = 0;

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
    virtual double combine_costs(const VectorXd &x1,
                                 const VectorXd &x2,
                                 const int i,
                                 const int m,
                                 const int j,
                                 const double &c1,
                                 const double &c2) const = 0;

	///
	/// How to sample a leaf (ex: geometric is one dimension and kino is 2)
	///
	/// @param sample A vector to the sample
	/// @param dof An index to the degree of freedom to sample
	/// @return A random vector in the space
	///
	virtual void sample_leaf(VectorXd &sample, const int dof) = 0;

	///
	/// Function to get the final cost from all of the dimensions
	///
	/// @param cost Cost to get final form of
	/// @return Final cost of the aggregation
	///
	virtual double Cost(const double &cost) const { return cost; }

protected:

    /// Dimension of the space
    uint dimension_;
};

///
/// Geometric Heirarchical Rejection Sampler
///
class GeometricHierarchicalRejectionSampler: public HierarchicalRejectionSampler
{
public:
    ///
    /// Constructor
    ///
    /// @param si Pointer to the space information
    /// @param problem Problem deficition pointer
    /// @param level_set Level set of the problem
    ///
	GeometricHierarchicalRejectionSampler(const ompl::base::SpaceInformationPtr& si,
                                          const ompl::base::ProblemDefinitionPtr& problem,
                                          const double level_set)
		: HierarchicalRejectionSampler(si, problem, level_set)
	{
        // Get the limits of the state
        std::tie(min_, max_) = state_limits();

        // Set up the random number generator
        std::random_device rd;
        gen_ = std::mt19937(rd());

        // Figure out the dimension. For Geometric it is the size of the space
        dimension_ = start_state().size();
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
	virtual double calculate_leaf(const VectorXd &x1, const VectorXd &x2, const int &i) override;

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
    virtual double combine_costs(const VectorXd &x1,
                                 const VectorXd &x2,
                                 const int i,
                                 const int m,
                                 const int j,
                                 const double &c1,
                                 const double &c2) const override;

	///
	/// How to sample a leaf (ex: geometric is one dimension and kino is 2)
	///
	/// @param sample A vector to the sample
	/// @param dof An index to the degree of freedom to sample
	/// @return A random vector in the space
	///
	virtual void sample_leaf(VectorXd &sample, const int dof) override;

	///
	/// Function to get the final cost from all of the dimensions
	///
	/// @param cost Cost to get final form of
	/// @return Final cost of the aggregation
	///
	virtual double Cost(const double &cost) const override { return std::sqrt(cost); }

    // Random number generator
    std::mt19937 gen_;

    // Limits
    VectorXd max_;
    VectorXd min_;
};

///
/// Dimt Hierarchical Rejection Sampler
///
class DimtHierarchicalRejectionSampler: public HierarchicalRejectionSampler
{
public:
    ///
    /// Constructor
    ///
    /// @param si Pointer to the space information
    /// @param problem Problem deficition pointer
    /// @param level_set Level set of the problem
    /// @param double_integrator_1dof A 1DOF double integrator model
    ///
    DimtHierarchicalRejectionSampler(const ompl::base::SpaceInformationPtr& si,
                                     const ompl::base::ProblemDefinitionPtr& problem,
                                     const double level_set,
                                     const DoubleIntegrator<1> double_integrator_1dof)
        : HierarchicalRejectionSampler(si, problem, level_set),
          double_integrator_1dof_(double_integrator_1dof),
          infeasible_intervals_(param.dof, std::make_pair(0.0, 0.0)),
          costs_(param.dof, 0.0)
    {
        // Get the limits of the state
        std::tie(min_, max_) = state_limits();

        // Set up the random number generator
        std::random_device rd;
        gen_ = std::mt19937(rd());

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
    virtual double calculate_leaf(const VectorXd &x1, const VectorXd &x2, const int &i) override;

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
    virtual double combine_costs(const VectorXd &x1,
                                 const VectorXd &x2,
                                 const int i,
                                 const int m,
                                 const int j,
                                 const double &c1,
                                 const double &c2) const override;

    ///
    /// How to sample a leaf (ex: geometric is one dimension and kino is 2)
    ///
    /// @param sample A vector to the sample
    /// @param dof An index to the degree of freedom to sample
    /// @return A random vector in the space
    ///
    virtual void sample_leaf(VectorXd &sample, const int dof) override;

    ///
    /// Function to get the final cost from all of the dimensions
    ///
    /// @param cost Cost to get final form of
    /// @return Final cost of the aggregation
    ///
    virtual double Cost(const double &cost) const override { return cost; }

    // Random number generator
    std::mt19937 gen_;

    // Limits
    VectorXd max_;
    VectorXd min_;

    // Double Integrator Model
    DoubleIntegrator<1> double_integrator_1dof_;

    // Infeasible time intervals
    std::vector<std::pair<double, double>> infeasible_intervals_;

    // Costs for the dofs
    std::vector<double> costs_;
};
