// Standard Librry
#include <iostream>
#include <tuple>
#include <vector>
#include <fstream>

// Eigen
#include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;

// OMPL
#include <ompl/base/spaces/RealVectorStateSpace.h>

// Internal Libraries
#include "Sampler/RejectionSampler.h"
#include "DimtStateSpace.h"
#include "Dimt/DoubleIntegrator.h"
#include "Dimt/Dimt.h"
#include "Dimt/Params.h"
#include <Sampler/RejectionSampler.h>
#include <Sampler/MonteCarloSamplers.h>
#include <OmplWrappers/OmplHelpers.h>

//
// From stackoverflow:
// http://stackoverflow.com/questions/865668/how-to-parse-command-line-arguments-in-c
//
#include <algorithm>

char* getCmdOption(char ** begin, char ** end, const std::string & option)
{
    char ** itr = std::find(begin, end, option);
    if (itr != end && ++itr != end)
    {
        return *itr;
    }
    return 0;
}

bool cmdOptionExists(char** begin, char** end, const std::string& option)
{
    return std::find(begin, end, option) != end;
}

std::tuple<bool, std::vector<int>> handle_arguments(int argc, char * argv[])
{
	if(cmdOptionExists(argv, argv+argc, "-h"))
    {
		std::cout << "________________________________________________" << std::endl;
		std::cout << "Main function for sampling" << std::endl;
		std::cout << "________________________________________________" << std::endl;
		std::cout << "Arguments:" << std::endl;
                std::cout << "\t -batch - NUmber of batches to run" << std::endl;
		std::cout << "\t -samples - Number of samples to get" << std::endl;
		std::cout << "\t -time - Boolean (0,1)" << std::endl;
		std::cout << "\t -hmc - Boolean (0,1)" << std::endl;
		std::cout << "\t -mcmc - Boolean (0,1)" << std::endl;
		std::cout << "\t -rej - Boolean (0,1)" << std::endl;
        std::cout << "\t -ghrej - Boolean(0,1)" << std::endl;
		std::cout << "\t -filename - Filename to save the samples to" << std::endl;
		std::cout << "________________________________________________" << std::endl;
		return std::make_tuple(false, std::vector<int>{});
    }
    else
    {
    	std::vector<int> args;

    	// Get the number of samples
    	if(cmdOptionExists(argv, argv+argc, "-samples"))
    		args.push_back(atoi(getCmdOption(argv, argv+argc, "-samples")));
    	else
    		args.push_back(100); // Default to 100 samples
        // Get the batch number
    	if(cmdOptionExists(argv, argv+argc, "-batch"))
    		args.push_back(atoi(getCmdOption(argv, argv+argc, "-batch")));
    	else
    		args.push_back(20); // Default to 20 batches

    	// Get the boolean to determine if we should time
    	if(cmdOptionExists(argv, argv+argc, "-time"))
    		args.push_back(atoi(getCmdOption(argv, argv+argc, "-time")));
    	else
    		args.push_back(0); // Default to not print time

    	// Get the boolean to determine if we run hmc
		if(cmdOptionExists(argv, argv+argc, "-hmc"))
    		args.push_back(atoi(getCmdOption(argv, argv+argc, "-hmc")));
    	else
    		args.push_back(1); // Default to run hmc

    	// Get the boolean to determine if we run mcmc
		if(cmdOptionExists(argv, argv+argc, "-mcmc"))
    		args.push_back(atoi(getCmdOption(argv, argv+argc, "-mcmc")));
    	else
    		args.push_back(1); // Default to run mcmc

    	// Get the boolean to determine if we run rej
		if(cmdOptionExists(argv, argv+argc, "-rej"))
    		args.push_back(atoi(getCmdOption(argv, argv+argc, "-rej")));
    	else
    		args.push_back(1); // Default to run rej

        // Get the boolean to determine if we run geometric hierarchical rejection sampling
        if(cmdOptionExists(argv, argv+argc, "-ghrej"))
            args.push_back(atoi(getCmdOption(argv, argv+argc, "-ghrej")));
        else
            args.push_back(1); // Default to run rej

    	return std::make_tuple(true, args);
    }
}

std::tuple<bool, std::string> get_filename(int argc, char * argv[])
{
	if(cmdOptionExists(argv, argv+argc, "-filename"))
		return std::make_tuple(true, std::string(getCmdOption(argv, argv+argc, "-filename")));
	else
		return std::make_tuple(false, "none");
}

std::vector<double> get_random_vector(const double& max, const double& min, const int& num_dim)
{
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<double> dis(min, max);

	std::vector<double> vec;

	for(int i = 0; i < num_dim; i++)
	{
		vec.push_back(dis(gen));
	}
}

int main(int argc, char * argv[])
{
	//
	// Example for how to use the above sampler
	//
	bool run; std::vector<int> args;
	std::tie(run, args) = handle_arguments(argc, argv);
	if(!run) return 0;

	int no_samples = args[0];
        int no_batch = args[1];
	bool time = (args[2] == 1) ? true : false;
	bool run_hmc = (args[3] == 1) ? true : false;
	bool run_mcmc = (args[4] == 1) ? true : false;
	bool run_rej = (args[5] == 1) ? true : false;
        bool run_ghrej = (args[6] == 1) ? true : false;

	std::string filename; bool save;
	std::tie(save, filename) = get_filename(argc, argv);

    // Create a problem definition
    int num_dim = 12;
    double maxval = 25; double minval = -25;
    VectorXd start_state(num_dim);
    VectorXd goal_state(num_dim);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(-25, 25);
    for(int i = 0; i < num_dim; i++)
    {
        start_state(i) = dis(gen);
        goal_state(i) = dis(gen);
    }

    // VectorXd state_min(num_dim);
    // state_min << VectorXd::constant(num_dim, minval);

    // VectorXd state_max(num_dim);
    // state_max << VectorXd::constant(num_dim, maxval);

    // Initializations
    Dimt dimt(param.a_max);
    DoubleIntegrator<param.dof>::Vector maxAccelerations, maxVelocities;
    for (unsigned int i = 0; i < param.dof; ++i)
    {
        maxVelocities[i] = 10;
        maxAccelerations[i] = param.a_max;
    }
    DoubleIntegrator<param.dof> double_integrator(maxAccelerations, maxVelocities);

    const double level_set = 1.4 * dimt.get_min_time(start_state, goal_state);
    // const double level_set = 1.4 * (goal_state - start_state).norm();
    high_resolution_clock::duration duration;
    std::cout << "Level set: " << level_set << std::endl;

    // Construct the state space we are planning in
    ompl::base::StateSpacePtr space(new ompl::base::DimtStateSpace(dimt, double_integrator, param.dimensions));
    ompl::base::RealVectorBounds bounds(param.dimensions);
    bounds.setLow(minval);
    bounds.setHigh(maxval);
    space->as<ompl::base::DimtStateSpace>()->setBounds(bounds);
    ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
    // si->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(new ValidityChecker(si)));
    // si->setStateValidityCheckingResolution(0.01); // 3%
    si->setup();

    // Set custom start and goal
    ompl::base::State *start_s = space->allocState();
    ompl::base::State *goal_s = space->allocState();
    for (int i=0; i<param.dimensions; i++)
    {
        if (i%2==0) // position
        {
            start_s->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = start_state[i];
            goal_s->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = goal_state[i];
        }
        else // velocity
        {
            start_s->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = start_state[i];
            goal_s->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = goal_state[i];
        }
    }

    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(space, start_s);
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal(space, goal_s);

    ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));
    pdef->setStartAndGoalStates(start, goal);

    auto opt = get_dimt_opt_ob(si, start_state, goal_state, no_samples, double_integrator);
    pdef->setOptimizationObjective(opt);

        std::vector<high_resolution_clock::duration> times(5 * no_batch);
        for(unsigned int i=0; i < no_batch; i++) {

                std::cout << "BATCH " << i << std::endl;
		// Initialize the sampler
		// HMC parameters
                {
			MatrixXd hmc_samples;
			double alpha = 0.5; double L = 5; double epsilon = 0.1; double sigma = 1;  int max_steps = 20;
			HMCSampler hmc_s = HMCSampler(si, pdef, level_set, alpha, L, epsilon, sigma, max_steps);
			//std::cout << "Running HMC Sampling..." << std::endl;
			hmc_samples = hmc_s.sample(no_samples, times[i]);
		}

                {
			MatrixXd hmc_samples;
			double alpha = 0.5; double L = 5; double epsilon = 0.1; double sigma = 1;  int max_steps = 20;
			HMCSampler hmc_s = HMCSampler(si, pdef, level_set, alpha, L, epsilon, sigma, max_steps);
			//std::cout << "Running HMC Sampling..." << std::endl;
			//hmc_samples = hmc_s.sample(no_samples, times[i]);
			hmc_samples = hmc_s.sample_batch_memorized(no_samples, times[no_batch+i]);
		}

		{
			MatrixXd mcmc_samples;

			double sigma = 5; int max_steps = 20; double alpha = 0.5;
			MCMCSampler mcmc_s = MCMCSampler(si, pdef, level_set, alpha, sigma, max_steps);
			//std::cout << "Running MCMC Sampling..." << std::endl;
			mcmc_samples = mcmc_s.sample(no_samples, times[2*no_batch+i]);
		}

		{
			MatrixXd rej_samples;
			RejectionSampler rej_s = RejectionSampler(si, pdef, level_set);
			//std::cout << "Running Rejection Sampling..." << std::endl;
			rej_samples = rej_s.sample(no_samples, times[3*no_batch+i]);
		}

		{
   //  			MatrixXd ghrej_samples;
  	//         	ProblemDefinition geo_prob = ProblemDefinition(start_state, goal_state, state_min,
   //      	                                               state_max, level_set,
   //     				 [dimt, start_state, goal_state](const VectorXd& state)
   //      			{
   //    		      			return (start_state - state).norm() + (goal_state - state).norm();
   //     			 	});

	  //   		GeometricHierarchicalRejectionSampler ghrej_s = GeometricHierarchicalRejectionSampler(geo_prob);
	  //      		//std::cout << "Running Geometric Hierarchical Rejection Sampling..." << std::endl;
			// ghrej_samples = ghrej_s.sample(no_samples, times[4*no_batch+i]);
		}
	}

        if(save)
        {
        	std::cout << "START SAVING" << std::endl;
		std::ofstream time_file(filename + "_time.log");
		if (time_file.is_open())
		{
			for(int i = 0; i < 5; i++)
			{
				for(int j=0;j<no_batch;j++)
				{
					time_file << duration_cast<milliseconds>( times[i*no_batch+j] ).count() << " ";
				}
                	        time_file << std::endl;
			}
		}
		time_file.close();
	}

}
