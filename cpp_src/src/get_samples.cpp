// Standard Librry
#include <iostream>
#include <tuple>
#include <vector>
#include <fstream>

// Eigen
#include <Eigen/Dense>
using Eigen::MatrixXd;

// Internal Libraries
#include "Sampler/RejectionSampler.h"
#include "Dimt/Dimt.h"
#include <Sampler/RejectionSampler.h>
#include <Sampler/MonteCarloSamplers.h>

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
		std::cout << "\t -samples - Number of samples to get" << std::endl;
		std::cout << "\t -time - Boolean (0,1)" << std::endl;
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
    	// Get the boolean to determine if we should time
    	if(cmdOptionExists(argv, argv+argc, "-time")) 
    		args.push_back(atoi(getCmdOption(argv, argv+argc, "-time")));
    	else
    		args.push_back(0); // Default to not print time

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

int main(int argc, char * argv[])
{
	// 
	// Example for how to use the above sampler
	// 	
	bool run; std::vector<int> args;
	std::tie(run, args) = handle_arguments(argc, argv);
	if(!run) return 0;

	int no_samples = args[0];
	bool time = (args[1] == 1) ? true : false;

	std::string filename; bool save;
	std::tie(save, filename) = get_filename(argc, argv);

	// Create a problem definition
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<double> dis(-10, 10);

	int num_dim = 4;
	VectorXd start_state(num_dim);
	// start_state << 0, 0, 0, 0;
	start_state << dis(gen), dis(gen), dis(gen), dis(gen);
	VectorXd goal_state(num_dim);
	// goal_state << 1, 1, 1, 1;
	goal_state << dis(gen), dis(gen), dis(gen), dis(gen);
	VectorXd state_min(num_dim);
	state_min << -10, -10, -10, -10;
	VectorXd state_max(num_dim);
	state_max << 10, 10, 10, 10;
	double a_max = 1;
  	Dimt dimt(a_max);
  	double level_set = 1.2 * dimt.get_min_time(start_state, goal_state);
  	std::cout << "Level set: " << level_set << std::endl;

	ProblemDefinition prob = ProblemDefinition(start_state, goal_state, state_min, state_max, level_set,
		[dimt, start_state, goal_state](const VectorXd& state)
		{
			// return (start_state - state).norm() + (goal_state - state).norm();
			return dimt.get_min_time(start_state, goal_state, state);
		});

	// Initialize the sampler
	// HMC parameters
	double alpha = 0.5; double L = 200; double epsilon = 0.01; double sigma = 0.5;  int max_steps = 20;
	HMCSampler hmc_s = HMCSampler(prob, alpha, L, epsilon, sigma, max_steps);
	RejectionSampler rej_s = RejectionSampler(prob);
	std::cout << "Created the samplers!" << std::endl;

	// Sampler
	std::cout << "Running HMC Sampling..." << std::endl;
	MatrixXd hmc_samples = hmc_s.sample(no_samples, time);
	std::cout << "Running Rejection Sampling..." << std::endl;
	MatrixXd rej_samples = rej_s.sample(no_samples, time);

	if(save)
	{
		std::ofstream hmc_file(filename + "_hmc.log");
		if (hmc_file.is_open())
		{
			for(int i = 0; i < hmc_samples.rows(); i++)
			{
				hmc_file << hmc_samples.row(i) << std::endl;	
			}
		}
		hmc_file.close();
		std::ofstream rej_file(filename + "_rej.log");
		if (rej_file.is_open())
		{
			for(int i = 0; i < rej_samples.rows(); i++)
			{
				rej_file << rej_samples.row(i) << std::endl;	
			}
		}
		rej_file.close();
		std::cout << "Saved samples and costs to " << filename << std::endl;
	}
}