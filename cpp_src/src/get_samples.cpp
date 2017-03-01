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
#include <Sampler/HitAndRun.h>

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

void printTime( high_resolution_clock::duration& duration )
{
	auto duration_s = duration_cast<seconds>( duration ).count();
	auto duration_ms = duration_cast<milliseconds>( duration ).count();
	auto duration_us = duration_cast<microseconds>( duration ).count();
	if (duration_s != 0)
		std::cout << "Total Sampling Time: " << duration_s << "s" << std::endl;
	else if (duration_ms != 0)
		std::cout << "Total Sampling Time: " << duration_ms << "ms" << std::endl;
	else
		std::cout << "Total Sampling Time: " << duration_us << "us" << std::endl;
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
		std::cout << "\t -hmc - Boolean (0,1)" << std::endl;
		std::cout << "\t -mcmc - Boolean (0,1)" << std::endl;
		std::cout << "\t -rej - Boolean (0,1)" << std::endl;
        std::cout << "\t -ghrej - Boolean(0,1)" << std::endl;
        std::cout << "\t -dimthrs - Boolean(0,1)" << std::endl;
        std::cout << "\t -gibbs - Boolean(0,1)" << std::endl;
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
            args.push_back(1); // Default to run ghrej

        // Get the boolean to determine if we run dimt hierarchical rejection sampling
        if(cmdOptionExists(argv, argv+argc, "-dimthrs"))
            args.push_back(atoi(getCmdOption(argv, argv+argc, "-dimthrs")));
        else
            args.push_back(1); // Default to run dimthrs

        // Get the boolean to determine if we run dimt hierarchical rejection sampling
        if(cmdOptionExists(argv, argv+argc, "-gibbs"))
            args.push_back(atoi(getCmdOption(argv, argv+argc, "-gibbs")));
        else
            args.push_back(1); // Default to run dimthrs

        // Get the boolean to determine if we run dimt hierarchical rejection sampling
        if(cmdOptionExists(argv, argv+argc, "-gibbs"))
            args.push_back(atoi(getCmdOption(argv, argv+argc, "-gibbs")));
        else
            args.push_back(1); // Default to run dimthrs

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
	bool time = (args[1] == 1) ? true : false;
	bool run_hmc = (args[2] == 1) ? true : false;
	bool run_mcmc = (args[3] == 1) ? true : false;
	bool run_rej = (args[4] == 1) ? true : false;
  bool run_ghrej = (args[5] == 1) ? true : false;
  bool run_dimthrs = (args[6] == 1) ? true : false;
  bool run_gibbs = (args[7] == 1) ? true : false;

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

	VectorXd state_min(num_dim);
	state_min << VectorXd::Constant(num_dim, minval);

	VectorXd state_max(num_dim);
	state_max << VectorXd::Constant(num_dim, maxval);

	double a_max = 1;
  	Dimt dimt(a_max);
  	// double level_set = 1.4 * dimt.get_min_time(start_state, goal_state);
    	double level_set = 1.4 * (goal_state - start_state).norm();
	high_resolution_clock::duration duration;
  	std::cout << "Level set: " << level_set << std::endl;

	ProblemDefinition prob = ProblemDefinition(start_state, goal_state, state_min, state_max, level_set,
		[dimt, start_state, goal_state](const VectorXd& state)
		{
			return (start_state - state).norm() + (goal_state - state).norm();
			// return dimt.get_min_time(start_state, goal_state, state);
		});

	// Initialize the sampler
	// HMC parameters
	MatrixXd hmc_samples;
	MatrixXd hmc_samples2;
	if (run_hmc)
	{
		double alpha = 0.5; double L = 5; double epsilon = 0.1; double sigma = 1;  int max_steps = 20;
		HMCSampler hmc_s = HMCSampler(prob, alpha, L, epsilon, sigma, max_steps);
		std::cout << "Running HMC Sampling..." << std::endl;
		hmc_samples = hmc_s.sample(no_samples, duration);
                if(time)
                {
                	printTime(duration);
		}
		std::cout << "Running HMC2 Sampling..." << std::endl;
                hmc_samples2 = hmc_s.sample_batch_memorized(no_samples, duration);
                if(time)
                {
                	printTime(duration);
		}
	}

	MatrixXd mcmc_samples;
	if (run_mcmc)
	{
		double sigma = 5; int max_steps = 20; double alpha = 0.5;
		MCMCSampler mcmc_s = MCMCSampler(prob, alpha, sigma, max_steps);
		std::cout << "Running MCMC Sampling..." << std::endl;
		mcmc_samples = mcmc_s.sample(no_samples, duration);
                if(time)
                {
                	printTime(duration);
		}
	}

	MatrixXd rej_samples;
	if(run_rej)
	{
		RejectionSampler rej_s = RejectionSampler(prob);
		std::cout << "Running Rejection Sampling..." << std::endl;
		rej_samples = rej_s.sample(no_samples, duration);
        if(time)
        {
        	printTime(duration);
		}
	}

	MatrixXd ghrej_samples;
	if(run_ghrej)
	{
        ProblemDefinition geo_prob = ProblemDefinition(start_state, goal_state, state_min,
                                                       state_max, level_set,
        [dimt, start_state, goal_state](const VectorXd& state)
        {
            return (start_state - state).norm() + (goal_state - state).norm();
        });

        GeometricHierarchicalRejectionSampler ghrej_s =
            GeometricHierarchicalRejectionSampler(geo_prob);
        std::cout << "Running Geometric Hierarchical Rejection Sampling..." << std::endl;
        ghrej_samples = ghrej_s.sample(no_samples, duration);
        if(time)
        {
        	printTime(duration);
	    }
	}

  MatrixXd dimthrs_samples;
  if(run_dimthrs)
  {
      DoubleIntegrator<1>::Vector maxAccelerations1, maxVelocities1;
      for (unsigned int i = 0; i < 1; ++i)
      {
          maxVelocities1[i] = 10;
          maxAccelerations1[i] = param.a_max;
      }
      DoubleIntegrator<1> double_integrator_1dof(maxAccelerations1, maxVelocities1);

      DimtHierarchicalRejectionSampler dimthrs_s(prob, double_integrator_1dof);
      std::cout << "Running DIMT HRS..." << std::endl;
      dimthrs_samples = dimthrs_s.sample(no_samples, duration);
      if(time)
      {
          printTime(duration);
      }

  }

  MatrixXd gibbs_samples;
  if(run_gibbs)
  {
      DoubleIntegrator<1>::Vector maxAccelerations1, maxVelocities1;
      for (unsigned int i = 0; i < 1; ++i)
      {
          maxVelocities1[i] = 10;
          maxAccelerations1[i] = param.a_max;
      }
      DoubleIntegrator<1> double_integrator_1dof(maxAccelerations1, maxVelocities1);

      GibbsSampler gibbs_s = GibbsSampler(prob);
      std::cout << "Running Gibbs Sampler..." << std::endl;
      gibbs_samples = gibbs_s.sample(no_samples, duration);
      if(time)
      {
        printTime(duration);
      }
  }

	if(save)
	{
        std::cout << "START SAVING" << std::endl;
		if(run_hmc)
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
			std::ofstream hmc2_file(filename + "_hmc2.log");
			if (hmc2_file.is_open())
			{
				for(int i = 0; i < hmc_samples2.rows(); i++)
				{
					hmc2_file << hmc_samples2.row(i) << std::endl;
				}
			}
			hmc2_file.close();
		}
		if(run_mcmc)
		{
			std::ofstream mcmc_file(filename + "_mcmc.log");
			if (mcmc_file.is_open())
			{
				for(int i = 0; i < mcmc_samples.rows(); i++)
				{
					mcmc_file << mcmc_samples.row(i) << std::endl;
				}
			}
			mcmc_file.close();
		}
		if(run_rej)
		{
			std::ofstream rej_file(filename + "_rej.log");
			if (rej_file.is_open())
			{
				for(int i = 0; i < rej_samples.rows(); i++)
				{
					rej_file << rej_samples.row(i) << std::endl;
				}
			}
			rej_file.close();
		}
        if(run_ghrej)
        {
          std::ofstream ghrej_file(filename + "_ghrej.log");
          if (ghrej_file.is_open())
          {
            for(int i = 0; i < ghrej_samples.rows(); i++)
            {
                ghrej_file << ghrej_samples.row(i) << std::endl;
            }
     	    }
          ghrej_file.close();
        }
        if(run_dimthrs)
        {
            std::ofstream dimthrs_file(filename + "_dimthrs.log");
            if(dimthrs_file.is_open())
            {
                for(int i = 0; i < dimthrs_samples.rows(); i++)
                {
                    dimthrs_file << dimthrs_samples.row(i) << std::endl;
                }
            }
            dimthrs_file.close();
        }
    	if(run_gibbs)
        {
          std::ofstream gibbs_file(filename + "_gibbs.log");
          if (gibbs_file.is_open())
          {
              for(int i = 0; i < gibbs_samples.rows(); i++)
              {
                gibbs_file << gibbs_samples.row(i) << std::endl;
              }
          }
          gibbs_file.close();
        }

		std::cout << "Saved samples and costs to " << filename << std::endl;
	}
}
