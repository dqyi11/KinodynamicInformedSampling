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
                std::cout << "\t -batch - NUmber of batches to run" << std::endl;
		std::cout << "\t -filename - Filename to save the samples to" << std::endl;
		std::cout << "________________________________________________" << std::endl;
		return std::make_tuple(false, std::vector<int>{});
    }
    else
    {
    	std::vector<int> args;

        // Get the batch number
    	if(cmdOptionExists(argv, argv+argc, "-batch"))
    		args.push_back(atoi(getCmdOption(argv, argv+argc, "-batch")));
    	else
    		args.push_back(20); // Default to 20 batches
    
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

        int no_batch = args[0];
        int no_samples = 100;

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

        int sample_num_sets[] = {100, 200, 500, 1000, 3000, 5000};
        int sample_num_sets_num = sizeof(sample_num_sets)/sizeof(int);
        std::cout << "sample set num " << sample_num_sets_num << std::endl;
	double a_max = 1;
  	Dimt dimt(a_max);
        double level_set = 1.4 * dimt.get_min_time(start_state, goal_state);
        std::cout << "Level set: " << level_set << std::endl;
        //double level_set = level_set_ratios[j] * (goal_state - start_state).norm();

      	std::vector<high_resolution_clock::duration> times_hmc1(sample_num_sets_num * no_batch);
      	std::vector<high_resolution_clock::duration> times_hmc2(sample_num_sets_num * no_batch);
	std::vector<high_resolution_clock::duration> times_mcmc(sample_num_sets_num * no_batch);
	std::vector<high_resolution_clock::duration> times_rs(sample_num_sets_num * no_batch);
	std::vector<high_resolution_clock::duration> times_hrs(sample_num_sets_num * no_batch);
        for(unsigned int j=0; j<sample_num_sets_num;j++) {
	  	
	        no_samples = sample_num_sets[j];
                std::cout << "NO SAMPLE " << no_samples << std::endl;
	
		ProblemDefinition prob = ProblemDefinition(start_state, goal_state, state_min, state_max, level_set,
			[dimt, start_state, goal_state](const VectorXd& state)
			{
				return (start_state - state).norm() + (goal_state - state).norm();
				// return dimt.get_min_time(start_state, goal_state, state);
			});
        
    		for(unsigned int i=0; i < no_batch; i++) {

                	std::cout << "BATCH " << i << std::endl;
			// Initialize the sampler
			// HMC parameters
                	{
				MatrixXd hmc_samples;
				double alpha = 0.5; double L = 5; double epsilon = 0.1; double sigma = 1;  int max_steps = 20;
				HMCSampler hmc_s = HMCSampler(prob, alpha, L, epsilon, sigma, max_steps);
				//std::cout << "Running HMC Sampling..." << std::endl;
				hmc_samples = hmc_s.sample(no_samples, times_hmc1[j*no_batch+i]);
			}
	
        	        {
				MatrixXd hmc_samples;
				double alpha = 0.5; double L = 5; double epsilon = 0.1; double sigma = 1;  int max_steps = 20;
				HMCSampler hmc_s = HMCSampler(prob, alpha, L, epsilon, sigma, max_steps);
				//std::cout << "Running HMC Sampling..." << std::endl;
				//hmc_samples = hmc_s.sample(no_samples, times[i]);
				hmc_samples = hmc_s.sample_batch_memorized(no_samples, times_hmc2[j*no_batch+i]);
			}

			{
				MatrixXd mcmc_samples;
		
				double sigma = 5; int max_steps = 20; double alpha = 0.5;
				MCMCSampler mcmc_s = MCMCSampler(prob, alpha, sigma, max_steps);
				//std::cout << "Running MCMC Sampling..." << std::endl;
				mcmc_samples = mcmc_s.sample(no_samples, times_mcmc[j*no_batch+i]);
			}

			{
				MatrixXd rej_samples;
				RejectionSampler rej_s = RejectionSampler(prob);
				//std::cout << "Running Rejection Sampling..." << std::endl;
				rej_samples = rej_s.sample(no_samples, times_rs[j*no_batch+i]);
			}

			{
	    			MatrixXd ghrej_samples;
	  	        	ProblemDefinition geo_prob = ProblemDefinition(start_state, goal_state, state_min,
			                                               state_max, level_set,
	       				 [dimt, start_state, goal_state](const VectorXd& state)
					{
	      		      			return (start_state - state).norm() + (goal_state - state).norm();
	       			 	});
	
		    		GeometricHierarchicalRejectionSampler ghrej_s = GeometricHierarchicalRejectionSampler(geo_prob);
		       		//std::cout << "Running Geometric Hierarchical Rejection Sampling..." << std::endl;
				ghrej_samples = ghrej_s.sample(no_samples, times_hrs[j*no_batch+i]);
			}


		}
	}

        if(save)
        {
        	std::cout << "START SAVING" << std::endl;
		std::ofstream time1_file(filename + "_time_sn_hmc1.log");	
		if (time1_file.is_open())
		{
			for(int i = 0; i < sample_num_sets_num; i++)
			{
				for(int j=0;j<no_batch;j++)
				{
					time1_file << duration_cast<milliseconds>( times_hmc1[i*no_batch+j] ).count() << " ";			
				}
                	        time1_file << std::endl;		
			}
		}
		time1_file.close();

		std::ofstream time2_file(filename + "_time_sn_hmc2.log");	
		if (time2_file.is_open())
		{
			for(int i = 0; i < sample_num_sets_num; i++)
			{
				for(int j=0;j<no_batch;j++)
				{
					time2_file << duration_cast<milliseconds>( times_hmc2[i*no_batch+j] ).count() << " ";			
				}
                	        time2_file << std::endl;		
			}
		}
		time2_file.close();

		std::ofstream time3_file(filename + "_time_sn_mcmc.log");	
		if (time3_file.is_open())
		{
			for(int i = 0; i < sample_num_sets_num; i++)
			{
				for(int j=0;j<no_batch;j++)
				{
					time3_file << duration_cast<milliseconds>( times_mcmc[i*no_batch+j] ).count() << " ";			
				}
                	        time3_file << std::endl;		
			}
		}
		time3_file.close();

		std::ofstream time4_file(filename + "_time_sn_rs.log");	
		if (time4_file.is_open())
		{
			for(int i = 0; i < sample_num_sets_num; i++)
			{
				for(int j=0;j<no_batch;j++)
				{
					time4_file << duration_cast<milliseconds>( times_rs[i*no_batch+j] ).count() << " ";			
				}
                	        time4_file << std::endl;		
			}
		}
		time4_file.close();

		std::ofstream time5_file(filename + "_time_sn_hrs.log");	
		if (time5_file.is_open())
		{
			for(int i = 0; i < sample_num_sets_num; i++)
			{
				for(int j=0;j<no_batch;j++)
				{
					time5_file << duration_cast<milliseconds>( times_hrs[i*no_batch+j] ).count() << " ";			
				}
                	        time5_file << std::endl;		
			}
		}
		time5_file.close();

	}

}
