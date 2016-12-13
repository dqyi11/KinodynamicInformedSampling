// Standard Librry
#include <iostream>
#include <tuple>
#include <vector>
#include <fstream>

// Eigen
#include <Eigen/Dense>
using Eigen::MatrixXd;

// #include <ProblemDefinition/ProblemDefinition.h>
#include <Sampler/RejectionSampler.h>

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
//
//

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
	int num_dim = 3;
	VectorXd start_state(num_dim);
	start_state << 0, 0, 0;
	VectorXd goal_state(num_dim);
	goal_state << 0, 1, 0;
	VectorXd state_min(num_dim);
	state_min << -10, -10, -10;
	VectorXd state_max(num_dim);
	state_max << 10, 10, 10;
	double level_set = 2;
	ProblemDefinition prob = ProblemDefinition(start_state, goal_state, state_min, state_max, level_set,
		[start_state, goal_state](const VectorXd& state)
		{
			return (start_state - state).norm() + (goal_state - state).norm();
		});

	// Initialize the sampler
	RejectionSampler s = RejectionSampler(prob);
	std::cout << "Created the sampler" << std::endl;

	// Sampler
	MatrixXd samples = s.sample(no_samples, time);

	std::cout << "Got the samples" << std::endl;

	if(save)
	{
		std::ofstream file(filename);
		if (file.is_open())
		{
			for(int i = 0; i < samples.rows(); i++)
			{
				file << samples.row(i) << std::endl;	
			}
		}
		file.close();
		std::cout << "Saved samples and costs to " << filename << std::endl;
	}

	// print 
}