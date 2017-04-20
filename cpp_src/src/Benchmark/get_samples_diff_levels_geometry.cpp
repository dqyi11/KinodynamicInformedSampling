// Standard Librry
#include <iostream>
#include <tuple>
#include <vector>
#include <fstream>
#include <limits>


// Eigen
#include <Eigen/Dense>
using Eigen::MatrixXd;

// OMPL
#include <ompl/base/spaces/RealVectorStateSpace.h>

// Internal Libraries
#include "Sampler/RejectionSampler.h"
#include "Sampler/MonteCarloSamplers.h"
#include "Sampler/RejectionSampler.h"
#include "OmplWrappers/OmplHelpers.h"
#include "OmplWrappers/DimtStateSpace.h"
#include "Dimt/DoubleIntegrator.h"
#include "Dimt/Dimt.h"
#include "Dimt/Params.h"
#include "Benchmark/TimeBenchmark.h"

//
// From stackoverflow:
// http://stackoverflow.com/questions/865668/how-to-parse-command-line-arguments-in-c
//
#include <algorithm>

char *getCmdOption(char **begin, char **end, const std::string &option)
{
    char **itr = std::find(begin, end, option);
    if (itr != end && ++itr != end)
    {
        return *itr;
    }
    return 0;
}

bool cmdOptionExists(char **begin, char **end, const std::string &option)
{
    return std::find(begin, end, option) != end;
}

std::tuple<bool, std::vector<int>> handle_arguments(int argc, char *argv[])
{
    if (cmdOptionExists(argv, argv + argc, "-h"))
    {
        std::cout << "________________________________________________" << std::endl;
        std::cout << "Main function for sampling" << std::endl;
        std::cout << "________________________________________________" << std::endl;
        std::cout << "Arguments:" << std::endl;
        std::cout << "\t -batch - NUmber of batches to run" << std::endl;
        std::cout << "\t -samples - Number of samples to get" << std::endl;
        std::cout << "\t -filename - Filename to save the samples to" << std::endl;
        std::cout << "________________________________________________" << std::endl;
        return std::make_tuple(false, std::vector<int>{});
    }
    else
    {
        std::vector<int> args;

        // Get the number of samples
        if (cmdOptionExists(argv, argv + argc, "-samples"))
            args.push_back(atoi(getCmdOption(argv, argv + argc, "-samples")));
        else
            args.push_back(100);  // Default to 100 samples
                                  // Get the batch number
        if (cmdOptionExists(argv, argv + argc, "-batch"))
            args.push_back(atoi(getCmdOption(argv, argv + argc, "-batch")));
        else
            args.push_back(20);  // Default to 20 batches

        return std::make_tuple(true, args);
    }
}

std::tuple<bool, std::string> get_filename(int argc, char *argv[])
{
    if (cmdOptionExists(argv, argv + argc, "-filename"))
        return std::make_tuple(true, std::string(getCmdOption(argv, argv + argc, "-filename")));
    else
        return std::make_tuple(false, "none");
}

std::vector<double> get_random_vector(const double &max, const double &min, const int &num_dim)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(min, max);

    std::vector<double> vec;

    for (int i = 0; i < num_dim; i++)
    {
        vec.push_back(dis(gen));
    }
}

int main(int argc, char *argv[])
{
    //
    // Example for how to use the above sampler
    //
    bool run;
    std::vector<int> args;
    std::tie(run, args) = handle_arguments(argc, argv);
    if (!run)
        return 0;

    int no_samples = args[0];
    int no_batch = args[1];

    std::string filename;
    bool save;
    std::tie(save, filename) = get_filename(argc, argv);

    // Create a problem definition
    int num_dim = 12;
    double maxval = 25;
    double minval = -25;
    VectorXd start_state(num_dim);
    VectorXd goal_state(num_dim);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(-25, 25);
    for (int i = 0; i < num_dim; i++)
    {
        start_state(i) = dis(gen);
        goal_state(i) = dis(gen);
    }

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
    std::chrono::high_resolution_clock::duration duration;
    std::cout << "Level set: " << level_set << std::endl;

    // Construct the state space we are planning in
    ompl::base::StateSpacePtr space(new ompl::base::DimtStateSpace(dimt, double_integrator, param.dimensions));
    ompl::base::RealVectorBounds bounds(param.dimensions);
    bounds.setLow(minval);
    bounds.setHigh(maxval);
    space->as<ompl::base::DimtStateSpace>()->setBounds(bounds);
    ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
    si->setup();

    // Set custom start and goal
    ompl::base::State *start_s = space->allocState();
    ompl::base::State *goal_s = space->allocState();
    for (int i = 0; i < param.dimensions; i++)
    {
        if (i % 2 == 0)  // position
        {
            start_s->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = start_state[i];
            goal_s->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = goal_state[i];
        }
        else  // velocity
        {
            start_s->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = start_state[i];
            goal_s->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = goal_state[i];
        }
    }

    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(space, start_s);
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal(space, goal_s);

    ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));
    pdef->setStartAndGoalStates(start, goal);

    // const auto opt = get_geom_opt_obj(si, start_state, goal_state, no_samples);
    const ompl::base::OptimizationObjectivePtr opt =
        ompl::base::OptimizationObjectivePtr(new ompl::base::GeometricObjective(si, start_state, goal_state));

    pdef->setOptimizationObjective(opt);

    double level_set_ratios[] = {1.6, 1.5, 1.4, 1.3, 1.2, 1.1};
    int level_sets_num = sizeof(level_set_ratios) / sizeof(double);
    std::cout << "level set num " << level_sets_num << std::endl;

    std::vector<std::chrono::high_resolution_clock::duration> times_hmc1(level_sets_num * no_batch);
    std::vector<std::chrono::high_resolution_clock::duration> times_hmc2(level_sets_num * no_batch);
    std::vector<std::chrono::high_resolution_clock::duration> times_mcmc(level_sets_num * no_batch);
    std::vector<std::chrono::high_resolution_clock::duration> times_rs(level_sets_num * no_batch);
    std::vector<std::chrono::high_resolution_clock::duration> times_hrs(level_sets_num * no_batch);

    for (unsigned int j = 0; j < level_sets_num; j++)
    {
        double level_set = level_set_ratios[j] * (goal_state - start_state).norm();
        std::cout << "Level set: " << level_set << std::endl;

        for (unsigned int i = 0; i < no_batch; i++)
        {
            std::cout << "BATCH " << i << std::endl;

            {
                MatrixXd hmc_samples;
                double alpha = 0.5;
                double L = 5;
                double epsilon = 0.1;
                double sigma = 1;
                int max_steps = 20;
                ompl::base::HMCSampler hmc_s(si, pdef, level_set, 100, 100, alpha, L, epsilon, sigma, max_steps);
                hmc_samples = hmc_s.sample(no_samples, times_hmc1[j * no_batch + i]);
            }

            {
                MatrixXd hmc_samples;
                double alpha = 0.5;
                double L = 5;
                double epsilon = 0.1;
                double sigma = 1;
                int max_steps = 20;
                ompl::base::HMCSampler hmc_s(si, pdef, level_set, 100, 100, alpha, L, epsilon, sigma, max_steps);
                hmc_samples = hmc_s.sampleBatchMemorized(no_samples, times_hmc2[j * no_batch + i]);
            }

            {
                MatrixXd mcmc_samples;
                double sigma = 5;
                int max_steps = 20;
                double alpha = 0.5;
                ompl::base::MCMCSampler mcmc_s(si, pdef, level_set, 100, 100, alpha, sigma, max_steps);
                mcmc_samples = mcmc_s.sample(no_samples, times_mcmc[j * no_batch + i]);
            }

            {
                MatrixXd rej_samples;
                ompl::base::RejectionSampler rej_s(si, pdef, level_set, 100, 100);
                rej_samples = rej_s.sample(no_samples, times_rs[j * no_batch + i]);
            }
        }
    }

    if (save)
    {
        std::cout << "START SAVING" << std::endl;
        std::ofstream time1_file(filename + "_time_lvl_hmc1.log");
        printTimeToFile(times_hmc1, no_batch, level_sets_num, time1_file);

        std::ofstream time2_file(filename + "_time_lvl_hmc2.log");
        printTimeToFile(times_hmc2, no_batch, level_sets_num, time2_file);

        std::ofstream time3_file(filename + "_time_lvl_mcmc.log");
        printTimeToFile(times_mcmc, no_batch, level_sets_num, time3_file);

        std::ofstream time4_file(filename + "_time_lvl_rs.log");
        printTimeToFile(times_rs, no_batch, level_sets_num, time4_file);

        std::ofstream time5_file(filename + "_time_lvl_hrs.log");
        printTimeToFile(times_hrs, no_batch, level_sets_num, time5_file);
    }
}
