// Standard Librry
#include <iostream>
#include <tuple>
#include <vector>

// Eigen
#include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;

// OMPL
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/OptimizationObjective.h>

// Internal Libraries
#include "Sampler/RejectionSampler.h"
#include "Sampler/RejectionSampler.h"
#include "Sampler/MonteCarloSamplers.h"
#include "Sampler/HitAndRun.h"
#include "OmplWrappers/MyOptimizationObjective.h"
#include "OmplWrappers/OmplHelpers.h"
#include "OmplWrappers/DimtStateSpace.h"
#include "Dimt/DoubleIntegrator.h"
#include "Dimt/Dimt.h"
#include "Dimt/Params.h"
#include "Benchmark/TimeBenchmark.h"
#include "Benchmark/SampleBenchmark.h"

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
        std::cout << "\t -samples - Number of samples to get" << std::endl;
        std::cout << "\t -time - Boolean (0,1)" << std::endl;
        std::cout << "\t -hmc - Boolean (0,1)" << std::endl;
        std::cout << "\t -mcmc - Boolean (0,1)" << std::endl;
        std::cout << "\t -rej - Boolean (0,1)" << std::endl;
        std::cout << "\t -ghrej - Boolean(0,1)" << std::endl;
        std::cout << "\t -dimthrs - Boolean(0,1)" << std::endl;
        std::cout << "\t -gibbs - Boolean(0,1)" << std::endl;
        std::cout << "\t -hitnrun - Boolean(0,1)" << std::endl;
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

        // Get the boolean to determine if we should time
        if (cmdOptionExists(argv, argv + argc, "-time"))
            args.push_back(atoi(getCmdOption(argv, argv + argc, "-time")));
        else
            args.push_back(0);  // Default to not print time

        // Get the boolean to determine if we run hmc
        if (cmdOptionExists(argv, argv + argc, "-hmc"))
            args.push_back(atoi(getCmdOption(argv, argv + argc, "-hmc")));
        else
            args.push_back(1);  // Default to run hmc

        // Get the boolean to determine if we run mcmc
        if (cmdOptionExists(argv, argv + argc, "-mcmc"))
            args.push_back(atoi(getCmdOption(argv, argv + argc, "-mcmc")));
        else
            args.push_back(1);  // Default to run mcmc

        // Get the boolean to determine if we run rej
        if (cmdOptionExists(argv, argv + argc, "-rej"))
            args.push_back(atoi(getCmdOption(argv, argv + argc, "-rej")));
        else
            args.push_back(1);  // Default to run rej

        // Get the boolean to determine if we run geometric hierarchical rejection
        // sampling
        if (cmdOptionExists(argv, argv + argc, "-ghrej"))
            args.push_back(atoi(getCmdOption(argv, argv + argc, "-ghrej")));
        else
            args.push_back(1);  // Default to run ghrej

        // Get the boolean to determine if we run dimt hierarchical rejection
        // sampling
        if (cmdOptionExists(argv, argv + argc, "-dimthrs"))
            args.push_back(atoi(getCmdOption(argv, argv + argc, "-dimthrs")));
        else
            args.push_back(1);  // Default to run dimthrs

        // Get the boolean to determine if we run gibbs sampling
        if (cmdOptionExists(argv, argv + argc, "-gibbs"))
            args.push_back(atoi(getCmdOption(argv, argv + argc, "-gibbs")));
        else
            args.push_back(1);  // Default to run dimthrs

        // Get the boolean to determine if we run hit and run sampling
        if (cmdOptionExists(argv, argv + argc, "-hitnrun"))
            args.push_back(atoi(getCmdOption(argv, argv + argc, "-hitnrun")));
        else
            args.push_back(1);  // Default to run dimthrs

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
    bool time = (args[1] == 1) ? true : false;
    bool run_hmc = (args[2] == 1) ? true : false;
    bool run_mcmc = (args[3] == 1) ? true : false;
    bool run_rej = (args[4] == 1) ? true : false;
    bool run_ghrej = (args[5] == 1) ? true : false;
    bool run_dimthrs = (args[6] == 1) ? true : false;
    bool run_gibbs = (args[7] == 1) ? true : false;
    bool run_hitnrun = (args[8] == 1) ? true : false;

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
    gen.seed(1); /* TODO remove */
    std::uniform_real_distribution<double> dis(-25, 25);
    for (int i = 0; i < num_dim; i++)
    {
        start_state(i) = dis(gen);
        goal_state(i) = dis(gen);
    }
    // std::cout << " start " << start_state << std::endl;
    // std::cout << " goal " << goal_state << std::endl;

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

    const ompl::base::OptimizationObjectivePtr opt = ompl::base::OptimizationObjectivePtr(
        new ompl::base::DimtObjective<param.dof>(si, start_state, goal_state, double_integrator));
    pdef->setOptimizationObjective(opt);

    // Initialize the sampler
    // HMC parameters
    MatrixXd hmc_samples;
    MatrixXd hmc_samples2;
    if (run_hmc)
    {
        double alpha = 0.5;
        double L = 5;
        double epsilon = 0.1;
        double sigma = 1;
        int max_steps = 20;
        ompl::base::HMCSampler hmc_s(si, pdef, level_set, 100, 100, alpha, L, epsilon, sigma, max_steps);
        std::cout << "Running HMC Sampling..." << std::endl;
        hmc_samples = hmc_s.sample(no_samples, duration);
        if (time)
        {
            std::cout << "Total time ";
            printTime(duration, std::cout);
            std::cout << std::endl;
        }
        std::cout << "Running HMC2 Sampling..." << std::endl;
        hmc_samples2 = hmc_s.sampleBatchMemorized(no_samples, duration);
        if (time)
        {
            std::cout << "Total time ";
            printTime(duration, std::cout);
            std::cout << std::endl;
        }
    }

    MatrixXd mcmc_samples;
    if (run_mcmc)
    {
        double sigma = 5;
        int max_steps = 20;
        double alpha = 0.5;
        ompl::base::MCMCSampler mcmc_s(si, pdef, level_set, 100, 100, alpha, sigma, max_steps);
        std::cout << "Running MCMC Sampling..." << std::endl;
        mcmc_samples = mcmc_s.sample(no_samples, duration);
        if (time)
        {
            std::cout << "Total time ";
            printTime(duration, std::cout);
            std::cout << std::endl;
        }
    }

    MatrixXd rej_samples;
    if (run_rej)
    {
        ompl::base::RejectionSampler rej_s(si, pdef, level_set, 100, 100);
        std::cout << "Running Rejection Sampling..." << std::endl;
        rej_samples = rej_s.sample(no_samples, duration);
        if (time)
        {
            std::cout << "Total time ";
            printTime(duration, std::cout);
            std::cout << std::endl;
        }
    }

    MatrixXd ghrej_samples;
    if (run_ghrej)
    {
        //   	ProblemDefinition geo_prob = ProblemDefinition(start_state,
        //   goal_state, state_min,
        //                                                  state_max, level_set,
        //   	[dimt, start_state, goal_state](const VectorXd& state)
        //   	{
        //       		return (start_state - state).norm() + (goal_state -
        //       state).norm();
        //   	});

        //   	GeometricHierarchicalRejectionSampler ghrej_s =
        //       		GeometricHierarchicalRejectionSampler(geo_prob);
        //   	std::cout << "Running Geometric Hierarchical Rejection
        //   Sampling..." << std::endl;
        //   	ghrej_samples = ghrej_s.sample(no_samples, duration);
        //   	if(time)
        //   	{
        //   		printTime(duration);
        // }
    }

    MatrixXd dimthrs_samples;
    if (run_dimthrs)
    {
        DoubleIntegrator<1>::Vector maxAccelerations1, maxVelocities1;
        for (unsigned int i = 0; i < 1; ++i)
        {
            maxVelocities1[i] = 10;
            maxAccelerations1[i] = param.a_max;
        }
        DoubleIntegrator<1> double_integrator_1dof(maxAccelerations1, maxVelocities1);

        ompl::base::DimtHierarchicalRejectionSampler dimthrs_s(si, pdef, level_set, 100, 100, double_integrator_1dof);
        std::cout << "Running DIMT HRS..." << std::endl;
        dimthrs_samples = dimthrs_s.sample(no_samples, duration);
        if (time)
        {
            std::cout << "Total time ";
            printTime(duration, std::cout);
            std::cout << std::endl;
        }
    }

    MatrixXd gibbs_samples;
    if (run_gibbs)
    {
        ompl::base::GibbsSampler gibbs_s(si, pdef, level_set, 100, 100);
        std::cout << "Running Gibbs Sampler..." << std::endl;
        gibbs_samples = gibbs_s.sample(no_samples, duration);
        if (time)
        {
            std::cout << "Total time ";
            printTime(duration, std::cout);
            std::cout << std::endl;
        }
    }

    MatrixXd hitnrun_samples;
    if (run_hitnrun)
    {
        ompl::base::HitAndRun hitnrun_s(si, pdef, level_set, 100, 100);
        std::cout << "Running Hit and Run Sampler..." << std::endl;
        gibbs_samples = hitnrun_s.sample(no_samples, duration);
        if (time)
        {
            std::cout << "Total time ";
            printTime(duration, std::cout);
            std::cout << std::endl;
        }
    }

    if (save)
    {
        std::cout << "START SAVING" << std::endl;
        if (run_hmc)
        {
            std::ofstream hmc_file(filename + "_hmc.log");
            printSampleToFile(hmc_samples, hmc_file);
            std::ofstream hmc2_file(filename + "_hmc2.log");
            printSampleToFile(hmc2_samples, hmc2_file);
        }

        if (run_mcmc)
        {
            std::ofstream mcmc_file(filename + "_mcmc.log");
            printSampleToFile(mcmc_samples, mcmc_file);
        }

        if (run_rej)
        {
            std::ofstream rej_file(filename + "_rej.log");
            printSampleToFile(rej_samples, rej_file);
        }

        if (run_ghrej)
        {
            std::ofstream ghrej_file(filename + "_ghrej.log");
            printSampleToFile(ghrej_samples, ghrej_file);
        }

        if (run_dimthrs)
        {
            std::ofstream dimthrs_file(filename + "_dimthrs.log");
            printSampleToFile(dimthrs_samples, dimthrs_file);
        }

        if (run_gibbs)
        {
            std::ofstream gibbs_file(filename + "_gibbs.log");
            printSampleToFile(gibbs_samples, gibbs_file);
        }

        if (run_hitnrun)
        {
            std::ofstream hitnrun_file(filename + "_hitnrun.log");
            printSampleToFile(hitnrun_samples, hitnrun_file);
        }

        std::cout << "Saved samples and costs to " << filename << std::endl;
    }
}