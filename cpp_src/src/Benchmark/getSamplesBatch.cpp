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
#include "Sampler/MonteCarloSamplers.h"
#include "Sampler/RejectionSampler.h"
#include "OmplWrappers/DimtStateSpace.h"
#include "OmplWrappers/OmplHelpers.h"
#include "Dimt/DoubleIntegrator.h"
#include "Dimt/Dimt.h"
#include "Dimt/Params.h"
#include "Benchmark/TimeBenchmark.h"
#include "Benchmark/OptionParse.h"

std::tuple<bool, std::vector<int>> handleArguments(int argc, char *argv[])
{
    if (cmdOptionExists(argv, argv + argc, "-h"))
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
        if (cmdOptionExists(argv, argv + argc, "-samples"))
            args.push_back(atoi(getCmdOption(argv, argv + argc, "-samples")));
        else
            args.push_back(100);  // Default to 100 samples
                                  // Get the batch number
        if (cmdOptionExists(argv, argv + argc, "-batch"))
            args.push_back(atoi(getCmdOption(argv, argv + argc, "-batch")));
        else
            args.push_back(20);  // Default to 20 batches

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
            args.push_back(1);  // Default to run rej

        return std::make_tuple(true, args);
    }
}

int main(int argc, char *argv[])
{
    //
    // Example for how to use the above sampler
    //
    bool run;
    std::vector<int> args;
    std::tie(run, args) = handleArguments(argc, argv);
    if (!run)
        return 0;

    int numSamples = args[0];
    int numbatch = args[1];
    bool time = (args[2] == 1) ? true : false;
    bool runHmc = (args[3] == 1) ? true : false;
    bool runMcmc = (args[4] == 1) ? true : false;
    bool runRej = (args[5] == 1) ? true : false;
    bool runGhrej = (args[6] == 1) ? true : false;

    std::string filename;
    bool save;
    std::tie(save, filename) = get_filename(argc, argv);

    // Create a problem definition
    int numDim = 12;
    double maxval = 25;
    double minval = -25;
    VectorXd startVec(numDim);
    VectorXd goalVec(numDim);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(-25, 25);
    for (int i = 0; i < numDim; i++)
    {
        startVec(i) = dis(gen);
        goalVec(i) = dis(gen);
    }

    // Initializations
    Dimt dimt(param.a_max);
    DoubleIntegrator<param.dof>::Vector maxAccelerations, maxVelocities;
    for (unsigned int i = 0; i < param.dof; ++i)
    {
        maxVelocities[i] = 10;
        maxAccelerations[i] = param.a_max;
    }
    DoubleIntegrator<param.dof> doubleIntegrator(maxAccelerations, maxVelocities);

    const double levelSet = 1.4 * dimt.get_min_time(startVec, goalVec);
    //std::chrono::high_resolution_clock::duration duration;
    std::cout << "Level set: " << levelSet << std::endl;

    // Construct the state space we are planning in
    ompl::base::StateSpacePtr space(new ompl::base::DimtStateSpace(dimt, doubleIntegrator, param.dimensions));
    ompl::base::RealVectorBounds bounds(param.dimensions);
    bounds.setLow(minval);
    bounds.setHigh(maxval);
    space->as<ompl::base::DimtStateSpace>()->setBounds(bounds);
    ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
    si->setup();

    // Set custom start and goal
    ompl::base::State *startState = space->allocState();
    ompl::base::State *goalState = space->allocState();
    for (int i = 0; i < param.dimensions; i++)
    {
        if (i % 2 == 0)  // position
        {
            startState->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = startVec[i];
            goalState->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = goalVec[i];
        }
        else  // velocity
        {
            startState->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = startVec[i];
            goalState->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = goalVec[i];
        }
    }

    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(space, startState);
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal(space, goalState);

    ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));
    pdef->setStartAndGoalStates(start, goal);

    const ompl::base::OptimizationObjectivePtr opt = ompl::base::OptimizationObjectivePtr(
        new ompl::base::DimtObjective<param.dof>(si, startVec, goalVec, dimt));
    pdef->setOptimizationObjective(opt);

    std::vector<std::chrono::high_resolution_clock::duration> times(5 * numbatch);
    for (unsigned int i = 0; i < numbatch; i++)
    {
        std::cout << "BATCH " << i << std::endl;

        {
            MatrixXd hmcSamples;
            double alpha = 0.5;
            double L = 5;
            double epsilon = 0.1;
            double sigma = 1;
            int maxSteps = 20;
            ompl::base::HMCSampler hmcSampler(si, pdef, levelSet, 100, 100, alpha, L, epsilon, sigma, maxSteps);
            hmcSamples = hmcSampler.sample(numSamples, times[i]);
        }

        {
            MatrixXd hmcSamples;
            double alpha = 0.5;
            double L = 5;
            double epsilon = 0.1;
            double sigma = 1;
            int maxSteps = 20;
            ompl::base::HMCSampler hmcSampler(si, pdef, levelSet, 100, 100, alpha, L, epsilon, sigma, maxSteps);
            hmcSamples = hmcSampler.sampleBatchMemorized(numSamples, times[numbatch + i]);
        }

        {
            MatrixXd mcmcSamples;
            double sigma = 5;
            int maxSteps = 20;
            double alpha = 0.5;
            ompl::base::MCMCSampler mcmcSampler(si, pdef, levelSet, 100, 100, alpha, sigma, maxSteps);
            mcmcSamples = mcmcSampler.sample(numSamples, times[2 * numbatch + i]);
        }

        {
            MatrixXd rejSamples;
            double rejectionRatio = 0.0;
            ompl::base::RejectionSampler rejSampler(si, pdef, levelSet, 100, 100);
            rejSamples = rejSampler.sample(numSamples, times[3 * numbatch + i]);
        }
    }

    if (save)
    {
        std::cout << "START SAVING" << std::endl;
        std::ofstream timeFile(filename + "_time.log");
        printTimeToFile(times, numbatch, 5, timeFile);
    }
}
