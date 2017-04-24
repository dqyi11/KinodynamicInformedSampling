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
    int numBatch = args[1];

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
    std::chrono::high_resolution_clock::duration duration;
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

    // const auto opt = get_geom_opt_obj(si, start_state, goal_state, no_samples);
    const ompl::base::OptimizationObjectivePtr opt =
        ompl::base::OptimizationObjectivePtr(new ompl::base::GeometricObjective(si, startVec, goalVec));

    pdef->setOptimizationObjective(opt);

    double levelSetRatios[] = {1.6, 1.5, 1.4, 1.3, 1.2, 1.1};
    int levelSetsNum = sizeof(levelSetRatios) / sizeof(double);
    std::cout << "level set num " << levelSetsNum << std::endl;

    std::vector<std::chrono::high_resolution_clock::duration> timesHmc1(levelSetsNum * numBatch);
    std::vector<std::chrono::high_resolution_clock::duration> timesHmc2(levelSetsNum * numBatch);
    std::vector<std::chrono::high_resolution_clock::duration> timesMcmc(levelSetsNum * numBatch);
    std::vector<std::chrono::high_resolution_clock::duration> timesRs(levelSetsNum * numBatch);
    std::vector<std::chrono::high_resolution_clock::duration> timesHrs(levelSetsNum * numBatch);

    for (unsigned int j = 0; j < levelSetsNum; j++)
    {
        double levelSet = levelSetRatios[j] * (goalVec - startVec).norm();
        std::cout << "Level set: " << levelSet << std::endl;

        for (unsigned int i = 0; i < numBatch; i++)
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
                hmcSamples = hmcSampler.sample(numSamples, timesHmc1[j * numBatch + i]);
            }

            {
                MatrixXd hmcSamples;
                double alpha = 0.5;
                double L = 5;
                double epsilon = 0.1;
                double sigma = 1;
                int max_steps = 20;
                ompl::base::HMCSampler hmcSampler(si, pdef, levelSet, 100, 100, alpha, L, epsilon, sigma, max_steps);
                hmcSamples = hmcSampler.sampleBatchMemorized(numSamples, timesHmc2[j * numBatch + i]);
            }

            {
                MatrixXd mcmcSamples;
                double sigma = 5;
                int max_steps = 20;
                double alpha = 0.5;
                ompl::base::MCMCSampler mcmcSampler(si, pdef, levelSet, 100, 100, alpha, sigma, max_steps);
                mcmcSamples = mcmcSampler.sample(numSamples, timesMcmc[j * numBatch + i]);
            }

            {
                MatrixXd rejSamples;
                double rejectionRatio = 0.0;
                ompl::base::RejectionSampler rejSampler(si, pdef, levelSet, 100, 100);
                rejSamples = rejSampler.sample(numSamples, timesRs[j * numBatch + i]);
            }
        }
    }

    if (save)
    {
        std::cout << "START SAVING" << std::endl;
        std::ofstream time1_file(filename + "_time_lvl_hmc1.log");
        printTimeToFile(timesHmc1, numBatch, levelSetsNum, time1_file);

        std::ofstream time2_file(filename + "_time_lvl_hmc2.log");
        printTimeToFile(timesHmc2, numBatch, levelSetsNum, time2_file);

        std::ofstream time3_file(filename + "_time_lvl_mcmc.log");
        printTimeToFile(timesMcmc, numBatch, levelSetsNum, time3_file);

        std::ofstream time4_file(filename + "_time_lvl_rs.log");
        printTimeToFile(timesRs, numBatch, levelSetsNum, time4_file);

        std::ofstream time5_file(filename + "_time_lvl_hrs.log");
        printTimeToFile(timesHrs, numBatch, levelSetsNum, time5_file);
    }
}
