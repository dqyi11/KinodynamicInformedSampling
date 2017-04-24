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
#include "Sampler/HitAndRun.h"
#include "OmplWrappers/DimtStateSpace.h"
#include "OmplWrappers/OmplHelpers.h"
#include "Dimt/DoubleIntegrator.h"
#include "Dimt/Dimt.h"
#include "Dimt/Params.h"
#include "Benchmark/SampleBenchmark.h"
#include "Benchmark/TimeBenchmark.h"
#include "Benchmark/OptionParse.h"
#include "Benchmark/ProblemGeneration.h"

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
    int numbatch = args[1];

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
    std::uniform_real_distribution<double> dis01(0.0, 1.0);

    // Initializations
    Dimt dimt(param.a_max);
    DoubleIntegrator<param.dof>::Vector maxAccelerations, maxVelocities;
    for (unsigned int i = 0; i < param.dof; ++i)
    {
        maxVelocities[i] = 10;
        maxAccelerations[i] = param.a_max;
    }
    DoubleIntegrator<param.dof> doubleIntegrator(maxAccelerations, maxVelocities);

    // Construct the state space we are planning in
    ompl::base::StateSpacePtr space(new ompl::base::DimtStateSpace(dimt, doubleIntegrator, dof.dimensions));
    ompl::base::RealVectorBounds bounds(dof.dimensions);
    bounds.setLow(minval);
    bounds.setHigh(maxval);
    space->as<ompl::base::DimtStateSpace>()->setBounds(bounds);
    ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
    si->setup();

    std::ofstream logFile(filename + ".log");
    if(!logFile.is_open())
    {
        throw std::runtime_error("file not opened");
    }

    int numSamplers = 4;
    std::vector<std::chrono::high_resolution_clock::duration> times(numSamplers);
    std::vector<double> rejectionRatios(numbatch);
    for (unsigned int i = 0; i < numbatch; i++)
    {
        std::cout << "BATCH " << i << std::endl;

        // sample new start and goal
        for (int d = 0; d < numDim; d++)
        {
            startVec(d) = dis(gen);
            goalVec(d) = dis(gen);
        }

        // Set custom start and goal
        ompl::base::State *startState = space->allocState();
        ompl::base::State *goalState = space->allocState();
        for (int d = 0; d < dof.dimensions; d++)
        {
            if (d % 2 == 0)  // position
            {
                startState->as<ompl::base::RealVectorStateSpace::StateType>()->values[d] = startVec[d];
                goalState->as<ompl::base::RealVectorStateSpace::StateType>()->values[d] = goalVec[d];
            }
            else  // velocity
            {
                startState->as<ompl::base::RealVectorStateSpace::StateType>()->values[d] = startVec[d];
                goalState->as<ompl::base::RealVectorStateSpace::StateType>()->values[d] = goalVec[d];
            }
        }
        ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(space, startState);
        ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal(space, goalState);

        // create a level set
        double rndNum = 0.5*dis01(gen)+1;
        const double levelSet = rndNum * dimt.get_min_time(startVec, goalVec);
        //std::chrono::high_resolution_clock::duration duration;
        //std::cout << "Level set: " << levelSet << std::endl;

        // create new problem definition
        ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));
        pdef->setStartAndGoalStates(start, goal);

        const ompl::base::OptimizationObjectivePtr opt = ompl::base::OptimizationObjectivePtr(
            new ompl::base::DimtObjective<param.dof>(si, startVec, goalVec, doubleIntegrator));
        pdef->setOptimizationObjective(opt);



        int curr=0;

        /*
        {
            MatrixXd hmcSamples;
            double alpha = 0.5;
            double L = 5;
            double epsilon = 0.1;
            double sigma = 1;
            int maxSteps = 20;
            ompl::base::HMCSampler hmcSampler(si, pdef, levelSet, 100, 100, alpha, L, epsilon, sigma, maxSteps);
            hmcSamples = hmcSampler.sample(numSamples, times[curr]);
        }

        curr++;

        {
            MatrixXd hmcSamples;
            double alpha = 0.5;
            double L = 5;
            double epsilon = 0.1;
            double sigma = 1;
            int maxSteps = 20;
            ompl::base::HMCSampler hmcSampler(si, pdef, levelSet, 100, 100, alpha, L, epsilon, sigma, maxSteps);
            hmcSamples = hmcSampler.sampleBatchMemorized(numSamples, times[curr]);
        }

        curr++;
        */
        {
            MatrixXd mcmcSamples;
            double sigma = 5;
            int maxSteps = 20;
            double alpha = 0.5;
            ompl::base::MCMCSampler mcmcSampler(si, pdef, levelSet, 100, 100, alpha, sigma, maxSteps);
            mcmcSamples = mcmcSampler.sample(numSamples, times[curr]);
        }

        curr++;

        {
            MatrixXd rejSamples;
            ompl::base::RejectionSampler rejSampler(si, pdef, levelSet, 100, 100);
            rejSamples = rejSampler.sample(numSamples, times[curr]);
            rejectionRatios[i] = rejSampler.getRejectionRatio();
        }

        curr++;

        {
            MatrixXd dimthrsSamples;
            DoubleIntegrator<1>::Vector maxAccelerations1, maxVelocities1;
            for (unsigned int i = 0; i < 1; ++i)
            {
                maxVelocities1[i] = 10;
                maxAccelerations1[i] = dof.a_max;
            }
            DoubleIntegrator<1> doubleIntegrator1dof(maxAccelerations1, maxVelocities1);
            ompl::base::DimtHierarchicalRejectionSampler dimthrsSampler(si, pdef, levelSet, 100, 100, doubleIntegrator1dof);
            dimthrsSamples = dimthrsSampler.sample(numSamples, times[curr]);

        }

        curr++;

        {
            MatrixXd gibbsSamples;
            ompl::base::GibbsSampler gibbsSampler(si, pdef, levelSet, 100, 100);
            gibbsSamples = gibbsSampler.sample(numSamples, times[curr]);
        }

        curr++;

        {
            MatrixXd hitnrunSamples;
            ompl::base::HitAndRun hitnrunSampler(si, pdef, levelSet, 100, 100);
            hitnrunSamples = hitnrunSampler.sample(numSamples, times[curr]);
        }

        appendTimeAndRatioToFile(times, rejectionRatios[i], numSamples, logFile);
    }

    logFile.close();

}
