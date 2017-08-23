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
#include "Sampler/MonteCarloSampler.h"
#include "Sampler/HitAndRunSampler.h"
#include "OmplWrappers/MyOptimizationObjective.h"
#include "OmplWrappers/OmplHelpers.h"
#include "OmplWrappers/DimtStateSpace.h"
#include "Dimt/DoubleIntegratorMinimumTime.h"
#include "Dimt/ProblemGeneration.h"
#include "Benchmark/TimeBenchmark.h"
#include "Benchmark/SampleBenchmark.h"
#include "Benchmark/OptionParse.h"

std::tuple<bool, std::vector<int>> handleArguments(int argc, char *argv[])
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
        std::cout << "\t -hrs - Boolean(0,1)" << std::endl;
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
            args.push_back(1);  // Default to run hmc

        // Get the boolean to determine if we run rej
        if (cmdOptionExists(argv, argv + argc, "-rej"))
            args.push_back(atoi(getCmdOption(argv, argv + argc, "-rej")));
        else
            args.push_back(1);  // Default to run rej

        // Get the boolean to determine if we run dimt hierarchical rejection
        // sampling
        if (cmdOptionExists(argv, argv + argc, "-hrs"))
            args.push_back(atoi(getCmdOption(argv, argv + argc, "-hrs")));
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
    bool time = (args[1] == 1) ? true : false;
    bool runHmc = (args[2] == 1) ? true : false;
    bool runMcmc = (args[3] == 1) ? true : false;
    bool runRej = (args[4] == 1) ? true : false;
    bool runHrs = (args[5] == 1) ? true : false;
    bool runHitnrun = (args[6] == 1) ? true : false;

    std::string filename;
    bool save;
    std::tie(save, filename) = get_filename(argc, argv);

    // Create a problem definition
    int numDim = param.dimensions;
    double maxval = 25;
    double minval = -25;
    UniformRealRandomGenerator uniRndGnr;
    std::chrono::high_resolution_clock::duration duration;

    // Initializations
    std::vector<double> maxVelocities(param.dof, param.v_max);
    std::vector<double> maxAccelerations(param.dof, param.a_max);
    //maxVelocities[1] = 0.1;
    //maxAccelerations[1] = 0.1;
    DIMTPtr dimt = std::make_shared<DIMT>(maxVelocities, maxAccelerations);

    ompl::base::SpaceInformationPtr si = createDimtSpaceInformation(dimt, minval, maxval);
    auto startState = si->allocState();
    auto goalState = si->allocState();
    for (int i = 0; i < numDim; i++)
    {
        startState->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = uniRndGnr.sample(minval, maxval);
        goalState->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = uniRndGnr.sample(minval, maxval);
    }

    ompl::base::ProblemDefinitionPtr pdef = createDimtProblem(startState, goalState, si, dimt);

    const double levelSet = 1.4 * dimt->getMinTime(startState, goalState);

    // Initialize the sampler
    // HMC parameters
    MatrixXd hmcSamples;
    if (runHmc)
    {
        double alpha = 0.5;
        double L = 5;
        double epsilon = 0.1;
        double sigma = 1;
        int maxSteps = 50000000;
        ompl::base::HMCSampler hmcSampler(si, pdef, levelSet, 100, 100, alpha, L, epsilon, sigma, maxSteps);
        std::cout << "Running HMC Sampling..." << std::endl;
        hmcSamples = hmcSampler.sample(numSamples, duration);
        if (time)
        {
            std::cout << "Total time ";
            printTime(duration, std::cout);
            std::cout << std::endl;
        }
    }

    MatrixXd mcmcSamples;
    if (runMcmc)
    {
        double alpha = 0.5;
        double sigma = 1;
        int maxSteps = 50000000;
        ompl::base::MCMCSampler mcmcSampler(si, pdef, levelSet, 100, 100, alpha, sigma, maxSteps);
        mcmcSamples = mcmcSampler.sample(numSamples, duration);
        std::cout << "Running MCMC Sampler..." << std::endl;
        if (time)
        {
            std::cout << "Total time ";
            printTime(duration, std::cout);
            std::cout << std::endl;
        }
    }

    MatrixXd rejSamples;
    if (runRej)
    {
        ompl::base::RejectionSampler rejSampler(si, pdef, levelSet, 100, 100);
        std::cout << "Running Rejection Sampler..." << std::endl;
        rejSamples = rejSampler.sample(numSamples, duration);
        if (time)
        {
            std::cout << "Total time ";
            printTime(duration, std::cout);
            std::cout << std::endl;
        }
    }

    MatrixXd dimthrsSamples;
    if (runHrs)
    {
        ompl::base::DimtHierarchicalRejectionSampler dimthrsSampler(si, pdef, dimt,
                                                                    levelSet, 100, 100);
        std::cout << "Running DIMT HRS..." << std::endl;
        dimthrsSamples = dimthrsSampler.sample(numSamples, duration);
        if (time)
        {
            std::cout << "Total time ";
            printTime(duration, std::cout);
            std::cout << std::endl;
        }
    }

    MatrixXd hitnrunSamples;
    if (runHitnrun)
    {
        ompl::base::HitAndRunSampler hitnrunSampler(si, pdef, levelSet, 100, 100, 10);
        std::cout << "Running Hit and Run Sampler..." << std::endl;
        hitnrunSamples = hitnrunSampler.sample(numSamples, duration);
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
        if (runHmc)
        {
            std::ofstream hmcFile(filename + "_hmc.log");
            printSampleToFile(hmcSamples, hmcFile);
        }

        if (runMcmc)
        {
            std::ofstream mcmcFile(filename + "_mcmc.log");
            printSampleToFile(mcmcSamples, mcmcFile);
        }

        if (runRej)
        {
            std::ofstream rejFile(filename + "_rej.log");
            printSampleToFile(rejSamples, rejFile);
        }

        if (runHrs)
        {
            std::ofstream dimthrsFile(filename + "_hrs.log");
            printSampleToFile(dimthrsSamples, dimthrsFile);
        }

        if (runHitnrun)
        {
            std::ofstream hitnrunFile(filename + "_hitnrun.log");
            printSampleToFile(hitnrunSamples, hitnrunFile);
        }

        std::cout << "Saved samples and costs to " << filename << std::endl;
    }
}
