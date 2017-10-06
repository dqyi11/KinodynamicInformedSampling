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
#include "Sampler/MonteCarloSampler.h"
#include "Sampler/RejectionSampler.h"
#include "Sampler/HitAndRunSampler.h"
#include "OmplWrappers/DimtStateSpace.h"
#include "OmplWrappers/OmplHelpers.h"
#include "Dimt/Params.h"
#include "Dimt/DoubleIntegratorMinimumTime.h"
#include "Dimt/ProblemGeneration.h"
#include "Benchmark/SampleBenchmark.h"
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
            args.push_back(10);  // Default to 20 batches

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
    int numDim = param.dimensions;
    double maxval = param.s_max;
    double minval = -param.s_max;
    UniformRealRandomGenerator uniRndGnr;

    // Initializations
    std::vector<double> maxVelocities(param.dof, param.v_max);
    std::vector<double> maxAccelerations(param.dof, param.a_max);
    //maxVelocities[1] = 0.1;
    //maxAccelerations[1] = 0.1;
    DIMTPtr dimt = std::make_shared<DIMT>(maxVelocities, maxAccelerations);

    ompl::base::SpaceInformationPtr si = createDimtSpaceInformation(dimt, minval, maxval);
    auto startState = si->allocState();
    auto goalState = si->allocState();


    const double levelSet = 1.4 * dimt->getMinTime(startState, goalState);

    std::ofstream logFile(filename + ".log");
    if(!logFile.is_open())
    {
        throw std::runtime_error("file not opened");
    }

    int numSamplers = 1;
    numbatch = 50;
    numSamplers = 500;
    for (int i = 0; i < numbatch; i++)
    {
        //std::cout << "BATCH " << i << std::flush;
        std::vector<std::chrono::high_resolution_clock::duration> times(numSamplers);
        std::vector<uint> sampleNums(numSamplers);
        std::vector<double> rejectionRatios(numSamplers, 1.0);

        // sample new start and goal
        for (int i = 0; i < numDim; i++)
        {
            startState->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = uniRndGnr.sample(minval, maxval);
            goalState->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = uniRndGnr.sample(minval, maxval);
        }
        // create a level set
        //double rndNum = uniRndGnr.sample(1.0, 2.0);
        //const double levelSet = rndNum * dimt->getMinTime(startState, goalState);
        
        const double levelSet = 1.51 - i * 0.01;
        //std::cout <<  " ratio " << rndNum << " " << std::flush;

        // create new problem definition
        ompl::base::ProblemDefinitionPtr pdef = createDimtProblem(startState, goalState, si, dimt);

        int curr=0;

        /*
        std::cout << " MCMC " << std::flush;
        {
            MatrixXd mcmcSamples;
            double alpha = 0.5;
            double L = 5;
            double epsilon = 0.1;
            double sigma = 1;
            int maxSteps = 50000000;
            ompl::base::MCMCSampler mcmcSampler(si, pdef, levelSet, 100, 100, epsilon, sigma, maxSteps);
            mcmcSamples = mcmcSampler.sample(numSamples, times[curr]);
            rejectionRatios[curr] = mcmcSampler.getAcceptanceRatio();
            sampleNums[curr] = mcmcSamples.rows();
        }
        printTime(times[curr], std::cout);
        curr++;

        std::cout << " HRS " << std::flush;
        {
            MatrixXd dimthrsSamples;
            ompl::base::DimtHierarchicalRejectionSampler dimthrsSampler(si, pdef, dimt,
                                                                        levelSet, 100, 100);
            dimthrsSamples = dimthrsSampler.sample(numSamples, times[curr]);
            rejectionRatios[curr] = dimthrsSampler.getAcceptanceRatio();
            sampleNums[curr] = dimthrsSamples.rows();

        }
        printTime(times[curr], std::cout);
        curr++;

        int numTrials = 5;
        std::cout << " H&R (" << numTrials << ") " << std::flush;
        {
            MatrixXd hitnrunSamples;
            ompl::base::HitAndRunSampler hitnrunSampler(si, pdef, levelSet, 100, 100, numTrials);
            hitnrunSamples = hitnrunSampler.sample(numSamples, times[curr]);
            rejectionRatios[curr] = hitnrunSampler.getAcceptanceRatio();
            sampleNums[curr] = hitnrunSamples.rows();
        }
        printTime(times[curr], std::cout);
        curr++;
        */
        //std::cout << " REJ " << std::flush;
        {
            std::cout << levelSet << " " << std::flush;
            MatrixXd rejSamples;
            ompl::base::RejectionSampler rejSampler(si, pdef, levelSet, 100, 100);
            rejSamples = rejSampler.sample(numSamples, times[curr]);
            rejectionRatios[curr] = rejSampler.getAcceptanceRatio();
            sampleNums[curr] = rejSamples.rows();

            std::cout << rejectionRatios[curr] << std::endl;
        }
        //printTime(times[curr], std::cout);
        //curr++;

        //std::cout << "        REJ RATIO[" << rejectionRatios[3] << "]" << std::flush;

        //appendTimeAndRatioToFile(times, rejectionRatios, sampleNums, logFile);

        //std::cout << 
 

        //std::cout << std::endl;
    }

    logFile.close();

}
