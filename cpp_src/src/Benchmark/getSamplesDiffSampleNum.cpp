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
#include "Sampler/HitAndRun.h"
#include "OmplWrappers/OmplHelpers.h"
#include "OmplWrappers/DimtStateSpace.h"
#include "Dimt/Params.h"
#include "Dimt/DoubleIntegratorMinimumTime.h"
#include "Dimt/ProblemGeneration.h"
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
        std::cout << "\t -filename - Filename to save the samples to" << std::endl;
        std::cout << "________________________________________________" << std::endl;
        return std::make_tuple(false, std::vector<int>{});
    }
    else
    {
        std::vector<int> args;

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

    int numBatch = args[0];

    std::string filename;
    bool save;
    std::tie(save, filename) = get_filename(argc, argv);

    // Create a problem definition
    int numDim = param.dimensions;
    double maxval = 25;
    double minval = -25;
    VectorXd startVec(numDim);
    VectorXd goalVec(numDim);
    UniformRealRandomGenerator uniRndGnr;
    for (int i = 0; i < numDim; i++)
    {
        startVec(i) = uniRndGnr.sample(minval, maxval);
        goalVec(i) = uniRndGnr.sample(minval, maxval);
    }

    // Initializations
    std::vector<double> maxVelocities(param.dof, param.v_max);
    std::vector<double> maxAccelerations(param.dof, param.a_max);
    DIMTPtr dimt = std::make_shared<DIMT>(maxVelocities, maxAccelerations);

    const double levelSet = 1.4 * dimt->getMinTime(startVec, goalVec);
    // const double level_set = 1.4 * (goal_state - start_state).norm();
    std::cout << "Level set: " << levelSet << std::endl;

    ompl::base::SpaceInformationPtr si = createDimtSpaceInformation(dimt, minval, maxval);



    int sampleNumSets[] = {100, 200, 500, 1000, 3000, 5000};
    int sampleNumSetsNum = sizeof(sampleNumSets) / sizeof(int);
    std::cout << "sample set num " << sampleNumSetsNum << std::endl;

    int numSampler = 7;
    std::ofstream timeFile(filename + ".log");

    for (int j = 0; j < sampleNumSetsNum; j++)
    {
        uint numSamples = sampleNumSets[j];
        std::cout << "NO SAMPLE " << numSamples << std::endl;

         ompl::base::ProblemDefinitionPtr pdef = createDimtProblem(startVec, goalVec, si, dimt);

        for (int i = 0; i < numBatch; i++)
        {
            std::cout << "BATCH " << i << std::endl;
            std::vector<std::chrono::high_resolution_clock::duration> times(numSampler);
            double rejectionRatio = 0.0;

            int curr = 0;

            {
                MatrixXd hmcSamples;
                double alpha = 0.5;
                double L = 5;
                double epsilon = 0.1;
                double sigma = 1;
                int max_steps = 20;
                ompl::base::HMCSampler hmcSampler(si, pdef, levelSet, 100, 100, alpha, L, epsilon, sigma, max_steps);
                hmcSamples = hmcSampler.sample(numSamples, times[curr]);
            }
            curr++;

            {
                MatrixXd hmcSamples;
                double alpha = 0.5;
                double L = 5;
                double epsilon = 0.1;
                double sigma = 1;
                int max_steps = 20;
                ompl::base::HMCSampler hmcSampler(si, pdef, levelSet, 100, 100, alpha, L, epsilon, sigma, max_steps);
                hmcSamples = hmcSampler.sampleBatchMemorized(numSamples, times[curr]);
            }
            curr++;

            {
                MatrixXd mcmcSamples;
                double sigma = 5;
                int max_steps = 20;
                double alpha = 0.5;
                ompl::base::MCMCSampler mcmcSampler(si, pdef, levelSet, 100, 100, alpha, sigma, max_steps);
                mcmcSamples = mcmcSampler.sample(numSamples, times[curr]);
            }
            curr++;

            {
                MatrixXd rejSamples;
                ompl::base::RejectionSampler rejSampler(si, pdef, levelSet, 100, 100);
                rejSamples = rejSampler.sample(numSamples, times[curr]);
                rejectionRatio = rejSampler.getRejectionRatio();
            }
            curr++;

            {
                MatrixXd dimthrsSamples;
                ompl::base::DimtHierarchicalRejectionSampler dimthrsSampler(si, pdef, dimt, levelSet,
                                                                            100, 100);
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

            appendTimeAndRatioToFile(times, rejectionRatio, numSamples, timeFile);
        }
    }

    if (save)
    {
        timeFile.close();
    }
}
