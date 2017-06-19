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
#include "Sampler/MonteCarloSampler.h"
#include "Sampler/HitAndRunSampler.h"
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

    const double levelSet = 1.1 * dimt->getMinTime(startVec, goalVec);
    std::chrono::high_resolution_clock::duration duration;
    std::cout << "Level set: " << levelSet << std::endl;

    ompl::base::SpaceInformationPtr si = createDimtSpaceInformation(dimt, minval, maxval);

    ompl::base::ProblemDefinitionPtr pdef = createDimtProblem(startVec, goalVec, si, dimt);

    double levelSetRatios[] = {1.6, 1.5, 1.4, 1.3, 1.2, 1.1};
    int levelSetsNum = sizeof(levelSetRatios) / sizeof(double);
    std::cout << "level set num " << levelSetsNum << std::endl;

    std::ofstream timeFile(filename + ".log");

    int samplerNum = 7;

    for (int j = 0; j < levelSetsNum; j++)
    {
        double levelSet = levelSetRatios[j] * (goalVec - startVec).norm();
        std::cout << "Level set: " << levelSet << std::endl;


        for (int i = 0; i < numBatch; i++)
        {
            std::cout << "BATCH " << i << std::endl;

            std::vector<std::chrono::high_resolution_clock::duration> times(samplerNum);
            std::vector<uint> sampleNums(samplerNum);
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
                sampleNums[curr] = hmcSamples.cols();
            }
            curr++;

            {
                MatrixXd mcmcSamples;
                double sigma = 5;
                int max_steps = 20;
                double alpha = 0.5;
                ompl::base::MCMCSampler mcmcSampler(si, pdef, levelSet, 100, 100, alpha, sigma, max_steps);
                mcmcSamples = mcmcSampler.sample(numSamples, times[curr]);
                sampleNums[curr] = mcmcSamples.cols();
            }
            curr++;

            {
                MatrixXd rejSamples;                
                ompl::base::RejectionSampler rejSampler(si, pdef, levelSet, 100, 100);
                rejSamples = rejSampler.sample(numSamples, times[curr]);
                rejectionRatio = rejSampler.getRejectionRatio();
                sampleNums[curr] = rejSamples.cols();
            }
            curr++;

            {
                MatrixXd dimthrsSamples;
                ompl::base::DimtHierarchicalRejectionSampler dimthrsSampler(si, pdef, dimt, levelSet,
                                                                            100, 100);
                dimthrsSamples = dimthrsSampler.sample(numSamples, times[curr]);
                sampleNums[curr] = dimthrsSamples.cols();

            }
            curr++;

            {
                MatrixXd gibbsSamples;
                ompl::base::GibbsSampler gibbsSampler(si, pdef, levelSet, 100, 100);
                gibbsSamples = gibbsSampler.sample(numSamples, times[curr]);
                sampleNums[curr] = gibbsSamples.cols();
            }
            curr++;

            {
                MatrixXd hitnrunSamples;
                ompl::base::HitAndRunSampler hitnrunSampler(si, pdef, levelSet, 100, 100);
                hitnrunSamples = hitnrunSampler.sample(numSamples, times[curr]);
                sampleNums[curr] = hitnrunSamples.cols();
            }

            appendTimeAndRatioToFile(times, rejectionRatio, sampleNums, timeFile);
        }

    }

    if (save)
    {
        timeFile.close();
    }
}
