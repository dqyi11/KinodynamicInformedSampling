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
#include "Benchmark/GeometryProblemGeneration.h"
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
    int dimension = 4;
    double vmax = 10.0, amax = 1.0;

    std::string filename;
    bool save;
    std::tie(save, filename) = get_filename(argc, argv);

    // Create a problem definition
    double maxval = 25;
    double minval = -25;
    VectorXd startVec(dimension);
    VectorXd goalVec(dimension);
    std::mt19937 gen( std::random_device{}());
    std::uniform_real_distribution<double> dis(-25, 25);
    for (int i = 0; i < dimension; i++)
    {
        startVec(i) = dis(gen);
        goalVec(i) = dis(gen);
    }

    ompl::base::SpaceInformationPtr si = createGeometrySpaceInformation(dimension, minval, maxval);
    ompl::base::ProblemDefinitionPtr pdef = createGeometryProblem(startVec, goalVec, si);

    // Initializations
    const double levelSet = 1.4 *  getCost(pdef->getOptimizationObjective(), si, startVec, goalVec);
    std::cout << "Level set: " << levelSet << std::endl;

    double levelSetRatios[] = {1.6, 1.5, 1.4, 1.3, 1.2, 1.1};
    int levelSetsNum = sizeof(levelSetRatios) / sizeof(double);
    std::cout << "level set num " << levelSetsNum << std::endl;

    int numSampler = 7;

    std::ofstream timeFile(filename + ".log");

    for (int j = 0; j < levelSetsNum; j++)
    {
        double levelSet = levelSetRatios[j] * (goalVec - startVec).norm();
        std::cout << "Level set: " << levelSet << std::endl;

        for (int i = 0; i < numBatch; i++)
        {
            std::cout << "BATCH " << i << std::endl;
            std::vector<std::chrono::high_resolution_clock::duration> times(numSampler);
            double rejectionRatio = 1.0;

            int curr = 0;

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
            curr ++;

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
            curr ++;

            {
                MatrixXd mcmcSamples;
                double sigma = 5;
                int max_steps = 20;
                double alpha = 0.5;
                ompl::base::MCMCSampler mcmcSampler(si, pdef, levelSet, 100, 100, alpha, sigma, max_steps);
                mcmcSamples = mcmcSampler.sample(numSamples, times[curr]);
            }
            curr ++;

            {
                MatrixXd rejSamples;
                ompl::base::RejectionSampler rejSampler(si, pdef, levelSet, 100, 100);
                rejSamples = rejSampler.sample(numSamples, times[curr]);
            }
            curr ++;

            {
                MatrixXd dimthrsSamples;
                ompl::base::GeometricHierarchicalRejectionSampler dimthrsSampler(si, pdef, levelSet,
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
