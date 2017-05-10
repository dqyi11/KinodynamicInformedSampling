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
#include "Sampler/HitAndRun.h"
#include "OmplWrappers/DimtStateSpace.h"
#include "OmplWrappers/OmplHelpers.h"
#include "Dimt/DoubleIntegratorMinimumTime.h"
#include "Dimt/Params.h"
#include "Dimt/ProblemGeneration.h"
#include "Benchmark/TimeBenchmark.h"
#include "Benchmark/OptionParse.h"

std::tuple<bool, std::vector<int>> handleArguments(int argc, char *argv[])
{
    if (cmdOptionExists(argv, argv + argc, "-h"))
    {
        std::cout << "________________________________________________" << std::endl;
        std::cout << "Main function for sampling" << std::endl;
        std::cout << "__________________________________        double rejectionRatio = 0.0;______________" << std::endl;
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

    ompl::base::SpaceInformationPtr si = createDimtSpaceInformation(dimt, minval, maxval);

    ompl::base::ProblemDefinitionPtr pdef = createDimtProblem(startVec, goalVec, si, dimt);

    std::ofstream timeFile(filename + "_time.log");
    if(save)
    {
        if(!timeFile.is_open())
            throw std::runtime_error("file not open");
    }

    int samplerNum = 7;
    for (int i = 0; i < numbatch; i++)
    {
        std::cout << "BATCH " << i << std::endl;

        double rejectionRatio = 1.0;
        std::vector<std::chrono::high_resolution_clock::duration> times(samplerNum);

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
        curr++;

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
        curr++;

        {
            MatrixXd mcmcSamples;
            double sigma = 5;
            int maxSteps = 20;
            double alpha = 0.5;
            ompl::base::MCMCSampler mcmcSampler(si, pdef, levelSet, 100, 100, alpha, sigma, maxSteps);
            mcmcSamples = mcmcSampler.sample(numSamples, times[2 * numbatch + i]);
        }        
        curr++;

        {
            MatrixXd rejSamples;
            ompl::base::RejectionSampler rejSampler(si, pdef, levelSet, 100, 100);
            rejSamples = rejSampler.sample(numSamples, times[3 * numbatch + i]);
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

        if(save)
        {
            appendTimeAndRatioToFile(times, rejectionRatio, numSamples, timeFile);
        }
    }

    if (save)
    {
        timeFile.close();
    }
}
