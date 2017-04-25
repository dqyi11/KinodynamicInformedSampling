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
    bool runGhrej = (args[5] == 1) ? true : false;
    bool runDimthrs = (args[6] == 1) ? true : false;
    bool runGibbs = (args[7] == 1) ? true : false;
    bool runHitnrun = (args[8] == 1) ? true : false;

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
    gen.seed(1); /* TODO remove */
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

    const ompl::base::OptimizationObjectivePtr opt = ompl::base::OptimizationObjectivePtr(
        new ompl::base::DimtObjective<param.dof>(si, startVec, goalVec, dimt));
    pdef->setOptimizationObjective(opt);

    // Initialize the sampler
    // HMC parameters
    MatrixXd hmcSamples;
    MatrixXd hmc2Samples;
    if (runHmc)
    {
        double alpha = 0.5;
        double L = 5;
        double epsilon = 0.1;
        double sigma = 1;
        int maxSteps = 20;
        ompl::base::HMCSampler hmcSampler(si, pdef, levelSet, 100, 100, alpha, L, epsilon, sigma, maxSteps);
        std::cout << "Running HMC Sampling..." << std::endl;
        hmcSamples = hmcSampler.sample(numSamples, duration);
        if (time)
        {
            std::cout << "Total time ";
            printTime(duration, std::cout);
            std::cout << std::endl;
        }
        std::cout << "Running HMC2 Sampling..." << std::endl;
        hmc2Samples = hmcSampler.sampleBatchMemorized(numSamples, duration);
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
        double sigma = 5;
        int maxSteps = 20;
        double alpha = 0.5;
        ompl::base::MCMCSampler mcmcSampler(si, pdef, levelSet, 100, 100, alpha, sigma, maxSteps);
        std::cout << "Running MCMC Sampling..." << std::endl;
        mcmcSamples = mcmcSampler.sample(numSamples, duration);
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
        double rejectionRatio = 0.0;
        std::cout << "Running Rejection Sampling..." << std::endl;
        rejSamples = rejSampler.sample(numSamples, duration);
        if (time)
        {
            std::cout << "Total time ";
            printTime(duration, std::cout);
            std::cout << std::endl;
        }
    }

    MatrixXd ghrejSamples;
    if (runGhrej)
    {
        //   	ProblemDefinition geo_prob = ProblemDefinition(start_state,
        //   goal_state, state_min,
        //                                                  state_max, level_set,
        //   	[dimt, start_state, goal_state](const VectorXd& state)
        //   	{
        //       		return (start_state - state).norm() + (goal_state -
        //       state).norm();
        //   	});

        //   	GeometricHierarchicalRejectionSampler ghrejSampler =
        //       		GeometricHierarchicalRejectionSampler(geo_prob);
        //   	std::cout << "Running Geometric Hierarchical Rejection
        //   Sampling..." << std::endl;
        //   	ghrejSamples = ghrejSampler.sample(no_samples, duration);
        //   	if(time)
        //   	{
        //   		printTime(duration);
        // }
    }

    MatrixXd dimthrsSamples;
    if (runDimthrs)
    {
        DoubleIntegrator<1>::Vector maxAccelerations1, maxVelocities1;
        for (unsigned int i = 0; i < 1; ++i)
        {
            maxVelocities1[i] = 10;
            maxAccelerations1[i] = param.a_max;
        }
        DoubleIntegrator<1> doubleIntegrator1dof(maxAccelerations1, maxVelocities1);

        ompl::base::DimtHierarchicalRejectionSampler dimthrsSampler(si, pdef, levelSet, 100, 100, doubleIntegrator1dof);
        std::cout << "Running DIMT HRS..." << std::endl;
        dimthrsSamples = dimthrsSampler.sample(numSamples, duration);
        if (time)
        {
            std::cout << "Total time ";
            printTime(duration, std::cout);
            std::cout << std::endl;
        }
    }

    MatrixXd gibbsSamples;
    if (runGibbs)
    {
        ompl::base::GibbsSampler gibbsSampler(si, pdef, levelSet, 100, 100);
        std::cout << "Running Gibbs Sampler..." << std::endl;
        gibbsSamples = gibbsSampler.sample(numSamples, duration);
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
        ompl::base::HitAndRun hitnrunSampler(si, pdef, levelSet, 100, 100);
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
            std::ofstream hmc2File(filename + "_hmc2.log");
            printSampleToFile(hmc2Samples, hmc2File);
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

        if (runGhrej)
        {
            std::ofstream ghrejFile(filename + "_ghrej.log");
            printSampleToFile(ghrejSamples, ghrejFile);
        }

        if (runDimthrs)
        {
            std::ofstream dimthrsFile(filename + "_dimthrs.log");
            printSampleToFile(dimthrsSamples, dimthrsFile);
        }

        if (runGibbs)
        {
            std::ofstream gibbsFile(filename + "_gibbs.log");
            printSampleToFile(gibbsSamples, gibbsFile);
        }

        if (runHitnrun)
        {
            std::ofstream hitnrunFile(filename + "_hitnrun.log");
            printSampleToFile(hitnrunSamples, hitnrunFile);
        }

        std::cout << "Saved samples and costs to " << filename << std::endl;
    }
}
