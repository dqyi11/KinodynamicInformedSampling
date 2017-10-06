#ifndef MY_BIT_STAR_H_
#define MY_BIT_STAR_H_

// OMPL
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include "ompl/datastructures/NearestNeighborsLinear.h"
#include "ompl/datastructures/NearestNeighborsGNAT.h"
#include <iostream>
#include <fstream>
#include <ios>

namespace ompl
{
    namespace base
    {
        class MyBITstar : public ompl::geometric::BITstar
        {
        public:
            typedef enum { LOAD_SAMPLES, SAVE_SAMPLES, RANDOM_SAMPLES } PlannerMode;
            MyBITstar(const ompl::base::SpaceInformationPtr &si);

            virtual ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc) override;

            void initLogFile(std::string scenarioName, std::string samplerName, int id)
            {
                std::stringstream ss;
                ss << scenarioName.c_str() << "_" << samplerName.c_str() << "_" << id << ".csv";
                out_.open(ss.str());

                std::cout << "SAVING FILE TO " << ss.str() << " = " << out_.is_open() << std::endl;
            }

            ompl::base::PlannerStatus solve(double solveTime)
            {
                if (solveTime < 1.0)
                    return solve(timedPlannerTerminationCondition(solveTime));
                return solve(timedPlannerTerminationCondition(solveTime, std::min(solveTime / 100.0, 0.1)));
            }

            ompl::base::PlannerStatus solveAfterLoadingSamples(std::string filename, double solveTime)
            {
                mode_ = LOAD_SAMPLES;

                /*
                if(nn_)
                {
                    //nn_.reset(new NearestNeighborsLinear<Motion *>());
                    nn_.reset(new NearestNeighborsGNAT<Motion*>());
                    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
                }*/

                sampleLoadStream_.open(filename.c_str(), std::ios::in);
                if (solveTime < 1.0)
                    return solve(timedPlannerTerminationCondition(solveTime));
                return solve(timedPlannerTerminationCondition(solveTime, std::min(solveTime / 100.0, 0.1)));
            }

            ompl::base::PlannerStatus solveAndSaveSamples(std::string filename, double solveTime)
            {
                mode_ = SAVE_SAMPLES;

                /*
                if(nn_)
                {
                    //nn_.reset(new NearestNeighborsLinear<Motion *>());
                    nn_.reset(new NearestNeighborsGNAT<Motion*>());
                    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
                }*/

                sampleSaveStream_.open(filename.c_str(), std::ios::out);
                if (solveTime < 1.0)
                    return solve(timedPlannerTerminationCondition(solveTime));
                return solve(timedPlannerTerminationCondition(solveTime, std::min(solveTime / 100.0, 0.1)));
            }

            std::string fromState(ompl::base::State* fromState);
            bool toState(std::string stateString, ompl::base::State* toState);

            uint64_t getSamplesGeneratedNum() { return samplesGeneratedNum_; }


        private:
            PlannerMode mode_;
            std::ofstream out_;
            std::ofstream sampleSaveStream_;
            std::ifstream sampleLoadStream_;

            uint64_t samplesGeneratedNum_;

        };

        using MyBITstarPtr = std::shared_ptr<MyBITstar>;

    }
}

#endif // MY_BIT_STAR_H_
