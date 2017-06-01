#pragma once

// OMPL
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <iostream>
#include <fstream>

namespace ompl
{
    namespace base
    {
        class MyInformedRRTstar : public ompl::geometric::InformedRRTstar
        {
        public:
            MyInformedRRTstar(const ompl::base::SpaceInformationPtr &si);

            virtual ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc) override;

            void initLogFile(std::string scenarioName, std::string samplerName, int id)
            {
                std::stringstream ss;
                ss << scenarioName.c_str() << "_" << samplerName.c_str() << "_" << id << ".csv";
                out_.open(ss.str());

                std::cout << "SAVING FILE TO " << ss.str() << " = " << out_.is_open() << std::endl;
            }

        private:
            std::ofstream out_;
        };

        using MyInformedRRTstarPtr = std::shared_ptr<MyInformedRRTstar>;

    }
}
