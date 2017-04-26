#include "Dimt/Dimt.h"
#include "Dimt/DoubleIntegrator.h"
#include "Dimt/Params.h"
#include <iostream>

using Eigen::VectorXd;

int main()
{
    // DIMT Test
    double a_max = 1;
    Dimt dimt(param.a_max, param.v_max);
    VectorXd start_state(4);
    start_state << 0, 0, 0, 0;
    VectorXd goal_state(4);
    goal_state << 1, 1, 1, 1;
    VectorXd state(4);
    state << 0.5, 0.5, 0.5, 0.5;
    std::cout << "Error = " << dimt.get_min_time(start_state, goal_state, state) - 1.7025 << std::endl;

    // Double_Integrator Test
    //  std::cout << "Time = " << dimt.get_min_time(start_state, goal_state,  state) << std::endl;
    DoubleIntegrator<2>::Vector maxAccelerations, maxVelocities;
    for (unsigned int i = 0; i < 2; ++i)
    {
        maxVelocities[i] = 10;
        maxAccelerations[i] = 1;
    }
    DoubleIntegrator<2> double_integrator(maxAccelerations, maxVelocities);
    DoubleIntegrator<2>::StateVector eig_from, eig_to, eig_state;
    for (unsigned int i = 0; i < 4; ++i)
    {
        eig_from[i] = 0;
        eig_to[i] = 1;
        eig_state[i] = 0.5;
    }
    double time = double_integrator.getMinTime(eig_from, eig_state) + double_integrator.getMinTime(state, eig_to);
    std::cout << "Error = " << time - 1.7025 << std::endl;
    double time1, time2;
    double startVelocity = -4.1694302294950321;
    double goalVelocity = 0.80184277892207689;
    double distance = -8.3705982982587575;
    double acceleration1 = 1;
    double_integrator.getPPTime(startVelocity, goalVelocity, distance, acceleration1, NULL, NULL, &time1, &time2);
    std::cout << "T1= " << time1 << " T2=" << time2 << std::endl;

    for(int i=0;i<param.dof;i++)
    {

        std::cout << "START TO STATE ";
        std::cout << i << " DIMT " << dimt.get_min_time_1dof(start_state[i], start_state[i+param.dof], state[i], state[i+param.dof]);
        std::cout << " DI " << double_integrator.getMinTime1D(start_state[i+param.dof], state[i+param.dof], state[i]-start_state[i],
                                                             maxAccelerations[i], maxVelocities[i], std::numeric_limits<double>::max(),
                                                             acceleration1) << std::endl;


        std::cout << "STATE TO GOAL ";
        std::cout << i << " DIMT " << dimt.get_min_time_1dof(state[i], state[i+param.dof], goal_state[i], goal_state[i+param.dof]);
        std::cout << " DI " << double_integrator.getMinTime1D(state[i+param.dof], goal_state[i+param.dof], goal_state[i]-state[i],
                                                             maxAccelerations[i], maxVelocities[i], std::numeric_limits<double>::max(),
                                                             acceleration1) << std::endl;
    }

    return 0;
}
