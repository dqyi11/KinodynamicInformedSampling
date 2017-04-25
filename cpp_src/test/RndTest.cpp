#include "Dimt/Dimt.h"
#include "Dimt/DoubleIntegrator.h"
#include "Dimt/Params.h"
#include <iostream>

int main()
{
    int numDim = param.dimensions;
    double maxval = 25;
    double minval = -25;
    VectorXd startVec(numDim);
    VectorXd goalVec(numDim);
    VectorXd stateVec(numDim);
    std::mt19937 gen( std::random_device{}());
    std::uniform_real_distribution<double> disPos(minval, maxval);
    std::uniform_real_distribution<double> disVel(-param.v_max, param.v_max);
    gen.seed(1);
    for (int i = 0; i < numDim; i++)
    {
        if(i<param.dof)
        {
            startVec(i) = disPos(gen);
            goalVec(i) = disPos(gen);
            stateVec(i) = disPos(gen);
        }
        else
        {
            startVec(i) = disVel(gen);
            goalVec(i) = disVel(gen);
            stateVec(i) = disVel(gen);
        }
    }

    std::cout << "START " << startVec << std::endl;
    std::cout << "GOAL " << goalVec << std::endl;
    std::cout << "STATE " << stateVec << std::endl;

    // DIMT Test
    Dimt dimt(param.a_max, param.v_max);

    /*
    VectorXd start_state(4);
    start_state << 0, 0, 0, 0;
    VectorXd goal_state(4);
    goal_state << 1, 1, 1, 1;
    VectorXd state(4);
    state << 0.5, 0.5, 0.5, 0.5;
    */
    double dimt_t1 = dimt.get_min_time(startVec, stateVec);
    double dimt_t2 = dimt.get_min_time(stateVec, goalVec);
    std::cout << "DIMT MINI TIME " << dimt_t1  + dimt_t2  << std::endl;

    // Double_Integrator Test
    //  std::cout << "Time = " << dimt.get_min_time(start_state, goal_state,  state) << std::endl;
    DoubleIntegrator<2>::Vector maxAccelerations, maxVelocities;
    for (unsigned int i = 0; i < param.dof; ++i)
    {
        maxVelocities[i] = 10;
        maxAccelerations[i] = 1;
    }
    DoubleIntegrator<2> double_integrator(maxAccelerations, maxVelocities);
    double di_t1 = double_integrator.getMinTime(startVec, stateVec);
    double di_t2 = double_integrator.getMinTime(stateVec, goalVec);
    std::cout << "DI MINI TIME " << di_t1 + di_t2 << std::endl;

    /*
    double acceleration1 = 1;

    for(int i=0;i<param.dof;i++)
    {

        std::cout << "START TO STATE ";
        std::cout << i << " DIMT " << dimt.get_min_time_1dof(startVec[i], startVec[i+param.dof], stateVec[i], stateVec[i+param.dof]);
        std::cout << " DI " << double_integrator.getMinTime1D(startVec[i+param.dof], stateVec[i+param.dof], stateVec[i]-startVec[i],
                                                             maxAccelerations[i], maxVelocities[i], std::numeric_limits<double>::max(),
                                                             acceleration1) << std::endl;


        std::cout << "STATE TO GOAL ";
        std::cout << i << " DIMT " << dimt.get_min_time_1dof(stateVec[i], stateVec[i+param.dof], goalVec[i], goalVec[i+param.dof]);
        std::cout << " DI " << double_integrator.getMinTime1D(stateVec[i+param.dof], goalVec[i+param.dof], goalVec[i]-stateVec[i],
                                                             maxAccelerations[i], maxVelocities[i], std::numeric_limits<double>::max(),
                                                             acceleration1) << std::endl;
    }
    */
    return 0;
}
