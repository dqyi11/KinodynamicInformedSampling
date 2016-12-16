#include "Dimt/Dimt.h"
#include <iostream>

int main()
{
	double a_max = 1;
  Dimt dimt(a_max);
  VectorXd start_state(4);
  start_state << 0,0,0,0;
  VectorXd goal_state(4);
  goal_state << 1,1,1,1;
  VectorXd state(4);
  state << 0.5,0.5,0.5,0.5;
  std::cout << "Error = " << dimt.get_min_time(start_state, goal_state, state)-1.7025 << std::endl;
 //  std::cout << "Time = " << dimt.get_min_time(start_state, goal_state, state) << std::endl;
	// std::cout << "Time1 = " << dimt.get_min_time_1dof(0, 0, 0.5, 0.5, 1, 1) << std::endl;
	// std::cout << "Time2 = " << dimt.get_min_time_1dof(0, 0, 1, 1) << std::endl;
	return 0;
}