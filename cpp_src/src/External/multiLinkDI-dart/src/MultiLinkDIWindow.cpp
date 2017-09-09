#include "MultiLinkDI.hpp"
#include "MultiLinkDIWindow.hpp"
#include <iostream>

void MultiLinkDIWindow::draw()
{
    SimWindow::draw();


}

void MultiLinkDIWindow::setConfigPath(Eigen::MatrixXd& configPath)
{
    clearPath();
    uint waypoint_num = configPath.rows();
    path_ = Eigen::MatrixXd(waypoint_num, 3);
    if(di_)
    {
        for(uint i=0; i<waypoint_num; i++)
        {
            Eigen::Vector3d config = configPath.row(i);
            path_.row(i) = di_->getEndEffectorPos(config);
        }
    }

}

void MultiLinkDIWindow::keyboard(unsigned char key, int x, int y)
{
  switch(key)
  {
    case 'a':
    {
        prevWaypoint();
        std::cout << "PRESS A " << waypointIdx_ << std::endl;
        Eigen::VectorXd configA = di_->getWaypoint(waypointIdx_);
        di_->setConfiguration( configA );
        break;
    }
    case 'd':
    {
        nextWaypoint();
        std::cout << "PRESS D " << waypointIdx_ << std::endl;
        Eigen::VectorXd configD = di_->getWaypoint(waypointIdx_);
        di_->setConfiguration( configD );
        break;
    }
    case 's':
    {
        std::cout << "PRESS S " << waypointIdx_ << std::endl;
        simulateCurrentWaypoint();
        break;
    }
    case 'x':
    {
        std::cout << "PRESS X " << waypointIdx_ << std::endl;
        int originalWaypointIdx = waypointIdx_;
        // start from zero
        waypointIdx_ = 0;
        while(waypointIdx_ < di_->getWaypointNum()-1)
        {
            simulateCurrentWaypoint();
            nextWaypoint();
        }
        waypointIdx_ = originalWaypointIdx;
        render();
        break;
    }
    default:
    {
        SimWindow::keyboard(key, x, y);
    }
  }
}

void MultiLinkDIWindow::simulateCurrentWaypoint()
{
    if(waypointIdx_ < di_->getWaypointNum()-1)
    {
        Eigen::VectorXd currConfig = di_->getWaypoint(waypointIdx_);
        Eigen::VectorXd nextConfig = di_->getWaypoint(waypointIdx_+1);
        Eigen::VectorXd deltaConfig = nextConfig - currConfig;
        for(double i=di_->getResolutionSize();
            i <= 1.0+di_->getResolutionSize(); i+= di_->getResolutionSize())
        {
            usleep(default_step_time);
            Eigen::VectorXd newConfig = currConfig + i * deltaConfig;
            bool collision = di_->isCollided(newConfig);
            di_->setConfiguration( newConfig );

            render();
            if(collision==true)
            {
                std::cout << "time step " << i << " " << collision << std::endl;
            }
        }
        usleep(default_end_delay_time);
        di_->setConfiguration(currConfig);
    }
}

void MultiLinkDIWindow::prevWaypoint()
{
    waypointIdx_ --;
    if(waypointIdx_ < 0)
    {
       waypointIdx_ = di_->getWaypointNum() - 1;
    }
}

void MultiLinkDIWindow::nextWaypoint()
{
    waypointIdx_ ++;
    if(waypointIdx_ >= di_->getWaypointNum())
    {
        waypointIdx_ = 0;
    }
}
