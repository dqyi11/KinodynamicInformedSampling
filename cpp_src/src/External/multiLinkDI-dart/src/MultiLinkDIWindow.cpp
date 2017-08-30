#include "MultiLinkDI.hpp"
#include "MultiLinkDIWindow.hpp"

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
