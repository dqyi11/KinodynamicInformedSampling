#ifndef MULTILINKDI_WINDOW_HPP_
#define MULTILINKDI_WINDOW_HPP_

#include <dart/gui/gui.hpp>

class MultiLinkDI;

class MultiLinkDIWindow : public dart::gui::SimWindow
{
public:
    MultiLinkDIWindow() : di_(NULL)
    {
        waypointIdx_ = 0;
    }

    void setMultiLinkDI(MultiLinkDI* di)
    {
        di_ = di;
    }

    // Override virtual function on demand
    void draw() override;

    /*
    void setPath(Eigen::MatrixXd& path)
    {
        path_ = path;
    }*/

    void clearPath()
    {
        pathA_ = Eigen::MatrixXd(0,0);
        pathB_ = Eigen::MatrixXd(0,0);
    }

    void setConfigPath(Eigen::MatrixXd& configPath);

    void animate(int perMs= 100);

    void keyboard(unsigned char key, int x, int y) override;

    void prevWaypoint();

    void nextWaypoint();

protected:
    void simulateCurrentWaypoint();

    Eigen::MatrixXd pathA_;
    Eigen::MatrixXd pathB_;
    MultiLinkDI* di_;
    int waypointIdx_;


    int default_step_time = 500;
    int default_end_delay_time = 500;
};

#endif // MULTILINKDI_WINDOW_HPP_
