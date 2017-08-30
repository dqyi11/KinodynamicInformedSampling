#ifndef MULTILINKDI_WINDOW_HPP_
#define MULTILINKDI_WINDOW_HPP_

#include <dart/gui/gui.hpp>

class MultiLinkDI;

class MultiLinkDIWindow : public dart::gui::SimWindow
{
public:
    MultiLinkDIWindow() : di_(NULL)
    {
    }

    void setMultiLinkDI(MultiLinkDI* di)
    {
        di_ = di;
    }

    // Override virtual function on demand
    void draw() override;

    void setPath(Eigen::MatrixXd& path)
    {
        path_ = path;
    }

    void clearPath()
    {
        path_ = Eigen::MatrixXd(0,0);
    }

    void setConfigPath(Eigen::MatrixXd& configPath);

    void animate(int perMs= 100);

protected:
    Eigen::MatrixXd path_;
    MultiLinkDI* di_;
};

#endif // MULTILINKDI_WINDOW_HPP_
