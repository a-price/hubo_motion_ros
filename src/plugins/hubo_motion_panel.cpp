
#include "hubo_motion_ros/hubo_motion_panel.h"

namespace hubo_motion_ros
{

HuboMotionPanel::HuboMotionPanel(QWidget *parent)
    : rviz::Panel(parent)
{
    // TODO: Add channels

    achManager = new AchNetworkWidget;
    achManager->setNetworkName("Manipulation");

    achManager->addChannel(QString::fromLocal8Bit(CHAN_HUBO_MANIP_CMD),
                           AchNetworkWidget::ACHD_PUSH);

    achManager->addChannel(QString::fromLocal8Bit(CHAN_HUBO_MANIP_STATE),
                           AchNetworkWidget::ACHD_PULL);

    achManager->addChannel(QString::fromLocal8Bit(CHAN_HUBO_MANIP_PARAM),
                           AchNetworkWidget::ACHD_PUSH);

    achManager->addChannel(QString::fromLocal8Bit(CHAN_HUBO_MANIP_TRAJ),
                           AchNetworkWidget::ACHD_PUSH, 3, 1000000);


    QHBoxLayout* dumbLayout = new QHBoxLayout;
    dumbLayout->addWidget(achManager);

    setLayout(dumbLayout);
}

HuboMotionPanel::~HuboMotionPanel()
{

}

void HuboMotionPanel::save(rviz::Config config) const
{
    rviz::Panel::save(config);
    achManager->saveIP(config);

}

void HuboMotionPanel::load(const rviz::Config &config)
{
    rviz::Panel::load(config);
    achManager->loadIP(config);
}







}



#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( hubo_motion_ros::HuboMotionPanel, rviz::Panel )
