
#include "hubo_motion_ros/hubo_motion_panel.h"


namespace hubo_motion_ros
{

void HuboMotionPanel::handleCheckToggle(bool active)
{
    if(active)
        libertyThread.start();
    else
        stopLiberty();
}

void HuboMotionPanel::getRefreshData(hubo_manip_cmd_t cmd)
{
    for(int i=0; i<datas.size(); i++)
        for(int j=0; j<datas[i].size(); j++)
            datas[i][j]->setText(QString::number(cmd.pose[i].data[j]));
}

void HuboMotionPanel::handleLibQuit() { libertyCheck->setChecked(false); }


void LibertyRelay::haltOperation()
{
    alive = false;
}

void LibertyRelay::getUpdateFrequency(double value)
{
    updateFreq = value;
}


void LibertyRelay::graspL() { cmd.m_grasp[LEFT] = MC_GRASP_NOW; }
void LibertyRelay::graspR() { cmd.m_grasp[RIGHT] = MC_GRASP_NOW; }
void LibertyRelay::openL() { cmd.m_grasp[LEFT] = MC_RELEASE_NOW; }
void LibertyRelay::openR() { cmd.m_grasp[RIGHT] = MC_RELEASE_NOW; }
void LibertyRelay::loosenL() { cmd.m_grasp[LEFT] = MC_GRASP_LIMP; }
void LibertyRelay::loosenR() { cmd.m_grasp[RIGHT] = MC_GRASP_LIMP; }



}
