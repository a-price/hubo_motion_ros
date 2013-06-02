
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

void HuboMotionPanel::getRefreshData(double data, int i, int j)
{
    datas[i][j]->setText(QString::number(data, 'g', 3));
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

void LibertyRelay::getWaistValue(int val)
{
    waistAngle = val*M_PI/180.0;
}


void LibertyRelay::graspL() { cmd.m_grasp[LEFT] = MC_GRASP_NOW; }
void LibertyRelay::graspR() { cmd.m_grasp[RIGHT] = MC_GRASP_NOW; }
void LibertyRelay::openL() { cmd.m_grasp[LEFT] = MC_RELEASE_NOW; }
void LibertyRelay::openR() { cmd.m_grasp[RIGHT] = MC_RELEASE_NOW; }
void LibertyRelay::loosenL() { cmd.m_grasp[LEFT] = MC_GRASP_LIMP; }
void LibertyRelay::loosenR() { cmd.m_grasp[RIGHT] = MC_GRASP_LIMP; }



}
