
#include "hubo_motion_ros/hubo_motion_panel.h"


namespace hubo_motion_ros
{


void HuboMotionPanel::handleLibCheckToggle(bool active)
{
    if(active)
        libertyThread.start();
    else
        stopLiberty();
}

void HuboMotionPanel::handleNavCheckToggle(bool active)
{
    if(active)
        spacenavThread.start();
    else
        stopSpacenav();
}

void HuboMotionPanel::getRefreshData(double data, int i, int j)
{
    datas[i][j]->setText(QString::number(data, 'g', 3));
}

void HuboMotionPanel::handleLibQuit() { libertyCheck->setChecked(false); }

void HuboMotionPanel::handleNavQuit() { spacenavCheck->setChecked(false); }


void ManipRelay::haltOperation()
{
    alive = false;
}

void ManipRelay::getUpdateFrequency(double value)
{
    updateFreq = value;
}

void ManipRelay::getWaistValue(int val)
{
    waistAngle = val*M_PI/180.0;
}

void SpacenavRelay::switchLeft(bool active)
{
    if(active)
    {
        queueSide = LEFT;
        restart = true;
    }
}

void SpacenavRelay::switchRight(bool active)
{
    if(active)
    {
        queueSide = RIGHT;
        restart = true;
    }
}

void ManipRelay::graspL() { cmd.m_grasp[LEFT] = MC_GRASP_NOW; }
void ManipRelay::graspR() { cmd.m_grasp[RIGHT] = MC_GRASP_NOW; }
void ManipRelay::openL() { cmd.m_grasp[LEFT] = MC_RELEASE_NOW; }
void ManipRelay::openR() { cmd.m_grasp[RIGHT] = MC_RELEASE_NOW; }
void ManipRelay::loosenL() { cmd.m_grasp[LEFT] = MC_GRASP_LIMP; }
void ManipRelay::loosenR() { cmd.m_grasp[RIGHT] = MC_GRASP_LIMP; }



}
