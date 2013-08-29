
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

void SpacenavRelay::switchBoth(bool active)
{
    if(active)
    {
        // Maybe do something here? Probably doesn't matter...
    }
}


void HuboMotionPanel::switchLeft(bool active)
{
    if(active)
    {
        param.arm = T_LEFT;
        ach_put(&teleopParamChan, &param, sizeof(param));
    }
}

void HuboMotionPanel::switchRight(bool active)
{
    if(active)
    {
        param.arm = T_RIGHT;
        ach_put(&teleopParamChan, &param, sizeof(param));
    }
}

void HuboMotionPanel::switchBoth(bool active)
{
    if(active)
    {
        param.arm = T_BOTH;
        ach_put(&teleopParamChan, &param, sizeof(param));
    }
}


void HuboMotionPanel::graspL(bool active)
{
    if(active)
    {
        param.leftFin = T_GRASP;
        ach_put(&teleopParamChan, &param, sizeof(param));
    }
}

void HuboMotionPanel::openL(bool active)
{
    if(active)
    {
        param.leftFin = T_OPEN;
        ach_put(&teleopParamChan, &param, sizeof(param));
    }
}

void HuboMotionPanel::loosenL(bool active)
{
    if(active)
    {
        param.leftFin = T_LOOSEN;
        ach_put(&teleopParamChan, &param, sizeof(param));
    }
}


void HuboMotionPanel::graspR(bool active)
{
    if(active)
    {
        param.rightFin = T_GRASP;
        ach_put(&teleopParamChan, &param, sizeof(param));
    }
}

void HuboMotionPanel::openR(bool active)
{
    if(active)
    {
        param.rightFin = T_OPEN;
        ach_put(&teleopParamChan, &param, sizeof(param));
    }
}

void HuboMotionPanel::loosenR(bool active)
{
    if(active)
    {
        param.rightFin = T_LOOSEN;
        ach_put(&teleopParamChan, &param, sizeof(param));
    }
}


void HuboMotionPanel::graspT(bool active)
{
    if(active)
    {
        param.trigFin = T_GRASP;
        ach_put(&teleopParamChan, &param, sizeof(param));
    }
}

void HuboMotionPanel::openT(bool active)
{
    if(active)
    {
        param.trigFin = T_OPEN;
        ach_put(&teleopParamChan, &param, sizeof(param));
    }
}

void HuboMotionPanel::loosenT(bool active)
{
    if(active)
    {
        param.trigFin = T_LOOSEN;
        ach_put(&teleopParamChan, &param, sizeof(param));
    }
}


void ManipRelay::graspL() { cmd.m_grasp[LEFT] = MC_GRASP_NOW; }
void ManipRelay::graspR() { cmd.m_grasp[RIGHT] = MC_GRASP_NOW; }
void ManipRelay::openL() { cmd.m_grasp[LEFT] = MC_RELEASE_NOW; }
void ManipRelay::openR() { cmd.m_grasp[RIGHT] = MC_RELEASE_NOW; }
void ManipRelay::loosenL() { cmd.m_grasp[LEFT] = MC_GRASP_LIMP; }
void ManipRelay::loosenR() { cmd.m_grasp[RIGHT] = MC_GRASP_LIMP; }





}
