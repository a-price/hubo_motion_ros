
#include "hubo_motion_ros/hubo_motion_panel.h"
#include <control_msgs/GripperCommandAction.h>

#include <actionlib/client/simple_action_client.h>


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


void HuboMotionPanel::graspL()
{
//    if(active)
//    {
//        param.leftFin = T_GRASP;
//        ach_put(&teleopParamChan, &param, sizeof(param));
//    }

}

void HuboMotionPanel::openL()
{
//    if(active)
//    {
//        param.leftFin = T_OPEN;
//        ach_put(&teleopParamChan, &param, sizeof(param));
//    }
}

void HuboMotionPanel::loosenL()
{
//    if(active)
//    {
//        param.leftFin = T_LOOSEN;
//        ach_put(&teleopParamChan, &param, sizeof(param));
//    }
}


void HuboMotionPanel::graspR()
{
//    if(active)
//    {
//        param.rightFin = T_GRASP;
//        ach_put(&teleopParamChan, &param, sizeof(param));
//    }
}

void HuboMotionPanel::openR()
{
//    if(active)
//    {
//        param.rightFin = T_OPEN;
//        ach_put(&teleopParamChan, &param, sizeof(param));
//    }
}

void HuboMotionPanel::loosenR()
{
//    if(active)
//    {
//        param.rightFin = T_LOOSEN;
//        ach_put(&teleopParamChan, &param, sizeof(param));
//    }
}


void HuboMotionPanel::graspT()
{
//    if(active)
//    {
//        param.trigFin = T_GRASP;
//        ach_put(&teleopParamChan, &param, sizeof(param));
//    }
}

void HuboMotionPanel::openT()
{
//    if(active)
//    {
//        param.trigFin = T_OPEN;
//        ach_put(&teleopParamChan, &param, sizeof(param));
//    }
}

void HuboMotionPanel::loosenT()
{
    control_msgs::GripperCommandGoal goal;
    goal.command.position = 0.0;

    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> ac("/hubo_trajectory_server_gripper", true);
    ac.waitForServer();
    ac.sendGoal(goal);

//    if(active)
//    {
//        param.trigFin = T_LOOSEN;
//        ach_put(&teleopParamChan, &param, sizeof(param));
//    }
}


void ManipRelay::graspL() { cmd.m_grasp[LEFT] = MC_GRASP_NOW; }
void ManipRelay::graspR() { cmd.m_grasp[RIGHT] = MC_GRASP_NOW; }
void ManipRelay::openL() { cmd.m_grasp[LEFT] = MC_RELEASE_NOW; }
void ManipRelay::openR() { cmd.m_grasp[RIGHT] = MC_RELEASE_NOW; }
void ManipRelay::loosenL() { cmd.m_grasp[LEFT] = MC_GRASP_LIMP; }
void ManipRelay::loosenR() { cmd.m_grasp[RIGHT] = MC_GRASP_LIMP; }





}
