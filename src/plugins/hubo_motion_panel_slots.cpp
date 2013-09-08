
#include "hubo_motion_ros/hubo_motion_panel.h"
#include "hubo_motion_ros/ExecuteGripperAction.h"
#include "hubo_motion_ros/TeleopCmd.h"
//#include <control_msgs/GripperCommandAction.h>

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


//void HuboMotionPanel::switchLeft(bool active)
void HuboMotionPanel::switchLeft()
{
//    if(active)
//    {
        hubo_motion_ros::TeleopCmd cmd;
        cmd.CommandType = hubo_motion_ros::TeleopCmd::SWITCH_LEFT;
        cmdPublisher.publish(cmd);
        
        leftSel->setStyleSheet(sideSelectedStyle);
        rightSel->setStyleSheet("");
        bothSel->setStyleSheet("");
//    }
}

//void HuboMotionPanel::switchRight(bool active)
void HuboMotionPanel::switchRight()
{
//    if(active)
//    {
        hubo_motion_ros::TeleopCmd cmd;
        cmd.CommandType = hubo_motion_ros::TeleopCmd::SWITCH_RIGHT;
        cmdPublisher.publish(cmd);
        
        leftSel->setStyleSheet("");
        rightSel->setStyleSheet(sideSelectedStyle);
        bothSel->setStyleSheet("");
//    }
}

//void HuboMotionPanel::switchBoth(bool active)
void HuboMotionPanel::switchBoth()
{
//    if(active)
//    {
        hubo_motion_ros::TeleopCmd cmd;
        cmd.CommandType = hubo_motion_ros::TeleopCmd::SWITCH_DUAL;
        cmdPublisher.publish(cmd); // TODO: Make this do stuff
        
        leftSel->setStyleSheet("");
        rightSel->setStyleSheet("");
        bothSel->setStyleSheet(sideSelectedStyle);
//    }
}

void HuboMotionPanel::eeCmdSlot()
{
    hubo_motion_ros::TeleopCmd cmd;
    cmd.CommandType = hubo_motion_ros::TeleopCmd::END_EFFECTOR;
    cmdPublisher.publish(cmd);
}

void HuboMotionPanel::jsCmdSlot()
{
    hubo_motion_ros::TeleopCmd cmd;
    cmd.CommandType = hubo_motion_ros::TeleopCmd::JOINTSPACE;
    cmdPublisher.publish(cmd);
}

void HuboMotionPanel::resetCmdSlot()
{
    hubo_motion_ros::TeleopCmd cmd;
    cmd.CommandType = hubo_motion_ros::TeleopCmd::RESET;
    cmdPublisher.publish(cmd);
}

void HuboMotionPanel::zerosCmdSlot()
{
    hubo_motion_ros::TeleopCmd cmd;
    cmd.CommandType = hubo_motion_ros::TeleopCmd::ZEROS;
    cmdPublisher.publish(cmd);
}


void HuboMotionPanel::graspL()
{
    hubo_motion_ros::ExecuteGripperGoal goal;
    goal.grip.push_back(hubo_motion_ros::ExecuteGripperGoal::PTA_GRASP_NOW);
    goal.ArmIndex.push_back(hubo_motion_ros::ExecuteGripperGoal::PTA_LEFT);

    actionlib::SimpleActionClient<hubo_motion_ros::ExecuteGripperAction> ac("/hubo_trajectory_server_gripper", true);
    bool response = ac.waitForServer(ros::Duration(actionWait));

    ac.sendGoal(goal);

    if(response)
    {
        loosenLB->setStyleSheet("");
        openLB->setStyleSheet("");
        graspLB->setStyleSheet(selectedStyle);
    }

//    if(active)
//    {
//        param.leftFin = T_GRASP;
//        ach_put(&teleopParamChan, &param, sizeof(param));
//    }

}

void HuboMotionPanel::openL()
{
    hubo_motion_ros::ExecuteGripperGoal goal;
    goal.grip.push_back(hubo_motion_ros::ExecuteGripperGoal::PTA_GRASP_RELEASE_NOW);
    goal.ArmIndex.push_back(hubo_motion_ros::ExecuteGripperGoal::PTA_LEFT);

    actionlib::SimpleActionClient<hubo_motion_ros::ExecuteGripperAction> ac("/hubo_trajectory_server_gripper", true);
    bool response = ac.waitForServer(ros::Duration(actionWait));

    ac.sendGoal(goal);

    if(response)
    {
        loosenLB->setStyleSheet("");
        openLB->setStyleSheet(selectedStyle);
        graspLB->setStyleSheet("");
    }



//    if(active)
//    {
//        param.leftFin = T_OPEN;
//        ach_put(&teleopParamChan, &param, sizeof(param));
//    }
}

void HuboMotionPanel::loosenL()
{
    hubo_motion_ros::ExecuteGripperGoal goal;
    goal.grip.push_back(hubo_motion_ros::ExecuteGripperGoal::PTA_GRASP_LIMP);
    goal.ArmIndex.push_back(hubo_motion_ros::ExecuteGripperGoal::PTA_LEFT);

    actionlib::SimpleActionClient<hubo_motion_ros::ExecuteGripperAction> ac("/hubo_trajectory_server_gripper", true);
    bool response = ac.waitForServer(ros::Duration(actionWait));

    ac.sendGoal(goal);

    if(response)
    {
        loosenLB->setStyleSheet(selectedStyle);
        openLB->setStyleSheet("");
        graspLB->setStyleSheet("");
    }


//    if(active)
//    {
//        param.leftFin = T_LOOSEN;
//        ach_put(&teleopParamChan, &param, sizeof(param));
//    }
}


void HuboMotionPanel::graspR()
{
    hubo_motion_ros::ExecuteGripperGoal goal;
    goal.grip.push_back(hubo_motion_ros::ExecuteGripperGoal::PTA_GRASP_NOW);
    goal.ArmIndex.push_back(hubo_motion_ros::ExecuteGripperGoal::PTA_RIGHT);

    actionlib::SimpleActionClient<hubo_motion_ros::ExecuteGripperAction> ac("/hubo_trajectory_server_gripper", true);
    bool response = ac.waitForServer(ros::Duration(actionWait));

    ac.sendGoal(goal);

    if(response)
    {
        loosenRB->setStyleSheet("");
        openRB->setStyleSheet("");
        graspRB->setStyleSheet(selectedStyle);
    }


//    if(active)
//    {
//        param.rightFin = T_GRASP;
//        ach_put(&teleopParamChan, &param, sizeof(param));
//    }
}

void HuboMotionPanel::openR()
{
    hubo_motion_ros::ExecuteGripperGoal goal;
    goal.grip.push_back(hubo_motion_ros::ExecuteGripperGoal::PTA_GRASP_RELEASE_NOW);
    goal.ArmIndex.push_back(hubo_motion_ros::ExecuteGripperGoal::PTA_RIGHT);

    actionlib::SimpleActionClient<hubo_motion_ros::ExecuteGripperAction> ac("/hubo_trajectory_server_gripper", true);
    bool response = ac.waitForServer(ros::Duration(actionWait));

    ac.sendGoal(goal);

    if(response)
    {
        loosenRB->setStyleSheet("");
        openRB->setStyleSheet(selectedStyle);
        graspRB->setStyleSheet("");
    }


//    if(active)
//    {
//        param.rightFin = T_OPEN;
//        ach_put(&teleopParamChan, &param, sizeof(param));
//    }
}

void HuboMotionPanel::loosenR()
{
    hubo_motion_ros::ExecuteGripperGoal goal;
    goal.grip.push_back(hubo_motion_ros::ExecuteGripperGoal::PTA_GRASP_LIMP);
    goal.ArmIndex.push_back(hubo_motion_ros::ExecuteGripperGoal::PTA_RIGHT);

    actionlib::SimpleActionClient<hubo_motion_ros::ExecuteGripperAction> ac("/hubo_trajectory_server_gripper", true);
    bool response = ac.waitForServer(ros::Duration(actionWait));

    ac.sendGoal(goal);

    if(response)
    {
        loosenRB->setStyleSheet(selectedStyle);
        openRB->setStyleSheet("");
        graspRB->setStyleSheet("");
    }


//    if(active)
//    {
//        param.rightFin = T_LOOSEN;
//        ach_put(&teleopParamChan, &param, sizeof(param));
//    }
}



void HuboMotionPanel::graspT()
{
    hubo_motion_ros::ExecuteGripperGoal goal;
    goal.grip.push_back(hubo_motion_ros::ExecuteGripperGoal::PTA_GRASP_NOW);
    goal.ArmIndex.push_back(hubo_motion_ros::ExecuteGripperGoal::PTA_TRIG);


    actionlib::SimpleActionClient<hubo_motion_ros::ExecuteGripperAction> ac("/hubo_trajectory_server_gripper", true);
    bool response = ac.waitForServer(ros::Duration(actionWait));

    ac.sendGoal(goal);

    if(response)
    {
        loosenTB->setStyleSheet("");
        openTB->setStyleSheet("");
        graspTB->setStyleSheet(selectedStyle);
    }


//    if(active)
//    {
//        param.trigFin = T_GRASP;
//        ach_put(&teleopParamChan, &param, sizeof(param));
//    }
}

void HuboMotionPanel::openT()
{
    hubo_motion_ros::ExecuteGripperGoal goal;
    goal.grip.push_back(hubo_motion_ros::ExecuteGripperGoal::PTA_GRASP_RELEASE_NOW);
    goal.ArmIndex.push_back(hubo_motion_ros::ExecuteGripperGoal::PTA_TRIG);

    actionlib::SimpleActionClient<hubo_motion_ros::ExecuteGripperAction> ac("/hubo_trajectory_server_gripper", true);
    bool response = ac.waitForServer(ros::Duration(actionWait));

    ac.sendGoal(goal);

    if(response)
    {
        loosenTB->setStyleSheet("");
        openTB->setStyleSheet(selectedStyle);
        graspTB->setStyleSheet("");
    }


//    if(active)
//    {
//        param.trigFin = T_OPEN;
//        ach_put(&teleopParamChan, &param, sizeof(param));
//    }
}

void HuboMotionPanel::loosenT()
{
    hubo_motion_ros::ExecuteGripperGoal goal;
    goal.grip.push_back(hubo_motion_ros::ExecuteGripperGoal::PTA_GRASP_LIMP);
    goal.ArmIndex.push_back(hubo_motion_ros::ExecuteGripperGoal::PTA_TRIG);

    actionlib::SimpleActionClient<hubo_motion_ros::ExecuteGripperAction> ac("/hubo_trajectory_server_gripper", true);
    bool response = ac.waitForServer(ros::Duration(actionWait));

    ac.sendGoal(goal);


    if(response)
    {
        loosenTB->setStyleSheet(selectedStyle);
        openTB->setStyleSheet("");
        graspTB->setStyleSheet("");
    }


//    control_msgs::GripperCommandGoal goal;
//    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> ac("/hubo_trajectory_server_gripper", true);

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
