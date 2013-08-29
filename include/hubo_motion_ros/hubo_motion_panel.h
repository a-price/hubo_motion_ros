#ifndef HUBO_MOTION_PANEL_H
#define HUBO_MOTION_PANEL_H

#include <stdio.h>

#include <QApplication>
#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <ros/ros.h>
#include <QTableWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QRadioButton>
#include <QSpinBox>
#include <QLabel>
#include <QProcess>
#include <QGroupBox>
#include <QButtonGroup>
#include <QProcess>
#include <QString>
#include <QStringList>
#include <QTextStream>
#include <QClipboard>
#include <QPalette>
#include <QColor>
#include <QThread>
#include <QCheckBox>
#include <QTime>

#include <vector>

#include <rviz/panel.h>

#include <hubo.h>
#include <hubo-jointparams.h>

#include "AchNetworkWidget.h"
#ifndef _SYS_SYSLOG_H
#define _SYS_SYSLOG_H
#endif //_SYS_SYSLOG_H
#include <manip.h>
#include <liberty.h>

#include <sensor_msgs/Joy.h>

#include <Eigen/Geometry>


#include "DummyParams.h"

namespace hubo_motion_ros
{

class HuboMotionPanel;

class ManipRelay : public QThread
{
Q_OBJECT
public:
    ach_channel_t manipCmdChan;
    ach_channel_t manipStateChan;
    bool alive;
    double updateFreq;
    void openChannels();
    double waistAngle;

    hubo_manip_cmd_t cmd;

public Q_SLOTS:
    void haltOperation();
    void getUpdateFrequency(double value);
    void graspL();
    void graspR();
    void openL();
    void openR();
    void loosenL();
    void loosenR();

    void getWaistValue(int val);

signals:
    void refreshData(double data, int i, int j);

};

class LibertyRelay : public ManipRelay
{
Q_OBJECT
public:

    ach_channel_t libertyChan;
    void openLiberty();

protected:
    virtual void run();


signals:
    void libertyQuitting();

};

class SpacenavRelay : public ManipRelay
{
Q_OBJECT
public:
    void spacenav_callback(const sensor_msgs::JoyConstPtr joystick);
    ros::NodeHandle nh;
    Eigen::Vector3d pos[2];
    Eigen::Vector3d angles[2];

    bool rotationOn;
    bool rotRegistered;

    bool zOn;
    bool zRegistered;

    double sn_freq;

    int side;
    int queueSide;
    bool restart;

    QTime refClock;

public Q_SLOTS:
    void switchLeft(bool active);
    void switchRight(bool active);
    void switchBoth(bool active);

protected:
    virtual void run();

signals:
    void spacenavQuitting();

};

class HuboMotionPanel : public rviz::Panel
{
    Q_OBJECT
public:
    HuboMotionPanel(QWidget *parent = 0);
    ~HuboMotionPanel();
    
    
    ach_channel_t teleopParamChan;

    LibertyRelay libertyThread;
    SpacenavRelay spacenavThread;

    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;



    teleop_params_t param;

private:
    AchNetworkWidget* achManager;
    QCheckBox* libertyCheck;
    QCheckBox* spacenavCheck;
    QDoubleSpinBox* libertyFreq;
    QVector< QVector<QLineEdit*> > datas;
    QButtonGroup* graspSelL;
    QRadioButton* graspLB;
    QRadioButton* openLB;
    QRadioButton* loosenLB;

    QButtonGroup* graspSelR;
    QRadioButton* graspRB;
    QRadioButton* openRB;
    QRadioButton* loosenRB;

    QButtonGroup* graspSelT;
    QRadioButton* graspTB;
    QRadioButton* openTB;
    QRadioButton* loosenTB;



    QButtonGroup* sideSel;
    QRadioButton* leftSel;
    QRadioButton* rightSel;
    QRadioButton* bothSel;

    QSlider* waistSlide;
    double waistScale;

signals:
    void stopLiberty();
    void stopSpacenav();

protected Q_SLOTS:


    void handleLibCheckToggle(bool active);
    void handleNavCheckToggle(bool active);
    void getRefreshData(double data, int i, int j);
    void handleLibQuit();
    void handleNavQuit();
    
    void graspR(bool active);
    void graspL(bool active);
    void graspT(bool active);

    void openR(bool active);
    void openL(bool active);
    void openT(bool active);

    void loosenR(bool active);
    void loosenL(bool active);
    void loosenT(bool active);

    
    void switchLeft(bool active);
    void switchRight(bool active);
    void switchBoth(bool active);

};


}



#endif // HUBO_MOTION_PANEL_H
