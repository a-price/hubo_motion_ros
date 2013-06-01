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


namespace hubo_motion_ros
{

class HuboMotionPanel;

class LibertyRelay : public QThread
{
Q_OBJECT
public:

    ach_channel_t libertyChan;
    ach_channel_t manipCmdChan;
    ach_channel_t manipStateChan;
    bool alive;
    double updateFreq;
    void openChannels();

    hubo_manip_cmd_t cmd;

protected:
    virtual void run();

public Q_SLOTS:
    void haltOperation();
    void getUpdateFrequency(double value);
    void graspL();
    void graspR();
    void openL();
    void openR();
    void loosenL();
    void loosenR();

signals:
    void refreshData(hubo_manip_cmd_t lib);
    void libertyQuitting();

};

class HuboMotionPanel : public rviz::Panel
{
    Q_OBJECT
public:
    HuboMotionPanel(QWidget *parent = 0);
    ~HuboMotionPanel();

    LibertyRelay libertyThread;

    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;

private:
    AchNetworkWidget* achManager;
    QCheckBox* libertyCheck;
    QDoubleSpinBox* libertyFreq;
    QVector< QVector<QLineEdit*> > datas;
    QPushButton* graspLB;
    QPushButton* graspRB;
    QPushButton* openLB;
    QPushButton* openRB;
    QPushButton* loosenLB;
    QPushButton* loosenRB;

signals:
    void stopLiberty();

protected Q_SLOTS:

    void handleCheckToggle(bool active);
    void getRefreshData(hubo_manip_cmd_t cmd);
    void handleLibQuit();

};


}



#endif // HUBO_MOTION_PANEL_H
