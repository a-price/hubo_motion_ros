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

#include <vector>

#include <rviz/panel.h>

#include <hubo.h>
#include <hubo-jointparams.h>

#include "AchNetworkWidget.h"
#ifndef _SYS_SYSLOG_H
#define _SYS_SYSLOG_H
#endif //_SYS_SYSLOG_H
#include <manip.h>


namespace hubo_motion_ros
{


class HuboMotionPanel : public rviz::Panel
{
    Q_OBJECT
public:
    HuboMotionPanel(QWidget *parent = 0);
    ~HuboMotionPanel();


    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;

private:
    AchNetworkWidget* achManager;

};


}



#endif // HUBO_MOTION_PANEL_H
