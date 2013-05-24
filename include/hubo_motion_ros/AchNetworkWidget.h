/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Michael X. Grey <mxgrey@gatech.edu>
 * Date: May 23, 2013
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef ACHNETWORKWIDGET_H
#define ACHNETWORKWIDGET_H



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
#include <QVector>


#include <QProcess>


#include <vector>

#include <rviz/panel.h>


#include <stdint.h>
#include <time.h>
#include <string.h>
#include <pthread.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <ach.h>



namespace hubo_motion_ros
{

class AchNetworkWidget : public QWidget
{
    Q_OBJECT
public:

    typedef enum {
        ACHD_CREATE_ONLY,
        ACHD_PUSH,
        ACHD_PULL
    } achd_create_t;

    AchNetworkWidget( QWidget* parent = 0 );
    ~AchNetworkWidget();


    void setNetworkName(QString name);

    QVector<QProcess*> achds;
    QVector<ach_channel_t> channels;
    QVector<QString> channelNames;
    QVector<achd_create_t> modes;

    QProcess channelMaker;

    void addChannel(QString name, achd_create_t mode=ACHD_CREATE_ONLY, int message_count=10, int nominal_size=3000);
    void setIPAddress(int a, int b, int c, int d);
    int getIPAddress(int index);

    QString groupStyleSheet;

    void loadIP( const rviz::Config& config );
    void saveIP( rviz::Config &config );

protected:


    int ipAddrA;
    int ipAddrB;
    int ipAddrC;
    int ipAddrD;

protected Q_SLOTS:


    void achdConnectSlot();
    void achdDisconnectSlot();
    void handleDead(int exitStatus);
    void ipEditHandle(const QString &text);

private:

    QPushButton* achdConnect;
    QPushButton* achdDisconnect;
    QLabel* statusLabel;
    QLineEdit* ipAddrAEdit;
    QLineEdit* ipAddrBEdit;
    QLineEdit* ipAddrCEdit;
    QLineEdit* ipAddrDEdit;

    QGroupBox* networkBox;


};



}









#endif // ACHNETWORKWIDGET_H
