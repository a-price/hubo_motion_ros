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

#include "hubo_motion_ros/AchNetworkWidget.h"

namespace hubo_motion_ros
{

AchNetworkWidget::AchNetworkWidget(QWidget *parent)
    : QWidget(parent)
{
    groupStyleSheet = "QGroupBox {"
                      "border: 1px solid gray;"
                      "border-radius: 9px;"
                      "margin-top: 0.5em;"
                      "}"
                      "QGroupBox::title {"
                      "subcontrol-origin: margin;"
                      "left: 10px;"
                      "padding: 0 3px 0 3px;"
                      "}";

    // Set up the networking box
    QVBoxLayout* achdLayout = new QVBoxLayout;


    achdConnect = new QPushButton;
    achdConnect->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    achdConnect->setText("Connect");
    achdConnect->setToolTip("Connect to Hubo's on board computer");
    achdLayout->addWidget(achdConnect, 0, Qt::AlignCenter);
    connect(achdConnect, SIGNAL(clicked()), this, SLOT(achdConnectSlot()));


    achdDisconnect = new QPushButton;
    achdDisconnect->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    achdDisconnect->setText("Disconnect");
    achdDisconnect->setToolTip("Disconnect from Hubo's on board computer");
    achdLayout->addWidget(achdDisconnect, 0, Qt::AlignCenter);
    connect(achdDisconnect, SIGNAL(clicked()), this, SLOT(achdDisconnectSlot()));


    QHBoxLayout* statusLayout = new QHBoxLayout;
    QLabel* staticLabel = new QLabel;
    staticLabel->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    staticLabel->setText("Status: ");
    statusLayout->addWidget(staticLabel);
    statusLabel = new QLabel;
    statusLabel->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    statusLabel->setText("Not Connected");
    statusLayout->addWidget(statusLabel);
    achdLayout->addLayout(statusLayout);

    QHBoxLayout* networkLayout = new QHBoxLayout;
    networkLayout->addLayout(achdLayout);


    QHBoxLayout* ipLayout = new QHBoxLayout;
    ipLayout->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    ipAddrAEdit = new QLineEdit;
    ipAddrAEdit->setMaxLength(3);
    ipAddrAEdit->setMaximumWidth(50);
    ipAddrAEdit->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    ipAddrAEdit->setToolTip("IP Address for Hubo's on board computer");
    ipLayout->addWidget(ipAddrAEdit);
    connect(ipAddrAEdit, SIGNAL(textEdited(QString)), this, SLOT(ipEditHandle(QString)));
    QLabel* dot1 = new QLabel;
    dot1->setText(".");
    ipLayout->addWidget(dot1);
    ipAddrBEdit = new QLineEdit;
    ipAddrBEdit->setMaxLength(3);
    ipAddrBEdit->setMaximumWidth(50);
    ipAddrBEdit->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    ipAddrBEdit->setToolTip("IP Address for Hubo's on board computer");
    ipLayout->addWidget(ipAddrBEdit);
    connect(ipAddrBEdit, SIGNAL(textEdited(QString)), this, SLOT(ipEditHandle(QString)));
    QLabel* dot2 = new QLabel;
    dot2->setText(".");
    ipLayout->addWidget(dot2);
    ipAddrCEdit = new QLineEdit;
    ipAddrCEdit->setMaxLength(3);
    ipAddrCEdit->setMaximumWidth(50);
    ipAddrCEdit->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    ipAddrCEdit->setToolTip("IP Address for Hubo's on board computer");
    ipLayout->addWidget(ipAddrCEdit);
    connect(ipAddrCEdit, SIGNAL(textEdited(QString)), this, SLOT(ipEditHandle(QString)));
    QLabel* dot3 = new QLabel;
    dot3->setText(".");
    ipLayout->addWidget(dot3);
    ipAddrDEdit = new QLineEdit;
    ipAddrDEdit->setMaxLength(3);
    ipAddrDEdit->setMaximumWidth(50);
    ipAddrDEdit->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    ipAddrDEdit->setToolTip("IP Address for Hubo's on board computer");
    ipLayout->addWidget(ipAddrDEdit);
    connect(ipAddrDEdit, SIGNAL(textEdited(QString)), this, SLOT(ipEditHandle(QString)));


    QLabel* ipTitle = new QLabel;
    ipTitle->setText("IP Address");
    ipTitle->setToolTip("IP Address for Hubo's on board computer");

    QVBoxLayout* ipUpperLayout = new QVBoxLayout;
    ipUpperLayout->addWidget(ipTitle, 0, Qt::AlignLeft | Qt::AlignBottom);
    ipUpperLayout->addLayout(ipLayout);

    networkLayout->addLayout(ipUpperLayout);



    networkBox = new QGroupBox;
    networkBox->setStyleSheet(groupStyleSheet);
    networkBox->setTitle("Ach Networking");
    networkBox->setLayout(networkLayout);

    QHBoxLayout* bullshitLayout = new QHBoxLayout;
    bullshitLayout->addWidget(networkBox);
    this->setLayout(bullshitLayout);

    setIPAddress(192, 168, 1, 0);

}

AchNetworkWidget::~AchNetworkWidget()
{
    for(int i=0; i<achds.size(); i++)
    {
        achds[i]->kill();
        ach_close(&channels[i]);
    }
}


void AchNetworkWidget::setIPAddress(int a, int b, int c, int d)
{
    ipAddrA = a;
    ipAddrB = b;
    ipAddrC = c;
    ipAddrD = d;

    ipAddrAEdit->setText(QString::number(ipAddrA));
    ipAddrBEdit->setText(QString::number(ipAddrB));
    ipAddrCEdit->setText(QString::number(ipAddrC));
    ipAddrDEdit->setText(QString::number(ipAddrD));
}

int AchNetworkWidget::getIPAddress(int index)
{
    switch(index)
    {
    case 0:
        return ipAddrA;
    case 1:
        return ipAddrB;
    case 2:
        return ipAddrC;
    case 3:
        return ipAddrD;
    default:
    	return 0;
    }
}

void AchNetworkWidget::addChannel(QString name, achd_create_t mode,
                                  int message_count, int nominal_size)
{
    channelMaker.start("ach mk " + name + " -1 "
                          + "-m " + QString::number(message_count)
                          + " -n " + QString::number(nominal_size)
                          + " -o 666", QIODevice::ReadWrite);

    channelMaker.waitForFinished(5000);

    ach_channel_t tempChan;
    ach_status_t r = ach_open(&tempChan, name.toLocal8Bit(), NULL);
    if( r != ACH_OK )
        std::cerr << ach_result_to_string(r) << std::endl;
    else
    {
        channelNames.push_back(name);
        channels.push_back(tempChan);
        modes.push_back(mode);
    }

    QProcess* tempProc = new QProcess;
    achds.push_back(tempProc);
}


void AchNetworkWidget::achdConnectSlot()
{
    statusLabel->setText("Connected");
    for(int i=0; i<channels.size(); i++)
    {
        QString startType;
        if( modes[i] == ACHD_PUSH )
            startType = "achd push ";
        else if( modes[i] == ACHD_PULL )
            startType = "achd pull ";
        else
            continue;

        achds[i]->start( startType + QString::number(ipAddrA)
                        + "." + QString::number(ipAddrB)
                        + "." + QString::number(ipAddrC)
                        + "." + QString::number(ipAddrD)
                                + " " + channelNames[i] );

        connect(achds[i], SIGNAL(finished(int)), this, SLOT(handleDead(int)));

    }
}

void AchNetworkWidget::handleDead(int exitStatus)
{
    statusLabel->setText("Disconnected");
}

void AchNetworkWidget::achdDisconnectSlot()
{
    for(int i=0; i<achds.size(); i++)
        achds[i]->kill();
}

void AchNetworkWidget::setNetworkName(QString name)
{
    networkBox->setTitle("Ach Networking - " + name);
}

void AchNetworkWidget::saveIP(rviz::Config &config)
{
    rviz::Config ip_config = config.mapMakeChild("HuboIP");

    QVariant a = QVariant(getIPAddress(0));
    QVariant b = QVariant(getIPAddress(1));
    QVariant c = QVariant(getIPAddress(2));
    QVariant d = QVariant(getIPAddress(3));

    ip_config.mapSetValue("ipAddrA", a);
    ip_config.mapSetValue("ipAddrB", b);
    ip_config.mapSetValue("ipAddrC", c);
    ip_config.mapSetValue("ipAddrD", d);
}

void AchNetworkWidget::loadIP(const rviz::Config &config)
{
    rviz::Config ip_config = config.mapGetChild("HuboIP");
    QVariant a, b, c, d;
    if( !ip_config.mapGetValue("ipAddrA", &a) || !ip_config.mapGetValue("ipAddrB", &b)
     || !ip_config.mapGetValue("ipAddrC", &c) || !ip_config.mapGetValue("ipAddrD", &d))
        ROS_INFO("Loading the IP Address Failed");
    else
        setIPAddress(a.toInt(), b.toInt(), c.toInt(), d.toInt());
}

void AchNetworkWidget::ipEditHandle(const QString &text)
{
    ipAddrA = ipAddrAEdit->text().toInt();
    ipAddrB = ipAddrBEdit->text().toInt();
    ipAddrC = ipAddrCEdit->text().toInt();
    ipAddrD = ipAddrDEdit->text().toInt();
}


}



#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( hubo_motion_ros::AchNetworkWidget, QWidget )
