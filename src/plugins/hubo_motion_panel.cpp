
#include "hubo_motion_ros/hubo_motion_panel.h"

namespace hubo_motion_ros
{

HuboMotionPanel::HuboMotionPanel(QWidget *parent)
    : rviz::Panel(parent)
{

    achManager = new AchNetworkWidget;
    achManager->setNetworkName("Manipulation");

    achManager->addChannel("liberty");

    achManager->addChannel(QString::fromLocal8Bit(CHAN_HUBO_MANIP_CMD),
                           AchNetworkWidget::ACHD_PUSH);

    achManager->addChannel(QString::fromLocal8Bit(CHAN_HUBO_MANIP_STATE),
                           AchNetworkWidget::ACHD_PULL);

    achManager->addChannel(QString::fromLocal8Bit(CHAN_HUBO_MANIP_PARAM),
                           AchNetworkWidget::ACHD_PUSH);

    achManager->addChannel(QString::fromLocal8Bit(CHAN_HUBO_MANIP_TRAJ),
                           AchNetworkWidget::ACHD_PUSH, 3, 1000000);

    QVBoxLayout* dumbLayout = new QVBoxLayout;
    dumbLayout->addWidget(achManager);


    QHBoxLayout* checkLayout = new QHBoxLayout;
    libertyCheck = new QCheckBox;
    libertyCheck->setText("Liberty");
    libertyCheck->setToolTip("Use the Liberty sensor system for teleoperation");
    connect(libertyCheck, SIGNAL(toggled(bool)), this, SLOT(handleLibCheckToggle(bool)));
    checkLayout->addWidget(libertyCheck, 0, Qt::AlignLeft);

    spacenavCheck = new QCheckBox;
    spacenavCheck->setText("Spacenav");
    spacenavCheck->setToolTip("Use a 6-axis spacenav joystick for teleoperation");
    connect(spacenavCheck, SIGNAL(toggled(bool)), this, SLOT(handleNavCheckToggle(bool)));
    checkLayout->addWidget(spacenavCheck, 0, Qt::AlignLeft);

    QLabel* freqLab = new QLabel;
    freqLab->setText("Display Frequency:");
    freqLab->setToolTip("Rate (Hz) at which to display data from Liberty");
    checkLayout->addWidget(freqLab, 0, Qt::AlignRight);

    libertyFreq = new QDoubleSpinBox;
    libertyFreq->setToolTip(freqLab->toolTip());
    libertyFreq->setMinimum(0);
    connect(libertyFreq, SIGNAL(valueChanged(double)), &libertyThread, SLOT(getUpdateFrequency(double)));
    connect(libertyFreq, SIGNAL(valueChanged(double)), &spacenavThread, SLOT(getUpdateFrequency(double)));
    libertyFreq->setValue(5);

    checkLayout->addWidget(libertyFreq, 0, Qt::AlignLeft);
    dumbLayout->addLayout(checkLayout);

    connect(this, SIGNAL(stopLiberty()), &libertyThread, SLOT(haltOperation()));
    connect(&libertyThread, SIGNAL(refreshData(double,int,int)), this, SLOT(getRefreshData(double,int,int)));
    connect(&libertyThread, SIGNAL(libertyQuitting()), this, SLOT(handleLibQuit()));

    connect(this, SIGNAL(stopSpacenav()), &spacenavThread, SLOT(haltOperation()));
    connect(&spacenavThread, SIGNAL(refreshData(double,int,int)), this, SLOT(getRefreshData(double,int,int)));
    connect(&spacenavThread, SIGNAL(spacenavQuitting()), this, SLOT(handleNavQuit()));


    QGridLayout* grid = new QGridLayout;
    datas.resize(2);
    for(int i=0; i<2; i++)
    {
        datas[i].resize(7);
        for(int j=0; j<7; j++)
        {
            datas[i][j] = new QLineEdit;
            datas[i][j]->setReadOnly(true);
            grid->addWidget(datas[i][j], i, j, 1, 1);
        }
    }

    QHBoxLayout* rightGraspLay = new QHBoxLayout;
    graspRB = new QPushButton;
    graspRB->setText("Grasp R");
    rightGraspLay->addWidget(graspRB);
    connect(graspRB, SIGNAL(clicked()), &libertyThread, SLOT(graspR()));
    connect(graspRB, SIGNAL(clicked()), &spacenavThread, SLOT(graspR()));
    openRB = new QPushButton;
    openRB->setText("Open R");
    rightGraspLay->addWidget(openRB);
    connect(openRB, SIGNAL(clicked()), &libertyThread, SLOT(openR()));
    connect(openRB, SIGNAL(clicked()), &spacenavThread, SLOT(openR()));
    loosenRB = new QPushButton;
    loosenRB->setText("Loosen R");
    rightGraspLay->addWidget(loosenRB);
    connect(loosenRB, SIGNAL(clicked()), &libertyThread, SLOT(loosenR()));
    connect(loosenRB, SIGNAL(clicked()), &spacenavThread, SLOT(loosenR()));

    dumbLayout->addLayout(rightGraspLay);

    QHBoxLayout* leftGraspLay = new QHBoxLayout;
    graspLB = new QPushButton;
    graspLB->setText("Grasp L");
    leftGraspLay->addWidget(graspLB);
    connect(graspLB, SIGNAL(clicked()), &libertyThread, SLOT(graspL()));
    connect(graspLB, SIGNAL(clicked()), &spacenavThread, SLOT(graspL()));
    openLB = new QPushButton;
    openLB->setText("Open L");
    leftGraspLay->addWidget(openLB);
    connect(openLB, SIGNAL(clicked()), &libertyThread, SLOT(openL()));
    connect(openLB, SIGNAL(clicked()), &spacenavThread, SLOT(openL()));
    loosenLB = new QPushButton;
    loosenLB->setText("Loosen L");
    leftGraspLay->addWidget(loosenLB);
    connect(loosenLB, SIGNAL(clicked()), &libertyThread, SLOT(loosenL()));
    connect(loosenLB, SIGNAL(clicked()), &spacenavThread, SLOT(loosenL()));

    dumbLayout->addLayout(leftGraspLay);

    libertyThread.openChannels();
    libertyThread.openLiberty();
    spacenavThread.openChannels();

    dumbLayout->addLayout(grid);

    waistSlide = new QSlider(Qt::Horizontal);
    waistSlide->setMaximum(160);
    waistSlide->setMinimum(-160);
    waistSlide->setValue(0);
    waistSlide->setToolTip("Waist Angle Value");
    connect(waistSlide, SIGNAL(valueChanged(int)), &libertyThread, SLOT(getWaistValue(int)));
    connect(waistSlide, SIGNAL(valueChanged(int)), &spacenavThread, SLOT(getWaistValue(int)));
    dumbLayout->addWidget(waistSlide);

    setLayout(dumbLayout);




}

HuboMotionPanel::~HuboMotionPanel()
{
    achManager->achdDisconnectSlot();
}

void HuboMotionPanel::save(rviz::Config config) const
{
    rviz::Panel::save(config);
    achManager->saveIP(config);

}

void HuboMotionPanel::load(const rviz::Config &config)
{
    rviz::Panel::load(config);
    achManager->loadIP(config);
}


void SpacenavRelay::run()
{
    alive = false;



    hubo_manip_state_t state;
    memset(&state, 0, sizeof(state));

    size_t fs;
    ach_status_t manR = ach_get( &manipStateChan, &state, sizeof(state), &fs, NULL, ACH_O_LAST );



    if(ACH_OK==manR || ACH_MISSED_FRAME==manR || ACH_STALE_FRAMES==manR)
        alive = true;
    else // FIXME: Remove this
        alive = true;

    if(alive)
    {
        ros::Subscriber spacenavJoy = nh.subscribe("spacenav/joy", 1,
                            &hubo_motion_ros::SpacenavRelay::spacenav_callback, this);

        refClock.start();

        for(int i=0; i<2; i++)
            for(int j=0; j<3; j++)
                pos[i][j] = state.pose[i].data[j];

        if(ACH_OK==manR || ACH_MISSED_FRAME==manR || ACH_STALE_FRAMES==manR) // FIXME: Remove this condition
        {
            std::cout << "Initializing euler angles\t" << ach_result_to_string(manR) << std::endl;
            double q0 = /*state.pose[RIGHT].w;*/ 1;
            double q1 = state.pose[RIGHT].i;
            double q2 = state.pose[RIGHT].j;
            double q3 = state.pose[RIGHT].k;
            angles[RIGHT][0] = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
            angles[RIGHT][1] = asin(2*(q0*q2 - q3*q1));
            angles[RIGHT][2] = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
        }
        else
            angles[RIGHT] << 0, 0, 0;

        sn_freq = 10;

        ros::Rate r(sn_freq);

        rotationOn = false;
        zOn = true;

        while(alive)
        {
            ros::spinOnce();
            r.sleep();
        }

        cmd.m_mode[0] = MC_HALT;
        cmd.m_mode[1] = MC_HALT;
        cmd.interrupt[0] = true;
        cmd.interrupt[1] = true;

        cmd.waistAngle = waistAngle;
        ach_put( &manipCmdChan, &cmd, sizeof(cmd) );

        spacenavJoy.shutdown();
    }

    emit spacenavQuitting();
}

void SpacenavRelay::spacenav_callback(const sensor_msgs::JoyConstPtr joystick)
{
    if(rotationOn && joystick->buttons[0] == 1 && !rotRegistered)
    {
        rotationOn = false;
        rotRegistered = true;
    }
    else if(!rotationOn && joystick->buttons[0] == 1 && !rotRegistered)
    {
        rotationOn = true;
        rotRegistered = true;
    }
    else if(joystick->buttons[0] == 0)
        rotRegistered = false;

    if(zOn && joystick->buttons[1] == 1 && !zRegistered)
    {
        zOn = false;
        zRegistered = true;
    }
    else if(!zOn && joystick->buttons[1] == 1 && !zRegistered)
    {
        zOn = true;
        zRegistered = true;
    }
    else if(joystick->buttons[1] == 0)
        zRegistered = false;


    // TODO: Distinguish left and right
    cmd.m_mode[0] = MC_TRANS_EULER;
    cmd.m_mode[1] = MC_TRANS_EULER;
    cmd.interrupt[0] = true;
    cmd.interrupt[1] = true;

    double posScale = 0.1/sn_freq;
    double rotScale = 0.1/sn_freq;

    if(!rotationOn)
    {
        int num;
        if(zOn)
            num = 3;
        else
            num = 2;

        for(int j=0; j<num; j++)
            pos[RIGHT][j] += joystick->axes[j]*posScale;

        for(int j=0; j<num; j++)
            cmd.pose[RIGHT].data[j] = pos[RIGHT][j];
    }

    if(rotationOn)
    {
        for(int j=0; j<3; j++)
            angles[RIGHT][j] += joystick->axes[j+3]*rotScale;

        cmd.pose[RIGHT].alpha = angles[RIGHT][0];
        cmd.pose[RIGHT].beta = angles[RIGHT][1];
        cmd.pose[RIGHT].gamma = angles[RIGHT][2];
    }

    double elapsed = refClock.elapsed()/1000.0;
    if( updateFreq > 0 )
    {
        if( elapsed > 1.0/updateFreq )
        {
            for(int j=0; j<7; j++)
                emit refreshData(cmd.pose[RIGHT].data[j], RIGHT, j);
            refClock.restart();
        }
    }
    cmd.waistAngle = waistAngle;
    ach_put( &manipCmdChan, &cmd, sizeof(cmd) );
}

void LibertyRelay::run()
{

    alive = false;

    liberty_data_t lib, lib0;
    hubo_manip_state_t state;
    memset(&lib, 0, sizeof(lib));
    memset(&lib0, 0, sizeof(lib0));
    memset(&cmd, 0, sizeof(cmd));
    QTime refClock; refClock.start();
    size_t fs;

    ach_status_t libR = ach_get( &libertyChan, &lib0, sizeof(lib0), &fs, NULL, ACH_O_WAIT );
    ach_status_t manR = ach_get( &manipStateChan, &state, sizeof(state), &fs, NULL, ACH_O_WAIT );

    std::cout << "lib:" << ach_result_to_string(libR) << "\tman:" << ach_result_to_string(manR) << std::endl;

    if( (ACH_OK==libR || ACH_MISSED_FRAME==libR || ACH_STALE_FRAMES==libR)
     && (ACH_OK==manR || ACH_MISSED_FRAME==manR || ACH_STALE_FRAMES==manR) )
        alive = true;


    while(alive)
    {
        ach_get( &libertyChan, &lib, sizeof(lib), &fs, NULL, ACH_O_WAIT );
        cmd.m_mode[0] = MC_TRANS_QUAT;
        cmd.m_mode[1] = MC_TRANS_QUAT;
        cmd.interrupt[0] = true;
        cmd.interrupt[1] = true;

        for(int i=0; i<2; i++)
            for(int j=0; j<3; j++)
                cmd.pose[i].data[j] = lib.sensor[i][j] - lib0.sensor[i][j] + state.pose[i].data[j];

        for(int i=0; i<2; i++)
            for(int j=3; j<7; j++)
                cmd.pose[i].data[j] = lib.sensor[i][j];


        double elapsed = refClock.elapsed()/1000.0;
        if( updateFreq > 0 )
        {
            if( elapsed > 1.0/updateFreq )
            {
                for(int i=0; i<2; i++)
                    for(int j=0; j<7; j++)
                        emit refreshData(cmd.pose[i].data[j], i, j);
                refClock.restart();
            }
        }
        cmd.waistAngle = waistAngle;
        ach_put( &manipCmdChan, &cmd, sizeof(cmd) );
    }

    cmd.m_mode[0] = MC_HALT;
    cmd.m_mode[1] = MC_HALT;
    cmd.interrupt[0] = true;
    cmd.interrupt[1] = true;

    cmd.waistAngle = waistAngle;
    ach_put( &manipCmdChan, &cmd, sizeof(cmd) );


    emit libertyQuitting();

}

void ManipRelay::openChannels()
{
    ach_status_t r = ach_open(&manipCmdChan, CHAN_HUBO_MANIP_CMD, NULL);
    if( ACH_OK != r )
        std::cout << "Command Channel Error: " << ach_result_to_string(r) << std::endl;

    r = ach_open(&manipStateChan, CHAN_HUBO_MANIP_STATE, NULL);
    if( ACH_OK != r )
        std::cout << "Command Channel Error: " << ach_result_to_string(r) << std::endl;
}

void LibertyRelay::openLiberty()
{
    ach_status_t r = ach_open(&libertyChan, "liberty", NULL);
    if( ACH_OK != r )
        std::cout << "Liberty Channel Error: " << ach_result_to_string(r) << std::endl;

}


}



#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( hubo_motion_ros::HuboMotionPanel, rviz::Panel )
