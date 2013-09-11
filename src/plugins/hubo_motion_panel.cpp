
#include "hubo_motion_ros/hubo_motion_panel.h"
#include "hubo_motion_ros/TeleopCmd.h"

namespace hubo_motion_ros
{

HuboMotionPanel::HuboMotionPanel(QWidget *parent)
    : rviz::Panel(parent)
{
//    int argc=0;
//    char** argv;
//    ros::init(argc, argv, "hubo_motion_panel");

    actionWait = 3.0;
    
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
    
    std::cout << "HMP -- Advertising topics" << std::endl;

    cmdPublisher = nh.advertise<hubo_motion_ros::TeleopCmd>("teleop_cmd_req", 1);
    nudgePublisher = nh.advertise<hubo_motion_ros::TeleopPoseNudge>("teleop_nudge_req", 1);

    achManager = new AchNetworkWidget;
    achManager->setNetworkName("Manipulation");

    std::cout << "HMP -- Opening Ach channels" << std::endl;

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
    
    ach_open(&teleopParamChan, "teleop-param", NULL);

    QColor color(100, 230, 100);
    selectedStyle = "background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #dadbde, stop: 1 "
            + color.name() + ")";
    
    QColor sideColor(100, 100, 230);
    sideSelectedStyle = "background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #dadbde, stop: 1 "
            + sideColor.name() + ")";

    QHBoxLayout* checkLayout = new QHBoxLayout;
//    libertyCheck = new QCheckBox;
//    libertyCheck->setText("Liberty");
//    libertyCheck->setToolTip("Use the Liberty sensor system for teleoperation");
//    connect(libertyCheck, SIGNAL(toggled(bool)), this, SLOT(handleLibCheckToggle(bool)));
//    checkLayout->addWidget(libertyCheck, 0, Qt::AlignLeft);
//    libertyCheck->setEnabled(false);

//    spacenavCheck = new QCheckBox;
//    spacenavCheck->setText("Spacenav");
//    spacenavCheck->setToolTip("Use a 6-axis spacenav joystick for teleoperation");
//    connect(spacenavCheck, SIGNAL(toggled(bool)), this, SLOT(handleNavCheckToggle(bool)));
//    checkLayout->addWidget(spacenavCheck, 0, Qt::AlignLeft);
//    spacenavCheck->setEnabled(false);

    std::cout << "HMP -- Selection Buttons" << std::endl;

    sideSel = new QButtonGroup;
    sideSel->setExclusive(true);
    QVBoxLayout* selLayout = new QVBoxLayout;
    leftSel = new QPushButton;
    leftSel->setText("Left");
    leftSel->setToolTip("Control left arm with SpaceNav");
//    leftSel->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
//    connect(leftSel, SIGNAL(clicked()), &spacenavThread, SLOT(switchLeft(bool)));
    connect(leftSel, SIGNAL(clicked()), this, SLOT(switchLeft()));
    selLayout->addWidget(leftSel);
    rightSel = new QPushButton;
    rightSel->setText("Right");
    rightSel->setToolTip("Control right arm with SpaceNav");
//    rightSel->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
//    connect(rightSel, SIGNAL(clicked()), &spacenavThread, SLOT(switchRight(bool)));
    connect(rightSel, SIGNAL(clicked()), this, SLOT(switchRight()));
    selLayout->addWidget(rightSel);
    bothSel = new QPushButton;
    bothSel->setText("Both");
    bothSel->setToolTip("Control both arms simultaneously");
//    bothSel->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
//    connect(bothSel, SIGNAL(clicked()), &spacenavThread, SLOT(switchBoth(bool)));
    connect(bothSel, SIGNAL(clicked()), this, SLOT(switchBoth()));
    selLayout->addWidget(bothSel);
//    bothSel->setEnabled(false);

    checkLayout->addLayout(selLayout);

//    rightSel->setChecked(true);
    
    std::cout << "HMP -- Command Buttons" << std::endl;
    
    QVBoxLayout* cmdLayout = new QVBoxLayout;
    
    eeCmd = new QPushButton;
    eeCmd->setText("Pose Command");
    eeCmd->setToolTip("Instruct the physical Hubo to go\nto the displayed Pose in a straight line.");
    connect(eeCmd, SIGNAL(clicked()), this, SLOT(eeCmdSlot()));
    cmdLayout->addWidget(eeCmd);
    
    jsCmd = new QPushButton;
    jsCmd->setText("Joint Command");
    jsCmd->setToolTip("Instruct the physical Hubo to go\nto the displayed joint configuration.");
    connect(jsCmd, SIGNAL(clicked()), this, SLOT(jsCmdSlot()));
    cmdLayout->addWidget(jsCmd);
    
    resetCmd = new QPushButton;
    resetCmd->setText("Reset");
    resetCmd->setToolTip("Move the virtual Hubo's arms back to\nthe last commanded state.");
    connect(resetCmd, SIGNAL(clicked()), this, SLOT(resetCmdSlot()));
    cmdLayout->addWidget(resetCmd);
    
    zerosCmd = new QPushButton;
    zerosCmd->setText("Zero");
    zerosCmd->setToolTip("Move the virtual Hubo's arm joints back to all zeros");
    connect(zerosCmd, SIGNAL(clicked()), this, SLOT(zerosCmdSlot()));
    cmdLayout->addWidget(zerosCmd);
    
    checkLayout->addLayout(cmdLayout);


    QLabel* freqLab = new QLabel;
    freqLab->setText("Display Frequency:");
    freqLab->setToolTip("Rate (Hz) at which to display data from Liberty");
//    checkLayout->addWidget(freqLab, 0, Qt::AlignRight);

    libertyFreq = new QDoubleSpinBox;
    libertyFreq->setToolTip(freqLab->toolTip());
    libertyFreq->setMinimum(0);
    connect(libertyFreq, SIGNAL(valueChanged(double)), &libertyThread, SLOT(getUpdateFrequency(double)));
    connect(libertyFreq, SIGNAL(valueChanged(double)), &spacenavThread, SLOT(getUpdateFrequency(double)));
    libertyFreq->setValue(5);

//    checkLayout->addWidget(libertyFreq, 0, Qt::AlignLeft);
    dumbLayout->addLayout(checkLayout);

    connect(this, SIGNAL(stopLiberty()), &libertyThread, SLOT(haltOperation()));
    connect(&libertyThread, SIGNAL(refreshData(double,int,int)), this, SLOT(getRefreshData(double,int,int)));
    connect(&libertyThread, SIGNAL(libertyQuitting()), this, SLOT(handleLibQuit()));

    connect(this, SIGNAL(stopSpacenav()), &spacenavThread, SLOT(haltOperation()));
    connect(&spacenavThread, SIGNAL(refreshData(double,int,int)), this, SLOT(getRefreshData(double,int,int)));
    connect(&spacenavThread, SIGNAL(spacenavQuitting()), this, SLOT(handleNavQuit()));


//    QGridLayout* grid = new QGridLayout;
//    datas.resize(2);
//    for(int i=0; i<2; i++)
//    {
//        datas[i].resize(7);
//        for(int j=0; j<7; j++)
//        {
//            datas[i][j] = new QLineEdit;
//            datas[i][j]->setReadOnly(true);
//            grid->addWidget(datas[i][j], i, j, 1, 1);
//        }
//    }

    std::cout << "HMP -- Grasp Commands" << std::endl;
    
    QVBoxLayout* graspLayout = new QVBoxLayout;

    QHBoxLayout* leftGraspLay = new QHBoxLayout;
    graspSelL = new QButtonGroup;
    graspSelL->setExclusive(true);
    graspLB = new QPushButton;
    graspLB->setText("Grasp L");
    leftGraspLay->addWidget(graspLB);
    graspSelL->addButton(graspLB);
    connect(graspLB, SIGNAL(clicked()), this, SLOT(graspL()));
//    connect(graspLB, SIGNAL(clicked()), &libertyThread, SLOT(graspL()));
//    connect(graspLB, SIGNAL(clicked()), &spacenavThread, SLOT(graspL()));
    openLB = new QPushButton;
    openLB->setText("Open L");
    leftGraspLay->addWidget(openLB);
    graspSelL->addButton(openLB);
    connect(openLB, SIGNAL(clicked()), this, SLOT(openL()));
//    connect(openLB, SIGNAL(clicked()), &libertyThread, SLOT(openL()));
//    connect(openLB, SIGNAL(clicked()), &spacenavThread, SLOT(openL()));
    loosenLB = new QPushButton;
    loosenLB->setText("Loosen L");
    leftGraspLay->addWidget(loosenLB);
    graspSelL->addButton(loosenLB);
    connect(loosenLB, SIGNAL(clicked()), this, SLOT(loosenL()));
//    connect(loosenLB, SIGNAL(clicked()), &libertyThread, SLOT(loosenL()));
//    connect(loosenLB, SIGNAL(clicked()), &spacenavThread, SLOT(loosenL()));

    graspLayout->addLayout(leftGraspLay);

    QHBoxLayout* rightGraspLay = new QHBoxLayout;
    graspSelR = new QButtonGroup;
    graspSelR->setExclusive(true);
    graspRB = new QPushButton;
    graspRB->setText("Grasp R");
    rightGraspLay->addWidget(graspRB);
    graspSelR->addButton(graspRB);
    connect(graspRB, SIGNAL(clicked()), this, SLOT(graspR()));
//    connect(graspRB, SIGNAL(clicked()), &libertyThread, SLOT(graspR()));
//    connect(graspRB, SIGNAL(clicked()), &spacenavThread, SLOT(graspR()));
    openRB = new QPushButton;
    openRB->setText("Open R");
    rightGraspLay->addWidget(openRB);
    graspSelR->addButton(openRB);
    connect(openRB, SIGNAL(clicked()), this, SLOT(openR()));
//    connect(openRB, SIGNAL(clicked()), &libertyThread, SLOT(openR()));
//    connect(openRB, SIGNAL(clicked()), &spacenavThread, SLOT(openR()));
    loosenRB = new QPushButton;
    loosenRB->setText("Loosen R");
    rightGraspLay->addWidget(loosenRB);
    graspSelR->addButton(loosenRB);
    connect(loosenRB, SIGNAL(clicked()), this, SLOT(loosenR()));
//    connect(loosenRB, SIGNAL(clicked()), &libertyThread, SLOT(loosenR()));
//    connect(loosenRB, SIGNAL(clicked()), &spacenavThread, SLOT(loosenR()));

    graspLayout->addLayout(rightGraspLay);

    QHBoxLayout* trigGraspLay = new QHBoxLayout;
    graspSelT = new QButtonGroup;
    graspSelT->setExclusive(true);
    graspTB = new QPushButton;
    graspTB->setText("Grasp T");
    trigGraspLay->addWidget(graspTB);
    graspSelT->addButton(graspTB);
    connect(graspTB, SIGNAL(clicked()), this, SLOT(graspT()));
    openTB = new QPushButton;
    openTB->setText("Open T");
    trigGraspLay->addWidget(openTB);
    graspSelT->addButton(openTB);
    connect(openTB, SIGNAL(clicked()), this, SLOT(openT()));
    loosenTB = new QPushButton;
    loosenTB->setText("Loosen T");
    trigGraspLay->addWidget(loosenTB);
    graspSelT->addButton(loosenTB);
    connect(loosenTB, SIGNAL(clicked()), this, SLOT(loosenT()));

    graspLayout->addLayout(trigGraspLay);

    graspBox = new QGroupBox;
    graspBox->setStyleSheet(groupStyleSheet);
    graspBox->setLayout(graspLayout);
    graspBox->setTitle("Grasp Commands");
    dumbLayout->addWidget(graspBox);

    libertyThread.openChannels();
    libertyThread.openLiberty();
    spacenavThread.openChannels();

//    dumbLayout->addLayout(grid);

    std::cout << "HMP -- Nudge Requests" << std::endl;
    
    QHBoxLayout* waistBoxLayout = new QHBoxLayout;
    QLabel* waistLab = new QLabel;
    waistLab->setText("Waist Angle");
    waistBoxLayout->addWidget(waistLab);
    
    waistSpin = new QDoubleSpinBox;
    waistSpin->setMaximum(160);
    waistSpin->setMinimum(-160);
    waistSpin->setValue(0);
    waistSpin->setToolTip("Waist Angle Value (degrees)");
    connect(waistSpin, SIGNAL(editingFinished()), this, SLOT(handleWaistSpin()));
    waistBoxLayout->addWidget(waistSpin);
    
    QLabel* degLab = new QLabel;
    degLab->setText("(deg)");
    waistBoxLayout->addWidget(degLab);
    
    dumbLayout->addLayout(waistBoxLayout);

    waistSlide = new QSlider(Qt::Horizontal);
    waistSlide->setMaximum(160);
    waistSlide->setMinimum(-160);
    waistSlide->setValue(0);
    waistSlide->setToolTip("Waist Angle Value");
    connect(waistSlide, SIGNAL(valueChanged(int)), this, SLOT(handleWaistSlide(int)));
    connect(waistSlide, SIGNAL(sliderReleased()), this, SLOT(handleWaistRelease()));
//    connect(waistSlide, SIGNAL(valueChanged(int)), &libertyThread, SLOT(getWaistValue(int)));
//    connect(waistSlide, SIGNAL(valueChanged(int)), &spacenavThread, SLOT(getWaistValue(int)));
    
    dumbLayout->addWidget(waistSlide);
    
    
    nudgeBox = new QGroupBox;
    nudgeBox->setStyleSheet(groupStyleSheet);
    nudgeBox->setTitle("Target Nudging");
    QVBoxLayout* nudgeLayout = new QVBoxLayout;
    
    QHBoxLayout* selectLayout = new QHBoxLayout;
    
    frameGroup = new QButtonGroup;
    frameGroup->setExclusive(true);
    
    globalRad = new QRadioButton;
    globalRad->setText("Global");
    globalRad->setToolTip("Nudge the end effector in Global Coordinates");
    connect(globalRad, SIGNAL(toggled(bool)), this, SLOT(handleGlobalToggle(bool)));
    frameGroup->addButton(globalRad);
    selectLayout->addWidget(globalRad, 0, Qt::AlignLeft);
    
    localRad = new QRadioButton;
    localRad->setText("Local");
    localRad->setToolTip("Nudge the end effector in Local Coordinates");
    connect(localRad, SIGNAL(toggled(bool)), this, SLOT(handleLocalToggle(bool)));
    frameGroup->addButton(localRad);
    selectLayout->addWidget(localRad, 0, Qt::AlignLeft);
    
    transStep = 1;
    rotStep = 5;
    stepBox = new QDoubleSpinBox;
    stepBox->setToolTip("Distance for target to step with each button click\nTranslation (cm) or Rotation (deg)");
    stepBox->setValue(transStep);
    stepBox->setDecimals(1);
    stepBox->setMinimum(0);
    stepBox->setSingleStep(1);
    connect(stepBox, SIGNAL(valueChanged(double)), this, SLOT(handleStepChange(double)));
    selectLayout->addWidget(stepBox, 0, Qt::AlignCenter);
    
    
    dGroup = new QButtonGroup;
    dGroup->setExclusive(true);
    
    transRad = new QRadioButton;
    transRad->setText("Translate");
    transRad->setToolTip("Set the nudging to translation mode");
    connect(transRad, SIGNAL(toggled(bool)), this, SLOT(handleTransToggle(bool)));
    dGroup->addButton(transRad);
    selectLayout->addWidget(transRad, 0, Qt::AlignRight);
    
    rotRad = new QRadioButton;
    rotRad->setText("Rotate");
    rotRad->setToolTip("Set the nudging to rotation mode");
    connect(rotRad, SIGNAL(toggled(bool)), this, SLOT(handleRotToggle(bool)));
    dGroup->addButton(rotRad);
    selectLayout->addWidget(rotRad, 0, Qt::AlignRight);
    
    
    globalRad->setChecked(true);
    transRad->setChecked(true);
    
    
    nudgeLayout->addLayout(selectLayout);
    
    
    QHBoxLayout* posLayout = new QHBoxLayout;
    
    plusX = new QPushButton;
    plusX->setText("+ X");
    plusX->setToolTip("Nudge Positively along the X Axis");
    connect(plusX, SIGNAL(clicked()), this, SLOT(handlePlusX()));
    posLayout->addWidget(plusX);
    
    plusY = new QPushButton;
    plusY->setText("+ Y");
    plusY->setToolTip("Nudge Positively along the Y Axis");
    connect(plusY, SIGNAL(clicked()), this, SLOT(handlePlusY()));
    posLayout->addWidget(plusY);
    
    plusZ = new QPushButton;
    plusZ->setText("+ Z");
    plusZ->setToolTip("Nudge Positively along the Z Axis");
    connect(plusZ, SIGNAL(clicked()), this, SLOT(handlePlusZ()));
    posLayout->addWidget(plusZ);
    
    nudgeLayout->addLayout(posLayout);
    
    QHBoxLayout* negLayout = new QHBoxLayout;
    
    minusX = new QPushButton;
    minusX->setText("- X");
    minusX->setToolTip("Nudge Negatively along the X Axis");
    connect(minusX, SIGNAL(clicked()), this, SLOT(handleMinusX()));
    negLayout->addWidget(minusX);
    
    minusY = new QPushButton;
    minusY->setText("- Y");
    minusY->setToolTip("Nudge Negatively along the Y Axis");
    connect(minusY, SIGNAL(clicked()), this, SLOT(handleMinusY()));
    negLayout->addWidget(minusY);
    
    minusZ = new QPushButton;
    minusZ->setText("- Z");
    minusZ->setToolTip("Nudge Negatively along the Z Axis");
    connect(minusZ, SIGNAL(clicked()), this, SLOT(handleMinusZ()));
    negLayout->addWidget(minusZ);
    
    nudgeLayout->addLayout(negLayout);
    
    nudgeBox->setLayout(nudgeLayout);
    dumbLayout->addWidget(nudgeBox);
    
    setLayout(dumbLayout);



    rightSel->click();


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


    for(int i=0; i<2; i++)
        for(int j=0; j<3; j++)
            pos[i][j] = state.pose[i].data[j];

    for(int i=0; i<2; i++)
    {
        if(ACH_OK==manR || ACH_MISSED_FRAME==manR || ACH_STALE_FRAMES==manR) // FIXME: Remove this condition
        {
            std::cout << "Initializing euler angles\t" << ach_result_to_string(manR) << std::endl;
            double q0 = /*state.pose[i].w;*/ 1; // TODO: FIXME
            double q1 = state.pose[i].i;
            double q2 = state.pose[i].j;
            double q3 = state.pose[i].k;
            angles[i][0] = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
            angles[i][1] = asin(2*(q0*q2 - q3*q1));
            angles[i][2] = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
        }
        else
            angles[i] << 0, 0, 0;
    }

    sn_freq = 10;

    if(alive)
    {
        ros::Subscriber spacenavJoy = nh.subscribe("spacenav/joy", 1,
                            &hubo_motion_ros::SpacenavRelay::spacenav_callback, this);

        refClock.start();

        ros::Rate r(sn_freq);

        restart = true;

        while(alive)
        {
            if(restart)
            {
                side = queueSide;
//                for(int i=0; i<2; i++)
//                    for(int j=0; j<3; j++)
//                        pos[i][j] = state.pose[i].data[j];

//                if(ACH_OK==manR || ACH_MISSED_FRAME==manR || ACH_STALE_FRAMES==manR) // FIXME: Remove this condition
//                {
//                    std::cout << "Initializing euler angles\t" << ach_result_to_string(manR) << std::endl;
//                    double q0 = state.pose[side].w;// 1; // TODO: FIXME
//                    double q1 = state.pose[side].i;
//                    double q2 = state.pose[side].j;
//                    double q3 = state.pose[side].k;
//                    angles[side][0] = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
//                    angles[side][1] = asin(2*(q0*q2 - q3*q1));
//                    angles[side][2] = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
//                }
//                else
//                    angles[side] << 0, 0, 0;


                rotationOn = false;
                zOn = true;

                for(int j=0; j<3; j++)
                    cmd.pose[side].data[j] = pos[side][j];

                cmd.pose[side].alpha = angles[side][0];
                cmd.pose[side].beta = angles[side][1];
                cmd.pose[side].gamma = angles[side][2];

                restart = false;
            }

            ros::spinOnce();
            r.sleep();

        }

        cmd.m_mode[0] = MC_HALT;
        cmd.m_mode[1] = MC_HALT;
        cmd.interrupt[0] = true;
        cmd.interrupt[1] = true;

        cmd.waistAngle = waistAngle;
        ach_put( &manipCmdChan, &cmd, sizeof(cmd) );

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
            pos[side][j] += joystick->axes[j]*posScale;

        for(int j=0; j<num; j++)
            cmd.pose[side].data[j] = pos[side][j];
    }

    if(rotationOn)
    {
        for(int j=0; j<3; j++)
            angles[side][j] += joystick->axes[j+3]*rotScale;

        cmd.pose[side].alpha = angles[side][0];
        cmd.pose[side].beta = angles[side][1];
        cmd.pose[side].gamma = angles[side][2];
    }

    double elapsed = refClock.elapsed()/1000.0;
    if( updateFreq > 0 )
    {
        if( elapsed > 1.0/updateFreq )
        {
            for(int j=0; j<7; j++)
                emit refreshData(cmd.pose[side].data[j], side, j);
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
