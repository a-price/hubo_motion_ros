
#include "hubo_motion_ros/hubo_motion_panel.h"

namespace hubo_motion_ros
{

HuboMotionPanel::HuboMotionPanel(QWidget *parent)
    : rviz::Panel(parent)
{
    // TODO: Add channels

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
    connect(libertyCheck, SIGNAL(toggled(bool)), this, SLOT(handleCheckToggle(bool)));
    checkLayout->addWidget(libertyCheck, 0, Qt::AlignLeft);

    QLabel* freqLab = new QLabel;
    freqLab->setText("Display Frequency:");
    freqLab->setToolTip("Rate (Hz) at which to display data from Liberty");
    checkLayout->addWidget(freqLab, 0, Qt::AlignRight);

    libertyFreq = new QDoubleSpinBox;
    libertyFreq->setToolTip(freqLab->toolTip());
    libertyFreq->setMinimum(0);
    connect(libertyFreq, SIGNAL(valueChanged(double)), &libertyThread, SLOT(getUpdateFrequency(double)));
    libertyFreq->setValue(5);

    checkLayout->addWidget(libertyFreq, 0, Qt::AlignLeft);
    dumbLayout->addLayout(checkLayout);

    connect(this, SIGNAL(stopLiberty()), &libertyThread, SLOT(haltOperation()));
    connect(&libertyThread, SIGNAL(refreshData(hubo_manip_cmd_t)), this, SLOT(getRefreshData(hubo_manip_cmd_t)));
    connect(&libertyThread, SIGNAL(libertyQuitting()), this, SLOT(handleLibQuit()));


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

    libertyThread.openChannels();

    dumbLayout->addLayout(grid);

    setLayout(dumbLayout);
}

HuboMotionPanel::~HuboMotionPanel()
{

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

        double elapsed = refClock.elapsed()*1000;
        if( updateFreq > 0 )
        {
            if( elapsed > 1.0/updateFreq )
            {
                refClock.restart();
                emit refreshData(cmd);
            }
        }
        ach_put( &manipCmdChan, &cmd, sizeof(cmd) );
    }

    emit libertyQuitting();
}

void LibertyRelay::openChannels()
{
    ach_status_t r = ach_open(&libertyChan, "liberty", NULL);
    if( ACH_OK != r )
        std::cout << "Liberty Channel Error: " << ach_result_to_string(r) << std::endl;

    r = ach_open(&manipCmdChan, CHAN_HUBO_MANIP_CMD, NULL);
    if( ACH_OK != r )
        std::cout << "Command Channel Error: " << ach_result_to_string(r) << std::endl;

    r = ach_open(&manipStateChan, CHAN_HUBO_MANIP_STATE, NULL);
    if( ACH_OK != r )
        std::cout << "Command Channel Error: " << ach_result_to_string(r) << std::endl;
}


}



#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( hubo_motion_ros::HuboMotionPanel, rviz::Panel )
