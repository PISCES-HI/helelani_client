#include "HelelaniMissionControlClient.h"
#include <std_srvs/Empty.h>
#include <pluginlib/class_list_macros.h>

namespace helelani_client {

void HelelaniMissionControlClient::initPlugin(qt_gui_cpp::PluginContext& context)
{
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    m_widget = new QWidget();
    // extend the widget with all attributes and children from UI file
    m_ui.setupUi(m_widget);
    m_widget->setObjectName("HelelaniMissionControlClient");
    if (context.serialNumber() > 1)
        m_widget->setWindowTitle(m_widget->windowTitle() +
                                 " (" + QString::number(context.serialNumber()) + ")");
    // add widget to the user interface
    context.addWidget(m_widget);
    m_ui.timer->display("+00:00:00");
    connect(m_ui.startButton, SIGNAL(clicked()),
            this, SLOT(startClicked()));
    connect(m_ui.endButton, SIGNAL(clicked()),
            this, SLOT(endClicked()));
    m_ui.initialTimerEdit->setValidator(&m_intValidator);
    m_ui.transmitEdit->setValidator(&m_doubleValidator);
    m_ui.receiveEdit->setValidator(&m_doubleValidator);

    m_ui.initialTimerEdit->setText("0");
    m_ui.transmitEdit->setText("0");
    m_ui.receiveEdit->setText("0");

    ros::NodeHandle n;
    m_missionSub = n.subscribe("/helelani/mission", 10,
                               &HelelaniMissionControlClient::_missionUpdate, this);
    m_analogSub = n.subscribe("/helelani/analog", 10,
                              &HelelaniMissionControlClient::_analogUpdate, this);
    m_leftMotorSub = n.subscribe("/helelani/left_motor", 10,
                                 &HelelaniMissionControlClient::_leftMotorUpdate, this);
    m_rightMotorSub = n.subscribe("/helelani/right_motor", 10,
                                  &HelelaniMissionControlClient::_rightMotorUpdate, this);
    m_startMission = n.serviceClient<helelani_common::MissionStart>("/helelani/start_mission");
    m_endMission = n.serviceClient<std_srvs::Empty>("/helelani/end_mission");
}

void HelelaniMissionControlClient::shutdownPlugin()
{
    m_missionSub.shutdown();
    m_analogSub.shutdown();
    m_leftMotorSub.shutdown();
    m_rightMotorSub.shutdown();
    m_startMission.shutdown();
    m_endMission.shutdown();
}

void HelelaniMissionControlClient::saveSettings(qt_gui_cpp::Settings& plugin_settings,
                                                qt_gui_cpp::Settings& instance_settings) const
{

}

void HelelaniMissionControlClient::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                                                   const qt_gui_cpp::Settings& instance_settings)
{

}

void HelelaniMissionControlClient::customEvent(QEvent* ev)
{
    if (ev->type() == QEvent::Type::User + 11)
    {
        auto mue = static_cast<MissionUpdateEvent*>(ev);
        helelani_common::Mission& msg = mue->m_msg;
        m_ui.timer->setEnabled(true);
        QString timerStr;
        int seconds = std::abs(msg.elapsed_seconds);
        int minutes = seconds / 60;
        int hours = minutes / 60;
        minutes = minutes % 60;
        seconds = seconds % 60;
        timerStr.sprintf("%c%02d:%02d:%02d", msg.elapsed_seconds >= 0 ? '+' : '-',
                         hours, minutes, seconds);
        m_ui.timer->display(timerStr);
        if (!msg.mission_active && m_state != State::Ended)
        {
            m_ui.startButton->setEnabled(true);
            m_ui.endButton->setEnabled(false);
            m_ui.initialTimerEdit->setText(QString::number(msg.elapsed_seconds));
            m_ui.initialTimerEdit->setEnabled(true);
            m_ui.transmitEdit->setText(QString::number(msg.to_rover_delay));
            m_ui.transmitEdit->setEnabled(true);
            m_ui.receiveEdit->setText(QString::number(msg.from_rover_delay));
            m_ui.receiveEdit->setEnabled(true);
            m_ui.stereoType->setCurrentIndex(msg.stereo_camera_type);
            m_ui.stereoType->setEnabled(true);
            m_state = State::Ended;
        }
        else if (msg.mission_active && m_state != State::Started)
        {
            m_ui.startButton->setEnabled(false);
            m_ui.endButton->setEnabled(true);
            m_ui.initialTimerEdit->setEnabled(false);
            m_ui.transmitEdit->setText(QString::number(msg.to_rover_delay));
            m_ui.transmitEdit->setEnabled(false);
            m_ui.receiveEdit->setText(QString::number(msg.from_rover_delay));
            m_ui.receiveEdit->setEnabled(false);
            m_ui.stereoType->setCurrentIndex(msg.stereo_camera_type);
            m_ui.stereoType->setEnabled(false);
            m_state = State::Started;
        }
    }
    else if (ev->type() == QEvent::Type::User + 12)
    {
        auto aue = static_cast<AnalogUpdateEvent*>(ev);
        helelani_common::Analog& msg = aue->m_msg;
        m_ui.volt12->display(QString::number(msg.voltage_12, 'f', 2));
        m_ui.amp12->display(QString::number(msg.current_12, 'f', 2));
        m_ui.volt48->display(QString::number(msg.voltage_48, 'f', 2));
        m_ui.amp24->display(QString::number(msg.current_24, 'f', 2));
        m_ui.tempLeft->display(QString::number(msg.temp_l, 'f', 2));
        m_ui.tempRight->display(QString::number(msg.temp_r, 'f', 2));
        m_ui.tempBox->display(QString::number(msg.temp_box, 'f', 2));
    }
    else if (ev->type() == QEvent::Type::User + 13)
    {
        auto mue = static_cast<MotorUpdateEvent*>(ev);
        helelani_common::Motor& msg = mue->m_msg;
        m_ui.ampLeft->display(QString::number(msg.current, 'f', 2));
    }
    else if (ev->type() == QEvent::Type::User + 14)
    {
        auto mue = static_cast<MotorUpdateEvent*>(ev);
        helelani_common::Motor& msg = mue->m_msg;
        m_ui.ampRight->display(QString::number(msg.current, 'f', 2));
    }
}

void HelelaniMissionControlClient::startClicked()
{
    helelani_common::MissionStart data = {};
    data.request.elapsed_seconds = m_ui.initialTimerEdit->text().toInt();
    data.request.to_rover_delay = float(m_ui.transmitEdit->text().toDouble());
    data.request.from_rover_delay = float(m_ui.receiveEdit->text().toDouble());
    data.request.stereo_camera_type = uint8_t(m_ui.stereoType->currentIndex());
    m_startMission.call(data);
}

void HelelaniMissionControlClient::endClicked()
{
    std_srvs::Empty data = {};
    m_endMission.call(data);
}

void HelelaniMissionControlClient::_missionUpdate(const helelani_common::Mission& msg)
{
    QApplication::postEvent(this, new MissionUpdateEvent(msg));
}

void HelelaniMissionControlClient::_analogUpdate(const helelani_common::Analog& msg)
{
    QApplication::postEvent(this, new AnalogUpdateEvent(msg));
}

void HelelaniMissionControlClient::_leftMotorUpdate(const helelani_common::Motor& msg)
{
    QApplication::postEvent(this, new LeftMotorUpdateEvent(msg));
}

void HelelaniMissionControlClient::_rightMotorUpdate(const helelani_common::Motor& msg)
{
    QApplication::postEvent(this, new RightMotorUpdateEvent(msg));
}

}

PLUGINLIB_EXPORT_CLASS(helelani_client::HelelaniMissionControlClient, rqt_gui_cpp::Plugin)
