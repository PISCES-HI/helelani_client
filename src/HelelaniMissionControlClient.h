#ifndef HELELANIMISSIONCONTROLCLIENT_HPP
#define HELELANIMISSIONCONTROLCLIENT_HPP

#include <QMainWindow>
#include <QtCore>
#include <QIntValidator>
#include <QDoubleValidator>
#include <rqt_gui_cpp/plugin.h>
#include <ros/ros.h>
#include <helelani_common/Mission.h>
#include <helelani_common/MissionStart.h>
#include <helelani_common/Analog.h>
#include <helelani_common/Motor.h>
#include <helelani_common/Imu.h>
#include "ui_HelelaniMissionControlClient.h"
#include <mutex>

namespace helelani_client {

class MissionUpdateEvent : public QEvent
{
    friend class HelelaniMissionControlClient;
    helelani_common::Mission m_msg;
public:
    MissionUpdateEvent(const helelani_common::Mission& msg)
    : QEvent(Type(int(Type::User) + 11)), m_msg(msg) {}
};

class AnalogUpdateEvent : public QEvent
{
    friend class HelelaniMissionControlClient;
    helelani_common::Analog m_msg;
public:
    AnalogUpdateEvent(const helelani_common::Analog& msg)
            : QEvent(Type(int(Type::User) + 12)), m_msg(msg) {}
};

class MotorUpdateEvent : public QEvent
{
    friend class HelelaniMissionControlClient;
    helelani_common::Motor m_msg;
public:
    MotorUpdateEvent(QEvent::Type tp, const helelani_common::Motor& msg)
            : QEvent(tp), m_msg(msg) {}
};

class LeftMotorUpdateEvent : public MotorUpdateEvent
{
public:
    LeftMotorUpdateEvent(const helelani_common::Motor& msg)
            : MotorUpdateEvent(Type(int(Type::User) + 13), msg) {}
};

class RightMotorUpdateEvent : public MotorUpdateEvent
{
public:
    RightMotorUpdateEvent(const helelani_common::Motor& msg)
            : MotorUpdateEvent(Type(int(Type::User) + 14), msg) {}
};

class IMUUpdateEvent : public QEvent
{
    friend class HelelaniMissionControlClient;
    helelani_common::Imu m_msg;
public:
    IMUUpdateEvent(const helelani_common::Imu& msg)
            : QEvent(Type(int(Type::User) + 15)), m_msg(msg) {}
};

struct CSVData
{
    QString time;
    float wheelRot;
    float leftCurrent;
    float rightCurrent;
    float pitch;
    float roll;
    float heading;
    float _12V;
};

class CSVRecorder
{
    QString m_csvString;
public:
    CSVRecorder();
    void addData(const CSVData& data);
    void writeOut(const QString& path);
};

class HelelaniMissionControlClient : public rqt_gui_cpp::Plugin
{
    Q_OBJECT

public:
    void initPlugin(qt_gui_cpp::PluginContext& context);
    void shutdownPlugin();
    void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);
    void customEvent(QEvent* ev) override;

public slots:
    void startClicked();
    void endClicked();

private:
    Ui::HelelaniMissionControlClient m_ui;
    QWidget* m_widget = nullptr;
    QIntValidator m_intValidator;
    QDoubleValidator m_doubleValidator;
    CSVData m_csvData = {};
    CSVRecorder m_csv;

    float m_leftRotations = 0.f;
    float m_rightRotations = 0.f;

    void _missionUpdate(const helelani_common::Mission& msg);
    void _analogUpdate(const helelani_common::Analog& msg);
    void _leftMotorUpdate(const helelani_common::Motor& msg);
    void _rightMotorUpdate(const helelani_common::Motor& msg);
    void _imuUpdate(const helelani_common::Imu& msg);

    ros::Subscriber m_missionSub;
    ros::Subscriber m_analogSub;
    ros::Subscriber m_leftMotorSub;
    ros::Subscriber m_rightMotorSub;
    ros::Subscriber m_imuSub;
    ros::ServiceClient m_startMission;
    ros::ServiceClient m_endMission;

    enum State
    {
        Uninitialized,
        Started,
        Ended
    };
    State m_state = State::Uninitialized;
};

}

#endif // HELELANIMISSIONCONTROLCLIENT_HPP
