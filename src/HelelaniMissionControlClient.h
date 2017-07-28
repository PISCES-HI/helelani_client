#ifndef HELELANIMISSIONCONTROLCLIENT_HPP
#define HELELANIMISSIONCONTROLCLIENT_HPP

#include <QtGui/QMainWindow>
#include <QtCore/QtCore>
#include <QtGui/QIntValidator>
#include <QtGui/QDoubleValidator>
#include <rqt_gui_cpp/plugin.h>
#include <ros/ros.h>
#include <helelani_common/Mission.h>
#include <helelani_common/MissionStart.h>
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

    void _missionUpdate(const helelani_common::Mission& msg);

    ros::Subscriber m_missionSub;
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
