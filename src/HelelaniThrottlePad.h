#ifndef HELELANITHROTTLEPAD_HPP
#define HELELANITHROTTLEPAD_HPP

#include <QMainWindow>
#include <QtCore>
#include <rqt_gui_cpp/plugin.h>
#include <ros/ros.h>
#include "ui_HelelaniThrottlePad.h"

namespace helelani_client {

class HelelaniThrottlePad : public rqt_gui_cpp::Plugin
{
    Q_OBJECT

public:
    void initPlugin(qt_gui_cpp::PluginContext& context);
    void shutdownPlugin();
    void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

public slots:
    void driveMove(DriveTrackpad* sender, QMouseEvent* ev);
    void driveRelease(DriveTrackpad* sender, QMouseEvent* ev);

private:
    void changeThrottles(float left, float right);

    Ui::HelelaniThrottlePad m_ui;
    QWidget* m_widget = nullptr;
    ros::Publisher m_pub;
    bool m_moveDragging = false;
};

}

#endif // HELELANITHROTTLEPAD_HPP
