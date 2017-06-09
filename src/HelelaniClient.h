#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QMainWindow>
#include <rqt_gui_cpp/plugin.h>
#include <ros/node_handle.h>
#include <helelani_common/Imu.h>
#include "ui_HelelaniClient.h"

namespace helelani_client {

class HelelaniClient : public rqt_gui_cpp::Plugin
{
    Q_OBJECT

public:
    HelelaniClient();
    ~HelelaniClient();
    void initPlugin(qt_gui_cpp::PluginContext& context);
    void shutdownPlugin();
    void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

    void imuCallback(const helelani_common::Imu& message);

public slots:

private:
    ros::Subscriber m_imuSub;
    Ui::HelelaniClient m_ui;
    QWidget* m_widget = nullptr;
};

}

#endif // MAINWINDOW_HPP
