#ifndef HELELANIIMU_HPP
#define HELELANIIMU_HPP

#include <QtGui/QMainWindow>
#include <QtCore/QtCore>
#include <rqt_gui_cpp/plugin.h>
#include <ros/node_handle.h>
#include <helelani_common/Imu.h>
#include "ui_HelelaniIMU.h"
#include <mutex>

namespace helelani_client {

class HelelaniIMU : public rqt_gui_cpp::Plugin
{
    Q_OBJECT

public:
    void initPlugin(qt_gui_cpp::PluginContext& context);
    void shutdownPlugin();
    void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

    void imuCallback(const helelani_common::Imu& message);

signals:
    void imuUpdated();
public slots:
    void updateImuUI();

private:
    std::mutex m_imuLock;
    helelani_common::Imu m_imuData;
    ros::Subscriber m_imuSub;
    Ui::HelelaniIMU m_ui;
    QWidget* m_widget = nullptr;
};

}

#endif // HELELANIIMU_HPP
