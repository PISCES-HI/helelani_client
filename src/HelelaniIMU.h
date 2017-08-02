#ifndef HELELANIIMU_HPP
#define HELELANIIMU_HPP

#include <QtGui/QMainWindow>
#include <QtCore/QtCore>
#include <rqt_gui_cpp/plugin.h>
#include <ros/node_handle.h>
#include <helelani_common/Imu.h>
#include <helelani_common/Motor.h>
#include <sensor_msgs/NavSatFix.h>
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
    void leftMotorCallback(const helelani_common::Motor& message);
    void rightMotorCallback(const helelani_common::Motor& message);
    void navSatCallback(const sensor_msgs::NavSatFix& message);

signals:
    void imuUpdated();
public slots:
    void updateImuUI();
    void distanceMenuRequested(QPoint pt);
    void resetDistance();

private:
    std::mutex m_imuLock;
    helelani_common::Imu m_imuData;
    float m_leftRotations = 0.f;
    float m_rightRotations = 0.f;
    float m_leftSpeed = 0.f;
    float m_rightSpeed = 0.f;
    float m_startRotations = 0.f;
    double m_latitude = 0.0;
    double m_longitude = 0.0;
    ros::Subscriber m_imuSub;
    ros::Subscriber m_leftMotorSub;
    ros::Subscriber m_rightMotorSub;
    ros::Subscriber m_navSatSub;
    Ui::HelelaniIMU m_ui;
    QWidget* m_widget = nullptr;
};

}

#endif // HELELANIIMU_HPP
