#ifndef HELELANICAMERACONTROL_HPP
#define HELELANICAMERACONTROL_HPP

#include <QtGui/QMainWindow>
#include <QtCore/QtCore>
#include <rqt_gui_cpp/plugin.h>
#include <ros/node_handle.h>
#include <helelani_common/CameraCtrl.h>
#include "ui_HelelaniCameraControl.h"
#include <mutex>

namespace helelani_client {

class HelelaniCameraControl : public rqt_gui_cpp::Plugin
{
    Q_OBJECT

public:
    void initPlugin(qt_gui_cpp::PluginContext& context);
    void shutdownPlugin();
    void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

public slots:
    void situationUpdate(const helelani_common::CameraCtrl& msg);
    void stereoUpdate(const helelani_common::CameraCtrl& msg);
    void stereoCapture();

private:
    Ui::HelelaniCameraControl m_ui;
    QWidget* m_widget = nullptr;
    ros::Publisher m_situationCamPub;
    ros::Publisher m_stereoCamPub;
    ros::ServiceClient m_stereoImageRequest;
};

}

#endif // HELELANIIMU_HPP
