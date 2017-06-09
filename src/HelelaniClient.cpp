#include "HelelaniClient.h"
#include "ui_HelelaniClient.h"
#include <pluginlib/class_list_macros.h>

namespace helelani_client {

HelelaniClient::HelelaniClient()
{
    setObjectName("HelelaniClient");
}

HelelaniClient::~HelelaniClient()
{
}

void HelelaniClient::initPlugin(qt_gui_cpp::PluginContext& context)
{
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    m_widget = new QWidget();
    // extend the widget with all attributes and children from UI file
    m_ui.setupUi(m_widget);
    // add widget to the user interface
    context.addWidget(m_widget);

    ros::NodeHandle rosNode;
    m_imuSub = rosNode.subscribe("/helelani/imu", 1000, &HelelaniClient::imuCallback, this);
}

void HelelaniClient::shutdownPlugin()
{
    m_imuSub.shutdown();
}

void HelelaniClient::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{

}

void HelelaniClient::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{

}

void HelelaniClient::imuCallback(const helelani_common::Imu& message)
{
    geometry_msgs::Vector3 a;
    a.x = message.accel.y;
    a.y = -message.accel.z;
    a.z = message.accel.x;

    geometry_msgs::Vector3 m;
    m.x = message.mag.y;
    m.y = -message.mag.z;
    m.z = message.mag.x;

    const float roll = std::atan2(a.y, a.z);
    const float pitch = std::atan2(-a.x, a.y * std::sin(roll) + a.z * std::cos(roll));
    const float heading = std::atan2(m.z * std::sin(roll) - m.y * std::cos(roll),
                                     m.x * std::cos(pitch) + m.y * std::sin(pitch) *
                                     std::sin(roll) + m.z * std::sin(pitch) * std::cos(roll));

    m_ui.pitchDisplay->display(pitch);
    m_ui.rollDisplay->display(roll);
    m_ui.headingDisplay->display(heading);
}

}

PLUGINLIB_DECLARE_CLASS(helelani_client, HelelaniClient, helelani_client::HelelaniClient, rqt_gui_cpp::Plugin)
