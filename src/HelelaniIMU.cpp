#include "HelelaniIMU.h"
#include <pluginlib/class_list_macros.h>

static void InitResources() { Q_INIT_RESOURCE(resources); }
static void CleanupResources() { Q_CLEANUP_RESOURCE(resources); }

namespace helelani_client {

void HelelaniIMU::initPlugin(qt_gui_cpp::PluginContext& context)
{
    InitResources();

    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    m_widget = new QWidget();
    // extend the widget with all attributes and children from UI file
    m_ui.setupUi(m_widget);
    m_widget->setObjectName("HelelaniIMU");
    if (context.serialNumber() > 1)
        m_widget->setWindowTitle(m_widget->windowTitle() +
                                 " (" + QString::number(context.serialNumber()) + ")");
    // add widget to the user interface
    context.addWidget(m_widget);

    QObject::connect(this, SIGNAL(imuUpdated()),
                     this, SLOT(updateImuUI()));

    ros::NodeHandle rosNode;
    m_imuSub = rosNode.subscribe("/helelani/imu", 1000, &HelelaniIMU::imuCallback, this);
}

void HelelaniIMU::shutdownPlugin()
{
    m_imuSub.shutdown();
    CleanupResources();
}

void HelelaniIMU::saveSettings(qt_gui_cpp::Settings& plugin_settings,
                               qt_gui_cpp::Settings& instance_settings) const
{

}

void HelelaniIMU::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                                  const qt_gui_cpp::Settings& instance_settings)
{

}

void HelelaniIMU::updateImuUI()
{
    std::lock_guard<std::mutex> lk(m_imuLock);
    m_ui.attitude->setPitch(m_imuData.pitch);
    m_ui.attitude->setRoll(m_imuData.roll);
    m_ui.heading->setHeading(m_imuData.yaw);
}

void HelelaniIMU::imuCallback(const helelani_common::Imu& message)
{
    std::lock_guard<std::mutex> lk(m_imuLock);
    m_imuData = message;
    emit imuUpdated();
}

}

PLUGINLIB_EXPORT_CLASS(helelani_client::HelelaniIMU, rqt_gui_cpp::Plugin)
