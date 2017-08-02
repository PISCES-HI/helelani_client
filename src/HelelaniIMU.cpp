#include "HelelaniIMU.h"
#include <pluginlib/class_list_macros.h>
#include <helelani_common/helelani_common.h>
#include <QtGui/QMenu>

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

    m_ui.distanceNumber->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(m_ui.distanceNumber, SIGNAL(customContextMenuRequested(QPoint)),
            this, SLOT(distanceMenuRequested(QPoint)));

    ros::NodeHandle rosNode;
    m_imuSub = rosNode.subscribe("/helelani/imu", 10, &HelelaniIMU::imuCallback, this);
    m_leftMotorSub = rosNode.subscribe("/helelani/left_motor", 10, &HelelaniIMU::leftMotorCallback, this);
    m_rightMotorSub = rosNode.subscribe("/helelani/right_motor", 10, &HelelaniIMU::rightMotorCallback, this);
    m_navSatSub = rosNode.subscribe("/helelani/nav_sat_fix", 10, &HelelaniIMU::navSatCallback, this);
}

void HelelaniIMU::shutdownPlugin()
{
    m_imuSub.shutdown();
    m_leftMotorSub.shutdown();
    m_rightMotorSub.shutdown();
    m_navSatSub.shutdown();
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

    float avgRotations = (m_leftRotations + m_rightRotations) / 2.f;
    m_ui.distanceNumber->display(QString::number(helelani_common::RotationsToMeters
                                 (avgRotations - m_startRotations), 'f', 1));
    float avgSpeed = helelani_common::RotationsToMeters((m_leftSpeed + m_rightSpeed) / 2.f) / 60.f;
    m_ui.speedNumber->display(QString::number(avgSpeed, 'f', 2));

    {
        auto deg = int(m_latitude);
        double minSec = std::abs(m_latitude - deg) * 60.0;
        auto min = int(minSec);
        auto sec = int((minSec - min) * 60.0);
        m_ui.latDeg->display(deg);
        m_ui.latMin->display(min);
        m_ui.latSec->display(sec);
    }

    {
        auto deg = int(m_longitude);
        double minSec = std::abs(m_longitude - deg) * 60.0;
        auto min = int(minSec);
        auto sec = int((minSec - min) * 60.0);
        m_ui.lonDeg->display(deg);
        m_ui.lonMin->display(min);
        m_ui.lonSec->display(sec);
    }
}

void HelelaniIMU::distanceMenuRequested(QPoint pt)
{
    auto menu = new QMenu(m_widget);
    auto action = new QAction(QIcon::fromTheme("edit-clear"), "Reset Distance", m_widget);
    connect(action, SIGNAL(triggered(bool)), this, SLOT(resetDistance()));
    menu->addAction(action);
    menu->popup(m_ui.distanceNumber->mapToGlobal(pt));
}

void HelelaniIMU::resetDistance()
{
    float avgRotations = (m_leftRotations + m_rightRotations) / 2.f;
    m_startRotations = avgRotations;
    m_ui.distanceNumber->display(0.f);
}

void HelelaniIMU::imuCallback(const helelani_common::Imu& message)
{
    std::lock_guard<std::mutex> lk(m_imuLock);
    m_imuData = message;
    emit imuUpdated();
}

void HelelaniIMU::leftMotorCallback(const helelani_common::Motor& message)
{
    std::lock_guard<std::mutex> lk(m_imuLock);
    m_leftRotations = message.abs_rotations;
    m_leftSpeed = message.speed;
    emit imuUpdated();
}

void HelelaniIMU::rightMotorCallback(const helelani_common::Motor& message)
{
    std::lock_guard<std::mutex> lk(m_imuLock);
    m_rightRotations = message.abs_rotations;
    m_rightSpeed = message.speed;
    emit imuUpdated();
}

void HelelaniIMU::navSatCallback(const sensor_msgs::NavSatFix& message)
{
    std::lock_guard<std::mutex> lk(m_imuLock);
    m_latitude = message.latitude;
    m_longitude = message.longitude;
    emit imuUpdated();
}

}

PLUGINLIB_EXPORT_CLASS(helelani_client::HelelaniIMU, rqt_gui_cpp::Plugin)
