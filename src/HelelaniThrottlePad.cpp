#include "HelelaniThrottlePad.h"
#include <pluginlib/class_list_macros.h>
#include <QtGui/QMouseEvent>
#include <helelani_common/Throttle.h>

namespace helelani_client {

void HelelaniThrottlePad::initPlugin(qt_gui_cpp::PluginContext& context)
{
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    m_widget = new QWidget();
    // extend the widget with all attributes and children from UI file
    m_ui.setupUi(m_widget);
    m_widget->setObjectName("HelelaniThrottlePad");
    if (context.serialNumber() > 1)
        m_widget->setWindowTitle(m_widget->windowTitle() +
                                 " (" + QString::number(context.serialNumber()) + ")");
    // add widget to the user interface
    context.addWidget(m_widget);

    connect(m_ui.driveTrackpad, SIGNAL(_mouseMoveEvent(DriveTrackpad*, QMouseEvent*)),
            this, SLOT(driveMove(DriveTrackpad*, QMouseEvent*)));
    connect(m_ui.driveTrackpad, SIGNAL(_mouseReleaseEvent(DriveTrackpad*, QMouseEvent*)),
            this, SLOT(driveRelease(DriveTrackpad*, QMouseEvent*)));

    ros::NodeHandle n;
    m_pub = n.advertise<helelani_common::Throttle>("/helelani/throttle", 10);
}

void HelelaniThrottlePad::shutdownPlugin()
{
    m_pub.shutdown();
}

void HelelaniThrottlePad::saveSettings(qt_gui_cpp::Settings& plugin_settings,
                                       qt_gui_cpp::Settings& instance_settings) const
{

}

void HelelaniThrottlePad::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                                          const qt_gui_cpp::Settings& instance_settings)
{

}

void HelelaniThrottlePad::driveMove(DriveTrackpad* sender, QMouseEvent* ev)
{
    float x = -(ev->x() / float(sender->width()) * 2.f - 1.f);
    float y = std::max(0.f, std::min(ev->y() / float(sender->height()), 1.f)) * 2.f - 1.f;
    float leftMul = std::max(0.f, std::min(1.f - x, 1.f)) * 2.f - 1.f;
    float rightMul = std::max(0.f, std::min(x + 1.f, 1.f)) * 2.f - 1.f;
    changeThrottles(leftMul * -y, rightMul * -y);
    m_moveDragging = true;
}

void HelelaniThrottlePad::driveRelease(DriveTrackpad* sender, QMouseEvent* ev)
{
    changeThrottles(0.f, 0.f);
    m_moveDragging = false;
}

void HelelaniThrottlePad::changeThrottles(float left, float right)
{
    helelani_common::Throttle msg = {};
    msg.left = left;
    msg.right = right;
    m_pub.publish(msg);

    float factorLeft = -left / 2.f + 0.5f;
    float factorRight = -right / 2.f + 0.5f;
    const QRect& rect = m_ui.driveTrackpad->geometry();
    const QRect& lrect = m_ui.leftThrottleLine->geometry();
    const QRect& rrect = m_ui.rightThrottleLine->geometry();
    m_ui.leftThrottleLine->move(lrect.left(),
                                int(rect.top() + (rect.height() - lrect.height()) * factorLeft));
    m_ui.rightThrottleLine->move(rrect.left(),
                                 int(rect.top() + (rect.height() - rrect.height()) * factorRight));
}

}

PLUGINLIB_EXPORT_CLASS(helelani_client::HelelaniThrottlePad, rqt_gui_cpp::Plugin)
