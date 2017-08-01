#include "HelelaniCameraControl.h"
#include <pluginlib/class_list_macros.h>
#include <std_srvs/Empty.h>

static void InitResources() { Q_INIT_RESOURCE(resources); }
static void CleanupResources() { Q_CLEANUP_RESOURCE(resources); }

namespace helelani_client {

void HelelaniCameraControl::initPlugin(qt_gui_cpp::PluginContext& context)
{
    InitResources();

    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    m_widget = new QWidget();
    // extend the widget with all attributes and children from UI file
    m_ui.setupUi(m_widget);
    m_ui.leftHeading->setName("Situation Cam");
    connect(m_ui.leftPitch, SIGNAL(valueChanged(int)),
            m_ui.leftHeading, SLOT(pitchSliderChanged(int)));
    connect(m_ui.leftHeading, SIGNAL(camUpdate(const helelani_common::CameraCtrl&)),
            this, SLOT(situationUpdate(const helelani_common::CameraCtrl&)));
    m_ui.rightHeading->setName("Stereo Cam");
    m_ui.rightHeading->setAngleBias(50.3 * M_PI / 180.f);
    connect(m_ui.rightPitch, SIGNAL(valueChanged(int)),
            m_ui.rightHeading, SLOT(pitchSliderChanged(int)));
    connect(m_ui.rightHeading, SIGNAL(camUpdate(const helelani_common::CameraCtrl&)),
            this, SLOT(stereoUpdate(const helelani_common::CameraCtrl&)));
    connect(m_ui.captureStereo, SIGNAL(clicked()), this, SLOT(stereoCapture()));
    m_widget->setObjectName("HelelaniCameraControl");
    if (context.serialNumber() > 1)
        m_widget->setWindowTitle(m_widget->windowTitle() +
                                 " (" + QString::number(context.serialNumber()) + ")");
    // add widget to the user interface
    context.addWidget(m_widget);

    ros::NodeHandle rosNode;
    m_situationCamPub = rosNode.advertise<helelani_common::CameraCtrl>
            ("/helelani/situation_cam_ctrl", 1000);
    m_stereoCamPub = rosNode.advertise<helelani_common::CameraCtrl>
            ("/helelani/stereo_cam_ctrl", 1000);
    m_stereoImageRequest = rosNode.serviceClient<std_srvs::Empty>
            ("/helelani/stereo_image_request");
}

void HelelaniCameraControl::shutdownPlugin()
{
    CleanupResources();
    m_situationCamPub.shutdown();
    m_stereoCamPub.shutdown();
    m_stereoImageRequest.shutdown();
}

void HelelaniCameraControl::saveSettings(qt_gui_cpp::Settings& plugin_settings,
                                         qt_gui_cpp::Settings& instance_settings) const
{

}

void HelelaniCameraControl::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                                            const qt_gui_cpp::Settings& instance_settings)
{

}

void HelelaniCameraControl::situationUpdate(const helelani_common::CameraCtrl& msg)
{
    m_situationCamPub.publish(msg);
}

void HelelaniCameraControl::stereoUpdate(const helelani_common::CameraCtrl& msg)
{
    m_stereoCamPub.publish(msg);
}

void HelelaniCameraControl::stereoCapture()
{
    std_srvs::Empty empty;
    m_stereoImageRequest.call(empty);
}

}

PLUGINLIB_EXPORT_CLASS(helelani_client::HelelaniCameraControl, rqt_gui_cpp::Plugin)
