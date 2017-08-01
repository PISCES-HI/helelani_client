#include "HelelaniIPCamViewer.h"
#include <pluginlib/class_list_macros.h>
#include <QtGui/QMenu>
#include <QtGui/QFileDialog>
#include <QtGui/QMouseEvent>

namespace helelani_client {

// call this before loading the stream
static void tuneForZeroLatencyLiveStream(QtAV::AVPlayer& player)
{
    QVariantHash settings;

    // sets av_opt_set_xx and av_dict_set settings
    QVariantHash avFormat;

    {// these settings seem to have no impact on latency?
        //avFormat["fpsprobesize"] = 32;
        //avFormat["framerate"] = 100; // from: https://github.com/wang-bin/QtAV/issues/369
        //avFormat["rtbufsize"] = 512 * 1024;
        //avFormat["flush_packets"] = 1;
        //avFormat["avioflags"] = "direct";
        //avFormat["max_delay"] = 1000;
        //avFormat["fflags"] = "nobuffer";
        //avFormat["vprofile"] = "baseline";
    }

    //avFormat["probesize"] = 16384; // great impact but how to calc the optimum?  // from: https://github.com/wang-bin/QtAV/wiki/FFmpeg-dict-opt-in-QtAV
    avFormat["tune"] = "zerolatency,fastdecode";
    {
        // is what zerolatency does:
        //  avFormat["bframes"] = 0;
        //  avFormat["force-cfr"] = 1;
        //  avFormat["no-mbtree"] = 1;
        //  avFormat["sync-lookahead"] = 0;
        //  avFormat["sliced-threads"] = 1;
        //  avFormat["rc-lookahead"] = 0;
    }

    //settings["avformat"] = avFormat;
    player.setOptionsForFormat(avFormat);
}

static void startStream(QtAV::AVPlayer& player, QtAV::VideoRenderer* renderer, const char* url)
{
    //player.masterClock()->setClockType(QtAV::AVClock::VideoClock);
    //player.setBufferMode(QtAV::BufferPackets);
    //player.setBufferValue(1);
    tuneForZeroLatencyLiveStream(player);
    player.setRenderer(renderer);
    player.play(url);
}

void HelelaniIPCamViewer::mainMenuRequested(QPoint pt)
{
    auto menu = new QMenu(m_widget);
    auto action = new QAction(QIcon::fromTheme("camera-photo"), "Save Screenshot", m_widget);
    connect(action, SIGNAL(triggered(bool)), this, SLOT(mainShotSave()));
    menu->addAction(action);
    menu->popup(m_ui.mainRenderer->mapToGlobal(pt));
}

void HelelaniIPCamViewer::mainShotSave()
{
    for (QtAV::AVPlayer& subPlayer : m_cameras)
        if (subPlayer.renderer() == m_ui.mainRenderer)
            subPlayer.videoCapture()->capture();
}

void HelelaniIPCamViewer::leftMenuRequested(QPoint pt)
{
    auto menu = new QMenu(m_widget);
    auto action = new QAction(QIcon::fromTheme("camera-photo"), "Save Screenshot", m_widget);
    connect(action, SIGNAL(triggered(bool)), this, SLOT(leftShotSave()));
    menu->addAction(action);
    menu->popup(m_ui.subRendererLeft->mapToGlobal(pt));
}

void HelelaniIPCamViewer::leftShotSave()
{
    for (QtAV::AVPlayer& subPlayer : m_cameras)
        if (subPlayer.renderer() == m_ui.subRendererLeft)
            subPlayer.videoCapture()->capture();
}

void HelelaniIPCamViewer::rightMenuRequested(QPoint pt)
{
    auto menu = new QMenu(m_widget);
    auto action = new QAction(QIcon::fromTheme("camera-photo"), "Save Screenshot", m_widget);
    connect(action, SIGNAL(triggered(bool)), this, SLOT(rightShotSave()));
    menu->addAction(action);
    menu->popup(m_ui.subRendererRight->mapToGlobal(pt));
}

void HelelaniIPCamViewer::rightShotSave()
{
    for (QtAV::AVPlayer& subPlayer : m_cameras)
        if (subPlayer.renderer() == m_ui.subRendererRight)
            subPlayer.videoCapture()->capture();
}

void HelelaniIPCamViewer::frameAvailable(const QtAV::VideoFrame& frame)
{
    if (QFile::exists(m_lastImagePath + ".png"))
    {
        int numIdx = m_lastImagePath.lastIndexOf(QRegExp("[0-9]+"));
        if (numIdx < 0)
            m_lastImagePath += "-1";
        else
        {
            int num = m_lastImagePath.mid(numIdx).toInt();
            m_lastImagePath = m_lastImagePath.left(numIdx);
            if (m_lastImagePath.endsWith('-'))
                m_lastImagePath.chop(1);
            m_lastImagePath += "-" + QString::number(num + 1);
        }
    }
    QString fileName = QFileDialog::getSaveFileName(m_widget, tr("Save File"),
                                                    m_lastImagePath,
                                                    tr("Images (*.png)"));
    if (fileName.endsWith(".png"))
        fileName.chop(4);
    if (fileName.size())
    {
        m_lastImagePath = fileName;
        frame.toImage().save(fileName + ".png");
    }
}

void HelelaniIPCamViewer::subVideoClicked(SubVideoRendererWidget* src, QMouseEvent* ev)
{
    if (ev->button() != Qt::MouseButton::LeftButton)
        return;
    for (QtAV::AVPlayer& subPlayer : m_cameras)
    {
        if (subPlayer.renderer() == src)
        {
            for (QtAV::AVPlayer& mainPlayer : m_cameras)
            {
                if (mainPlayer.renderer() == m_ui.mainRenderer)
                {
                    subPlayer.setRenderer(m_ui.mainRenderer);
                    mainPlayer.setRenderer(src);
                    return;
                }
            }
        }
    }
}

void HelelaniIPCamViewer::initPlugin(qt_gui_cpp::PluginContext& context)
{
    m_widget = new QWidget();
    // access standalone command line arguments
    QStringList argv = context.argv();
    // extend the widget with all attributes and children from UI file
    m_ui.setupUi(m_widget);
    m_widget->setObjectName("HelelaniIPCamViewer");
    if (context.serialNumber() > 1)
        m_widget->setWindowTitle(m_widget->windowTitle() +
                                 " (" + QString::number(context.serialNumber()) + ")");
    // add widget to the user interface
    context.addWidget(m_widget);

    connect(m_ui.subRendererLeft, SIGNAL(clicked(SubVideoRendererWidget*, QMouseEvent*)),
            this, SLOT(subVideoClicked(SubVideoRendererWidget*, QMouseEvent*)));
    connect(m_ui.subRendererRight, SIGNAL(clicked(SubVideoRendererWidget*, QMouseEvent*)),
            this, SLOT(subVideoClicked(SubVideoRendererWidget*, QMouseEvent*)));

    m_ui.mainRenderer->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(m_ui.mainRenderer, SIGNAL(customContextMenuRequested(QPoint)),
            this, SLOT(mainMenuRequested(QPoint)));
    m_ui.subRendererLeft->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(m_ui.subRendererLeft, SIGNAL(customContextMenuRequested(QPoint)),
            this, SLOT(leftMenuRequested(QPoint)));
    m_ui.subRendererRight->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(m_ui.subRendererRight, SIGNAL(customContextMenuRequested(QPoint)),
            this, SLOT(rightMenuRequested(QPoint)));

    for (int i=0 ; i<3 ; ++i)
        connect(m_cameras[i].videoCapture(), SIGNAL(frameAvailable(const QtAV::VideoFrame&)),
                this, SLOT(frameAvailable(const QtAV::VideoFrame&)));

    startStream(m_cameras[0], m_ui.mainRenderer, "rtsp://10.10.153.9/axis-media/media.amp");
    startStream(m_cameras[1], m_ui.subRendererLeft, "rtsp://10.10.153.10/axis-media/media.amp");
    startStream(m_cameras[2], m_ui.subRendererRight, "rtsp://10.10.153.11/axis-media/media.amp");
}

void HelelaniIPCamViewer::shutdownPlugin()
{
    for (QtAV::AVPlayer& player : m_cameras)
        player.stop();
}

void HelelaniIPCamViewer::saveSettings(qt_gui_cpp::Settings& plugin_settings,
                                       qt_gui_cpp::Settings& instance_settings) const
{

}

void HelelaniIPCamViewer::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                                          const qt_gui_cpp::Settings& instance_settings)
{

}

}

PLUGINLIB_EXPORT_CLASS(helelani_client::HelelaniIPCamViewer, rqt_gui_cpp::Plugin)
