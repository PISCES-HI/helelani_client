#include "HelelaniIPCamViewer.h"
#include <pluginlib/class_list_macros.h>
#include <QtGui/QMenu>
#include <QtGui/QFileDialog>
#include <QtGui/QMouseEvent>
#include <VLCQtCore/Video.h>

namespace helelani_client {

#if 0
// call this before loading the stream
static void tuneForZeroLatencyLiveStream(VlcMediaPlayer& player)
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
#endif

void HelelaniIPCamViewer::startStream(VlcMediaPlayer& player, VlcWidgetVideo* renderer, VlcMedia* media)
{
    //player.masterClock()->setClockType(QtAV::AVClock::VideoClock);
    //player.setBufferMode(QtAV::BufferPackets);
    //player.setBufferValue(1);
    //tuneForZeroLatencyLiveStream(player);
    player.setVideoWidget(renderer);
    player.open(media);
}

void HelelaniIPCamViewer::mainMenuRequested(QPoint pt)
{
    auto menu = new QMenu(m_widget);
    auto action = new QAction(QIcon::fromTheme("camera-photo"), "Save Screenshot", m_widget);
    connect(action, SIGNAL(triggered(bool)), this, SLOT(mainShotSave()));
    menu->addAction(action);
    menu->popup(m_ui.mainRenderer->mapToGlobal(pt));
}

static void TakeSnapshot(VlcMediaPlayer& subPlayer)
{
    const char* tmpdir = getenv("TMPDIR");
    if (!tmpdir)
        tmpdir = "/tmp";
    QString tmpPath(tmpdir);
    tmpPath += "/mission_shot.png";
    subPlayer.video()->takeSnapshot(tmpPath);
}

void HelelaniIPCamViewer::mainShotSave()
{
    for (auto& subPlayer : m_cameras)
        if (subPlayer->videoWidget() == m_ui.mainRenderer)
            TakeSnapshot(*subPlayer);
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
    for (auto& subPlayer : m_cameras)
        if (subPlayer->videoWidget() == m_ui.subRendererLeft)
            TakeSnapshot(*subPlayer);
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
    for (auto& subPlayer : m_cameras)
        if (subPlayer->videoWidget() == m_ui.subRendererRight)
            TakeSnapshot(*subPlayer);
}

void HelelaniIPCamViewer::frameAvailable(const QString& path)
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
        QFile::copy(path, fileName);
    }
}

void HelelaniIPCamViewer::subVideoClicked(SubVideoRendererWidget* src, QMouseEvent* ev)
{
    if (ev->button() != Qt::MouseButton::LeftButton)
        return;
    for (auto& subPlayer : m_cameras)
    {
        if (subPlayer->videoWidget() == src)
        {
            for (auto& mainPlayer : m_cameras)
            {
                if (mainPlayer->videoWidget() == m_ui.mainRenderer)
                {
#if 0
                    subPlayer->stop();
                    subPlayer->setVideoWidget(m_ui.mainRenderer);
                    m_ui.mainRenderer->setMediaPlayer(subPlayer.get());
                    subPlayer->play();
                    mainPlayer->stop();
                    mainPlayer->setVideoWidget(src);
                    src->setMediaPlayer(mainPlayer.get());
                    mainPlayer->play();
#endif
                    VlcMedia* subMedia = subPlayer->currentMedia();
                    VlcMedia* mainMedia = mainPlayer->currentMedia();
                    subPlayer->open(mainMedia);
                    mainPlayer->open(subMedia);
                    printf("SWAPPED\n");
                    return;
                }
            }
        }
    }
}

HelelaniIPCamViewer::HelelaniIPCamViewer()
: m_vlcInst(QStringList()),
  m_mainMedia("rtsp://10.10.153.9/axis-media/media.amp", &m_vlcInst),
  m_leftMedia("rtsp://10.10.153.10/axis-media/media.amp", &m_vlcInst),
  m_rightMedia("rtsp://10.10.153.11/axis-media/media.amp", &m_vlcInst)
{
    for (int i=0 ; i<3 ; ++i)
        m_cameras[i].reset(new VlcMediaPlayer(&m_vlcInst));
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
        connect(m_cameras[i].get(), SIGNAL(snapshotTaken(const QString&)),
                this, SLOT(frameAvailable(const QString&)));

    startStream(*m_cameras[0], m_ui.mainRenderer, &m_mainMedia);
    startStream(*m_cameras[1], m_ui.subRendererLeft, &m_leftMedia);
    startStream(*m_cameras[2], m_ui.subRendererRight, &m_rightMedia);
}

void HelelaniIPCamViewer::shutdownPlugin()
{
    for (auto& player : m_cameras)
        player->stop();
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
