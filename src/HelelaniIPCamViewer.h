#ifndef HELELANIIPCAMVIEWER_HPP
#define HELELANIIPCAMVIEWER_HPP

#include <QMainWindow>
#include <QtCore>
#include <QAbstractItemModel>
#include <QCompleter>
#include <rqt_gui_cpp/plugin.h>
#include <ros/node_handle.h>
#include "ui_HelelaniIPCamViewer.h"
#include <mutex>
#include <QListView>
#include <VLCQtCore/Instance.h>
#include <VLCQtCore/MediaPlayer.h>
#include <VLCQtWidgets/WidgetVideo.h>
#include <VLCQtCore/Media.h>

namespace helelani_client {

class HelelaniIPCamViewer : public rqt_gui_cpp::Plugin
{
    Q_OBJECT

public slots:
    void mainMenuRequested(QPoint pt);
    void mainShotSave();
    void leftMenuRequested(QPoint pt);
    void leftShotSave();
    void rightMenuRequested(QPoint pt);
    void rightShotSave();
    void frameAvailable(const QString& path);
    void subVideoClicked(SubVideoRendererWidget*, QMouseEvent*);

public:
    HelelaniIPCamViewer();
    void initPlugin(qt_gui_cpp::PluginContext& context);
    void shutdownPlugin();
    void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

private:
    void startStream(VlcMediaPlayer& player, VlcWidgetVideo* renderer, VlcMedia* media);
    Ui::HelelaniIPCamViewer m_ui;
    VlcInstance m_vlcInst;
    VlcMedia m_mainMedia;
    VlcMedia m_leftMedia;
    VlcMedia m_rightMedia;
    std::unique_ptr<VlcMediaPlayer> m_cameras[3]; // main, forwardhaz, reversehaz
    QWidget* m_widget;
    QString m_lastImagePath;
};

}

#endif // HELELANICOMMAND_HPP
