#ifndef HELELANIIPCAMVIEWER_HPP
#define HELELANIIPCAMVIEWER_HPP

#include <QtGui/QMainWindow>
#include <QtCore/QtCore>
#include <QtCore/QAbstractItemModel>
#include <QtGui/QCompleter>
#include <rqt_gui_cpp/plugin.h>
#include <ros/node_handle.h>
#include "ui_HelelaniIPCamViewer.h"
#include <mutex>
#include <QtGui/QListView>
#include <QtAV/QtAV.h>

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
    void frameAvailable(const QtAV::VideoFrame& frame);
    void subVideoClicked(SubVideoRendererWidget*, QMouseEvent*);

public:
    HelelaniIPCamViewer() = default;
    void initPlugin(qt_gui_cpp::PluginContext& context);
    void shutdownPlugin();
    void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

private:
    Ui::HelelaniIPCamViewer m_ui;
    QtAV::AVPlayer m_cameras[3]; // main, forwardhaz, reversehaz
    QWidget* m_widget;
    QString m_lastImagePath;
};

}

#endif // HELELANICOMMAND_HPP
