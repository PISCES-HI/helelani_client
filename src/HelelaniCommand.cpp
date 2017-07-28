#include "HelelaniCommand.h"
#include <pluginlib/class_list_macros.h>
#include <QtGui/QMenu>

namespace helelani_client {

HelelaniCommand::HelelaniCommand()
: m_cmdModel(this), m_cmdCompleter(&m_cmdModel, this),
  m_cmdValidator(m_cmdCompleter, this)
{
}

void HelelaniCommand::initPlugin(qt_gui_cpp::PluginContext& context)
{
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    m_widget = new QWidget();
    // extend the widget with all attributes and children from UI file
    m_ui.setupUi(m_widget);
    m_widget->setObjectName("HelelaniCommand");
    if (context.serialNumber() > 1)
        m_widget->setWindowTitle(m_widget->windowTitle() +
                                 " (" + QString::number(context.serialNumber()) + ")");
    // add widget to the user interface
    context.addWidget(m_widget);

    connect(m_ui.commandEdit, SIGNAL(returnPressed()),
            this, SLOT(runCommand()));
    m_ui.commandEdit->setCompleter(&m_cmdCompleter);
    m_ui.commandEdit->setValidator(&m_cmdValidator);

    connect(m_ui.commandTable, SIGNAL(currentCellChanged(int, int, int, int)),
            this, SLOT(cellChanged(int, int)));
    m_ui.commandTable->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(m_ui.commandTable, SIGNAL(customContextMenuRequested(QPoint)),
            this, SLOT(tableMenuRequested(QPoint)));

    ros::NodeHandle n;
    m_missionSub = n.subscribe("/helelani/mission", 10, &HelelaniCommand::missionCallback, this);
    m_cmdPub = n.advertise<helelani_common::DriveCommand>("/helelani/drive_cmd", 10);
    connect(&m_updateTimer, SIGNAL(timeout()), this, SLOT(updateTick()));
    m_updateTimer.start(100);
}

void HelelaniCommand::shutdownPlugin()
{
    m_missionSub.shutdown();
    m_cmdPub.shutdown();
    m_updateTimer.stop();
}

void HelelaniCommand::saveSettings(qt_gui_cpp::Settings& plugin_settings,
                                   qt_gui_cpp::Settings& instance_settings) const
{

}

void HelelaniCommand::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                                      const qt_gui_cpp::Settings& instance_settings)
{

}

void HelelaniCommand::missionCallback(const helelani_common::Mission& msg)
{
    if (msg.mission_active)
    {
        m_delayUp = msg.to_rover_delay;
        m_delayDown = msg.from_rover_delay;
        m_elapsedSeconds = msg.elapsed_seconds;
    }
    else
    {
        m_delayUp = 0.f;
        m_delayDown = 0.f;
        m_elapsedSeconds = 0;
    }
}

void HelelaniCommand::tableMenuRequested(QPoint pt)
{
    auto menu = new QMenu(m_widget);
    auto action = new QAction(QIcon::fromTheme("edit-clear"), "Clear Inactive", m_widget);
    connect(action, SIGNAL(triggered(bool)), this, SLOT(clearInactive()));
    menu->addAction(action);
    menu->popup(m_ui.commandTable->viewport()->mapToGlobal(pt));
}

void HelelaniCommand::clearInactive()
{
    for (int i=0 ; i<m_ui.commandTable->rowCount() ;)
    {
        auto item = static_cast<CommandTimerWidgetItem*>(m_ui.commandTable->item(i, 2));
        if (item->isDone())
        {
            m_ui.commandTable->removeRow(i);
            continue;
        }
        ++i;
    }
}

boost::shared_ptr<helelani_common::DriveCommand>
HelelaniCommand::parseMessage(const QString& str) const
{
    auto ret = boost::make_shared<helelani_common::DriveCommand>();
    using State = QValidator::State;
    QAbstractItemModel* model = m_cmdCompleter.model();
    QStringList list = m_cmdCompleter.splitPath(str);
    if (list.isEmpty())
        return ret;

    QModelIndex idx = model->index(0, 0);
    QModelIndexList matches =
            model->match(idx, Qt::DisplayRole, list[0], 1, Qt::MatchExactly);
    if (matches.isEmpty())
        return ret;

    auto cmd = static_cast<CommandItem*>(matches[0].internalPointer());
    cmd->toMsg(*ret, list);

    return ret;
}

void HelelaniCommand::runCommand()
{
    auto msg = parseMessage(m_ui.commandEdit->text());
    if (msg->cmd == helelani_common::DriveCommand::CMD_NONE)
        return;

    QApplication::postEvent(this, new CommandSubmitEvent(msg));
}

void HelelaniCommand::cellChanged(int row, int column)
{
    m_ui.commandEdit->clear();
    if (QTableWidgetItem* item = m_ui.commandTable->item(row, 3))
    {
        m_ui.commandEdit->insert(item->text());
        m_ui.commandEdit->setFocus();
    }
}

static QString TimeString(int elapsedSeconds)
{
    QString timerStr;
    int seconds = std::abs(elapsedSeconds);
    int minutes = seconds / 60;
    int hours = minutes / 60;
    minutes = minutes % 60;
    seconds = seconds % 60;
    timerStr.sprintf("%c%02d:%02d:%02d", elapsedSeconds >= 0 ? '+' : '-',
                     hours, minutes, seconds);
    return timerStr;
}

void HelelaniCommand::customEvent(QEvent* e)
{
    if (e->type() == int(QEvent::User) + 10)
    {
        /* Posted event to submit command message */
        auto evc = static_cast<CommandSubmitEvent*>(e);
        int row = m_ui.commandTable->rowCount();
        m_ui.commandTable->insertRow(row);
        m_ui.commandTable->setItem(row, 0, new QTableWidgetItem(QIcon::fromTheme("go-jump"), "Command"));
        m_ui.commandTable->setItem(row, 1, new QTableWidgetItem(TimeString(m_elapsedSeconds)));
        m_ui.commandTable->setItem(row, 2, new CommandTimerWidgetItem(evc->m_msg, m_delayUp, m_delayDown));
        m_ui.commandTable->setItem(row, 3, new QTableWidgetItem(m_ui.commandEdit->text().trimmed()));
        m_ui.commandEdit->clear();

        if (evc->m_msg->cmd == helelani_common::DriveCommand::CMD_KILL)
        {
            // Cancel pending messages
            for (int i=0 ; i<m_ui.commandTable->rowCount() ; ++i)
            {
                auto item = static_cast<CommandTimerWidgetItem*>(m_ui.commandTable->item(i, 2));
                item->cancel();
            }

            // Bypass delay for kill command (safety feature)
            m_cmdPub.publish(evc->m_msg);
        }
    }
}

void HelelaniCommand::updateTick()
{
    for (int i=0 ; i<m_ui.commandTable->rowCount() ; ++i)
    {
        auto item = static_cast<CommandTimerWidgetItem*>(m_ui.commandTable->item(i, 2));
        item->update(m_updateTimer.interval() / 1000.f, m_cmdPub);
    }
}

}

PLUGINLIB_EXPORT_CLASS(helelani_client::HelelaniCommand, rqt_gui_cpp::Plugin)
