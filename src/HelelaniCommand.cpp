#include "HelelaniCommand.h"
#include <pluginlib/class_list_macros.h>

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

    QObject::connect(m_ui.commandEdit, SIGNAL(returnPressed()),
                     this, SLOT(runCommand()));
    m_ui.commandEdit->setCompleter(&m_cmdCompleter);
    m_ui.commandEdit->setValidator(&m_cmdValidator);

    QObject::connect(m_ui.commandTable, SIGNAL(currentCellChanged(int, int, int, int)),
                     this, SLOT(cellChanged(int, int)));

    ros::NodeHandle rosNode;
    m_cmdPub = rosNode.advertise<helelani_common::DriveCommand>("/helelani/drive_cmd", 10);
}

void HelelaniCommand::shutdownPlugin()
{
    m_cmdPub.shutdown();
}

void HelelaniCommand::saveSettings(qt_gui_cpp::Settings& plugin_settings,
                                   qt_gui_cpp::Settings& instance_settings) const
{

}

void HelelaniCommand::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                                      const qt_gui_cpp::Settings& instance_settings)
{

}

void HelelaniCommand::runCommand()
{
    helelani_common::DriveCommand msg = parseMessage(m_ui.commandEdit->text());
    if (msg.cmd == helelani_common::DriveCommand::CMD_NONE)
        return;

    QApplication::postEvent(this, new CommandSubmitEvent(msg));
}

void HelelaniCommand::cellChanged(int row, int column)
{
    QTableWidgetItem* item = m_ui.commandTable->item(row, 2);
    m_ui.commandEdit->clear();
    m_ui.commandEdit->insert(item->text());
    m_ui.commandEdit->setFocus();
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
        m_ui.commandTable->setItem(row, 1, new QTableWidgetItem("0.0"));
        m_ui.commandTable->setItem(row, 2, new QTableWidgetItem(m_ui.commandEdit->text().trimmed()));
        m_ui.commandEdit->clear();
        m_cmdPub.publish(evc->m_msg);
    }
}

}

PLUGINLIB_EXPORT_CLASS(helelani_client::HelelaniCommand, rqt_gui_cpp::Plugin)
