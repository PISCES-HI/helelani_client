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

    ros::NodeHandle rosNode;
}

void HelelaniCommand::shutdownPlugin()
{
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

}

}

PLUGINLIB_EXPORT_CLASS(helelani_client::HelelaniCommand, rqt_gui_cpp::Plugin)
