#ifndef HELELANICOMMAND_HPP
#define HELELANICOMMAND_HPP

#include <QtGui/QMainWindow>
#include <QtCore/QtCore>
#include <QtCore/QAbstractItemModel>
#include <QtGui/QCompleter>
#include <rqt_gui_cpp/plugin.h>
#include <ros/node_handle.h>
#include "ui_HelelaniCommand.h"
#include <mutex>
#include <QtGui/QListView>

namespace helelani_client {

class CommandItem
{
    int m_row;
    CommandItem* m_parent;
    CommandItem* m_next;
public:
    CommandItem(int row, CommandItem* parent, CommandItem* next)
            : m_row(row), m_parent(parent), m_next(next) {}
    int row() const { return m_row; }
    CommandItem* parent() const { return m_parent; }
    virtual bool forward() const { return false; }
    virtual int rowCount() const
    { return m_next ? (m_next->forward() ? m_next->rowCount() : 1) : 0; }
    virtual CommandItem* index(int row)
    { return m_next ? (m_next->forward() ? m_next->index(row) : m_next) : nullptr; }
    virtual QVariant data() const { return {}; }
};

class ArgumentEnum : public CommandItem
{
    QVariant m_rep;
public:
    ArgumentEnum(int row, CommandItem* parent, CommandItem* next,
                 const QVariant& rep)
            : CommandItem(row, parent, next), m_rep(rep) {}
    virtual QVariant data() const override { return m_rep; };
};

class NumberArg : public CommandItem
{
public:
    NumberArg(CommandItem* parent, CommandItem* next) :
            CommandItem(0, parent, next) {}
    QVariant data() const override { return "<number>"; }
};

class DriveDirArg : public CommandItem
{
    ArgumentEnum m_forward;
    ArgumentEnum m_backward;
public:
    DriveDirArg(CommandItem* parent, CommandItem* next) :
            CommandItem(0, parent, next),
            m_forward(0, parent, next, "forward"),
            m_backward(1, parent, next, "backward") {}
    bool forward() const override { return true; }
    int rowCount() const override { return 2; }
    CommandItem* index(int row) override
    {
        switch (row)
        {
        case 0:
            return &m_forward;
        case 1:
            return &m_backward;
        default:
            return nullptr;
        }
    }
};

class DriveUnitArg : public CommandItem
{
    ArgumentEnum m_seconds;
    ArgumentEnum m_rotations;
    ArgumentEnum m_meters;
public:
    DriveUnitArg(CommandItem* parent, CommandItem* next) :
            CommandItem(0, parent, next),
            m_seconds(0, parent, next, "seconds"),
            m_rotations(1, parent, next, "rotations"),
            m_meters(2, parent, next, "meters") {}
    bool forward() const override { return true; }
    int rowCount() const override { return 3; }
    CommandItem* index(int row) override
    {
        switch (row)
        {
        case 0:
            return &m_seconds;
        case 1:
            return &m_rotations;
        case 2:
            return &m_meters;
        default:
            return nullptr;
        }
    }
};

class DriveCommand : public CommandItem
{
    DriveDirArg m_dir;
    NumberArg m_val;
    DriveUnitArg m_unit;
    NumberArg m_throttle;
public:
    DriveCommand(int row, CommandItem* parent) :
            CommandItem(row, parent, &m_dir),
            m_dir(this, &m_val),
            m_val(&m_dir, &m_unit),
            m_unit(&m_val, &m_throttle),
            m_throttle(&m_unit, nullptr) {}
    QVariant data() const { return "drive"; }
};

class TurnDirArg : public CommandItem
{
    ArgumentEnum m_left;
    ArgumentEnum m_right;
public:
    TurnDirArg(CommandItem* parent, CommandItem* next) :
            CommandItem(0, parent, next),
            m_left(0, parent, next, "left"),
            m_right(1, parent, next, "right") {}
    bool forward() const override { return true; }
    int rowCount() const override { return 2; }
    CommandItem* index(int row) override
    {
        switch (row)
        {
        case 0:
            return &m_left;
        case 1:
            return &m_right;
        default:
            return nullptr;
        }
    }
};

class TurnUnitArg : public CommandItem
{
    ArgumentEnum m_seconds;
    ArgumentEnum m_degrees;
    ArgumentEnum m_rotations;
public:
    TurnUnitArg(CommandItem* parent, CommandItem* next) :
            CommandItem(0, parent, next),
            m_seconds(0, parent, next, "seconds"),
            m_degrees(1, parent, next, "degrees"),
            m_rotations(2, parent, next, "rotations") {}
    bool forward() const override { return true; }
    int rowCount() const override { return 3; }
    CommandItem* index(int row) override
    {
        switch (row)
        {
        case 0:
            return &m_seconds;
        case 1:
            return &m_degrees;
        case 2:
            return &m_rotations;
        default:
            return nullptr;
        }
    }
};

class TurnCommand : public CommandItem
{
    TurnDirArg m_dir;
    NumberArg m_val;
    TurnUnitArg m_unit;
    NumberArg m_throttle;
public:
    TurnCommand(int row, CommandItem* parent) :
            CommandItem(row, parent, &m_dir),
            m_dir(this, &m_val),
            m_val(&m_dir, &m_unit),
            m_unit(&m_val, &m_throttle),
            m_throttle(&m_unit, nullptr) {}
    QVariant data() const { return "turn"; }
};

class SadlDirArg : public CommandItem
{
    ArgumentEnum m_up;
    ArgumentEnum m_down;
public:
    SadlDirArg(CommandItem* parent, CommandItem* next) :
            CommandItem(0, parent, next),
            m_up(0, parent, next, "up"),
            m_down(1, parent, next, "down") {}
    bool forward() const override { return true; }
    int rowCount() const override { return 2; }
    CommandItem* index(int row) override
    {
        switch (row)
        {
        case 0:
            return &m_up;
        case 1:
            return &m_down;
        default:
            return nullptr;
        }
    }
};

class SadlCommand : public CommandItem
{
    SadlDirArg m_dir;
public:
    SadlCommand(int row, CommandItem* parent) :
            CommandItem(row, parent, &m_dir),
            m_dir(this, nullptr) {}
    QVariant data() const { return "sadl"; }
};

class KillCommand : public CommandItem
{
public:
    KillCommand(int row, CommandItem* parent) :
            CommandItem(row, parent, nullptr) {}
    QVariant data() const { return "kill"; }
};

class RootCommand : public CommandItem
{
    DriveCommand m_drive;
    TurnCommand m_turn;
    SadlCommand m_sadl;
    KillCommand m_kill;
public:
    RootCommand() : CommandItem(0, nullptr, nullptr),
                    m_drive(0, this),
                    m_turn(1, this),
                    m_sadl(2, this),
                    m_kill(3, this) {}
    int rowCount() const override { return 4; }
    CommandItem* index(int row) override
    {
        switch (row)
        {
        case 0:
            return &m_drive;
        case 1:
            return &m_turn;
        case 2:
            return &m_sadl;
        case 3:
            return &m_kill;
        default:
            return nullptr;
        }
    }
};

class CommandModel : public QAbstractItemModel
{
    Q_OBJECT

public:
    explicit CommandModel(QObject* parent = 0) : QAbstractItemModel(parent) {}
    QVariant data(const QModelIndex& index, int role) const override
    {
        if (!index.isValid())
            return {};

        if (role != Qt::DisplayRole && role != Qt::EditRole)
            return QVariant();

        return static_cast<CommandItem*>(index.internalPointer())->data();
    }
    Qt::ItemFlags flags(const QModelIndex& index) const override
    {
        if (!index.isValid())
            return Qt::ItemFlag::NoItemFlags;
        return QAbstractItemModel::flags(index);
    }
    QModelIndex index(int row, int column,
                      const QModelIndex& p) const override
    {
        if (!hasIndex(row, column, p))
            return {};

        CommandItem* parent = p.isValid() ?
            static_cast<CommandItem*>(p.internalPointer()) :
            &m_rootItem;

        CommandItem* child = parent->index(row);
        if (child)
            return createIndex(row, column, child);
        else
            return {};
    }
    QModelIndex parent(const QModelIndex& index) const override
    {
        if (!index.isValid())
            return {};

        CommandItem* child = static_cast<CommandItem*>(index.internalPointer());
        CommandItem* parent = child->parent();

        if (parent == &m_rootItem)
            return {};

        return createIndex(parent->row(), 0, parent);
    }
    int rowCount(const QModelIndex& p) const override
    {
        CommandItem* parent = p.isValid() ?
            static_cast<CommandItem*>(p.internalPointer()) :
            &m_rootItem;

        return parent->rowCount();
    }
    int columnCount(const QModelIndex& parent) const override
    {
        return 1;
    }

private:
    mutable RootCommand m_rootItem;
};

class CommandCompleter : public QCompleter
{
    Q_OBJECT
public:
    CommandCompleter(CommandModel* model, QObject* parent = nullptr) :
            QCompleter(model, parent)
    {
        setCaseSensitivity(Qt::CaseInsensitive);
        setCompletionMode(CompletionMode::UnfilteredPopupCompletion);
    }
    QStringList splitPath(const QString& path) const override
    {
        QStringList list = path.split(' ', QString::SkipEmptyParts);
        if (path.endsWith(' '))
            list.push_back(" ");
        return list;
    }
    QString pathFromIndex(const QModelIndex& index) const override
    {
        QString path = static_cast<QLineEdit*>(widget())->text();
        QStringList list = path.split(' ', QString::SkipEmptyParts);
        if (!path.endsWith(' '))
            list.pop_back();
        QString ret;
        for (const QString& str : list)
            ret += str + ' ';
        ret += index.data().toString();
        return ret;
    }
};

class HelelaniCommand : public rqt_gui_cpp::Plugin
{
    Q_OBJECT

public:
    HelelaniCommand();
    void initPlugin(qt_gui_cpp::PluginContext& context);
    void shutdownPlugin();
    void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

public slots:
    void runCommand();

private:
    Ui::HelelaniCommand m_ui;
    QWidget* m_widget = nullptr;
    CommandModel m_cmdModel;
    CommandCompleter m_cmdCompleter;
};

}

#endif // HELELANICOMMAND_HPP
