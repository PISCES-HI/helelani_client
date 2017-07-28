#ifndef HELELANICOMMAND_HPP
#define HELELANICOMMAND_HPP

#include <QtGui/QMainWindow>
#include <QtCore/QtCore>
#include <QtCore/QAbstractItemModel>
#include <QtGui/QCompleter>
#include <QtGui/QTableWidgetItem>
#include <rqt_gui_cpp/plugin.h>
#include <ros/node_handle.h>
#include "ui_HelelaniCommand.h"
#include <mutex>
#include <QtGui/QListView>
#include <helelani_common/DriveCommand.h>
#include <helelani_common/Mission.h>

namespace helelani_client {

class CommandSubmitEvent : public QEvent
{
    friend class HelelaniCommand;
    boost::shared_ptr<helelani_common::DriveCommand> m_msg;
public:
    CommandSubmitEvent(boost::shared_ptr<helelani_common::DriveCommand> msg)
    : QEvent(QEvent::Type(int(QEvent::User) + 10)), m_msg(std::move(msg)) {}
};

class CommandTimerWidgetItem : public QTableWidgetItem
{
    boost::shared_ptr<helelani_common::DriveCommand> m_msg;
    float m_timerUp;
    float m_timerDown;
    enum class Icon
    {
        Uninitialized,
        Up,
        Down
    };
    Icon m_curIcon = Icon::Uninitialized;
    bool m_done = false;

    void _updateDisplay()
    {
        if (m_timerUp > 0.f)
        {
            if (m_curIcon != Icon::Up)
            {
                setIcon(QIcon::fromTheme("go-up"));
                m_curIcon = Icon::Up;
            }
            setText(QString::number(m_timerUp, 'f', 1));
        }
        else
        {
            if (m_curIcon != Icon::Down)
            {
                setIcon(QIcon::fromTheme("go-down"));
                m_curIcon = Icon::Down;
            }
            setText(QString::number(m_timerDown, 'f', 1));
        }
    }

public:
    CommandTimerWidgetItem(boost::shared_ptr<helelani_common::DriveCommand> msg,
                           float timerUp, float timerDown)
    : m_msg(std::move(msg)), m_timerUp(timerUp), m_timerDown(timerDown)
    {
        _updateDisplay();
    }

    void update(float dt, ros::Publisher& pub)
    {
        if (m_done)
            return;

        if (m_timerUp > 0.f)
        {
            m_timerUp -= dt;
            if (m_timerUp < 0.f)
            {
                m_timerDown += m_timerUp;
                if (m_timerDown < 0.f)
                    m_timerDown = 0.f;
                m_timerUp = 0.f;
            }
        }

        if (m_timerUp > 0.f)
        {
            _updateDisplay();
            return;
        }
        else if (m_msg)
        {
            pub.publish(*m_msg);
            m_msg.reset();
        }

        if (m_timerDown > 0.f)
        {
            m_timerDown -= dt;
            if (m_timerDown < 0.f)
                m_timerDown = 0.f;
        }

        if (m_timerUp <= 0.f && m_timerDown <= 0.f)
            m_done = true;

        _updateDisplay();
    }

    void cancel()
    {
        m_timerUp = 0.f;
        m_timerDown = 0.f;
        m_msg.reset();
    }

    bool isDone() const { return m_done; }
};

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
    virtual QVariant editData() const { return data(); }
    virtual Qt::ItemFlags flags() const
    { return Qt::ItemIsSelectable | Qt::ItemIsEnabled; }
    virtual QValidator::State validate(const QString& str) const
    { return QValidator::State::Invalid; }
    virtual void toMsg(helelani_common::DriveCommand& msg,
                       const QStringList& args) const {}
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
    mutable QString m_enteredString;
    QDoubleValidator m_doubleValidator;
public:
    NumberArg(CommandItem* parent, CommandItem* next) :
            CommandItem(0, parent, next) {}
    QVariant data() const override { return "<number>"; }
    QVariant editData() const override { return m_enteredString; }
    Qt::ItemFlags flags() const override { return Qt::ItemIsSelectable; }
    QValidator::State validate(const QString& str) const override
    {
        m_enteredString = str;
        int pos = 0;
        return m_doubleValidator.validate(m_enteredString, pos);
    }
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
    static uint8_t DirEnum(const QString& str)
    {
        if (!str.compare("forward", Qt::CaseInsensitive))
            return helelani_common::DriveCommand::DIR_FORWARD;
        if (!str.compare("backward", Qt::CaseInsensitive))
            return helelani_common::DriveCommand::DIR_BACKWARD;
        return helelani_common::DriveCommand::DIR_FORWARD;
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
    static uint8_t UnitEnum(const QString& str)
    {
        if (!str.compare("seconds", Qt::CaseInsensitive))
            return helelani_common::DriveCommand::UNIT_SECONDS;
        if (!str.compare("rotations", Qt::CaseInsensitive))
            return helelani_common::DriveCommand::UNIT_ROTATIONS;
        if (!str.compare("meters", Qt::CaseInsensitive))
            return helelani_common::DriveCommand::UNIT_METERS;
        return helelani_common::DriveCommand::UNIT_SECONDS;
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
    void toMsg(helelani_common::DriveCommand& msg,
               const QStringList& args) const
    {
        if (args.size() < 4)
            return;
        msg.cmd = helelani_common::DriveCommand::CMD_DRIVE;
        msg.dir = DriveDirArg::DirEnum(args[1]);
        msg.value = args[2].toFloat();
        msg.unit = DriveUnitArg::UnitEnum(args[3]);
        if (args.size() >= 5)
            msg.throttle = args[4].toFloat();
    }
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
    static uint8_t DirEnum(const QString& str)
    {
        if (!str.compare("left", Qt::CaseInsensitive))
            return helelani_common::DriveCommand::DIR_LEFT;
        if (!str.compare("right", Qt::CaseInsensitive))
            return helelani_common::DriveCommand::DIR_RIGHT;
        return helelani_common::DriveCommand::DIR_LEFT;
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
    static uint8_t UnitEnum(const QString& str)
    {
        if (!str.compare("seconds", Qt::CaseInsensitive))
            return helelani_common::DriveCommand::UNIT_SECONDS;
        if (!str.compare("degrees", Qt::CaseInsensitive))
            return helelani_common::DriveCommand::UNIT_DEGREES;
        if (!str.compare("rotations", Qt::CaseInsensitive))
            return helelani_common::DriveCommand::UNIT_ROTATIONS;
        return helelani_common::DriveCommand::UNIT_SECONDS;
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
    void toMsg(helelani_common::DriveCommand& msg,
               const QStringList& args) const
    {
        if (args.size() < 4)
            return;
        msg.cmd = helelani_common::DriveCommand::CMD_TURN;
        msg.dir = TurnDirArg::DirEnum(args[1]);
        msg.value = args[2].toFloat();
        msg.unit = TurnUnitArg::UnitEnum(args[3]);
        if (args.size() >= 5)
            msg.throttle = args[4].toFloat();
    }
};

class SadlDirArg : public CommandItem
{
    ArgumentEnum m_up;
    ArgumentEnum m_down;
    ArgumentEnum m_autolevel;
public:
    SadlDirArg(CommandItem* parent, CommandItem* next) :
            CommandItem(0, parent, next),
            m_up(0, parent, next, "up"),
            m_down(1, parent, next, "down"),
            m_autolevel(2, parent, next, "autolevel") {}
    bool forward() const override { return true; }
    int rowCount() const override { return 3; }
    CommandItem* index(int row) override
    {
        switch (row)
        {
        case 0:
            return &m_up;
        case 1:
            return &m_down;
        case 2:
            return &m_autolevel;
        default:
            return nullptr;
        }
    }
    static uint8_t DirEnum(const QString& str)
    {
        if (!str.compare("up", Qt::CaseInsensitive))
            return helelani_common::DriveCommand::DIR_UP;
        if (!str.compare("down", Qt::CaseInsensitive))
            return helelani_common::DriveCommand::DIR_DOWN;
        if (!str.compare("autolevel", Qt::CaseInsensitive))
            return helelani_common::DriveCommand::DIR_AUTOLEVEL;
        return helelani_common::DriveCommand::DIR_UP;
    }
};

class SadlCommand : public CommandItem
{
    SadlDirArg m_dir;
    NumberArg m_val;
public:
    SadlCommand(int row, CommandItem* parent) :
            CommandItem(row, parent, &m_dir),
            m_dir(this, &m_val),
            m_val(&m_dir, nullptr) {}
    QVariant data() const { return "sadl"; }
    void toMsg(helelani_common::DriveCommand& msg,
               const QStringList& args) const
    {
        if (args.size() < 3)
            return;
        msg.cmd = helelani_common::DriveCommand::CMD_SADL;
        msg.dir = SadlDirArg::DirEnum(args[1]);
        msg.value = args[2].toFloat();
    }
};

class KillCommand : public CommandItem
{
public:
    KillCommand(int row, CommandItem* parent) :
            CommandItem(row, parent, nullptr) {}
    QVariant data() const { return "kill"; }
    void toMsg(helelani_common::DriveCommand& msg,
               const QStringList& args) const
    {
        msg.cmd = helelani_common::DriveCommand::CMD_KILL;
    }
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

        CommandItem* cmdItem =
                static_cast<CommandItem*>(index.internalPointer());
        switch (role)
        {
        case Qt::DisplayRole:
            return cmdItem->data();
        case Qt::EditRole:
            return cmdItem->editData();
        default:
            return QVariant();
        }
    }
    Qt::ItemFlags flags(const QModelIndex& index) const override
    {
        if (!index.isValid())
            return Qt::ItemFlag::NoItemFlags;
        return static_cast<CommandItem*>(index.internalPointer())->flags();
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
    CommandCompleter(CommandModel* model, QObject* parent = nullptr)
    : QCompleter(model, parent)
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
        if (!path.endsWith(' ') && list.size())
            list.pop_back();
        QString ret;
        for (const QString& str : list)
            ret += str + ' ';
        ret += index.data().toString();
        return ret;
    }
};

class CommandValidator : public QValidator
{
    Q_OBJECT
    CommandCompleter& m_completer;
public:
    CommandValidator(CommandCompleter& completer, QObject* parent = nullptr) :
    QValidator(parent), m_completer(completer)
    {}

    State validate(QString& str, int& pos) const override
    {
        State ret = State::Acceptable;
        QAbstractItemModel* model = m_completer.model();
        QStringList list = m_completer.splitPath(str);
        QModelIndex idx = model->index(0, 0);

        for (const QString& substr : list)
        {
            if (substr == " ")
                continue;
            if (ret == State::Intermediate)
                return State::Invalid;

            QModelIndexList matches =
                    model->match(idx, Qt::DisplayRole, substr, 1, Qt::MatchStartsWith);
            if (matches.isEmpty())
            {
                auto item = static_cast<CommandItem*>(idx.internalPointer());
                if (!item)
                    return State::Invalid;
                ret = item->validate(substr);
                if (ret == State::Invalid)
                    return State::Invalid;
                idx = idx.child(0, 0);
                continue;
            }

            if (matches[0].data(Qt::EditRole).toString().compare
                    (substr, Qt::CaseInsensitive) != 0)
                ret = State::Intermediate;
            else
                idx = matches[0].child(0, 0);
        }

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
    void cellChanged(int row, int column);
    void updateTick();
    void tableMenuRequested(QPoint pt);
    void clearInactive();

private:
    Ui::HelelaniCommand m_ui;
    QWidget* m_widget = nullptr;
    CommandModel m_cmdModel;
    CommandCompleter m_cmdCompleter;
    CommandValidator m_cmdValidator;
    ros::Subscriber m_missionSub;
    ros::Publisher m_cmdPub;
    int m_elapsedSeconds = 0;
    float m_delayUp = 0.f;
    float m_delayDown = 0.f;
    QTimer m_updateTimer;

    void missionCallback(const helelani_common::Mission& msg);
    boost::shared_ptr<helelani_common::DriveCommand> parseMessage(const QString& str) const;
    void customEvent(QEvent* e) override;
};

}

#endif // HELELANICOMMAND_HPP
