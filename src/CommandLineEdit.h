#ifndef COMMANDLINEEDIT_H
#define COMMANDLINEEDIT_H

#include <QLineEdit>
#include <QApplication>
#include <QCompleter>
#include <QKeyEvent>

class CommandLineEdit : public QLineEdit
{
    Q_OBJECT
public:
    explicit CommandLineEdit(QWidget* parent = 0)
    : QLineEdit(parent)
    {
        connect(this, SIGNAL(textEdited(QString)), this, SLOT(slotTextEdited()));
    }

    void focusInEvent(QFocusEvent* e) override
    {
        QLineEdit::focusInEvent(e);
        // force completion when user brings focus to widget
        if (text().isEmpty())
            completer()->complete();
    }

    void customEvent(QEvent* e) override
    {
        QLineEdit::customEvent(e);
        // force completion after text is deleted
        completer()->complete();
    }

    void keyPressEvent(QKeyEvent* e) override
    {
        if (e->key() == Qt::Key_Escape)
            emit escapePressed();
        QLineEdit::keyPressEvent(e);
    }

Q_SIGNALS:
    void escapePressed();

public slots:
    void slotTextEdited()
    {
        if (text().isEmpty())
            QApplication::postEvent(this, new QEvent(QEvent::User));
    };
};

#endif // COMMANDLINEEDIT_H
