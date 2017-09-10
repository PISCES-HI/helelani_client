#ifndef DRIVETRACKPAD_H
#define DRIVETRACKPAD_H

#include <QGroupBox>

class DriveTrackpad : public QGroupBox
{
    Q_OBJECT
public:
    explicit DriveTrackpad(QWidget *parent = 0);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);

signals:
    void _mousePressEvent(DriveTrackpad *sender, QMouseEvent *event);
    void _mouseReleaseEvent(DriveTrackpad *sender, QMouseEvent *event);
    void _mouseMoveEvent(DriveTrackpad *sender, QMouseEvent* ev);
};

#endif // DRIVETRACKPAD_H
