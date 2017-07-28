#include "DriveTrackpad.h"
#include <QMouseEvent>

DriveTrackpad::DriveTrackpad(QWidget *parent)
: QGroupBox(parent)
{
}

void DriveTrackpad::mousePressEvent(QMouseEvent *event)
{
    emit _mousePressEvent(this, event);
}

void DriveTrackpad::mouseReleaseEvent(QMouseEvent *event)
{
    emit _mouseReleaseEvent(this, event);
}

void DriveTrackpad::mouseMoveEvent(QMouseEvent *event)
{
    emit _mouseMoveEvent(this, event);
}