#include "CameraControlWidget.h"
#include "SVGHelper.h"
#include <QtGui/QPaintEvent>
#include <QtGui/QPainter>
#include <cmath>

CameraControlWidget::CameraControlWidget(QWidget* parent)
: QWidget(parent),
  m_renderer(QString(":/Camera_control.svg"), this)
{
    m_foregroundBounds = m_renderer.boundsOnElement("foreground");
    m_handleBounds = m_renderer.boundsOnElement("handle");
    m_backgroundBounds = m_renderer.boundsOnElement("background");
}

void CameraControlWidget::paintEvent(QPaintEvent* ev)
{
    /* Maintain 1:1 aspect ratio */
    QRect contentRect = contentsRect();
    contentRect.setWidth(contentRect.height());
    QTransform scaleDown = RectToRect(contentsRect(), contentRect);
    int leftOff = (contentsRect().width() - contentsRect().height()) / 2;
    QTransform centerTrans = QTransform::fromTranslate(leftOff, 0);

    /* Convert document rectangles to widget rectanges */
    QTransform svgToWidgetXf = RectToRect(m_renderer.viewBoxF(), contentRect);
    QRectF foregroundBounds = svgToWidgetXf.mapRect(m_foregroundBounds);
    QRectF handleBounds = svgToWidgetXf.mapRect(m_handleBounds);
    QRectF backgroundBounds = svgToWidgetXf.mapRect(m_backgroundBounds);

    /* Transform used by compass */
    QTransform rollXf =
            QTransform::fromTranslate(contentRect.width() / 2.f, contentRect.height() / 2.f).
                    rotateRadians(m_angle).
                    translate(-contentRect.width() / 2.f, -contentRect.height() / 2.f);

    /* Map document space to widget space */
    QTransform backgroundXf = RectToRect(contentRect, backgroundBounds);
    QTransform handleXf = RectToRect(contentRect, handleBounds) * rollXf;
    QTransform foregroundXf = RectToRect(contentRect, foregroundBounds);

    /* Draw components of widget */
    QPainter painter(this);
    painter.setTransform(scaleDown * backgroundXf * centerTrans);
    m_renderer.render(&painter, "background");
    painter.setTransform(scaleDown * handleXf * centerTrans);
    m_renderer.render(&painter, "handle");
    painter.setTransform(scaleDown * foregroundXf * centerTrans);
    m_renderer.render(&painter, "foreground");
    painter.resetTransform();

    QString headingStr;
    headingStr.sprintf("%s\nHeading: %0.1f\xb0\nPitch: %d\xb0",
                       m_name.toUtf8().data(),
                       m_angle * 180.0 / M_PI, m_pitch);
    painter.setPen(QColor(Qt::white));
    QRect textRect = contentRect;
    textRect.translate(leftOff, 0);
    QTextOption textOpts(Qt::AlignCenter);
    painter.drawText(textRect, headingStr, textOpts);
}

void CameraControlWidget::setName(const QString& name)
{
    m_name = name;
    update();
}

void CameraControlWidget::setAngle(double a)
{
    m_angle = std::max(std::min(a, M_PI / 2.0), -M_PI / 2.0);
    update();
}

void CameraControlWidget::doAngleSet(QMouseEvent* ev)
{
    QRect contentRect = contentsRect();
    double x = (contentRect.width() != 0 ? ev->x() / double(contentRect.width()) : 0.0) - 0.5;
    double y = (contentRect.height() != 0 ? ev->y() / double(contentRect.height()) : 0.0) - 0.5;
    setAngle(std::atan2(x, -y));
    doSignal();
}

void CameraControlWidget::doSignal()
{
    helelani_common::CameraCtrl msg;
    msg.pan = float(m_angle * 180.0 / M_PI) + 90.f;
    msg.tilt = float(m_pitch + 90);
    emit camUpdate(msg);
}

void CameraControlWidget::mousePressEvent(QMouseEvent* ev)
{
    m_mouseDown = true;
    doAngleSet(ev);
}

void CameraControlWidget::mouseMoveEvent(QMouseEvent* ev)
{
    if (m_mouseDown)
        doAngleSet(ev);
}

void CameraControlWidget::mouseReleaseEvent(QMouseEvent*)
{
    m_mouseDown = false;
}

void CameraControlWidget::pitchSliderChanged(int value)
{
    m_pitch = value;
    update();
    doSignal();
}
