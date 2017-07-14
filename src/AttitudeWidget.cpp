#include "AttitudeWidget.h"
#include "SVGHelper.h"
#include <QtGui/QPaintEvent>
#include <QtGui/QPainter>
#include <math.h>

AttitudeWidget::AttitudeWidget(QWidget* parent)
: QWidget(parent, Qt::WindowType::Widget),
  m_renderer(QString(":/Attitude_indicator_level_flight.svg"), this)
{
    m_horizonBounds = m_renderer.boundsOnElement("horizon");
    m_horizonclipBounds = m_renderer.boundsOnElement("horizonclip");
    m_maskBounds = m_renderer.boundsOnElement("mask");
    m_turretBounds = m_renderer.boundsOnElement("turret");
    m_tickdecoBounds = m_renderer.boundsOnElement("tickdeco");
}

void AttitudeWidget::paintEvent(QPaintEvent* ev)
{
    /* Maintain 1:1 aspect ratio */
    QRect contentRect = contentsRect();
    contentRect.setWidth(contentRect.height());
    QTransform scaleDown = RectToRect(contentsRect(), contentRect);

    /* Convert document rectangles to widget rectanges */
    QTransform svgToWidgetXf = RectToRect(m_renderer.viewBoxF(), contentRect);
    QRectF horizonBounds = svgToWidgetXf.mapRect(m_horizonBounds);
    QRectF horizonclipBounds = svgToWidgetXf.mapRect(m_horizonclipBounds);
    QRectF maskBounds = svgToWidgetXf.mapRect(m_maskBounds);
    QRectF turretBounds = svgToWidgetXf.mapRect(m_turretBounds);
    QRectF tickdecoBounds = svgToWidgetXf.mapRect(m_tickdecoBounds);

    /* Roll transform shared by horizon and ticks */
    QTransform rollXf =
            QTransform::fromTranslate(contentRect.width() / 2.f, contentRect.height() / 2.f).
            rotateRadians(m_rollAngle).
            translate(-contentRect.width() / 2.f, -contentRect.height() / 2.f);

    /* Map document space to widget space */
    QTransform horizonXf = RectToRect(contentRect, horizonBounds).
            translate(0, contentRect.height() * (m_pitchAngle / M_PI)) * rollXf;
    QTransform maskXf = RectToRect(contentRect, maskBounds);
    QTransform turretXf = RectToRect(contentRect, turretBounds);
    QTransform tickdecoXf = RectToRect(contentRect, tickdecoBounds) * rollXf;

    /* Draw components of widget */
    QPainter painter(this);
    painter.setClipRect(horizonclipBounds);
    painter.setTransform(scaleDown * horizonXf);
    m_renderer.render(&painter, "horizon", ev->rect());
    painter.setClipping(false);
    painter.setTransform(scaleDown * maskXf);
    m_renderer.render(&painter, "mask", ev->rect());
    painter.setTransform(scaleDown * turretXf);
    m_renderer.render(&painter, "turret", ev->rect());
    painter.setTransform(scaleDown * tickdecoXf);
    m_renderer.render(&painter, "tickdeco", ev->rect());
    painter.resetTransform();

    QString rollStr;
    rollStr.sprintf("Roll: %0.1f\xb0", -m_rollAngle * 180.0 / M_PI);
    painter.drawText(0, contentRect.height() - QFont().pointSize() - 3, rollStr);

    QString pitchStr;
    pitchStr.sprintf("Pitch: %0.1f\xb0", m_pitchAngle * 180.0 / M_PI);
    painter.drawText(0, contentRect.height(), pitchStr);
}

void AttitudeWidget::setPitch(double p)
{
    m_pitchAngle = p;
    update();
}

void AttitudeWidget::setRoll(double r)
{
    m_rollAngle = r;
    update();
}
