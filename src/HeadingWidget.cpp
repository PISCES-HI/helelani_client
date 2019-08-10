#include "HeadingWidget.h"
#include "SVGHelper.h"
#include <QtGui/QPaintEvent>
#include <QtGui/QPainter>
#include <cmath>

HeadingWidget::HeadingWidget(QWidget* parent)
: QWidget(parent, Qt::WindowType::Widget),
  m_renderer(QString(":/Heading_indicator.svg"), this)
{
    m_foregroundBounds = m_renderer.boundsOnElement("foreground");
    m_compassBounds = m_renderer.boundsOnElement("compass");
    m_backgroundBounds = m_renderer.boundsOnElement("background");
}

void HeadingWidget::paintEvent(QPaintEvent* ev)
{
    /* Maintain 1:1 aspect ratio */
    QRect contentRect = contentsRect();
    contentRect.setWidth(contentRect.height());
    QTransform scaleDown = RectToRect(contentsRect(), contentRect);

    /* Convert document rectangles to widget rectanges */
    QTransform svgToWidgetXf = RectToRect(m_renderer.viewBoxF(), contentRect);
    QRectF foregroundBounds = svgToWidgetXf.mapRect(m_foregroundBounds);
    QRectF compassBounds = svgToWidgetXf.mapRect(m_compassBounds);
    QRectF backgroundBounds = svgToWidgetXf.mapRect(m_backgroundBounds);

    /* Transform used by compass */
    QTransform rollXf =
            QTransform::fromTranslate(contentRect.width() / 2.f, contentRect.height() / 2.f).
                    rotateRadians(-m_angle).
                    translate(-contentRect.width() / 2.f, -contentRect.height() / 2.f);

    /* Map document space to widget space */
    QTransform backgroundXf = RectToRect(contentRect, backgroundBounds);
    QTransform compassXf = RectToRect(contentRect, compassBounds) * rollXf;
    QTransform foregroundXf = RectToRect(contentRect, foregroundBounds);

    /* Draw components of widget */
    QPainter painter(this);
    painter.setTransform(scaleDown * backgroundXf / 1.25f);		//Changed by Alec: '/ 1.25f'
    m_renderer.render(&painter, "background");
    painter.setTransform(scaleDown * compassXf / 1.25f);		//Changed by Alec: '/ 1.25f'
    m_renderer.render(&painter, "compass");
    painter.setTransform(scaleDown * foregroundXf / 1.25f);		//Changed by Alec: '/ 1.25f'
    m_renderer.render(&painter, "foreground");
    painter.resetTransform();

    double heading = std::fmod(m_angle * 180.0 / M_PI, 360.0);
    if (heading < 0.0)
        heading += 360.0;

    QString headingStr;
    headingStr.sprintf("Heading: %0.1f\xb0", heading);
    painter.drawText(0, contentRect.height() - 3, headingStr);
}

void HeadingWidget::setHeading(double h)
{
    m_angle = h;
    update();
}
