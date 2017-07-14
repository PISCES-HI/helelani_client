#ifndef ATTITUDEWIDGET_H
#define ATTITUDEWIDGET_H

#include <QtSvg/QSvgRenderer>
#include <QtGui/QWidget>

class AttitudeWidget : public QWidget
{
    QSvgRenderer m_renderer;

    /* Element rectangles in SVG document space */
    QRectF m_horizonBounds;
    QRectF m_horizonclipBounds;
    QRectF m_maskBounds;
    QRectF m_turretBounds;
    QRectF m_tickdecoBounds;

    double m_pitchAngle = 0.0;
    double m_rollAngle = 0.0;
public:
    explicit AttitudeWidget(QWidget* parent=0);
    void paintEvent(QPaintEvent*);
    void setPitch(double p);
    void setRoll(double r);
};


#endif // ATTITUDEWIDGET_H
