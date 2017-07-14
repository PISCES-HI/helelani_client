#ifndef HEADINGWIDGET_H
#define HEADINGWIDGET_H

#include <QtSvg/QSvgRenderer>
#include <QtGui/QWidget>

class HeadingWidget : public QWidget
{
    QSvgRenderer m_renderer;

    /* Element rectangles in SVG document space */
    QRectF m_foregroundBounds;
    QRectF m_compassBounds;
    QRectF m_backgroundBounds;

    double m_angle = 0.0;
public:
    explicit HeadingWidget(QWidget* parent=0);
    void paintEvent(QPaintEvent*);
    void setHeading(double h);
};


#endif // HEADINGWIDGET_H
