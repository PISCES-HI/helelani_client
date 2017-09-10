#ifndef HEADINGWIDGET_H
#define HEADINGWIDGET_H

#include <QSvgRenderer>
#include <QWidget>

class HeadingWidget : public QWidget
{
    Q_OBJECT
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
