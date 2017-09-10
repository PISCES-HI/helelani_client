#ifndef CAMERACONTROLWIDGET_H
#define CAMERACONTROLWIDGET_H

#include <QSvgRenderer>
#include <QWidget>
#include <QSlider>
#include <helelani_common/CameraCtrl.h>

class CameraControlWidget : public QWidget
{
    Q_OBJECT
    QSvgRenderer m_renderer;

    /* Element rectangles in SVG document space */
    QRectF m_foregroundBounds;
    QRectF m_handleBounds;
    QRectF m_backgroundBounds;

    QString m_name;
    double m_angleBias = 0.0;
    double m_angle = 0.0;
    int m_pitch = 0;
    int m_exposure = 0;
    bool m_mouseDown = false;
    void doAngleSet(QMouseEvent* ev);
    void doSignal();
public:
    explicit CameraControlWidget(QWidget* parent = 0);
    void paintEvent(QPaintEvent* ev) override;
    void setName(const QString& name);
    void setAngleBias(double a);
    void setAngle(double a);
    void mousePressEvent(QMouseEvent* ev) override;
    void mouseMoveEvent(QMouseEvent* ev) override;
    void mouseReleaseEvent(QMouseEvent* ev) override;

signals:
    void camUpdate(const helelani_common::CameraCtrl& msg);

public slots:
    void pitchSliderChanged(int value);
    void exposureSliderChanged(int value);
};

#endif //CAMERACONTROLWIDGET_H
