#ifndef SUBVIDEORENDERERWIDGET_H
#define SUBVIDEORENDERERWIDGET_H

#include <QtAVWidgets/GLWidgetRenderer2.h>

class SubVideoRendererWidget : public QtAV::GLWidgetRenderer2
{
    Q_OBJECT

public:
    explicit SubVideoRendererWidget(QWidget* parent = 0)
    : QtAV::GLWidgetRenderer2(parent)
    {
        QSizePolicy policy = sizePolicy();
        policy.setHeightForWidth(true);
        setSizePolicy(policy);
    }

    int heightForWidth(int width) const override
    {
        return width * 9 / 16;
    }

    void mousePressEvent(QMouseEvent* ev) override
    {
        QtAV::GLWidgetRenderer2::mousePressEvent(ev);
        emit clicked(this, ev);
    }

signals:
    void clicked(SubVideoRendererWidget* src, QMouseEvent* ev);
};

#endif // SUBVIDEORENDERERWIDGET_H
