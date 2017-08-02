#ifndef SUBVIDEORENDERERWIDGET_H
#define SUBVIDEORENDERERWIDGET_H

#include <VLCQtWidgets/WidgetVideo.h>

class SubVideoRendererWidget : public VlcWidgetVideo
{
    Q_OBJECT

public:
    explicit SubVideoRendererWidget(QWidget* parent = 0)
    : VlcWidgetVideo(parent)
    {
        QSizePolicy policy = sizePolicy();
        policy.setHeightForWidth(true);
        setSizePolicy(policy);
    }

    int heightForWidth(int width) const override
    {
        printf("HFW %d\n", width * 9 / 16);
        return width * 9 / 16;
    }

    void mousePressEvent(QMouseEvent* ev) override
    {
        VlcWidgetVideo::mousePressEvent(ev);
        emit clicked(this, ev);
    }

signals:
    void clicked(SubVideoRendererWidget* src, QMouseEvent* ev);
};

#endif // SUBVIDEORENDERERWIDGET_H
