#ifndef SVGHELPER_H
#define SVGHELPER_H

#include <QtGui/QTransform>

/* Used for generating transform matrices to map coordinate space */
static QTransform RectToRect(const QRectF& from, const QRectF& to)
{
    QPolygonF orig(from);
    orig.pop_back();
    QPolygonF resize(to);
    resize.pop_back();
    QTransform ret;
    QTransform::quadToQuad(orig, resize, ret);
    return ret;
}

#endif // SVGHELPER_H
