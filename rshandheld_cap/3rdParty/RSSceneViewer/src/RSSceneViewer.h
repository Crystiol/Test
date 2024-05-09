#ifndef RSSCENEVIEWER_H
#define RSSCENEVIEWER_H

#include "RSSceneViewer_global.h"

#include <QtCore/qobject.h>
#include <QVTKWidget.h>
#include <QVariant>
#include <vtkQuaternion.h>

class RSSceneViewerPrivate;

typedef struct tagPoint3D
{
    tagPoint3D() { p[0] = 0; p[1] = 0; p[2] = 0; }
    tagPoint3D(double x, double y, double z) { p[0] = x; p[1] = y; p[2] = z; }
    ~tagPoint3D() {}

    double* data() { return p; }

    double p[3];
    double c[3];
}Point3D,*PPoint3D;

class RSSCENEVIEWER_EXPORT RSSceneViewer :  public QVTKWidget
{
    Q_OBJECT
    Q_ENUMS(CAMERA_MODE)

public:
    enum CAMERA_MODE
    {
        CM_FIRSTVIEW,
        CM_THIRDVIEW
    };

public:
    RSSceneViewer(QWidget* parent = nullptr);
    ~RSSceneViewer();

    void setBackGround(const QVariantList& color, double alpha = 1.0);
    void setBackGround(const QVariantList& color1, const QVariantList& color2, double alpha = 1.0);

    void setAxesVisiable(bool bVisiable);

    void clear();

    void setCameraMode(CAMERA_MODE mode);

    void setPointSize(double pointSize);
    void setPointOpacity(double opacityValue);

    void addPointCloud(const QVariantList& pos, const QVariantList& rotate, const QVariantList& pointCloud, const QVariantList& pointColors, bool bCreateOctree = false);

    void addTrackPathPoint(const QVariantList& pos, const QVariantList& rotate);
    void addTrackPathPoints(const QVariantList& pointList);

    void lock();
    void unlock();
    void render();

private:
    void adjustCamera();
    void incrementalBuildOctree(const QVariantList& pointCloud);

private:
    Q_DECLARE_PRIVATE(RSSceneViewer)
    Q_DISABLE_COPY(RSSceneViewer)
    QScopedPointer<RSSceneViewerPrivate> d_ptr;
};

#endif // RSSCENEVIEWER_H
