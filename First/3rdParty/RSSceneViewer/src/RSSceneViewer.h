#ifndef RSSCENEVIEWER_H
#define RSSCENEVIEWER_H

#include "RSSceneViewer_global.h"

#include <QtCore/qobject.h>
//#include <QVector3D>
#include <QQuickVTKItem.h>

class RSSceneViewerPrivate;

typedef struct tagPoint3D
{
	tagPoint3D() { p[0] = 0; p[1] = 0; p[2] = 0; }
	tagPoint3D(double x, double y, double z) { p[0] = x; p[1] = y; p[2] = z; }
	~tagPoint3D() {}

    double* data() { return p; }

	double p[3];
	double n[3];
}Point3D,*PPoint3D;

class RSSCENEVIEWER_EXPORT RSSceneViewer : public QQuickVTKItem
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

    vtkUserData initializeVTK(vtkRenderWindow* renderWindow) override;

    Q_INVOKABLE void setBackGround(const QVariantList& color, double alpha = 1.0);
    Q_INVOKABLE void setBackGround(const QVariantList& color1, const QVariantList& color2, double alpha = 1.0);

    Q_INVOKABLE void setAxesVisiable(bool bVisiable);

    Q_INVOKABLE void clear();

    Q_INVOKABLE void setCameraMode(CAMERA_MODE mode);

    Q_INVOKABLE void setPointSize(double pointSize);
    Q_INVOKABLE void setPointOpacity(double opacityValue);

    Q_INVOKABLE void addPointCloud(const QVariantList& pointCloud, bool bCreateOctree = true);

    Q_INVOKABLE void addTrackPathPoint(const QVariantList& point);
    Q_INVOKABLE void addTrackPathPoints(const QVariantList& pointList);

    Q_INVOKABLE void lock();
    Q_INVOKABLE void unlock();
    Q_INVOKABLE void render();

private:
    void adjustCamera();
    void incrementalBuildOctree(const QVariantList& pointCloud);

private:
    Q_DECLARE_PRIVATE(RSSceneViewer)
    Q_DISABLE_COPY(RSSceneViewer)
    QScopedPointer<RSSceneViewerPrivate> d_ptr;
};

#endif // RSSCENEVIEWER_H
