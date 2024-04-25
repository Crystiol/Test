#include "RSSceneViewer.h"
#include "RSMultiTouchInteractorStyle.h"

#include <vtkSmartPointer.h>
#include <vtkNamedColors.h>
#include <vtkMath.h>
#include <vtkUnsignedCharArray.h>
#include <vtkLookupTable.h>

#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkProperty.h>
#include <vtkCamera.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkAxesActor.h>

#include <vtkSphereSource.h>
#include <vtkCubeSource.h>

#include <vtkVertexGlyphFilter.h>
#include <vtkOctreePointLocator.h>
#include <vtkIncrementalOctreePointLocator.h>

#include <vtkPointSource.h>

#include <vtkAppendPolyData.h>
#include <vtkPointData.h>
#include <vtkPolyLine.h>
#include <vtkSplineFilter.h>
#include <vtkParametricSpline.h>
#include <vtkParametricFunctionSource.h>
#include <vtkGlyph3DMapper.h>
#include <vtkTransform.h>
#include <vtkColorTransferFunction.h>
#include <vtkCleanPolyData.h>

#include <QVector>
#include <mutex>

enum ACTOR_INDEX
{
	AI_POINTCLOUD,
	AI_TRACKPATH,
	AI_TRACKPOINTS,
	AI_NUM
};

class RSSceneViewerPrivate
{
    Q_DECLARE_PUBLIC(RSSceneViewer)
public:
    RSSceneViewerPrivate(RSSceneViewer* viewer) : q_ptr(viewer)
	{
        Q_Q(RSSceneViewer);

        render_ = vtkSmartPointer<vtkRenderer>::New();
        render_->UseFXAAOn();

        reset();

		axes_ = vtkSmartPointer<vtkAxesActor>::New();
		axes_->AxisLabelsOff();
		axes_->SetConeRadius(0.1);
		render_->AddActor(axes_);

        //vtkSmartPointer<JMMultiTouchInteractorStyle> interactorStyle = vtkSmartPointer<JMMultiTouchInteractorStyle>::New();
        //renderWindow_->GetInteractor()->SetInteractorStyle(interactorStyle);

		render_->ResetCamera();
	}

    ~RSSceneViewerPrivate()
	{
	}

    void reset()
	{
		bOrignal_ = true;
        cameraMode_ = RSSceneViewer::CM_THIRDVIEW;
		cameraPos_ = { 0.0,0.0,0.0 };
		cameraLastPos_ = { 0.0,0.0,0.0 };
		lastTrackPoint = { 0.0,0.0,0.0 };

        trackPoints_ = vtkSmartPointer<vtkPoints>::New();
		octree_ = vtkSmartPointer<vtkIncrementalOctreePointLocator>::New();

        vtkNew<vtkNamedColors> colors;

        actors_[AI_POINTCLOUD] = vtkSmartPointer<vtkActor>::New();
        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        actors_[AI_POINTCLOUD]->SetMapper(mapper);
        actors_[AI_POINTCLOUD]->GetProperty()->SetPointSize(3.0);
        render_->AddActor(actors_[AI_POINTCLOUD]);

        actors_[AI_TRACKPOINTS] = vtkSmartPointer<vtkActor>::New();
        actors_[AI_TRACKPOINTS]->GetProperty()->SetColor(colors->GetColor3d("Fuchsia").GetData());
        actors_[AI_TRACKPOINTS]->GetProperty()->SetPointSize(5.0);
        render_->AddActor(actors_[AI_TRACKPOINTS]);

        actors_[AI_TRACKPATH] = vtkSmartPointer<vtkActor>::New();
        actors_[AI_TRACKPATH]->GetProperty()->SetLineWidth(2.0);
        actors_[AI_TRACKPATH]->GetProperty()->SetColor(colors->GetColor3d("Lime").GetData());
        render_->AddActor(actors_[AI_TRACKPATH]);
	}

	bool bOrignal_;
    RSSceneViewer::CAMERA_MODE cameraMode_;
	Point3D	cameraPos_;
	Point3D	cameraLastPos_;
	Point3D	lastTrackPoint;

	vtkSmartPointer<vtkPoints> trackPoints_;
	vtkSmartPointer<vtkAxesActor> axes_;
	std::array<vtkSmartPointer<vtkActor>, AI_NUM> actors_;
	vtkSmartPointer<vtkIncrementalOctreePointLocator> octree_;
	vtkSmartPointer<vtkRenderer> render_;
    vtkSmartPointer<vtkRenderWindow> renderWindow_;

    std::mutex lock_;

private:
    RSSceneViewer* q_ptr;
};

RSSceneViewer::RSSceneViewer(QWidget* parent) : d_ptr(new RSSceneViewerPrivate(this))
{
    Q_D(RSSceneViewer);
}

RSSceneViewer::~RSSceneViewer()
{

}

QQuickVTKItem::vtkUserData RSSceneViewer::initializeVTK(vtkRenderWindow *renderWindow)
{
    Q_D(RSSceneViewer);

    d_ptr->renderWindow_ = renderWindow;

    renderWindow->AddRenderer(d_ptr->render_);
}

void RSSceneViewer::setBackGround(const QVariantList& color, double alpha)
{
    vtkColor3d bkColor(color.at(0).toDouble(),color.at(1).toDouble(),color.at(2).toDouble());
    d_ptr->render_->SetBackground(bkColor.GetData());
	d_ptr->render_->SetBackgroundAlpha(alpha);

    render();
}

void RSSceneViewer::setBackGround(const QVariantList& color1, const QVariantList& color2, double alpha)
{
    Q_D(RSSceneViewer);

    vtkColor3d bkColor1(color1.at(0).toDouble(),color1.at(1).toDouble(),color1.at(2).toDouble());
    vtkColor3d bkColor2(color2.at(0).toDouble(),color2.at(1).toDouble(),color2.at(2).toDouble());
	d_ptr->render_->GradientBackgroundOn();
    d_ptr->render_->SetBackground(bkColor1.GetData());
    d_ptr->render_->SetBackground2(bkColor2.GetData());
	d_ptr->render_->SetBackgroundAlpha(alpha);

    render();
}

void RSSceneViewer::setAxesVisiable(bool bVisiable)
{
    Q_D(RSSceneViewer);
	d_ptr->axes_->SetVisibility(bVisiable);

    render();
}

void RSSceneViewer::clear()
{
    Q_D(RSSceneViewer);

	d_ptr->render_->RemoveAllViewProps();

    d_ptr->reset();

	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transform->Identity();
	d_ptr->axes_->SetUserTransform(transform);

	d_ptr->render_->AddActor(d_ptr->axes_);
	
	d_ptr->render_->GetActiveCamera()->SetPosition(0, 0, 1);
	d_ptr->render_->GetActiveCamera()->SetFocalPoint(0, 0, 0);
	d_ptr->render_->GetActiveCamera()->SetViewUp(0, 1, 0);

	d_ptr->render_->ResetCamera();

    render();
}

void RSSceneViewer::setCameraMode(CAMERA_MODE mode)
{
    Q_D(RSSceneViewer);
	
	d_ptr->cameraMode_ = mode;

    adjustCamera();
	
	//d_ptr->render_->ResetCamera();
    render();
}

void RSSceneViewer::setPointSize(double pointSize)
{
    Q_D(RSSceneViewer);

    d_ptr->actors_[AI_POINTCLOUD]->GetProperty()->SetPointSize(pointSize);

    render();
}

void RSSceneViewer::setPointOpacity(double opacityValue)
{
    Q_D(RSSceneViewer);

	d_ptr->actors_[AI_POINTCLOUD]->GetProperty()->SetOpacity(opacityValue);
    render();
}

void RSSceneViewer::addPointCloud(const QVariantList& pointCloud, bool bCreateOctree)
{
    Q_D(RSSceneViewer);

    if(pointCloud.empty())
        return;

	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    for (const QVariant &variant : pointCloud)
    {
        QVariantList innerList = variant.toList();
        Point3D vec3d(innerList.at(0).toDouble(), innerList.at(1).toDouble(), innerList.at(2).toDouble());
        points->InsertNextPoint(vec3d.data());
    }
	
	static double step = 0.0;
	
	/*Point3D center;
	center.p[0] = vtkMath::Random(-1+step, 1+step);
	center.p[1] = vtkMath::Random(-1+step, 1+step);
	center.p[2] = vtkMath::Random(-1+step, 1+step);*/

	static double c = 0.0;
	Point3D center = { c, 0, 0 };
	c += 0.4;

	step++;

	vtkSmartPointer<vtkPointSource> pointSource = vtkSmartPointer<vtkPointSource>::New();
	pointSource->SetNumberOfPoints(10000);
    pointSource->SetCenter(center.data());
	pointSource->SetRadius(0.2);
	pointSource->Update();

    //points = pointSource->GetOutput()->GetPoints();

	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	polydata->SetPoints(points);
	
#if 0
	//点云抽稀
	int nPointNum = points->GetNumberOfPoints();
	vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
	for (int i = 0; i < nPointNum; ++i)
	{
		vertices->InsertNextCell(1);
		vertices->InsertCellPoint(i);
	}
	polydata->SetVerts(vertices);

	vtkSmartPointer<vtkCleanPolyData> cleaner = vtkSmartPointer<vtkCleanPolyData>::New();
	cleaner->SetInputData(polydata);
	cleaner->SetTolerance(0.05);
	cleaner->Update();

	int nPts = cleaner->GetOutput()->GetPoints()->GetNumberOfPoints();
	int nCells = cleaner->GetOutput()->GetNumberOfCells();
#endif

	vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
    vertexFilter->SetInputData(polydata);
    //vertexFilter->SetInputConnection(cleaner->GetOutputPort());
	vertexFilter->Update();

	vtkSmartPointer<vtkAppendPolyData> appendFilter = vtkSmartPointer<vtkAppendPolyData>::New();
	vtkPolyData* curPolydata = dynamic_cast<vtkPolyData*>(d_ptr->actors_[AI_POINTCLOUD]->GetMapper()->GetInputAsDataSet());
	if (curPolydata)
	{
		appendFilter->AddInputData(curPolydata);
	}

	appendFilter->AddInputData(vertexFilter->GetOutput());
	appendFilter->Update();

	vtkSmartPointer<vtkLookupTable> colorLookupTable = vtkSmartPointer<vtkLookupTable>::New();
	colorLookupTable->SetHueRange(0.63, 0.32);
	colorLookupTable->SetTableRange(0.0, 20.0);
	colorLookupTable->Build();

	vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->SetName("Colors");

	vtkPoints* newpoints = appendFilter->GetOutput()->GetPoints();
	for (vtkIdType i = 0; i < newpoints->GetNumberOfPoints(); i++)
	{
		double val = newpoints->GetPoint(i)[0];

		double dcolor[3];
		colorLookupTable->GetColor(val, dcolor);

		unsigned char color[3];
		for (unsigned int j = 0; j < 3; j++)
		{
			color[j] = 255 * dcolor[j] / 1.0;
		}

		colors->InsertNextTypedTuple(color);
	}

	appendFilter->GetOutput()->GetPointData()->SetScalars(colors);

	if (bCreateOctree)
	{
        incrementalBuildOctree(pointCloud);
	}
	
	d_ptr->actors_[AI_POINTCLOUD]->GetMapper()->SetInputConnection(appendFilter->GetOutputPort());
	
	double addPointsBound[6];
	polydata->GetBounds(addPointsBound);
	Point3D addPointsCenter = {
		(addPointsBound[0] + addPointsBound[1]) / 2,
		(addPointsBound[2] + addPointsBound[3]) / 2,
		(addPointsBound[4] + addPointsBound[5]) / 2
	};

	double max_length = std::max(addPointsBound[1] - addPointsBound[0], std::max(addPointsBound[3] - addPointsBound[2], addPointsBound[5] - addPointsBound[4]));

    QVariantList trackPoint;
    trackPoint.append(addPointsCenter.p[0]);
    trackPoint.append(addPointsCenter.p[1]);
    trackPoint.append(addPointsCenter.p[2]);
    addTrackPathPoint(trackPoint);

	if (d_ptr->bOrignal_)
	{
		d_ptr->cameraPos_ = addPointsCenter;
		d_ptr->bOrignal_ = false;

		d_ptr->render_->GetActiveCamera()->ParallelProjectionOn();
		d_ptr->render_->ResetCamera();
	}
	else
	{
		d_ptr->cameraLastPos_ = d_ptr->cameraPos_;

		if (d_ptr->cameraMode_ == CM_FIRSTVIEW)
		{
            d_ptr->render_->GetActiveCamera()->SetPosition(d_ptr->cameraPos_.data());
            d_ptr->render_->GetActiveCamera()->SetFocalPoint(addPointsCenter.data());
			d_ptr->render_->ResetCameraClippingRange();
		}
		
		d_ptr->cameraPos_ = addPointsCenter;
	}

    render();
}

void RSSceneViewer::addTrackPathPoint(const QVariantList &point)
{
    Q_D(RSSceneViewer);

	bool bFirstPoint = (d_ptr->trackPoints_->GetNumberOfPoints() == 0);

    Point3D newTrackPoint(point.at(0).toDouble(), point.at(1).toDouble(), point.at(2).toDouble());
    d_ptr->trackPoints_->InsertNextPoint(newTrackPoint.data());

	vtkIdType nPts = d_ptr->trackPoints_->GetNumberOfPoints();

	vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
	for (int i = 0; i < nPts; ++i)
	{
		vertices->InsertNextCell(1);
		vertices->InsertCellPoint(i);
	}

	vtkSmartPointer<vtkPolyData> pointsdata = vtkSmartPointer<vtkPolyData>::New();
	pointsdata->SetPoints(d_ptr->trackPoints_);
	pointsdata->SetVerts(vertices);

	vtkSmartPointer<vtkPolyDataMapper> pointMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	pointMapper->SetInputData(pointsdata);

	d_ptr->actors_[AI_TRACKPOINTS]->SetMapper(pointMapper);

	if (bFirstPoint)
	{
		d_ptr->lastTrackPoint = newTrackPoint;
		return;
	}

	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    points->InsertNextPoint(d_ptr->lastTrackPoint.data());
    points->InsertNextPoint(newTrackPoint.data());

	vtkSmartPointer<vtkPolyLine> polyline = vtkSmartPointer<vtkPolyLine>::New();
	polyline->GetPointIds()->SetNumberOfIds(2);
	for (unsigned int i = 0; i < 2; ++i) 
	{
		polyline->GetPointIds()->SetId(i, i);
	}

	vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
	lines->InsertNextCell(polyline);

	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	polydata->SetPoints(points);
	polydata->SetLines(lines);
	
    if (d_ptr->trackPoints_->GetNumberOfPoints() == 2)
	{
		vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		mapper->SetInputData(polydata);
		d_ptr->actors_[AI_TRACKPATH]->SetMapper(mapper);
	}
	else
	{
		vtkSmartPointer<vtkAppendPolyData> appendFilter = vtkSmartPointer<vtkAppendPolyData>::New();
		vtkPolyData* curPolydata = dynamic_cast<vtkPolyData*>(d_ptr->actors_[AI_TRACKPATH]->GetMapper()->GetInputAsDataSet());
		if (curPolydata)
		{
			appendFilter->AddInputData(curPolydata);
		}
		appendFilter->AddInputData(polydata);
		appendFilter->Update();

		vtkSmartPointer<vtkParametricSpline> spline = vtkSmartPointer<vtkParametricSpline>::New();
		spline->SetPoints(appendFilter->GetOutput()->GetPoints());

		vtkSmartPointer<vtkParametricFunctionSource> parametricFunctionSource = vtkSmartPointer<vtkParametricFunctionSource>::New();
		parametricFunctionSource->SetParametricFunction(spline);
		parametricFunctionSource->SetUResolution(200);

		d_ptr->actors_[AI_TRACKPATH]->GetMapper()->SetInputConnection(parametricFunctionSource->GetOutputPort());
	}

    render();

	d_ptr->lastTrackPoint = newTrackPoint;
}

void RSSceneViewer::addTrackPathPoints(const QVariantList& pointList)
{
    Q_D(RSSceneViewer);

    QVector<Point3D> trackpoints;
    for (const QVariant &variant : pointList)
    {
        QVariantList innerList = variant.toList();
        Point3D vec3d(innerList.at(0).toDouble(), innerList.at(1).toDouble(), innerList.at(2).toDouble());
        trackpoints.append(vec3d);
    }

    int nPts = trackpoints.size();
	if (nPts == 0)
	{
		return;
	}

    d_ptr->trackPoints_->Initialize();
	for (auto& it : trackpoints)
	{
        d_ptr->trackPoints_->InsertNextPoint(it.data());
	}

    vtkIdType nPointsNum = d_ptr->trackPoints_->GetNumberOfPoints();

    vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
    for (int i = 0; i < nPointsNum; ++i)
    {
        vertices->InsertNextCell(1);
        vertices->InsertCellPoint(i);
    }

    vtkSmartPointer<vtkPolyData> pointsdata = vtkSmartPointer<vtkPolyData>::New();
    pointsdata->SetPoints(d_ptr->trackPoints_);
    pointsdata->SetVerts(vertices);

    vtkSmartPointer<vtkPolyDataMapper> pointMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    pointMapper->SetInputData(pointsdata);
    d_ptr->actors_[AI_TRACKPOINTS]->SetMapper(pointMapper);

	vtkSmartPointer<vtkParametricSpline> spline = vtkSmartPointer<vtkParametricSpline>::New();
	spline->SetPoints(d_ptr->trackPoints_);

	vtkSmartPointer<vtkParametricFunctionSource> parametricFunctionSource = vtkSmartPointer<vtkParametricFunctionSource>::New();
	parametricFunctionSource->SetParametricFunction(spline);
	parametricFunctionSource->SetUResolution(200);

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(parametricFunctionSource->GetOutputPort());
    d_ptr->actors_[AI_TRACKPATH]->SetMapper(mapper);

    render();

    d_ptr->lastTrackPoint = trackpoints[nPts - 1];
}

void RSSceneViewer::lock()
{
    Q_D(RSSceneViewer);
    d_ptr->lock_.lock();
}

void RSSceneViewer::unlock()
{
    Q_D(RSSceneViewer);
    d_ptr->lock_.unlock();
}

void RSSceneViewer::render()
{
    Q_D(RSSceneViewer);

    if(d_ptr->renderWindow_)
    {
        d_ptr->renderWindow_->Render();
    }
}

void RSSceneViewer::adjustCamera()
{
    Q_D(RSSceneViewer);

	if (d_ptr->cameraMode_ == CM_FIRSTVIEW)
	{
		if (d_ptr->cameraPos_.p[0] != d_ptr->cameraLastPos_.p[0] ||
			d_ptr->cameraPos_.p[1] != d_ptr->cameraLastPos_.p[1] ||
			d_ptr->cameraPos_.p[2] != d_ptr->cameraLastPos_.p[2])
		{
            d_ptr->render_->GetActiveCamera()->SetPosition(d_ptr->cameraLastPos_.data());
            d_ptr->render_->GetActiveCamera()->SetFocalPoint(d_ptr->cameraPos_.data());
			d_ptr->render_->ResetCameraClippingRange();
		}
	}
	else
	{
		if (!d_ptr->actors_[AI_POINTCLOUD])
		{
			return;
		}

		double bounds[6];
		d_ptr->actors_[AI_POINTCLOUD]->GetBounds(bounds);

		// 获取边界框的尺寸
		double width = bounds[1] - bounds[0];
		double height = bounds[3] - bounds[2];
		double depth = bounds[5] - bounds[4];

		// 计算边界框的中心点
		double center[3] = {
			(bounds[0] + bounds[1]) / 2,
			(bounds[2] + bounds[3]) / 2,
			(bounds[4] + bounds[5]) / 2
		};

		// 计算相机的新位置
		double distance = 2 * std::max(width, std::max(height, depth)) / (2 * tan(vtkMath::RadiansFromDegrees(0.5 * d_ptr->render_->GetActiveCamera()->GetViewAngle())));
		double cameraPosition[3] = { center[0], center[1], center[2] + distance };

		// 将相机定位到新位置并调整焦点
		d_ptr->render_->GetActiveCamera()->SetPosition(cameraPosition);
		d_ptr->render_->GetActiveCamera()->SetFocalPoint(center);

		// 更新渲染器
		d_ptr->render_->ResetCameraClippingRange();

		d_ptr->render_->ResetCamera();
	}
}

void RSSceneViewer::incrementalBuildOctree(const QVariantList& pointCloud)
{
    if (pointCloud.empty())
	{
		return;
	}

	if (d_ptr->octree_->GetNumberOfPoints() == 0)
	{
		vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
        for (const QVariant &variant : pointCloud)
        {
            QVariantList innerList = variant.toList();
            Point3D vec3d(innerList.at(0).toDouble(), innerList.at(1).toDouble(), innerList.at(2).toDouble());
            points->InsertNextPoint(vec3d.data());
        }

		vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
		polydata->SetPoints(points);

		d_ptr->octree_->SetDataSet(polydata);
		d_ptr->octree_->BuildLocator();
	}

    for (const QVariant &variant : pointCloud)
    {
        QVariantList innerList = variant.toList();
        Point3D vec3d(innerList.at(0).toDouble(), innerList.at(1).toDouble(), innerList.at(2).toDouble());
        d_ptr->octree_->InsertNextPoint(vec3d.data());
    }

	vtkSmartPointer<vtkCleanPolyData> cleanFilter = vtkSmartPointer<vtkCleanPolyData>::New();
	cleanFilter->SetInputData(d_ptr->octree_->GetDataSet());
	cleanFilter->Update();
	
	d_ptr->octree_->SetDataSet(cleanFilter->GetOutput());

	int nLevels = d_ptr->octree_->GetLevel();
	vtkSmartPointer<vtkPolyData> newpolydata = vtkSmartPointer<vtkPolyData>::New();
	d_ptr->octree_->GenerateRepresentation(0, newpolydata);
}
