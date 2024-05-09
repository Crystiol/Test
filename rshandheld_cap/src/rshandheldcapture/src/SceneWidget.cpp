#include "SceneWidget.h"
#include "./ui_scenewidget.h"
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QOpenGLWindow>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <QDir>

SceneWidget::SceneWidget(QWidget *parent)
    : QWidget(parent), viewer_(new RSSceneViewer(this)), control_(new RSDeviceControl)
    , ui(new Ui::SceneWidget), updateFlag_(true)
{
    ui->setupUi(this);

    initUI();

    QDir workDir;
    workDir.mkdir("/tmp/saveimg/");
    workDir.mkdir("/tmp/record/");

    camera_nh_.reset(new ros::NodeHandle);
    it_.reset(new image_transport::ImageTransport(*camera_nh_));

    lidar_track_nh_.reset(new ros::NodeHandle);
    lidar_cloud_nh_.reset(new ros::NodeHandle);

    ros_timer_ = new QTimer(this);
    connect(ros_timer_, &QTimer::timeout, this, &SceneWidget::onUpdate);
    ros_timer_->start(1000);

    camera_sub_ = it_->subscribeCamera("/rshandheld_cam/image_raw", 1000, &SceneWidget::cameraCallback, this);
    track_sub_ = lidar_track_nh_->subscribe("/mapping_odom", 1000, &SceneWidget::trackCallback, this);
    cloud_sub_ = lidar_cloud_nh_->subscribe("/cloud_registered_dense_world", 1000, &SceneWidget::cloudCallback, this);
}

SceneWidget::~SceneWidget()
{
    delete ui;
}

void SceneWidget::initUI()
{
    QPushButton* btnAddCloud = new QPushButton(this);
    btnAddCloud->setObjectName(QString::fromUtf8("btnAddCloud"));
    btnAddCloud->setFixedSize(80, 30);
    btnAddCloud->setText(tr("AddCloud"));
    connect(btnAddCloud, &QPushButton::clicked, [&] {
        //viewer_->SetPointOpacity(0.5);
        QVariantList points;
        points.append(QVariant::fromValue(QVariantList{ 0.2, 0.0, 0.0 }));
        points.append(QVariant::fromValue(QVariantList{ 0.5, 0.0, 0.0 }));
        points.append(QVariant::fromValue(QVariantList{ 1.0, 0.5, 0.0 }));
        points.append(QVariant::fromValue(QVariantList{ 1.5, 1.0, 0.0 }));
        points.append(QVariant::fromValue(QVariantList{ 2.5, 1.5, 0.0 }));
        points.append(QVariant::fromValue(QVariantList{ 1.5, 2.5, 0.0 }));
        points.append(QVariant::fromValue(QVariantList{ 0.5, 1.0, 0.0 }));

        //viewer_->addPointCloud(points, false);
    });
    QPushButton* btnAddTrack = new QPushButton(this);
    btnAddTrack->setObjectName(QString::fromUtf8("btnAddTrack"));
    connect(btnAddTrack, &QPushButton::clicked, [&] {

        // static int index = 0;
        // static std::vector<Point3D> points;
        // points.push_back(Point3D(0.2, 0.0, 0.0));
        // points.push_back(Point3D(0.5, 0.0, 0.0));
        // points.push_back(Point3D(1.0, 0.5, 0.0));
        // points.push_back(Point3D(1.5, 1.0, 0.0));
        // points.push_back(Point3D(2.5, 1.5, 0.0));
        // points.push_back(Point3D(1.5, 2.5, 0.0));
        // points.push_back(Point3D(0.5, 1.0, 0.0));

        // if (index > 6)
        // {
        //     return;
        // }
        // viewer_->addTrackPathPoints(points);

        QVariantList points;
        points.append(QVariant::fromValue(QVariantList{ 0.2, 0.0, 0.0 }));
        points.append(QVariant::fromValue(QVariantList{ 0.5, 0.0, 0.0 }));
        points.append(QVariant::fromValue(QVariantList{ 1.0, 0.5, 0.0 }));
        points.append(QVariant::fromValue(QVariantList{ 1.5, 1.0, 0.0 }));
        points.append(QVariant::fromValue(QVariantList{ 2.5, 1.5, 0.0 }));
        points.append(QVariant::fromValue(QVariantList{ 1.5, 2.5, 0.0 }));
        points.append(QVariant::fromValue(QVariantList{ 0.5, 1.0, 0.0 }));
        viewer_->addTrackPathPoints(points);
    });
    btnAddTrack->setFixedSize(80, 30);
    btnAddTrack->setText(tr("AddTrack"));

    QPushButton* btnChange = new QPushButton(this);
    btnChange->setObjectName(QString::fromUtf8("btnChange"));
    btnChange->setFixedSize(80, 30);
    btnChange->setText(tr("Change"));
    connect(btnChange, &QPushButton::clicked, [&] {
        viewer_->setBackGround(QVariantList{0.3, 0.4, 0.6},QVariantList{0.1, 0.2, 0.4}, 0.5);
        //viewer_->SetPointOpacity(0.5);
        //viewer_->SetPointSize(3.0);
    });

    QPushButton* btnChangeCamera = new QPushButton(this);
    btnChangeCamera->setObjectName(QString::fromUtf8("btnChangeCamera"));
    btnChangeCamera->setFixedSize(80, 30);
    btnChangeCamera->setText(tr("Camera"));
    connect(btnChangeCamera, &QPushButton::clicked, [&] {
        static RSSceneViewer::CAMERA_MODE m = RSSceneViewer::CM_THIRDVIEW;
        m = (m == RSSceneViewer::CM_FIRSTVIEW) ? RSSceneViewer::CM_THIRDVIEW : RSSceneViewer::CM_FIRSTVIEW;
        viewer_->setCameraMode(m);
    });

    QPushButton* btnClear = new QPushButton(this);
    btnClear->setObjectName(QString::fromUtf8("btnClear"));
    btnClear->setFixedSize(80, 30);
    btnClear->setText(tr("Clear"));
    connect(btnClear, &QPushButton::clicked, [&] {
        //index = 0;
        viewer_->clear();
    });

    QPushButton* btnStartCamera = new QPushButton(this);
    btnStartCamera->setObjectName(QString::fromUtf8("btnStartCamera"));
    btnStartCamera->setFixedSize(80, 30);
    btnStartCamera->setText(tr("Start Camera"));
    connect(btnStartCamera, &QPushButton::clicked, [&] {
        control_->startCamera();
    });
    QPushButton* btnStopCamera = new QPushButton(this);
    btnStopCamera->setObjectName(QString::fromUtf8("btnStopCamera"));
    btnStopCamera->setFixedSize(80, 30);
    btnStopCamera->setText(tr("Stop Camera"));
    connect(btnStopCamera, &QPushButton::clicked, [&] {
        control_->stopCamera();
    });
    QPushButton* btnStartRecord = new QPushButton(this);
    btnStartRecord->setObjectName(QString::fromUtf8("btnStartRecord"));
    btnStartRecord->setFixedSize(80, 30);
    btnStartRecord->setText(tr("Start Record"));
    connect(btnStartRecord, &QPushButton::clicked, [&] {
        control_->startRecordControl();
    });
    QPushButton* btnStopRecord = new QPushButton(this);
    btnStopRecord->setObjectName(QString::fromUtf8("btnStopRecord"));
    btnStopRecord->setFixedSize(80, 30);
    btnStopRecord->setText(tr("Stop Record"));
    connect(btnStopRecord, &QPushButton::clicked, [&] {
        control_->stopRecordControl();
    });
    QPushButton* btnStartMapping = new QPushButton(this);
    btnStartMapping->setObjectName(QString::fromUtf8("btnStartMapping"));
    btnStartMapping->setFixedSize(80, 30);
    btnStartMapping->setText(tr("Start Mapping"));
    connect(btnStartMapping, &QPushButton::clicked, [&] {
        control_->startMapping();
    });
    QPushButton* btnStopMapping = new QPushButton(this);
    btnStopMapping->setObjectName(QString::fromUtf8("btnStopMapping"));
    btnStopMapping->setFixedSize(80, 30);
    btnStopMapping->setText(tr("Stop Mapping"));
    connect(btnStopMapping, &QPushButton::clicked, [&] {
        control_->stopMapping();
    });

    QHBoxLayout* sub_layout = new QHBoxLayout;
    sub_layout->setSpacing(100);
    sub_layout->addWidget(btnAddCloud);
    sub_layout->setSpacing(20);
    sub_layout->addWidget(btnAddTrack);
    sub_layout->setSpacing(20);
    sub_layout->addWidget(btnChange);
    sub_layout->setSpacing(20);
    sub_layout->addWidget(btnChangeCamera);
    sub_layout->setSpacing(20);
    sub_layout->addWidget(btnClear);

    QHBoxLayout* sub_layout1 = new QHBoxLayout;
    sub_layout1->addWidget(btnStartCamera);
    sub_layout1->addWidget(btnStopCamera);
    sub_layout1->addWidget(btnStartRecord);
    sub_layout1->addWidget(btnStopRecord);
    sub_layout1->addWidget(btnStartMapping);
    sub_layout1->addWidget(btnStopMapping);

    QVBoxLayout* layout = new QVBoxLayout;
    layout->addWidget(viewer_.get());
    layout->addLayout(sub_layout);
    layout->addLayout(sub_layout1);
    setLayout(layout);
}

void SceneWidget::topicCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Receive: [%s]", msg->data.c_str());
}

void SceneWidget::cameraCallback(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr &info)
{
    try {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        static int i = 0;
        QString qstrImageName = QString("/tmp/saveimg/IMAGE_%1.jpg").arg(i++);
        cv::imwrite(qstrImageName.toStdString().c_str(), frame);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from ROS message to OpenCV image: %s", e.what());
    }
}

inline int32_t findChannelIndex(const sensor_msgs::PointCloud2ConstPtr& cloud, const std::string& channel)
{
    for (size_t i = 0; i < cloud->fields.size(); ++i)
    {
        if (cloud->fields[i].name == channel)
        {
            return i;
        }
    }

    return -1;
}

bool hasColor(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    int32_t ri = findChannelIndex(cloud, "r");
    int32_t gi = findChannelIndex(cloud, "g");
    int32_t bi = findChannelIndex(cloud, "b");
    if (ri == -1 || gi == -1 || bi == -1)
    {
        return false;
    }

    if (cloud->fields[ri].datatype == sensor_msgs::PointField::FLOAT32)
    {
        return true;
    }

    return false;
}

inline bool validateFloats(float val)
{
    return !(std::isnan(val) || std::isinf(val));
}

void SceneWidget::cloudCallback(const sensor_msgs::PointCloud2Ptr& cloud)
{
    ROS_INFO("Receive from lidar");

    cloud_lock_.lock();
    new_clouds_.enqueue(cloud);
    cloud_lock_.unlock();
}

void SceneWidget::trackCallback(const nav_msgs::Odometry::ConstPtr &message)
{
    ROS_INFO("Receive Track");

    // viewer_->lock();

    // QVariantList pos;
    // pos << message->pose.pose.position.x << message->pose.pose.position.y << message->pose.pose.position.z;
    // QVariantList rotate;
    // rotate << message->pose.pose.orientation.w << message->pose.pose.orientation.x << message->pose.pose.orientation.y
    //        << message->pose.pose.orientation.z;
    // viewer_->addTrackPathPoint(pos, rotate);

    // viewer_->unlock();

    track_lock_.lock();
    track_points_.enqueue(message);
    track_lock_.unlock();
}

void SceneWidget::onUpdate()
{
    if(ros::ok())
    {
        ros::spinOnce();
    }
    else
    {
        QApplication::quit();
        return;
    }

    {
        std::lock_guard<std::mutex> lock(track_lock_);
        if(track_points_.isEmpty())
        {
            return;
        }
    }

    sensor_msgs::PointCloud2Ptr cloud;
    cloud_lock_.lock();
    if(!new_clouds_.isEmpty())
    {
        // if(!syncFlag_)
        // {
        //     cloud = new_clouds_.front();

        //     track_lock_.lock();
        //     for(QQueue<nav_msgs::Odometry::ConstPtr>::iterator it = track_points_.begin(); it != track_points_.end(); it++)
        //     {
        //         if((*it)->header.seq == cloud->header.seq)
        //         {
        //             syncFlag_ = true;
        //             break;
        //         }
        //     }
        //     track_lock_.unlock();

        //     if(!syncFlag_)
        //     {
        //         cloud_lock_.unlock();
        //         return;
        //     }
        // }

        cloud = new_clouds_.dequeue();
    }
    cloud_lock_.unlock();

    transformToVtkCloud(cloud);
}

void SceneWidget::transformToVtkCloud(const sensor_msgs::PointCloud2Ptr &cloud)
{
    sensor_msgs::PointCloud2Ptr filtered(new sensor_msgs::PointCloud2);
    int32_t xi = findChannelIndex(cloud, "x");
    int32_t yi = findChannelIndex(cloud, "y");
    int32_t zi = findChannelIndex(cloud, "z");

    if (xi == -1 || yi == -1 || zi == -1)
    {
        return;
    }

    uint32_t xoff = cloud->fields[xi].offset;
    uint32_t yoff = cloud->fields[yi].offset;
    uint32_t zoff = cloud->fields[zi].offset;
    uint32_t point_step = cloud->point_step;
    const size_t point_count = cloud->width * cloud->height;

    if (point_count * point_step != cloud->data.size())
    {
        return;
    }

    filtered->data.resize(cloud->data.size());
    uint32_t output_count;
    if (point_count == 0)
    {
        output_count = 0;
    }
    else
    {
        uint8_t* output_ptr = &filtered->data.front();
        const uint8_t *ptr = &cloud->data.front(), *ptr_end = &cloud->data.back(), *ptr_init;
        size_t points_to_copy = 0;
        for (; ptr < ptr_end; ptr += point_step)
        {
            float x = *reinterpret_cast<const float*>(ptr + xoff);
            float y = *reinterpret_cast<const float*>(ptr + yoff);
            float z = *reinterpret_cast<const float*>(ptr + zoff);
            if (validateFloats(x) && validateFloats(y) && validateFloats(z))
            {
                if (points_to_copy == 0)
                {
                    ptr_init = ptr;
                    points_to_copy = 1;
                }
                else
                {
                    ++points_to_copy;
                }
            }
            else
            {
                if (points_to_copy)
                {
                    memcpy(output_ptr, ptr_init, point_step * points_to_copy);
                    output_ptr += point_step * points_to_copy;
                    points_to_copy = 0;
                }
            }
        }

        if (points_to_copy)
        {
            memcpy(output_ptr, ptr_init, point_step * points_to_copy);
            output_ptr += point_step * points_to_copy;
        }
        output_count = (output_ptr - &filtered->data.front()) / point_step;
    }

    filtered->header = cloud->header;
    filtered->fields = cloud->fields;
    filtered->data.resize(output_count * point_step);
    filtered->height = 1;
    filtered->width = output_count;
    filtered->is_bigendian = cloud->is_bigendian;
    filtered->point_step = point_step;
    filtered->row_step = output_count;

    QVariantList newPointCloud;
    const uint8_t *ptr = &filtered->data.front(), *ptr_end = &filtered->data.back();
    for (; ptr < ptr_end; ptr += point_step)
    {
        float point_x = *reinterpret_cast<const float*>(ptr + xoff);
        float point_y = *reinterpret_cast<const float*>(ptr + yoff);
        float point_z = *reinterpret_cast<const float*>(ptr + zoff);

        //ROS_INFO("Point: %f, %f, %f", point_x, point_y, point_z);
        newPointCloud.append(QVariant::fromValue(QVariantList{ point_x, point_y, point_z }));
    }

    ROS_INFO("Point NUM: %d", newPointCloud.size());

    QVariantList newPointColor;
    if(hasColor(filtered))
    {
        int32_t ri = findChannelIndex(cloud, "r");
        int32_t gi = findChannelIndex(cloud, "g");
        int32_t bi = findChannelIndex(cloud, "b");

        const uint32_t roff = filtered->fields[ri].offset;
        const uint32_t goff = filtered->fields[gi].offset;
        const uint32_t boff = filtered->fields[bi].offset;
        const uint32_t point_step = filtered->point_step;
        const uint32_t num_points = filtered->width * filtered->height;
        uint8_t const* point = &filtered->data.front();
        for (uint32_t i = 0; i < num_points; ++i, point += point_step)
        {
            float r = *reinterpret_cast<const float*>(point + roff);
            float g = *reinterpret_cast<const float*>(point + goff);
            float b = *reinterpret_cast<const float*>(point + boff);
            newPointColor.append(QVariant::fromValue(QVariantList{ r, g, b }));
        }
    }

    nav_msgs::Odometry::ConstPtr ori;
    QVariantList pos;
    QVariantList rotate;

    {
        std::lock_guard<std::mutex> lock(track_lock_);
        while(!track_points_.isEmpty())
        {
            ori = track_points_.dequeue();
            if(ori->header.seq == cloud->header.seq)
            {
                pos << ori->pose.pose.position.x << ori->pose.pose.position.y << ori->pose.pose.position.z;

                rotate << ori->pose.pose.orientation.w << ori->pose.pose.orientation.x << ori->pose.pose.orientation.y
                       << ori->pose.pose.orientation.z;
                break;
            }
        }
    }

    viewer_->addPointCloud(pos, rotate, newPointCloud, newPointColor, false);
}
