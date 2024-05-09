#ifndef SCENEWIDGET_H
#define SCENEWIDGET_H

#include <QWidget>
#include <QTimer>
#include <QQueue>
#include <QMap>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud2.h"
#include "RSSceneViewer.h"
#include "RSDeviceControl.h"


QT_BEGIN_NAMESPACE
namespace Ui {
class SceneWidget;
}
QT_END_NAMESPACE

class SceneWidget : public QWidget
{
    Q_OBJECT

public:
    SceneWidget(QWidget *parent = nullptr);
    ~SceneWidget();

    void initUI();

    void topicCallback(const std_msgs::String::ConstPtr& msg);
    void cameraCallback(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr &info);
    void cloudCallback(const sensor_msgs::PointCloud2Ptr& cloud);
    void trackCallback(const nav_msgs::Odometry::ConstPtr& msg);

public slots:
    void onUpdate();
    void transformToVtkCloud(const sensor_msgs::PointCloud2Ptr& cloud);

private:
    Ui::SceneWidget *ui;

    QTimer *ros_timer_;

    QScopedPointer<ros::NodeHandle> camera_nh_;
    QScopedPointer<image_transport::ImageTransport> it_;
    image_transport::CameraSubscriber camera_sub_;

    QScopedPointer<ros::NodeHandle> lidar_track_nh_;
    QScopedPointer<ros::NodeHandle> lidar_cloud_nh_;
    ros::Subscriber track_sub_;
    ros::Subscriber cloud_sub_;

    ros::NodeHandle* nh_;
    ros::Subscriber chatter_sub_;

    QScopedPointer<RSDeviceControl> control_;
    QScopedPointer<RSSceneViewer> viewer_;

    std::mutex cloud_lock_;
    QQueue<sensor_msgs::PointCloud2Ptr> new_clouds_;

    std::mutex track_lock_;
    QQueue<nav_msgs::Odometry::ConstPtr> track_points_;

    bool updateFlag_;
    bool syncFlag_;
};
#endif // SCENEWIDGET_H
