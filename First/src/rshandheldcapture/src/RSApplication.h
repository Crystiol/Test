#ifndef RSAPPLICATION_H
#define RSAPPLICATION_H

#include <QGuiApplication>
#include <QTimer>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "livox_ros_driver2/CustomMsg.h"
#include "RSSceneViewer.h"

class RSApplication : public QGuiApplication
{
public:
    RSApplication(int &argc, char **argv, const std::string& nodeName);
    ~RSApplication();

    void SetView(RSSceneViewer* viewer);

    void topicCallback(const std_msgs::String::ConstPtr& msg);
    void cameraCallback(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr &info);
    void lidarCallback(const livox_ros_driver2::CustomMsgPtr& msg);

public slots:
    void onUpdate();

private:
    QTimer *ros_timer_;

    QScopedPointer<ros::NodeHandle> camera_nh_;
    QScopedPointer<image_transport::ImageTransport> it_;
    image_transport::CameraSubscriber camera_sub_;

    QScopedPointer<ros::NodeHandle> lidar_nh_;

    ros::NodeHandle* nh_;
    ros::Subscriber chatter_sub_;

    RSSceneViewer* viewer_;
    QVariantList newPointCloud_;
    bool updateFlag_;
};

#endif // RSAPPLICATION_H
