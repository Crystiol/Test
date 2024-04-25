#include "RSApplication.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

RSApplication::RSApplication(int &argc, char **argv, const std::string& nodeName) : QGuiApplication(argc, argv), updateFlag_(false)
{
    ros::init(argc, argv, nodeName);

    camera_nh_.reset(new ros::NodeHandle);
    it_.reset(new image_transport::ImageTransport(*camera_nh_));

    lidar_nh_.reset(new ros::NodeHandle);

    ros_timer_ = new QTimer(this);
    connect(ros_timer_, &QTimer::timeout, this, &RSApplication::onUpdate);
    ros_timer_->start(30);

    //std::string listen_topic;
    //nh_->param<std::string>("listen_topic",listen_topic,"chatter");
    //chatter_sub_ = nh_->subscribe<std_msgs::String>(listen_topic, 1, &RSApplication::topicCallback, this);

    camera_sub_ = it_->subscribeCamera("/rshandheld_cam/image_raw", 1000, &RSApplication::cameraCallback, this);
    lidar_nh_->subscribe("livox/imu", 1000, &RSApplication::lidarCallback, this);
}

RSApplication::~RSApplication()
{
    ros_timer_->stop();
    delete ros_timer_;
}

void RSApplication::SetView(RSSceneViewer *viewer)
{
    if(viewer)
    {
        viewer_ = viewer;
    }
}

void RSApplication::topicCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Receive: [%s]", msg->data.c_str());
}

void RSApplication::cameraCallback(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr &info)
{
    try {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        static int i = 0;
        QString qstrImageName = QString("/home/tuxu/saveimg/IMAGE_%1.jpg").arg(i++);
        cv::imwrite(qstrImageName.toStdString().c_str(), frame);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from ROS message to OpenCV image: %s", e.what());
    }
}

void RSApplication::lidarCallback(const livox_ros_driver2::CustomMsgPtr& msg)
{
    ROS_INFO("Receive from lidar");
    if(!msg)
        return;

    viewer_->lock();

    newPointCloud_.clear();
    for(auto& it : msg->points)
    {
        newPointCloud_.append(QVariant::fromValue(QVariantList{ it.x, it.y, it.z }));
    }
    updateFlag_ = true;

    viewer_->unlock();
}

void RSApplication::onUpdate()
{
    if(ros::ok())
    {
        ros::spinOnce();
    }
    else
    {
        QGuiApplication::quit();
        return;
    }

    viewer_->lock();
    if(updateFlag_)
    {
        viewer_->addPointCloud(newPointCloud_);
    }
    viewer_->unlock();

    //viewer_->render();
}
