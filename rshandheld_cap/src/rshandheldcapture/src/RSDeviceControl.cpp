#include "RSDeviceControl.h"
#include "std_srvs/Empty.h"
#include "std_msgs/String.h"
#define RECORD_FILE "control.log"

RSDeviceControl::RSDeviceControl(QObject *parent) : QObject(parent)
{
    camera_node_.reset(new ros::NodeHandle);
    mapping_control_node_.reset(new ros::NodeHandle);
    lidar_control_node_.reset(new ros::NodeHandle);
    lidar_status_node_.reset(new ros::NodeHandle);
    mapping_control_publisher_ = mapping_control_node_->advertise<std_msgs::String>("/mapping/control", 10);
    lidar_control_publisher_ = lidar_control_node_->advertise<std_msgs::String>("/recording/control", 10);
    lidar_status_sub_ = lidar_status_node_->subscribe("/record/status", 1000, &RSDeviceControl::statusCallback, this);
}

RSDeviceControl::~RSDeviceControl()
{

}

void RSDeviceControl::startCamera()
{
    ros::ServiceClient client = camera_node_->serviceClient<std_srvs::Empty>("/camera_front/start_capture");

    std_srvs::Empty srv;
    if(client.call(srv))
    {
        ROS_INFO("Start Camera");
    }
    else
    {
        ROS_INFO("Start Camera Failed");
    }
}

void RSDeviceControl::stopCamera()
{
    ros::ServiceClient client = camera_node_->serviceClient<std_srvs::Empty>("/camera_front/stop_capture");

    std_srvs::Empty srv;
    if(client.call(srv))
    {
        ROS_INFO("Stop Camera");
    }
    else
    {
        ROS_INFO("Stop Camera Failed");
    }
}

void RSDeviceControl::startRecordControl()
{
    ROS_INFO("Start Record Control");

    std_msgs::String msg;
    msg.data = RECORD_FILE;

    ROS_INFO("%s",msg.data.c_str());
    lidar_control_publisher_.publish(msg);

    ros::spinOnce();
}

void RSDeviceControl::stopRecordControl()
{
    ROS_INFO("Stop Record Control");

    std_msgs::String msg;
    msg.data = "end";

    ROS_INFO("%s",msg.data.c_str());
    lidar_control_publisher_.publish(msg);

    ros::spinOnce();
}

void RSDeviceControl::startMapping()
{
    ROS_INFO("Start Mapping");

    std_msgs::String msg;
    msg.data = "start";

    ROS_INFO("%s",msg.data.c_str());
    mapping_control_publisher_.publish(msg);

    ros::spinOnce();
}

void RSDeviceControl::stopMapping()
{
    ROS_INFO("Stop Mapping");

    std_msgs::String msg;
    msg.data = "end";

    ROS_INFO("%s",msg.data.c_str());
    mapping_control_publisher_.publish(msg);

    ros::spinOnce();
}

void RSDeviceControl::statusCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Receive: [%s]", msg->data.c_str());
}
