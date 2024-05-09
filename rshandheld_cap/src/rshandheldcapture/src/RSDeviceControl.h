#ifndef RSDEVICECONTROL_H
#define RSDEVICECONTROL_H

#include <QtCore/qobject.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

class RSDeviceControl : public QObject
{
    Q_OBJECT
public:
    RSDeviceControl(QObject *parent = 0);
    ~RSDeviceControl();

    Q_INVOKABLE void startCamera();
    Q_INVOKABLE void stopCamera();
    
    Q_INVOKABLE void startRecordControl();
    Q_INVOKABLE void stopRecordControl();
    
    Q_INVOKABLE void startMapping();
    Q_INVOKABLE void stopMapping();
    
    void statusCallback(const std_msgs::String::ConstPtr& msg);

private:
    QScopedPointer<ros::NodeHandle> camera_node_;
    QScopedPointer<ros::NodeHandle> mapping_control_node_;
    QScopedPointer<ros::NodeHandle> lidar_control_node_;
    QScopedPointer<ros::NodeHandle> lidar_status_node_;
    
    ros::Publisher mapping_control_publisher_;
    ros::Publisher lidar_control_publisher_;

    ros::Subscriber lidar_status_sub_;
};

#endif // RSDEVICECONTROL_H
