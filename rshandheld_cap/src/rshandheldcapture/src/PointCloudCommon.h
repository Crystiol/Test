#ifndef POINTCLOUDCOMMON_H
#define POINTCLOUDCOMMON_H

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <QSharedPointer>

struct CloudInfo
{
    CloudInfo();
    ~CloudInfo();

    void clear();

    ros::Time receive_time_;

    sensor_msgs::PointCloud2ConstPtr message_;

    //Ogre::SceneNode* scene_node_;
    //boost::shared_ptr<PointCloud> cloud_;
    //PointCloudSelectionHandlerPtr selection_handler_;

    //std::vector<PointCloud::Point> transformed_points_;

    //Ogre::Quaternion orientation_;
    //Ogre::Vector3 position_;
};

typedef QSharedPointer<CloudInfo> CloudInfoPtr;

#endif // POINTCLOUDCOMMON_H
