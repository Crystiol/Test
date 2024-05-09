#include "SceneWidget.h"
#include "ros/ros.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "viewer");

    QApplication a(argc, argv);
    
    SceneWidget w;
    w.show();
    
    return a.exec();
}
