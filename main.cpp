#include "mainwindow.h"
#include <QApplication>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    
    QApplication a(argc, argv);
    ros::init(argc, argv, "rosmsg_player");
    ros::NodeHandle nh;


    MainWindow w;
    w.ROSInit(nh);


    w.show();
    return a.exec();
}
