#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <filesystem>
#include <dirent.h>
#include <numeric>
#include <algorithm>

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include "classes/ros_class.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void ROSInit(ros::NodeHandle & n);

private slots:
    void LoadButtonClicked();
    void PlayButtonClicked();
    void OnTimeUpdate(float time, float percent);
    void OnTimeReachedEnd();

    void TimeLineSliderPressed();
    void TimeLineSliderReleased();
    void PlayRateChanged(double play_rate);
    
signals:
    void TimeReset();

private:
    Ui::MainWindow *ui;
    void OnInitialization();
    void DataListCheck(const std::string & data_dir);

    std::string dir_path;
    std::map<std::string, std::pair<bool, int>> data_dir_map;

    ROSClass * ros_;

    void GetTimeAndPathLidar(const std::string sensor_dir, 
                             const std::string lidar_name,
                             std::vector<std::pair<long double, std::pair<std::string, std::string>>> & data);    
    bool data_ready;

    bool time_line_slider_pressed;

};
#endif // MAINWINDOW_H
