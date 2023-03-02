#ifndef ROSCLASS_H
#define ROSCLASS_H

#define UTM_ZON 52
#define Wa 6378137.0
#define Wb 6356752.314245
#define We 0.081819190842965
#define Weps 0.006739496742333

#include <QThread>
#include <QObject>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include "classes/data_manager.h"
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <image_transport/image_transport.h>
#include <tf/tf.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>

#include <sstream>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>


class ROSClass: public QThread {
    Q_OBJECT
public:
    ROSClass(QObject * parent = nullptr);
    ~ROSClass();

    void Initialize(ros::NodeHandle & n);
    void SetThreads();


    std::vector<std::pair<long double, std::pair<std::string, std::string>>> time_stamp_data;

    std::string stereo_dir_left;
    std::string stereo_dir_right;
    std::string infrared_dir;
    std::string omni_0_dir;
    std::string omni_1_dir;
    std::string omni_2_dir;
    std::string omni_3_dir;
    std::string omni_4_dir;
    std::string omni_5_dir;
    std::string radar_dir;


    bool play;
    void run() override;

    void DataLoaded();

    void OnPlay();

    void load_color_img(cv::Mat & img, std::string img_path);

    float play_rate;
    float percent;

    float infrared_min_temp;
    float infrared_max_temp;

    bool play_GPS;
    bool play_AHRS;
    bool play_Stereo;
    bool play_Infrared;
    bool play_Omni;
    bool play_LiDAR_Front;
    bool play_LiDAR_Port;
    bool play_LiDAR_Starboard;
    bool play_Radar;
    bool radar_Deg;




signals:
    void TimeChanged(float time, float percent);
    void TimeReachedEnd();

private slots:
    void OnTimeReset();

private:
    ros::NodeHandle nh;

    ros::Publisher lidar_front_pub;
    ros::Publisher lidar_port_pub;
    ros::Publisher lidar_starboard_pub;
    ros::Publisher ahrs_pub;
    ros::Publisher gps_pub;

    image_transport::ImageTransport * it_stereo_left;
    image_transport::Publisher it_pub_stereo_left;

    image_transport::ImageTransport * it_stereo_right;
    image_transport::Publisher it_pub_stereo_right;

    image_transport::ImageTransport * it_infrared;
    image_transport::Publisher it_pub_infrared;

    image_transport::ImageTransport * it_omni_0;
    image_transport::Publisher it_pub_omni_0;

    image_transport::ImageTransport * it_omni_1;
    image_transport::Publisher it_pub_omni_1;

    image_transport::ImageTransport * it_omni_2;
    image_transport::Publisher it_pub_omni_2;

    image_transport::ImageTransport * it_omni_3;
    image_transport::Publisher it_pub_omni_3;

    image_transport::ImageTransport * it_omni_4;
    image_transport::Publisher it_pub_omni_4;

    image_transport::ImageTransport * it_omni_5;
    image_transport::Publisher it_pub_omni_5;

    image_transport::ImageTransport * it_radar;
    image_transport::Publisher it_pub_radar;


    ros::Timer timer;
    void TimerCallBack(const ros::TimerEvent& event);

    DataManager lidar_front_dm;
    DataManager lidar_port_dm;
    DataManager lidar_starboard_dm;
    DataManager ahrs_dm;
    DataManager gps_dm;
    DataManager stereo_dm;
    DataManager infrared_dm;
    DataManager omni_dm;
    DataManager radar_dm;

    nav_msgs::Odometry gps2odom(long double time, std::vector<std::string> gps_data, bool & valid);

    long double data_time;
    long double initial_time;
    long double final_time;

    long double last_current_time;

    int data_stamp_idx;

    //Threads
    std::mutex data_push_mutex;
    std::thread data_push_thread;
    bool data_push_thread_run;

    int cur_idx;


    cv::Mat radar_img;
    cv::Mat radar_mask_half_rectangle;
    int radar_currently_loaded;
    cv::Mat radar_img_curr;

    void DataPushThread();
    void LidarFrontThread();
    void LidarPortThread();
    void LidarStarboardThread();
    void AHRSThread();
    void GPSThread();
    void StereoThread();
    void InfraredThread();
    void OmniThread();
    void RadarThread();

    void ToUtm(double dlat, double dlon, double &rutmx, double &rutmy);
};


#endif