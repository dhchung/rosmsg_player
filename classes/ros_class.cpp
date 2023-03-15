#include "ros_class.h"

struct PointXYZIR {
    PCL_ADD_POINT4D;
    float intensity;
    std::uint32_t t;
    std::uint8_t ring;
    std::uint16_t reflectivity;
    std::uint16_t ambient;
    std::uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (std::uint8_t, ring, ring) (std::uint32_t, t, t) (std::uint16_t, reflectivity, reflectivity) 
    (std::uint16_t, ambient, ambient) (std::uint32_t, range, range)
)

ROSClass::ROSClass(QObject * parent): QThread(parent){
    stereo_dir_left = "";
    stereo_dir_right = "";
    infrared_dir = "";
    omni_0_dir = "";
    omni_1_dir = "";
    omni_2_dir = "";
    omni_3_dir = "";
    omni_4_dir = "";
    omni_5_dir = "";
    radar_dir = "";

    play = false;
    data_stamp_idx = 0;
    play_rate = 1.0f;
    cur_idx = 0;
    percent = 0.0f;

    infrared_min_temp = 20.0;
    infrared_max_temp = 50.0;

    radar_img = cv::Mat::zeros(cv::Size(2048, 2048), CV_8UC1);
    radar_img_curr = cv::Mat::zeros(cv::Size(2048, 2048), CV_8UC1);
    radar_mask_half_rectangle = cv::Mat::zeros(2048, 2048, CV_8UC1);
    cv::rectangle(radar_mask_half_rectangle, cv::Rect(cv::Point(0, 0), cv::Point(1023, 2048)), cv::Scalar(255), -1);
    radar_currently_loaded = -1;

    play_GPS = false;
    play_AHRS = false;
    play_Stereo = false;
    play_Infrared = false;
    play_Omni = false;
    play_LiDAR_Front = false;
    play_LiDAR_Port = false;
    play_LiDAR_Starboard = false;
    play_Radar = false;
    radar_Deg = false;

}

ROSClass::~ROSClass(){
    lidar_front_dm.b_run = false;
    lidar_port_dm.b_run = false;
    lidar_starboard_dm.b_run = false;
    ahrs_dm.b_run = false;
    gps_dm.b_run = false;
    stereo_dm.b_run = false;
    infrared_dm.b_run = false;
    omni_dm.b_run = false;
    radar_dm.b_run = false;


    data_push_thread_run = false;
    lidar_front_dm.m_cv.notify_all();
    lidar_port_dm.m_cv.notify_all();
    lidar_starboard_dm.m_cv.notify_all();
    ahrs_dm.m_cv.notify_all();
    gps_dm.m_cv.notify_all();
    stereo_dm.m_cv.notify_all();
    infrared_dm.m_cv.notify_all();
    omni_dm.m_cv.notify_all();
    radar_dm.m_cv.notify_all();


    usleep(100000);

    if(data_push_thread.joinable()) {
        data_push_thread.join();
        std::cout<<"DataPush Thread Joined"<<std::endl;
    }

    if(lidar_front_dm.m_thread.joinable()) {
        lidar_front_dm.m_thread.join();
        std::cout<<"Front Lidar Thread Joined"<<std::endl;
    }

    if(lidar_port_dm.m_thread.joinable()) {
        lidar_port_dm.m_thread.join();
        std::cout<<"Port Lidar Thread Joined"<<std::endl;
    }

    if(lidar_starboard_dm.m_thread.joinable()) {
        lidar_starboard_dm.m_thread.join();
        std::cout<<"Starboard Lidar Thread Joined"<<std::endl;
    }

    if(ahrs_dm.m_thread.joinable()) {
        ahrs_dm.m_thread.join();
        std::cout<<"AHRS Thread Joined"<<std::endl;
    }

    if(gps_dm.m_thread.joinable()) {
        gps_dm.m_thread.join();
        std::cout<<"GPS Thread Joined"<<std::endl;
    }

    if(stereo_dm.m_thread.joinable()) {
        stereo_dm.m_thread.join();
        std::cout<<"Stereo Camera Thread Joined"<<std::endl;
    }

    if(infrared_dm.m_thread.joinable()) {
        infrared_dm.m_thread.join();
        std::cout<<"Infrared Camera Thread Joined"<<std::endl;
    }

    if(omni_dm.m_thread.joinable()) {
        omni_dm.m_thread.join();
        std::cout<<"Omnidirectional Camera Thread Joined"<<std::endl;
    }

    if(radar_dm.m_thread.joinable()) {
        radar_dm.m_thread.join();
        std::cout<<"Radar Thread Joined"<<std::endl;
    }

}


void ROSClass::Initialize(ros::NodeHandle & n){
    nh = n;

    lidar_front_pub = nh.advertise<sensor_msgs::PointCloud2>("/lidar_front/os_cloud_node/points", 1000);
    lidar_port_pub = nh.advertise<sensor_msgs::PointCloud2>("/lidar_port/os_cloud_node/points", 1000);
    lidar_starboard_pub = nh.advertise<sensor_msgs::PointCloud2>("/lidar_starboard/os_cloud_node/points", 1000);
    ahrs_pub = nh.advertise<sensor_msgs::Imu>("/gx5/imu/data", 1000);
    gps_pub = nh.advertise<nav_msgs::Odometry>("/gps_nav", 1000);

    it_stereo_left = new image_transport::ImageTransport(nh);
    it_pub_stereo_left = it_stereo_left->advertise("/stereo_cam/left_img", 1000);

    it_stereo_right = new image_transport::ImageTransport(nh);
    it_pub_stereo_right = it_stereo_right->advertise("/stereo_cam/right_img", 1000);

    it_stereo = new image_transport::ImageTransport(nh);
    it_pub_stereo = it_stereo->advertise("/stereo_cam/stereo_img", 1000);

    it_infrared = new image_transport::ImageTransport(nh);
    it_pub_infrared = it_infrared->advertise("/infrared/image", 1000);

    it_omni_0 = new image_transport::ImageTransport(nh);
    it_pub_omni_0 = it_omni_0->advertise("/omni_cam/cam_0", 1000);

    it_omni_1 = new image_transport::ImageTransport(nh);
    it_pub_omni_1 = it_omni_1->advertise("/omni_cam/cam_1", 1000);

    it_omni_2 = new image_transport::ImageTransport(nh);
    it_pub_omni_2 = it_omni_2->advertise("/omni_cam/cam_2", 1000);

    it_omni_3 = new image_transport::ImageTransport(nh);
    it_pub_omni_3 = it_omni_3->advertise("/omni_cam/cam_3", 1000);

    it_omni_4 = new image_transport::ImageTransport(nh);
    it_pub_omni_4 = it_omni_4->advertise("/omni_cam/cam_4", 1000);

    it_omni_5 = new image_transport::ImageTransport(nh);
    it_pub_omni_5 = it_omni_5->advertise("/omni_cam/cam_5", 1000);

    it_radar = new image_transport::ImageTransport(nh);
    it_pub_radar = it_radar->advertise("/radar/image", 1000);

    timer = nh.createTimer(ros::Duration(0.00001), boost::bind(&ROSClass::TimerCallBack, this, _1));
}

void ROSClass::run() {


    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "lidar_front/os_sensor";
    static_transformStamped.child_frame_id = "lidar_starboard/os_sensor";
    static_transformStamped.transform.translation.x = -1.099783063;
    static_transformStamped.transform.translation.y = -1.441996455;
    static_transformStamped.transform.translation.z = 0.845223963;
    static_transformStamped.transform.rotation.x = 0.194914430;
    static_transformStamped.transform.rotation.y = 0.177977756;
    static_transformStamped.transform.rotation.z = -0.740679443;
    static_transformStamped.transform.rotation.w = 0.617840052;
    static_broadcaster.sendTransform(static_transformStamped);

    static_transformStamped.header.frame_id = "lidar_front/os_sensor";
    static_transformStamped.child_frame_id = "lidar_port/os_sensor";
    static_transformStamped.transform.translation.x = -0.899391890;
    static_transformStamped.transform.translation.y = 1.571232200;
    static_transformStamped.transform.translation.z = 0.836276829;
    static_transformStamped.transform.rotation.x = -0.205456942;
    static_transformStamped.transform.rotation.y = 0.202020437;
    static_transformStamped.transform.rotation.z = 0.652350783;
    static_transformStamped.transform.rotation.w = 0.701009095;
    static_broadcaster.sendTransform(static_transformStamped);


    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
}

void ROSClass::SetThreads() {
    data_push_thread_run = true;
    data_push_thread = std::thread(&ROSClass::DataPushThread, this);

    lidar_front_dm.b_run = false;
    if(lidar_front_dm.m_thread.joinable()) {
        lidar_front_dm.m_thread.join();
    }

    lidar_port_dm.b_run = false;
    if(lidar_port_dm.m_thread.joinable()) {
        lidar_port_dm.m_thread.join();
    }

    lidar_starboard_dm.b_run = false;
    if(lidar_starboard_dm.m_thread.joinable()) {
        lidar_starboard_dm.m_thread.join();
    }

    ahrs_dm.b_run = false;
    if(ahrs_dm.m_thread.joinable()) {
        ahrs_dm.m_thread.join();
    }

    gps_dm.b_run = false;
    if(gps_dm.m_thread.joinable()) {
        gps_dm.m_thread.join();
    }

    stereo_dm.b_run = false;
    if(stereo_dm.m_thread.joinable()) {
        stereo_dm.m_thread.join();
    }

    infrared_dm.b_run = false;
    if(infrared_dm.m_thread.joinable()) {
        infrared_dm.m_thread.join();
    }

    omni_dm.b_run = false;
    if(omni_dm.m_thread.joinable()) {
        omni_dm.m_thread.join();
    }

    radar_dm.b_run = false;
    if(radar_dm.m_thread.joinable()) {
        radar_dm.m_thread.join();
    }

    lidar_front_dm.b_run = true;
    lidar_front_dm.m_thread = std::thread(&ROSClass::LidarFrontThread, this);

    lidar_port_dm.b_run = true;
    lidar_port_dm.m_thread = std::thread(&ROSClass::LidarPortThread, this);

    lidar_starboard_dm.b_run = true;
    lidar_starboard_dm.m_thread = std::thread(&ROSClass::LidarStarboardThread, this);

    ahrs_dm.b_run = true;
    ahrs_dm.m_thread = std::thread(&ROSClass::AHRSThread, this);

    gps_dm.b_run = true;
    gps_dm.m_thread = std::thread(&ROSClass::GPSThread, this);

    stereo_dm.b_run = true;
    stereo_dm.m_thread = std::thread(&ROSClass::StereoThread, this);

    infrared_dm.b_run = true;
    infrared_dm.m_thread = std::thread(&ROSClass::InfraredThread, this);

    omni_dm.b_run = true;
    omni_dm.m_thread = std::thread(&ROSClass::OmniThread, this);    

    radar_dm.b_run = true;
    radar_dm.m_thread = std::thread(&ROSClass::RadarThread, this);    

}



void ROSClass::TimerCallBack(const ros::TimerEvent & event) {
    if(play) {
        long double current_time = ros::Time::now().toSec();
        long double time_passed = play_rate * (current_time - last_current_time);
        data_time += time_passed;

        last_current_time = current_time;

        // std::cout<<"Data Time: "<<data_time<<std::endl;
        emit TimeChanged(data_time - initial_time, (data_time-initial_time)/(final_time-initial_time));

        if(data_time > final_time) {
            emit TimeReachedEnd();
            play = false;
        }

    } else {
        last_current_time = ros::Time::now().toSec();
    }

}

void ROSClass::DataLoaded() {
    initial_time = time_stamp_data[0].first;
    final_time = time_stamp_data.back().first;

    data_time = initial_time;
    cur_idx = 0;
    radar_img = cv::Mat::zeros(cv::Size(2048, 2048), CV_8UC1);
    radar_img_curr = cv::Mat::zeros(cv::Size(2048, 2048), CV_8UC1);
    radar_currently_loaded = -1;

}

void ROSClass::OnPlay() {
    // data_time = initial_time + (final_time - initial_time) * percent;
    if(data_time-initial_time == 0.0f) {
        cur_idx = 0;
    } else {
        for(int i = 0; i < time_stamp_data.size()-1; ++i) {
            if(data_time > time_stamp_data[i].first && data_time <= time_stamp_data[i+1].first) {
                cur_idx = i+1;
                break;
            }
        }
    }

    last_current_time = ros::Time::now().toSec();
}

void ROSClass::OnTimeReset() {
    data_push_mutex.lock();
    lidar_front_dm.clear();
    lidar_port_dm.clear();
    lidar_starboard_dm.clear();
    ahrs_dm.clear();
    gps_dm.clear();
    stereo_dm.clear();
    infrared_dm.clear();
    radar_dm.clear();


    radar_img = cv::Mat::zeros(cv::Size(2048, 2048), CV_8UC1);

    data_time = initial_time + (final_time - initial_time) * percent;
    if(percent == 0.0f) {
        cur_idx = 0;
    } else {
        for(int i = 0; i < time_stamp_data.size()-1; ++i) {
            if(data_time > time_stamp_data[i].first && data_time <= time_stamp_data[i+1].first) {
                cur_idx = i+1;
                break;
            }
        }
    }
    data_push_mutex.unlock();
}

void ROSClass::LidarFrontThread() {
    while(1){
        std::unique_lock<std::mutex> ul(lidar_front_dm.m_mutex);
        lidar_front_dm.m_cv.wait(ul);
        if(lidar_front_dm.b_run == false) {
            ul.unlock();
            return;
        }
        ul.unlock();

        while(!lidar_front_dm.m_data_queue.empty()) {
            if(lidar_front_dm.b_run == false) {
                return;
            }

            std::pair<long double, std::string> data = lidar_front_dm.pop();

            float time_diff = data_time - data.first;
            if(abs(time_diff) > 0.4) {
                printf("Time Difference is %.2f. Dropping lidar_front\n", time_diff);
                continue;
            }

            // pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
            pcl::PointCloud<PointXYZIR> pcl_cloud;
            sensor_msgs::PointCloud2 ros_cloud;

            
            int pt_no = 0;

            std::ifstream is(data.second, std::ios::in | std::ios::binary);
            while(is) {
                // pcl::PointXYZI pt;
                PointXYZIR pt;
                float x;
                float y;
                float z;
                float intensity;
                uint32_t time;
                uint16_t reflectivity;
                uint16_t ambient;
                uint32_t range;
                is.read((char*)&x, sizeof(float));
                is.read((char*)&y, sizeof(float));
                is.read((char*)&z, sizeof(float));
                is.read((char*)&intensity, sizeof(float));
                is.read((char*)&time, sizeof(uint32_t));
                is.read((char*)&reflectivity, sizeof(uint16_t));
                is.read((char*)&ambient, sizeof(uint16_t));
                is.read((char*)&range, sizeof(uint32_t));

                //USE XYZI for now
                pt.x = x;
                pt.y = y;
                pt.z = z;
                pt.intensity = intensity;
                pt.ring = (pt_no%64);
                pt.t = time;
                pt.reflectivity = reflectivity;
                pt.ambient = ambient;
                pt.range = range;
                ++pt_no;
                pcl_cloud.points.push_back(pt);
            }
            is.close();

            pcl::toROSMsg(pcl_cloud, ros_cloud);
            
            ros_cloud.header.stamp.fromSec(data.first);
            ros_cloud.header.frame_id = "lidar_front/os_sensor";
            lidar_front_pub.publish(ros_cloud);


        }
        if(lidar_front_dm.b_run == false) {
            return;
        }
    }
}

void ROSClass::LidarPortThread() {
    while(1){
        std::unique_lock<std::mutex> ul(lidar_port_dm.m_mutex);
        lidar_port_dm.m_cv.wait(ul);
        if(lidar_port_dm.b_run == false) {
            ul.unlock();
            return;
        }
        ul.unlock();

        while(!lidar_port_dm.m_data_queue.empty()) {
            if(lidar_port_dm.b_run == false) {
                return;
            }

            std::pair<long double, std::string> data = lidar_port_dm.pop();
            float time_diff = data_time - data.first;
            if(abs(time_diff) > 0.4) {
                printf("Time Difference is %.2f. Dropping lidar_port\n", time_diff);
                continue;
            }

            pcl::PointCloud<PointXYZIR> pcl_cloud;
            sensor_msgs::PointCloud2 ros_cloud;

            int pt_no = 0;

            std::ifstream is(data.second, std::ios::in | std::ios::binary);
            while(is) {
                PointXYZIR pt;
                float x;
                float y;
                float z;
                float intensity;
                uint32_t time;
                uint16_t reflectivity;
                uint16_t ambient;
                uint32_t range;
                is.read((char*)&x, sizeof(float));
                is.read((char*)&y, sizeof(float));
                is.read((char*)&z, sizeof(float));
                is.read((char*)&intensity, sizeof(float));
                is.read((char*)&time, sizeof(uint32_t));
                is.read((char*)&reflectivity, sizeof(uint16_t));
                is.read((char*)&ambient, sizeof(uint16_t));
                is.read((char*)&range, sizeof(uint32_t));

                //USE XYZI for now
                pt.x = x;
                pt.y = y;
                pt.z = z;
                pt.intensity = intensity;
                pt.ring = (pt_no%32);
                pt.t = time;
                pt.reflectivity = reflectivity;
                pt.ambient = ambient;
                pt.range = range;
                ++pt_no;
                pcl_cloud.points.push_back(pt);
            }
            is.close();

            pcl::toROSMsg(pcl_cloud, ros_cloud);

            ros_cloud.header.stamp.fromSec(data.first);
            ros_cloud.header.frame_id = "lidar_port/os_sensor";
            lidar_port_pub.publish(ros_cloud);


        }
        if(lidar_port_dm.b_run == false) {
            return;
        }
    }
}


void ROSClass::LidarStarboardThread() {
    while(1){
        std::unique_lock<std::mutex> ul(lidar_starboard_dm.m_mutex);
        lidar_starboard_dm.m_cv.wait(ul);
        if(lidar_starboard_dm.b_run == false) {
            ul.unlock();
            return;
        }
        ul.unlock();

        while(!lidar_starboard_dm.m_data_queue.empty()) {
            if(lidar_starboard_dm.b_run == false) {
                return;
            }

            std::pair<long double, std::string> data = lidar_starboard_dm.pop();
            float time_diff = data_time - data.first;
            if(abs(time_diff) > 0.4) {
                printf("Time Difference is %.2f. Dropping lidar_starboard\n", time_diff);
                continue;
            }

            pcl::PointCloud<PointXYZIR> pcl_cloud;
            sensor_msgs::PointCloud2 ros_cloud;

            int pt_no = 0;
            
            std::ifstream is(data.second, std::ios::in | std::ios::binary);
            while(is) {
                PointXYZIR pt;
                float x;
                float y;
                float z;
                float intensity;
                uint32_t time;
                uint16_t reflectivity;
                uint16_t ambient;
                uint32_t range;
                is.read((char*)&x, sizeof(float));
                is.read((char*)&y, sizeof(float));
                is.read((char*)&z, sizeof(float));
                is.read((char*)&intensity, sizeof(float));
                is.read((char*)&time, sizeof(uint32_t));
                is.read((char*)&reflectivity, sizeof(uint16_t));
                is.read((char*)&ambient, sizeof(uint16_t));
                is.read((char*)&range, sizeof(uint32_t));

                //USE XYZI for now
                pt.x = x;
                pt.y = y;
                pt.z = z;
                pt.intensity = intensity;
                pt.ring = (pt_no%64);
                pt.t = time;
                pt.reflectivity = reflectivity;
                pt.ambient = ambient;
                pt.range = range;
                ++pt_no;
                pcl_cloud.points.push_back(pt);
            }
            is.close();

            pcl::toROSMsg(pcl_cloud, ros_cloud);
            ros_cloud.header.stamp.fromSec(data.first);
            ros_cloud.header.frame_id = "lidar_starboard/os_sensor";
            lidar_starboard_pub.publish(ros_cloud);


        }
        if(lidar_starboard_dm.b_run == false) {
            return;
        }
    }
}

void ROSClass::AHRSThread() {
    while(1){
        std::unique_lock<std::mutex> ul(ahrs_dm.m_mutex);
        ahrs_dm.m_cv.wait(ul);
        if(ahrs_dm.b_run == false) {
            ul.unlock();
            return;
        }
        ul.unlock();

        while(!ahrs_dm.m_data_queue.empty()) {
            if(ahrs_dm.b_run == false) {
                return;
            }

            std::pair<long double, std::string> data = ahrs_dm.pop();
            float time_diff = data_time - data.first;
            if(abs(time_diff) > 0.4) {
                printf("Time Difference is %.2f. Dropping AHRS\n", time_diff);
                continue;
            }

            sensor_msgs::Imu ros_ahrs;
            ros_ahrs.header.stamp.fromSec(data.first);

            std::string phrase;
            std::stringstream ss(data.second);
            std::vector<std::string> values;
            std::string gps_data_type;

            while(std::getline(ss, phrase, '\t')) {
                values.push_back(phrase);
            }
            if(values.size() < 10) {
                continue;
            }

            ros_ahrs.orientation.x = std::stod(values[0]);
            ros_ahrs.orientation.y = std::stod(values[1]);
            ros_ahrs.orientation.z = std::stod(values[2]);
            ros_ahrs.orientation.w = std::stod(values[3]);
            ros_ahrs.angular_velocity.x = std::stod(values[4]);
            ros_ahrs.angular_velocity.y = std::stod(values[5]);
            ros_ahrs.angular_velocity.z = std::stod(values[6]);
            ros_ahrs.linear_acceleration.x = std::stod(values[7]);
            ros_ahrs.linear_acceleration.y = std::stod(values[8]);
            ros_ahrs.linear_acceleration.z = std::stod(values[9]);

            ros_ahrs.header.frame_id = "gx5_link";
            ahrs_pub.publish(ros_ahrs);



            // while(std::getline(ss, phrase, '\t')) {
            //     if(idx==0) {
            //         ros_ahrs.orientation.x = std::stod(phrase);
            //         ++idx;
            //         continue;
            //     }
            //     if(idx==1) {
            //         ros_ahrs.orientation.y = std::stod(phrase);
            //         ++idx;
            //         continue;
            //     }
            //     if(idx==2) {
            //         ros_ahrs.orientation.z = std::stod(phrase);
            //         ++idx;
            //         continue;
            //     }
            //     if(idx==3) {
            //         ros_ahrs.orientation.w = std::stod(phrase);
            //         ++idx;
            //         continue;
            //     }

            //     if(idx>=4 && idx <=12) {
            //         ros_ahrs.orientation_covariance[idx-4] = std::stod(phrase);
            //         ++idx;
            //         continue;
            //     }

            //     if(idx==13) {
            //         ros_ahrs.angular_velocity.x = std::stod(phrase);
            //         ++idx;
            //         continue;
            //     }

            //     if(idx==14) {
            //         ros_ahrs.angular_velocity.y = std::stod(phrase);
            //         ++idx;
            //         continue;
            //     }

            //     if(idx==15) {
            //         ros_ahrs.angular_velocity.z = std::stod(phrase);
            //         ++idx;
            //         continue;
            //     }

            //     if(idx>=16 && idx <=24) {
            //         ros_ahrs.angular_velocity_covariance[idx-16] = std::stod(phrase);
            //         ++idx;
            //         continue;
            //     }

            //     if(idx==25) {
            //         ros_ahrs.linear_acceleration.x = std::stod(phrase);
            //         ++idx;
            //         continue;
            //     }

            //     if(idx==26) {
            //         ros_ahrs.linear_acceleration.y = std::stod(phrase);
            //         ++idx;
            //         continue;
            //     }

            //     if(idx==27) {
            //         ros_ahrs.linear_acceleration.z = std::stod(phrase);
            //         ++idx;
            //         continue;
            //     }

            //     if(idx>=28 && idx <=36) {
            //         ros_ahrs.linear_acceleration_covariance[idx-28] = std::stod(phrase);
            //         ++idx;
            //         continue;
            //     }
            // }




        }
        if(ahrs_dm.b_run == false) {
            return;
        }
    }    
}


void ROSClass::GPSThread() {
    while(1){
        std::unique_lock<std::mutex> ul(gps_dm.m_mutex);
        gps_dm.m_cv.wait(ul);
        if(gps_dm.b_run == false) {
            ul.unlock();
            return;
        }
        ul.unlock();

        while(!gps_dm.m_data_queue.empty()) {
            if(gps_dm.b_run == false) {
                return;
            }

            std::pair<long double, std::string> data = gps_dm.pop();
            float time_diff = data_time - data.first;
            if(abs(time_diff) > 0.4) {
                printf("Time Difference is %.2f. Dropping GPS\n", time_diff);
                continue;
            }

            std::string phrase;
            std::stringstream ss(data.second);
            std::vector<std::string> values;
            while(std::getline(ss, phrase, '\t')) {
                values.push_back(phrase);
            }

            if(values.size() < 10) {
                continue;
            }


            //0. gps_time
            //1. latitude
            //2. Hemisphere of latitude
            //3. longitude
            //4. Hemisphere of longitude
            //5. Heading
            //6. GPS quality indicator
            //7. Number of satellites
            //8. Horizontal dilution of precision
            //9. Geoid height
            bool valid = true;
            nav_msgs::Odometry ros_gps = gps2odom(data.first, values, valid);

            ros_gps.header.frame_id = "gps_link";
            if(valid) {
                gps_pub.publish(ros_gps);
            }

        }
        if(gps_dm.b_run == false) {
            return;
        }
    }    
}

void ROSClass::StereoThread() {
    while(1) {
        std::unique_lock<std::mutex> ul(stereo_dm.m_mutex);
        stereo_dm.m_cv.wait(ul);


        if(!stereo_dm.b_run) {
            ul.unlock();
            return;
        }
        ul.unlock();

        while(!stereo_dm.m_data_queue.empty()) {
            if(!stereo_dm.b_run) {
                return;
            }

            std::pair<long double, std::string> data = stereo_dm.pop();
            float time_diff = data_time - data.first;
            if(abs(time_diff) > 0.4) {
                printf("Time Difference is %.2f. Dropping Stereo\n", time_diff);
                continue;
            }

            std::string left_img_dir = stereo_dir_left + "/" + data.second;
            std::string right_img_dir = stereo_dir_right + "/" + data.second;

            cv::Mat left_img = cv::imread(left_img_dir, cv::IMREAD_COLOR);
            cv::Mat right_img = cv::imread(right_img_dir, cv::IMREAD_COLOR);

            if(left_img.empty() || right_img.empty()) {
                continue;
            }

            cv::Mat stereo_img;
            cv::hconcat(left_img, right_img, stereo_img);


            std_msgs::Header left_img_msg_header;
            left_img_msg_header.frame_id = "stereo_left_link";
            left_img_msg_header.stamp.fromSec(data.first);
            sensor_msgs::ImagePtr left_img_msg = cv_bridge::CvImage(left_img_msg_header, "bgr8", left_img).toImageMsg();
            it_pub_stereo_left.publish(left_img_msg);

            std_msgs::Header right_img_msg_header;
            right_img_msg_header.frame_id = "stereo_right_link";
            right_img_msg_header.stamp.fromSec(data.first);
            sensor_msgs::ImagePtr right_img_msg = cv_bridge::CvImage(right_img_msg_header, "bgr8", right_img).toImageMsg();
            it_pub_stereo_right.publish(right_img_msg);

            std_msgs::Header stereo_img_msg_header;
            stereo_img_msg_header.frame_id = "stereo_link";
            stereo_img_msg_header.stamp.fromSec(data.first);
            sensor_msgs::ImagePtr stereo_img_msg = cv_bridge::CvImage(stereo_img_msg_header, "bgr8", stereo_img).toImageMsg();
            it_pub_stereo.publish(stereo_img_msg);
            
        }
        if(!stereo_dm.b_run) {
            return;
        }

    }
}

void ROSClass::InfraredThread() {
    while(1) {
        std::unique_lock<std::mutex> ul(infrared_dm.m_mutex);
        infrared_dm.m_cv.wait(ul);
        if(!infrared_dm.b_run) {
            ul.unlock();
            return;
        }
        ul.unlock();

        while(!infrared_dm.m_data_queue.empty()) {
            if(!infrared_dm.b_run) {
                return;
            }

            std::pair<long double, std::string> data = infrared_dm.pop();
            float time_diff = data_time - data.first;
            if(abs(time_diff) > 0.4) {
                printf("Time Difference is %.2f. Dropping Infrared\n", time_diff);
                continue;
            }

            int infrared_min_pixel_value = int((infrared_min_temp + 273.15f)*25); 
            int infrared_max_pixel_value = int((infrared_max_temp + 273.15f)*25); 

            cv::Mat infrared_img = cv::imread(data.second, CV_16UC1);

            if(infrared_img.empty()) {
                continue;
            }

            for(int i = 0; i < infrared_img.rows; ++i) {
                for(int j = 0; j < infrared_img.cols; ++j) {
                    int pixel_value = int(infrared_img.at<u_int16_t>(i, j));

                    if(pixel_value <= infrared_min_pixel_value) {
                        infrared_img.at<u_int16_t>(i,j) = 0;
                    } else if(pixel_value >= infrared_max_pixel_value) {
                        infrared_img.at<u_int16_t>(i,j) = 16383;
                    } else {
                        infrared_img.at<u_int16_t>(i,j) = int(float(infrared_img.at<u_int16_t>(i,j) - infrared_min_pixel_value) * (16383.0f/float(infrared_max_pixel_value - infrared_min_pixel_value)));
                    }
                    infrared_img.at<u_int16_t>(i,j) = infrared_img.at<u_int16_t>(i,j) * 4;
                }
            }

            std_msgs::Header infrared_msg_header;
            infrared_msg_header.frame_id = "infrared_camera_link";
            infrared_msg_header.stamp.fromSec(data.first);
            sensor_msgs::ImagePtr infrared_img_msg = cv_bridge::CvImage(infrared_msg_header, "mono16", infrared_img).toImageMsg();
            it_pub_infrared.publish(infrared_img_msg);

        }
        if(!infrared_dm.b_run) {
            return;
        }

    }
}

void ROSClass::OmniThread() {
    while(1) {
        std::unique_lock<std::mutex> ul(omni_dm.m_mutex);
        omni_dm.m_cv.wait(ul);
        if(!omni_dm.b_run) {
            ul.unlock();
            return;
        }
        ul.unlock();

        while(!omni_dm.m_data_queue.empty()) {
            if(!omni_dm.b_run) {
                return;
            }

            std::pair<long double, std::string> data = omni_dm.pop();
            float time_diff = data_time - data.first;
            if(abs(time_diff) > 0.4) {
                printf("Time Difference is %.2f. Dropping Omni\n", time_diff);
                continue;
            }

            std::string cam_0_img_dir = omni_0_dir + "/" + data.second;
            std::string cam_1_img_dir = omni_1_dir + "/" + data.second;
            std::string cam_2_img_dir = omni_2_dir + "/" + data.second;
            std::string cam_3_img_dir = omni_3_dir + "/" + data.second;
            std::string cam_4_img_dir = omni_4_dir + "/" + data.second;
            std::string cam_5_img_dir = omni_5_dir + "/" + data.second;

            cv::Mat omni_img_0;
            cv::Mat omni_img_1;
            cv::Mat omni_img_2;
            cv::Mat omni_img_3;
            cv::Mat omni_img_4;
            cv::Mat omni_img_5;

            // cv::Mat omni_img_0 = cv::imread(cam_0_img_dir, cv::IMREAD_COLOR);
            // cv::Mat omni_img_1 = cv::imread(cam_1_img_dir, cv::IMREAD_COLOR);
            // cv::Mat omni_img_2 = cv::imread(cam_2_img_dir, cv::IMREAD_COLOR);
            // cv::Mat omni_img_3 = cv::imread(cam_3_img_dir, cv::IMREAD_COLOR);
            // cv::Mat omni_img_4 = cv::imread(cam_4_img_dir, cv::IMREAD_COLOR);
            // cv::Mat omni_img_5 = cv::imread(cam_5_img_dir, cv::IMREAD_COLOR);

            std::thread load_omni_0(&ROSClass::load_color_img, this, std::ref(omni_img_0), cam_0_img_dir);
            std::thread load_omni_1(&ROSClass::load_color_img, this, std::ref(omni_img_1), cam_1_img_dir);
            std::thread load_omni_2(&ROSClass::load_color_img, this, std::ref(omni_img_2), cam_2_img_dir);
            std::thread load_omni_3(&ROSClass::load_color_img, this, std::ref(omni_img_3), cam_3_img_dir);
            std::thread load_omni_4(&ROSClass::load_color_img, this, std::ref(omni_img_4), cam_4_img_dir);
            std::thread load_omni_5(&ROSClass::load_color_img, this, std::ref(omni_img_5), cam_5_img_dir);

            load_omni_0.join();
            load_omni_1.join();
            load_omni_2.join();
            load_omni_3.join();
            load_omni_4.join();
            load_omni_5.join();

            if(omni_img_0.empty() || omni_img_1.empty() ||
               omni_img_2.empty() || omni_img_3.empty() ||
               omni_img_4.empty() || omni_img_5.empty()) {
                   continue;
               }


            std_msgs::Header cam_0_msg_header;
            cam_0_msg_header.frame_id = "omni_cam_0_link";
            cam_0_msg_header.stamp.fromSec(data.first);
            sensor_msgs::ImagePtr cam_0_msg = cv_bridge::CvImage(cam_0_msg_header, "bgr8", omni_img_0).toImageMsg();
            it_pub_omni_0.publish(cam_0_msg);

            std_msgs::Header cam_1_msg_header;
            cam_1_msg_header.frame_id = "omni_cam_1_link";
            cam_1_msg_header.stamp.fromSec(data.first);
            sensor_msgs::ImagePtr cam_1_msg = cv_bridge::CvImage(cam_1_msg_header, "bgr8", omni_img_1).toImageMsg();
            it_pub_omni_1.publish(cam_1_msg);

            std_msgs::Header cam_2_msg_header;
            cam_2_msg_header.frame_id = "omni_cam_2_link";
            cam_2_msg_header.stamp.fromSec(data.first);
            sensor_msgs::ImagePtr cam_2_msg = cv_bridge::CvImage(cam_2_msg_header, "bgr8", omni_img_2).toImageMsg();
            it_pub_omni_2.publish(cam_2_msg);

            std_msgs::Header cam_3_msg_header;
            cam_3_msg_header.frame_id = "omni_cam_3_link";
            cam_3_msg_header.stamp.fromSec(data.first);
            sensor_msgs::ImagePtr cam_3_msg = cv_bridge::CvImage(cam_3_msg_header, "bgr8", omni_img_3).toImageMsg();
            it_pub_omni_3.publish(cam_3_msg);

            std_msgs::Header cam_4_msg_header;
            cam_4_msg_header.frame_id = "omni_cam_4_link";
            cam_4_msg_header.stamp.fromSec(data.first);
            sensor_msgs::ImagePtr cam_4_msg = cv_bridge::CvImage(cam_4_msg_header, "bgr8", omni_img_4).toImageMsg();
            it_pub_omni_4.publish(cam_4_msg);

            std_msgs::Header cam_5_msg_header;
            cam_5_msg_header.frame_id = "omni_cam_5_link";
            cam_5_msg_header.stamp.fromSec(data.first);
            sensor_msgs::ImagePtr cam_5_msg = cv_bridge::CvImage(cam_5_msg_header, "bgr8", omni_img_5).toImageMsg();
            it_pub_omni_5.publish(cam_5_msg);

        }
        if(!omni_dm.b_run) {
            return;
        }
    }
}

void ROSClass::RadarThread() {
    while(1) {
        std::unique_lock<std::mutex> ul(radar_dm.m_mutex);
        radar_dm.m_cv.wait(ul);
        if(!radar_dm.b_run) {
            ul.unlock();
            return;
        }
        ul.unlock();

        while(!radar_dm.m_data_queue.empty()) {
            if(!radar_dm.b_run) {
                return;
            }

            std::pair<long double, std::string> data = radar_dm.pop();
            float time_diff = data_time - data.first;
            if(abs(time_diff) > 0.4) {
                printf("Time Difference is %.2f. Dropping Radar\n", time_diff);
                continue;
            }

            std::string phrase;
            std::vector<std::string> values;
            std::stringstream ss(data.second);
            while(std::getline(ss, phrase, '\t')) {
                values.push_back(phrase);
            }

            if(std::stoi(values[0]) != radar_currently_loaded) {
                std::string radar_img_path = radar_dir + "/" + values[0] + ".png";
                radar_img_curr = cv::imread(radar_img_path, cv::IMREAD_GRAYSCALE);
                if(radar_img_curr.empty()) {
                    continue;
                }
                radar_currently_loaded = stoi(values[0]);
            }


            if(radar_Deg) {
                double deg_start = std::stod(values[1]);
                double deg_end = std::stod(values[2]);

                cv::Mat R_start = cv::getRotationMatrix2D(cv::Point2d(1023.5, 1023.5), -deg_start, 2.0);
                cv::Mat R_end = cv::getRotationMatrix2D(cv::Point2d(1023.5, 1023.5), -deg_end, 2.0);

                cv::Mat mask_start, mask_end;
                cv::warpAffine(radar_mask_half_rectangle, mask_start, R_start, cv::Size(2048, 2048));
                cv::warpAffine(radar_mask_half_rectangle, mask_end, R_end, cv::Size(2048, 2048));

                cv::Mat mask_end_otherside;
                cv::Mat R_other_side = cv::getRotationMatrix2D(cv::Point2d(1023.5, 1023.5), -(deg_start+90), 2.0);
                cv::warpAffine(radar_mask_half_rectangle, mask_end_otherside, R_other_side, cv::Size(2048, 2048));

                cv::Mat subtract_mask, curr_mask;

                cv::bitwise_xor(mask_start, mask_end, subtract_mask);
                cv::bitwise_and(mask_end, subtract_mask, curr_mask);
                cv::bitwise_and(curr_mask, mask_end_otherside, curr_mask);

                cv::rectangle(curr_mask, cv::Rect(cv::Point(1023, 1023), cv::Point(1024, 1024)), cv::Scalar(255), -1);
                radar_img_curr.copyTo(radar_img, curr_mask);
            } else {
                radar_img_curr.copyTo(radar_img);
            }

            std_msgs::Header radar_msg_header;
            radar_msg_header.frame_id = "radar_link";
            radar_msg_header.stamp.fromSec(data.first);
            sensor_msgs::ImagePtr radar_img_msg = cv_bridge::CvImage(radar_msg_header, "mono8", radar_img).toImageMsg();
            it_pub_radar.publish(radar_img_msg);

        }
        if(!radar_dm.b_run) {
            return;
        }
    }
}



void ROSClass::load_color_img(cv::Mat & img, std::string img_path) {
    img = cv::imread(img_path, cv::IMREAD_COLOR);
}

void ROSClass::ToUtm(double dlat, double dlon, double &rutmx, double &rutmy)
{
	double lat, lon;

	// coordinates in radians
	lat = dlat * M_PI / 180;
	lon = dlon * M_PI / 180;

	// UTM parameters
	double lon0_f = floor(dlon / 6) * 6 + 3; // reference longitude in degrees
	double lon0 = lon0_f * M_PI / 180;		 // in radians
	double k0 = 0.9996;						 // scale on central meridian

	int FE = 500000;				// false easting
	int FN = (dlat < 0) * 10000000; // false northing

	// Equations parameters
	// N: radius of curvature of the earth perpendicular to meridian plane
	// Also, distance from point to polar axis
	double WN = Wa / sqrt(1 - pow(We, 2) * pow(sin(lat), 2));
	double WT = pow(tan(lat), 2);
	double WC = (pow(We, 2) / (1 - pow(We, 2))) * pow(cos(lat), 2);
	double WLA = (lon - lon0) * cos(lat);
	// M: true distance along the central meridian from the equator to lat
	double WM = Wa * ((1 - pow(We, 2) / 4 - 3 * pow(We, 4) / 64 - 5 * pow(We, 6) / 256) * lat - (3 * pow(We, 2) / 8 + 3 * pow(We, 4) / 32 + 45 * pow(We, 6) / 1024) * sin(2 * lat) + (15 * pow(We, 4) / 256 + 45 * pow(We, 6) / 1024) * sin(4 * lat) - (35 * pow(We, 6) / 3072) * sin(6 * lat));

	// northing
	// M(lat0) = 0 so not used in following formula
	rutmx = (FN + k0 * WM + k0 * WN * tan(lat) * (pow(WLA, 2) / 2 + (5 - WT + 9 * WC + 4 * pow(WC, 2)) * pow(WLA, 4) / 24 + (61 - 58 * WT + pow(WT, 2) + 600 * WC - 330 * Weps) * pow(WLA, 6) / 720));

	// easting
	rutmy = (FE + k0 * WN * (WLA + (1 - WT + WC) * pow(WLA, 3) / 6 + (5 - 18 * WT + pow(WT, 2) + 72 * WC - 58 * Weps) * pow(WLA, 5) / 120));
}



nav_msgs::Odometry ROSClass::gps2odom(long double time, std::vector<std::string> gps_data, bool & valid){
    nav_msgs::Odometry odom;
    odom.header.stamp.fromSec(time);

    double latitude = std::stod(gps_data[1]);
    double longitude = std::stod(gps_data[3]);
    double altitude = std::stod(gps_data[9]);
    double yaw = std::stod(gps_data[5]);

    // bool signal_type = (gps_data[5].compare("1")==0)? true : false;
    // int fix = std::stoi(gps_data[6]);
    // double HDOP = std::stoi(gps_data[7]);

    if(latitude == 0 || longitude == 0 || yaw == 180.0) {
        valid = false;
        return odom;
    }

    double utmx;
    double utmy;
    ToUtm(latitude, longitude, utmx, utmy);

    odom.pose.pose.position.x = utmx;
    odom.pose.pose.position.y = utmy;
    odom.pose.pose.position.z = 0.0f;

    tf::Quaternion Q;
    double yaw_rad = (yaw/180.0f)*M_PI;

    //pi2pi
    int num = std::abs(yaw_rad)/(2*M_PI);
    float sign = yaw_rad < 0 ? -1.0 : 1.0;
    yaw_rad = yaw_rad - sign * (2*M_PI) * num;
    sign = yaw_rad < 0 ? -1.0 : 1.0;

    if(std::abs(yaw_rad) > M_PI) {
        yaw_rad = yaw_rad - sign * (2*M_PI);
    }

    Q.setRPY(0, 0, yaw_rad);

    odom.pose.pose.orientation.x = Q.x();
    odom.pose.pose.orientation.y = Q.y();
    odom.pose.pose.orientation.z = Q.z();
    odom.pose.pose.orientation.w = Q.w();

    return odom;
}

void ROSClass::DataPushThread() {
    while(1){
        data_push_mutex.lock();
        if(!data_push_thread_run) {
            data_push_mutex.unlock();
            break;
        }
        if(!play) {
            data_push_mutex.unlock();
            continue;
        }

        while(time_stamp_data[cur_idx].first <= data_time) {
            if(cur_idx >= time_stamp_data.size()) {
                break;
            }

            float time_diff = data_time - time_stamp_data[cur_idx].first;
            if(time_diff > 0.05) {
                // std::cout<<"Time difference is too big"<<std::endl;
                // printf("[Data Push Thread] Time Difference is %.2f. Dropping %s\n", time_diff, time_stamp_data[cur_idx].second.first.c_str());
                ++cur_idx;
                continue;
            }

            if(time_stamp_data[cur_idx].second.first.compare("lidar_front") == 0 && play_LiDAR_Front) {
                std::pair<long double, std::string> time_path{
                    time_stamp_data[cur_idx].first,
                    time_stamp_data[cur_idx].second.second
                };
                lidar_front_dm.push(time_path);
                lidar_front_dm.m_cv.notify_all();

            }else if(time_stamp_data[cur_idx].second.first.compare("lidar_port") == 0 && play_LiDAR_Port) {
                std::pair<long double, std::string> time_path{
                    time_stamp_data[cur_idx].first,
                    time_stamp_data[cur_idx].second.second
                };
                lidar_port_dm.push(time_path);
                lidar_port_dm.m_cv.notify_all();
            }else if(time_stamp_data[cur_idx].second.first.compare("lidar_starboard") == 0 && play_LiDAR_Starboard) {
                std::pair<long double, std::string> time_path{
                    time_stamp_data[cur_idx].first,
                    time_stamp_data[cur_idx].second.second
                };
                lidar_starboard_dm.push(time_path);
                lidar_starboard_dm.m_cv.notify_all();
            }else if(time_stamp_data[cur_idx].second.first.compare("ahrs") == 0 && play_AHRS) {
                std::pair<long double, std::string> time_data{
                    time_stamp_data[cur_idx].first,
                    time_stamp_data[cur_idx].second.second
                };
                ahrs_dm.push(time_data);
                ahrs_dm.m_cv.notify_all();

            }else if(time_stamp_data[cur_idx].second.first.compare("gps") == 0 && play_GPS) {
                std::pair<long double, std::string> time_data{
                    time_stamp_data[cur_idx].first,
                    time_stamp_data[cur_idx].second.second
                };
                gps_dm.push(time_data);
                gps_dm.m_cv.notify_all();
            }else if(time_stamp_data[cur_idx].second.first.compare("stereo") == 0 && play_Stereo) {
                std::pair<long double, std::string> time_data{
                    time_stamp_data[cur_idx].first,
                    time_stamp_data[cur_idx].second.second
                };
                stereo_dm.push(time_data);
                stereo_dm.m_cv.notify_all();
            }else if(time_stamp_data[cur_idx].second.first.compare("infrared") == 0 && play_Infrared) {
                std::pair<long double, std::string> time_data{
                    time_stamp_data[cur_idx].first,
                    time_stamp_data[cur_idx].second.second
                };
                infrared_dm.push(time_data);
                infrared_dm.m_cv.notify_all();

            }else if(time_stamp_data[cur_idx].second.first.compare("omni") == 0 && play_Omni) {
                std::pair<long double, std::string> time_data{
                    time_stamp_data[cur_idx].first,
                    time_stamp_data[cur_idx].second.second
                };
                omni_dm.push(time_data);
                omni_dm.m_cv.notify_all();
            }else if(time_stamp_data[cur_idx].second.first.compare("radar") == 0 && play_Radar) {
                std::pair<long double, std::string> time_data{
                    time_stamp_data[cur_idx].first,
                    time_stamp_data[cur_idx].second.second
                };
                radar_dm.push(time_data);
                radar_dm.m_cv.notify_all();
            }

            ++cur_idx;
        }


        data_push_mutex.unlock();
    }
}