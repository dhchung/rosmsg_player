#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    OnInitialization();
}

MainWindow::~MainWindow()
{
    ros_->quit();    
    if(!ros_->wait(500)) {
        ros_->terminate();
        ros_->wait();
    }
    ros_->terminate();
    delete ui;
}

void MainWindow::ROSInit(ros::NodeHandle & n) {
    ros_->Initialize(n);
    ros_->SetThreads();
}


void MainWindow::OnInitialization(){

    dir_path = "";
    data_dir_map["lidar_front"] = std::pair<bool, int>{false, 0};
    data_dir_map["lidar_port"] = std::pair<bool, int>{false, 0};
    data_dir_map["lidar_starboard"] = std::pair<bool, int>{false, 0};
    data_dir_map["ahrs"] = std::pair<bool, int>{false, 0};
    data_dir_map["gps"] = std::pair<bool, int>{false, 0};
    data_dir_map["stereo"] = std::pair<bool, int>{false, 0};
    data_dir_map["infrared"] = std::pair<bool, int>{false, 0};
    data_dir_map["omni"] = std::pair<bool, int>{false, 0};

    ui->topic_tableWidget->setColumnCount(2);
    ui->topic_tableWidget->setRowCount(8);
    // ui->topic_tableWidget->blocke
    QStringList horizontal_list{"Data Name", "Data Num"};
    ui->topic_tableWidget->setHorizontalHeaderLabels(horizontal_list);
    QHeaderView * vertical_header = ui->topic_tableWidget->verticalHeader();
    vertical_header->setSectionResizeMode(QHeaderView::Fixed);
    vertical_header->setDefaultSectionSize(10);

    QHeaderView * horizontal_header = ui->topic_tableWidget->horizontalHeader();
    horizontal_header->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    horizontal_header->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    data_ready = false;


    ros_ = new ROSClass(this);
    ros_->start();
    
    ui->play_rate_doubleSpinBox->setValue(1.0f);
    ros_->play_rate = ui->play_rate_doubleSpinBox->value();

    connect(ui->load_pushButton, SIGNAL(clicked()), this, SLOT(LoadButtonClicked()));
    connect(ui->play_pushButton, SIGNAL(clicked()), this, SLOT(PlayButtonClicked()));
    connect(ui->timeline_horizontalSlider, SIGNAL(sliderPressed()), this, SLOT(TimeLineSliderPressed()));
    connect(ui->timeline_horizontalSlider, SIGNAL(sliderReleased()), this, SLOT(TimeLineSliderReleased()));
    connect(ui->play_rate_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(PlayRateChanged(double)));

    connect(ros_, SIGNAL(TimeChanged(float, float)), this, SLOT(OnTimeUpdate(float, float)));
    connect(ros_, SIGNAL(TimeReachedEnd()), this, SLOT(OnTimeReachedEnd()));
    connect(this, SIGNAL(TimeReset()), this->ros_, SLOT(OnTimeReset()));
    time_line_slider_pressed = false;

}

void MainWindow::LoadButtonClicked(){
    QString dir;
    dir = QFileDialog::getExistingDirectory(this, "Choose Directory", QDir::currentPath(), QFileDialog::ShowDirsOnly);
    ui->directory_label->setText(dir);
    dir_path = dir.toUtf8().constData();
    DataListCheck(dir_path);
}

void MainWindow::PlayRateChanged(double play_rate) {
    ros_->play_rate = play_rate;
}


void MainWindow::PlayButtonClicked(){
    if(!ros_->play) {
        if(data_ready) {
            ros_->OnPlay();
            ros_->play = true;
            ui->play_pushButton->setText("Pause");
        } else {

        }
    } else {
        ros_->play = false;
        ui->play_pushButton->setText("Play");
    }
}

void MainWindow::OnTimeReachedEnd() {
    if(data_ready) {
        ui->play_pushButton->setText("Play");
        ros_->DataLoaded();
        ui->timeline_horizontalSlider->setValue(0);
        ros_->percent = 0.0f;
    }
}


void MainWindow::GetTimeAndPathLidar(const std::string sensor_dir, 
                                     const std::string lidar_name,
                                     std::vector<std::pair<long double, std::pair<std::string, std::string>>> & data) {
    data.clear();

    std::string pt_dir = sensor_dir + "/points";
    DIR * dir_sensor = opendir(pt_dir.c_str());
    struct dirent *ent;
    if(dir_sensor!=NULL){
        while((ent = readdir(dir_sensor))!=NULL) {
            std::string filename = std::string(ent->d_name);
            if(filename == "." || filename == "..") {
                continue;
            }
            size_t slash_loc = filename.find_last_of("/");
            size_t bin_loc = filename.find(".bin");
            size_t length = bin_loc - slash_loc -1;

            std::pair<long double, std::pair<std::string, std::string>> time_pair{
                std::stold(filename.substr(slash_loc+1, length))/1e9, 
                std::pair<std::string, std::string>{lidar_name, pt_dir+"/"+filename}
            };

            data.push_back(time_pair);
        }
    }else {
        printf("No Points on %s", lidar_name.c_str());
    }
}


void MainWindow::DataListCheck(const std::string & data_dir){

    std::vector<std::pair<long double, std::pair<std::string, std::string>>> lidar_front_data;
    std::vector<std::pair<long double, std::pair<std::string, std::string>>> lidar_port_data;
    std::vector<std::pair<long double, std::pair<std::string, std::string>>> lidar_starboard_data;
    std::vector<std::pair<long double, std::pair<std::string, std::string>>> stereo_data;
    std::vector<std::pair<long double, std::pair<std::string, std::string>>> infrared_data;
    std::vector<std::pair<long double, std::pair<std::string, std::string>>> omni_data;
    std::vector<std::pair<long double, std::pair<std::string, std::string>>> ahrs_data;
    std::vector<std::pair<long double, std::pair<std::string, std::string>>> gps_data;
    std::vector<std::pair<long double, std::pair<std::string, std::string>>> radar_data;

    std::string sensor_data_dir = data_dir+"/lidar/lidar_front";
    if(std::filesystem::exists(sensor_data_dir.c_str())){
        GetTimeAndPathLidar(sensor_data_dir, "lidar_front", lidar_front_data);
        data_dir_map["lidar_front"] = std::pair<bool, int>{true, int(lidar_front_data.size())};

    } else {
        data_dir_map["lidar_front"] = std::pair<bool, int>{false, 0};
    }

    sensor_data_dir = data_dir + "/lidar/lidar_port";
    if(std::filesystem::exists(sensor_data_dir.c_str())){
        GetTimeAndPathLidar(sensor_data_dir, "lidar_port", lidar_port_data);
        data_dir_map["lidar_port"] = std::pair<bool, int>{true, int(lidar_port_data.size())};

    } else {
        data_dir_map["lidar_port"] = std::pair<bool, int>{false, 0};
    }

    sensor_data_dir = data_dir + "/lidar/lidar_starboard";
    if(std::filesystem::exists(sensor_data_dir.c_str())){
        GetTimeAndPathLidar(sensor_data_dir, "lidar_starboard", lidar_starboard_data);
        data_dir_map["lidar_starboard"] = std::pair<bool, int>{true, int(lidar_starboard_data.size())};

    } else {
        data_dir_map["lidar_starboard"] = std::pair<bool, int>{false, 0};
    }

    sensor_data_dir = data_dir + "/stereo";
    if(std::filesystem::exists(sensor_data_dir.c_str())){
        ros_->stereo_dir_left = sensor_data_dir + "/left_images";
        ros_->stereo_dir_right = sensor_data_dir + "/right_images";

        std::string stereo_timestamp_txt = sensor_data_dir+"/timestamp.txt";
        std::ifstream file(stereo_timestamp_txt);
        std::string str;
        while(std::getline(file, str)) {
            std::string phrase;
            std::stringstream ss(str);
            std::vector<std::string> row;
            while(std::getline(ss, phrase, '\t')) {
                row.push_back(phrase);
            }
            std::pair<long double, ros::StringPair> time_pair{std::stold(row[1]), 
                                                              std::pair<std::string, std::string>{"stereo", row[0]+".png"}};
            stereo_data.push_back(time_pair);                                                              
        }
        data_dir_map["stereo"] = std::pair<bool, int>{true, int(stereo_data.size())};

    } else {
        data_dir_map["stereo"] = std::pair<bool, int>{false, 0};
        ros_->stereo_dir_left = "";
    }

    sensor_data_dir = data_dir + "/infrared";
    if(std::filesystem::exists(sensor_data_dir.c_str())){

        std::string infrared_timestamp_txt = sensor_data_dir+"/timestamp.txt";
        std::ifstream file(infrared_timestamp_txt);
        std::string str;
        ros_->infrared_dir = sensor_data_dir + "/images";

        while(std::getline(file, str)) {
            std::string phrase;
            std::stringstream ss(str);
            std::vector<std::string> row;
            while(std::getline(ss, phrase, '\t')) {
                row.push_back(phrase);
            }
            std::pair<long double, ros::StringPair> time_pair{std::stold(row[1]), 
                                                              std::pair<std::string, std::string>{"infrared", sensor_data_dir +"/images/"+row[0] + ".png"}};
            infrared_data.push_back(time_pair);                                                              
        }
        data_dir_map["infrared"] = std::pair<bool, int>{true, int(infrared_data.size())};

    } else {
        data_dir_map["infrared"] = std::pair<bool, int>{false, 0};
        ros_->infrared_dir = "";
    }

    sensor_data_dir = data_dir + "/omni";
    if(std::filesystem::exists(sensor_data_dir.c_str())) {
        std::string omni_timestamp_txt = sensor_data_dir + "/timestamp.txt";
        std::ifstream file(omni_timestamp_txt);
        std::string str;
        while(std::getline(file, str)) {
            std::string phrase;
            std::stringstream ss(str);
            std::vector<std::string> row;
            while(std::getline(ss, phrase, '\t')) {
                row.push_back(phrase);
            }
            std::pair<long double, ros::StringPair> time_pair{std::stold(row[1]), 
                                                              std::pair<std::string, std::string>{"omni", row[0] + ".jpg"}};
            omni_data.push_back(time_pair);                                                              
        }
        data_dir_map["omni"] = std::pair<bool, int>{true, int(omni_data.size())};
        ros_->omni_0_dir = sensor_data_dir +"/cam_0";
        ros_->omni_1_dir = sensor_data_dir +"/cam_1";
        ros_->omni_2_dir = sensor_data_dir +"/cam_2";
        ros_->omni_3_dir = sensor_data_dir +"/cam_3";
        ros_->omni_4_dir = sensor_data_dir +"/cam_4";
        ros_->omni_5_dir = sensor_data_dir +"/cam_5";

    } else {
        data_dir_map["omni"] = std::pair<bool, int>{false, 0};
        ros_->omni_0_dir = "";
        ros_->omni_1_dir = "";
        ros_->omni_2_dir = "";
        ros_->omni_3_dir = "";
        ros_->omni_4_dir = "";
        ros_->omni_5_dir = "";
    }



    sensor_data_dir = data_dir + "/navigation";
    if(std::filesystem::exists(sensor_data_dir.c_str())){

        std::string ahrs_data_txt = sensor_data_dir+"/ahrs.txt";
        std::ifstream file(ahrs_data_txt);
        std::string str;
        while(std::getline(file, str)) {
            size_t time_loc = str.find_first_of("\t");
            size_t total_length = str.length();
            size_t rest_length = total_length - time_loc-1;

            std::pair<long double, ros::StringPair> time_pair{std::stold(str.substr(0, time_loc)),
                                                              std::pair<std::string, std::string>{"ahrs", str.substr(time_loc+1, rest_length)}};
            ahrs_data.push_back(time_pair);
        }
        data_dir_map["ahrs"] = std::pair<bool, int>{true, int(ahrs_data.size())};

    } else {
        data_dir_map["ahrs"] = std::pair<bool, int>{false, 0};
    }

    sensor_data_dir = data_dir + "/navigation";
    if(std::filesystem::exists(sensor_data_dir.c_str())){

        std::string gps_data_txt = sensor_data_dir+"/gps.txt";
        std::ifstream file(gps_data_txt);
        std::string str;

        while(std::getline(file, str)) {
            size_t time_loc = str.find_first_of("\t");
            size_t total_length = str.length();
            size_t rest_length = total_length - time_loc-1;

            std::pair<long double, ros::StringPair> time_pair{std::stold(str.substr(0, time_loc)),
                                                              std::pair<std::string, std::string>{"radar", str.substr(time_loc+1, rest_length)}};
            gps_data.push_back(time_pair);

        }
        data_dir_map["gps"] = std::pair<bool, int>{true, int(gps_data.size())};

    } else {
        data_dir_map["gps"] = std::pair<bool, int>{false, 0};
    }

    sensor_data_dir = data_dir + "/radar";
    if(std::filesystem::exists(sensor_data_dir.c_str())){
        ros_->radar_dir = sensor_data_dir + "/images";
        std::string radar_timestamp_txt = sensor_data_dir+"/timestamp_deg.txt";
        std::ifstream file(radar_timestamp_txt);
        std::string str;
        while(std::getline(file, str)) {
            size_t time_loc = str.find_first_of("\t");
            size_t total_length = str.length();
            size_t rest_length = total_length - time_loc-1;

            std::pair<long double, ros::StringPair> time_pair{std::stold(str.substr(0, time_loc)),
                                                              std::pair<std::string, std::string>{"radar", str.substr(time_loc+1, rest_length)}};
            radar_data.push_back(time_pair);
                                                             
        }
        data_dir_map["radar"] = std::pair<bool, int>{true, int(radar_data.size())};

    } else {
        data_dir_map["radar"] = std::pair<bool, int>{false, 0};
        ros_->radar_dir = "";
    }

    int row = 0;
    for(auto iter = data_dir_map.begin(); iter != data_dir_map.end(); ++iter) {

        QTableWidgetItem  * table_item_name = ui->topic_tableWidget->item(row, 0);
        QTableWidgetItem  * table_item_num = ui->topic_tableWidget->item(row, 1);
        
        if(!table_item_name){
            table_item_name = new QTableWidgetItem();
            ui->topic_tableWidget->setItem(row, 0, table_item_name);
        }
        table_item_name->setText(QString::fromStdString(iter->first));

        if(!table_item_num){
            table_item_num = new QTableWidgetItem();
            ui->topic_tableWidget->setItem(row, 1, table_item_num);
        }
        table_item_num->setText(QString::number(iter->second.second));
        ++row;
    }

    ros_->time_stamp_data.clear();
    ros_->time_stamp_data.reserve(ros_->time_stamp_data.size() + lidar_front_data.size());
    ros_->time_stamp_data.insert(ros_->time_stamp_data.end(),lidar_front_data.begin(), lidar_front_data.end());

    ros_->time_stamp_data.reserve(ros_->time_stamp_data.size() + lidar_port_data.size());
    ros_->time_stamp_data.insert(ros_->time_stamp_data.end(),lidar_port_data.begin(), lidar_port_data.end());

    ros_->time_stamp_data.reserve(ros_->time_stamp_data.size() + lidar_starboard_data.size());
    ros_->time_stamp_data.insert(ros_->time_stamp_data.end(),lidar_starboard_data.begin(), lidar_starboard_data.end());

    ros_->time_stamp_data.reserve(ros_->time_stamp_data.size() + stereo_data.size());
    ros_->time_stamp_data.insert(ros_->time_stamp_data.end(),stereo_data.begin(), stereo_data.end());

    ros_->time_stamp_data.reserve(ros_->time_stamp_data.size() + infrared_data.size());
    ros_->time_stamp_data.insert(ros_->time_stamp_data.end(),infrared_data.begin(), infrared_data.end());

    ros_->time_stamp_data.reserve(ros_->time_stamp_data.size() + omni_data.size());
    ros_->time_stamp_data.insert(ros_->time_stamp_data.end(),omni_data.begin(), omni_data.end());

    ros_->time_stamp_data.reserve(ros_->time_stamp_data.size() + ahrs_data.size());
    ros_->time_stamp_data.insert(ros_->time_stamp_data.end(),ahrs_data.begin(), ahrs_data.end());

    ros_->time_stamp_data.reserve(ros_->time_stamp_data.size() + gps_data.size());
    ros_->time_stamp_data.insert(ros_->time_stamp_data.end(),gps_data.begin(), gps_data.end());

    ros_->time_stamp_data.reserve(ros_->time_stamp_data.size() + radar_data.size());
    ros_->time_stamp_data.insert(ros_->time_stamp_data.end(),radar_data.begin(), radar_data.end());

    if(ros_->time_stamp_data.empty()) {
        ui->total_time_label->setText(QString("Data Empty"));
        data_ready = false;
    } else {
        std::stable_sort(ros_->time_stamp_data.begin(), ros_->time_stamp_data.end(),
                        [](std::pair<long double, ros::StringPair> first,
                            std::pair<long double, ros::StringPair> second) -> bool {
                                return first.first < second.first;
                            });

        long double start_time = ros_->time_stamp_data[0].first;
        long double end_time = ros_->time_stamp_data.back().first;
        long double duration = end_time - start_time;
        int minute = int(duration/60.0f);
        int second = duration - 60*minute;
        ui->total_time_label->setText(QString("%1m %2s").arg(minute).arg(second));
        data_ready = true;

        ros_->DataLoaded();
    }
}

void MainWindow::OnTimeUpdate(float time, float percent) {
    int minute = int(time/60.0f);
    int second = time - 60*minute;
    ui->current_time_label->setText(QString("%1m %2s").arg(minute).arg(second));

    if(!time_line_slider_pressed) {
        ui->timeline_horizontalSlider->setValue(1000*percent);
    }

}

void MainWindow::TimeLineSliderPressed() {
    time_line_slider_pressed = true;

}

void MainWindow::TimeLineSliderReleased() {
    time_line_slider_pressed = false;
    if(data_ready) {
        float percent = static_cast<float>(ui->timeline_horizontalSlider->value())/1000.f;
        ros_->percent = percent;
        emit TimeReset();
    }
}

void MainWindow::on_checkBox_GPS_clicked(bool checked){
    ros_->play_GPS = checked;
}

void MainWindow::on_checkBox_AHRS_clicked(bool checked){
    ros_->play_AHRS = checked;
}

void MainWindow::on_checkBox_Stereo_clicked(bool checked){
    ros_->play_Stereo = checked;
}

void MainWindow::on_checkBox_Infrared_toggled(bool checked){
    ros_->play_Infrared = checked;
}

void MainWindow::on_checkBox_Omni_clicked(bool checked){
    ros_->play_Omni = checked;
}

void MainWindow::on_checkBox_LiDAR_Front_clicked(bool checked){
    ros_->play_LiDAR_Front = checked;
}

void MainWindow::on_checkBox_LiDAR_Port_clicked(bool checked){
    ros_->play_LiDAR_Port = checked;
}

void MainWindow::on_checkBox_LiDAR_Starboard_clicked(bool checked){
    ros_->play_LiDAR_Starboard = checked;
}

void MainWindow::on_checkBox_Radar_clicked(bool checked){
    ros_->play_Radar = checked;
}
