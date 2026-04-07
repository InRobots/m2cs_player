#include <QMutexLocker>

#include "ROSThread.h"

using namespace std;

ROSThread::ROSThread(QObject *parent, QMutex *th_mutex) : QThread(parent), mutex_(th_mutex)
{

    processed_stamp_ = 0;
    play_rate_ = 1.0;
    loop_flag_ = false;
    stop_skip_flag_ = true;

    search_bound_ = 10;
    reset_process_stamp_flag_ = false;
    auto_start_flag_ = true;

    tf_flag_ = true;
    nav_flag_ = true;
    ouster_flag_ = true;
    livox_left_flag_ = true;
    livox_right_flag_ = true;
    camera_flag_ = true;

    stamp_show_count_ = 0;
    prev_clock_stamp_ = 0;
}

ROSThread::~ROSThread()
{

    data_stamp_thread_.active_ = false;
    nav_thread_.active_ = false;
    gps_thread_.active_ = false;
    imu_thread_.active_ = false;
    livox_left_imu_thread_.active_ = false;
    livox_right_imu_thread_.active_ = false;
    livox_left_thread_.active_ = false;
    livox_right_thread_.active_ = false;
    ouster_thread_.active_ = false;
    camera_thread_.active_ = false;
    usleep(100000);

    data_stamp_thread_.cv_.notify_all();
    if (data_stamp_thread_.thread_.joinable())
        data_stamp_thread_.thread_.join();

    nav_thread_.cv_.notify_all();
    if (nav_thread_.thread_.joinable())
        nav_thread_.thread_.join();

    gps_thread_.cv_.notify_all();
    if (gps_thread_.thread_.joinable())
        gps_thread_.thread_.join();

    imu_thread_.cv_.notify_all();
    if (imu_thread_.thread_.joinable())
        imu_thread_.thread_.join();

    livox_left_imu_thread_.cv_.notify_all();
    if (livox_left_imu_thread_.thread_.joinable())
        livox_left_imu_thread_.thread_.join();

    livox_right_imu_thread_.cv_.notify_all();
    if (livox_right_imu_thread_.thread_.joinable())
        livox_right_imu_thread_.thread_.join();

    ouster_imu_thread_.cv_.notify_all();
    if (ouster_imu_thread_.thread_.joinable())
        ouster_imu_thread_.thread_.join();

    livox_left_thread_.cv_.notify_all();
    if (livox_left_thread_.thread_.joinable())
        livox_left_thread_.thread_.join();

    livox_right_thread_.cv_.notify_all();
    if (livox_right_thread_.thread_.joinable())
        livox_right_thread_.thread_.join();

    ouster_thread_.cv_.notify_all();
    if (ouster_thread_.thread_.joinable())
        ouster_thread_.thread_.join();

    camera_thread_.cv_.notify_all();
    if (camera_thread_.thread_.joinable())
        camera_thread_.thread_.join();
}

void ROSThread::ros_initialize(ros::NodeHandle &nh)
{
    nh_ = nh;

    pre_timer_stamp_ = ros::Time::now().toNSec();
    timer_ = nh_.createTimer(ros::Duration(0.0001), boost::bind(&ROSThread::TimerCallback, this, _1));

    start_sub_ = nh_.subscribe<std_msgs::Bool>("/file_player_start", 1, boost::bind(&ROSThread::FilePlayerStart, this, _1));
    stop_sub_ = nh_.subscribe<std_msgs::Bool>("/file_player_stop", 1, boost::bind(&ROSThread::FilePlayerStop, this, _1));

    nav_pub_ = nh_.advertise<gwnav_ros_driver::GWNav>("/gwnav/nav", 1000);
    gps_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/gwnav/gps", 1000);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/gwnav/imu", 1000);
    
    livox_msg_left_pub_ = nh_.advertise<livox_ros_driver::CustomMsg>("/livox_left/points", 1000);
    livox_msg_right_pub_ = nh_.advertise<livox_ros_driver::CustomMsg>("/livox_right/points", 1000);

    livox_left_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/livox_left/point_cloud", 1000);
    livox_right_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/livox_right/point_cloud", 1000);
    
    livox_left_imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/livox_left/imu", 1000);
    livox_right_imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/livox_right/imu", 1000);

    ouster_imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/ouster/imu", 1000);
    ouster_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/ouster/points", 100);

    camera_pub_ = nh_.advertise<sensor_msgs::Image>("/camera/image_raw", 100);

    clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 1);
}

void ROSThread::run()
{
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
}

void ROSThread::Ready()
{

    data_stamp_thread_.active_ = false;
    data_stamp_thread_.cv_.notify_all();
    if (data_stamp_thread_.thread_.joinable())
        data_stamp_thread_.thread_.join();

    nav_thread_.active_ = false;
    nav_thread_.cv_.notify_all();
    if (nav_thread_.thread_.joinable())
        nav_thread_.thread_.join();

    gps_thread_.active_ = false;
    gps_thread_.cv_.notify_all();
    if (gps_thread_.thread_.joinable())
        gps_thread_.thread_.join();

    imu_thread_.active_ = false;
    imu_thread_.cv_.notify_all();
    if (imu_thread_.thread_.joinable())
        imu_thread_.thread_.join();

    livox_left_imu_thread_.active_ = false;
    livox_left_imu_thread_.cv_.notify_all();
    if (livox_left_imu_thread_.thread_.joinable())
        livox_left_imu_thread_.thread_.join();

    livox_right_imu_thread_.active_ = false;
    livox_right_imu_thread_.cv_.notify_all();
    if (livox_right_imu_thread_.thread_.joinable())
        livox_right_imu_thread_.thread_.join();

    ouster_imu_thread_.active_ = false;
    ouster_imu_thread_.cv_.notify_all();
    if (ouster_imu_thread_.thread_.joinable())
        ouster_imu_thread_.thread_.join();

    livox_left_thread_.active_ = false;
    livox_left_thread_.cv_.notify_all();
    if (livox_left_thread_.thread_.joinable())
        livox_left_thread_.thread_.join();

    livox_right_thread_.active_ = false;
    livox_right_thread_.cv_.notify_all();
    if (livox_right_thread_.thread_.joinable())
        livox_right_thread_.thread_.join();

    ouster_thread_.active_ = false;
    ouster_thread_.cv_.notify_all();
    if (ouster_thread_.thread_.joinable())
        ouster_thread_.thread_.join();

    camera_thread_.active_ = false;
    camera_thread_.cv_.notify_all();
    if (camera_thread_.thread_.joinable())
        camera_thread_.thread_.join();

    // check path is right or not
    ifstream f((data_folder_path_ + "/data_stamp.csv").c_str());
    if (!f.good())
    {
        // cout << "Please check file path. Input path is wrong" << endl;
        ROS_INFO("\033[1;31m----> Please check file path. Input path is wrong.\033[0m");
        return;
    }
    f.close();

    // Read CSV file and make map
    FILE *fp;
    int64_t stamp;
    // data stamp data load

    fp = fopen((data_folder_path_ + "/data_stamp.csv").c_str(), "r");
    char data_name[50];
    data_stamp_.clear();
    while (fscanf(fp, "%ld,%s\n", &stamp, data_name) == 2)
    {
        //  data_stamp_[stamp] = data_name;
        data_stamp_.insert(multimap<int64_t, string>::value_type(stamp, data_name));
    }
    // cout << "Stamp data are loaded" << endl;
    ROS_INFO("\033[1;33m----> Stamp data are loaded.\033[0m");
    fclose(fp);

    initial_data_stamp_ = data_stamp_.begin()->first - 1;
    last_data_stamp_ = prev(data_stamp_.end(), 1)->first - 1;

    double angular_velocity_x,
        angular_velocity_y,
        angular_velocity_z,
        linear_acceleration_x,
        linear_acceleration_y,
        linear_acceleration_z,
        orientation_x,
        orientation_y,
        orientation_z,
        orientation_w;

    // Read livox_left_imu data
    fp = fopen((data_folder_path_ + "/ins_data/livox_left_imu.csv").c_str(), "r");
    sensor_msgs::Imu livox_left_imu_msg;
    livox_left_imu_data_.clear();
    while (fscanf(fp, "%ld,%lf,%lf,%lf,%lf,%lf,%lf\n", &stamp,
                  &angular_velocity_x,
                  &angular_velocity_y,
                  &angular_velocity_z,
                  &linear_acceleration_x,
                  &linear_acceleration_y,
                  &linear_acceleration_z) == 7)
    {
        livox_left_imu_msg.header.stamp.fromNSec(stamp);
        livox_left_imu_msg.header.frame_id = "livox_left_imu";
        livox_left_imu_msg.angular_velocity.x = angular_velocity_x;
        livox_left_imu_msg.angular_velocity.y = angular_velocity_y;
        livox_left_imu_msg.angular_velocity.z = angular_velocity_z;
        livox_left_imu_msg.linear_acceleration.x = linear_acceleration_x;
        livox_left_imu_msg.linear_acceleration.y = linear_acceleration_y;
        livox_left_imu_msg.linear_acceleration.z = linear_acceleration_z;
        livox_left_imu_data_[stamp] = livox_left_imu_msg;
    }
    // cout << "Livox left imu data are loaded" << endl;
    ROS_INFO("\033[1;33m----> Livox left imu data are loaded.\033[0m");
    fclose(fp);

    // Read livox_right_imu data
    fp = fopen((data_folder_path_ + "/ins_data/livox_right_imu.csv").c_str(), "r");
    sensor_msgs::Imu livox_right_imu_msg;
    livox_right_imu_data_.clear();
    while (fscanf(fp, "%ld,%lf,%lf,%lf,%lf,%lf,%lf\n", &stamp,
                  &angular_velocity_x,
                  &angular_velocity_y,
                  &angular_velocity_z,
                  &linear_acceleration_x,
                  &linear_acceleration_y,
                  &linear_acceleration_z) == 7)
    {
        livox_right_imu_msg.header.stamp.fromNSec(stamp);
        livox_right_imu_msg.header.frame_id = "livox_right_imu";
        livox_right_imu_msg.angular_velocity.x = angular_velocity_x;
        livox_right_imu_msg.angular_velocity.y = angular_velocity_y;
        livox_right_imu_msg.angular_velocity.z = angular_velocity_z;
        livox_right_imu_msg.linear_acceleration.x = linear_acceleration_x;
        livox_right_imu_msg.linear_acceleration.y = linear_acceleration_y;
        livox_right_imu_msg.linear_acceleration.z = linear_acceleration_z;
        livox_right_imu_data_[stamp] = livox_right_imu_msg;
    }
    // cout << "Livox right imu data are loaded" << endl;
    ROS_INFO("\033[1;33m----> Livox right imu data are loaded.\033[0m");
    fclose(fp);

    // Read ouster_imu data
    fp = fopen((data_folder_path_ + "/ins_data/ouster_imu.csv").c_str(), "r");
    sensor_msgs::Imu ouster_imu_msg;
    ouster_imu_data_.clear();
    while (fscanf(fp, "%ld,%lf,%lf,%lf,%lf,%lf,%lf\n", &stamp,
                  &angular_velocity_x,
                  &angular_velocity_y,
                  &angular_velocity_z,
                  &linear_acceleration_x,
                  &linear_acceleration_y,
                  &linear_acceleration_z) == 7)
    {
        ouster_imu_msg.header.stamp.fromNSec(stamp);
        ouster_imu_msg.header.frame_id = "ouster_imu";
        ouster_imu_msg.angular_velocity.x = angular_velocity_x;
        ouster_imu_msg.angular_velocity.y = angular_velocity_y;
        ouster_imu_msg.angular_velocity.z = angular_velocity_z;
        ouster_imu_msg.linear_acceleration.x = linear_acceleration_x;
        ouster_imu_msg.linear_acceleration.y = linear_acceleration_y;
        ouster_imu_msg.linear_acceleration.z = linear_acceleration_z;
        ouster_imu_data_[stamp] = ouster_imu_msg;
    }
    // cout << "Ouster imu data are loaded" << endl;
    ROS_INFO("\033[1;33m----> Ouster imu data are loaded.\033[0m");
    fclose(fp);

    // Read nav data
    fp = fopen((data_folder_path_ + "/ins_data/gwnav_nav.csv").c_str(), "r");
    gwnav_ros_driver::GWNav nav_msg;
    char utc_buf[50];
    int work_time, work_mode, self_test, nosv, recv_pos_type, recv_position_state, recv_heading_state, pps_time, itow;
    double nue_velocity_x,
        nue_velocity_y,
        nue_velocity_z,
        gyr_drift_x,
        gyr_drift_y,
        gyr_drift_z,
        heaveing,
        latitude,
        longitude,
        height,
        roll,
        pitch,
        yaw,
        recv_hdop,
        recv_nue_velocity_x,
        recv_nue_velocity_y,
        recv_nue_velocity_z,
        recv_latitude,
        recv_longitude,
        recv_height,
        temperature,
        mag_heading,
        gnss_heading;
    nav_data_.clear();
    //                           4           7          10          13          16              20          23          27              31          34          37       40
    while (fscanf(fp, "%ld,%d,%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d,%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d,%s\n",
                  &stamp,
                  &work_time,
                  &work_mode,
                  &self_test, // 4
                  &angular_velocity_x,
                  &angular_velocity_y,
                  &angular_velocity_z, // 7
                  &linear_acceleration_x,
                  &linear_acceleration_y,
                  &linear_acceleration_z, // 10
                  &nue_velocity_x,
                  &nue_velocity_y,
                  &nue_velocity_z, // 13
                  &gyr_drift_x,
                  &gyr_drift_y,
                  &gyr_drift_z, // 16
                  &heaveing,
                  &latitude,
                  &longitude,
                  &height, // 20
                  &roll,
                  &pitch,
                  &yaw, // 23
                  &nosv,
                  &recv_pos_type,
                  &recv_position_state,
                  &recv_heading_state, // 27
                  &recv_hdop,
                  &recv_nue_velocity_x,
                  &recv_nue_velocity_y,
                  &recv_nue_velocity_z, // 31
                  &recv_latitude,
                  &recv_longitude,
                  &recv_height, // 34
                  &temperature,
                  &mag_heading,
                  &gnss_heading, // 37
                  &pps_time,
                  &itow,
                  utc_buf) == 40) // 40
    {
        nav_msg.header.stamp.fromNSec(stamp);
        nav_msg.header.frame_id = "gwnav";
        nav_msg.work_time = work_time;
        nav_msg.work_mode = work_mode;
        nav_msg.self_test = self_test;
        nav_msg.angular_velocity.x = angular_velocity_x;
        nav_msg.angular_velocity.y = angular_velocity_y;
        nav_msg.angular_velocity.z = angular_velocity_z;
        nav_msg.linear_acceleration.x = linear_acceleration_x;
        nav_msg.linear_acceleration.y = linear_acceleration_y;
        nav_msg.linear_acceleration.z = linear_acceleration_z;
        nav_msg.nue_velocity.x = nue_velocity_x;
        nav_msg.nue_velocity.y = nue_velocity_y;
        nav_msg.nue_velocity.z = nue_velocity_z;
        nav_msg.gyr_drift.x = gyr_drift_x;
        nav_msg.gyr_drift.y = gyr_drift_y;
        nav_msg.gyr_drift.z = gyr_drift_z;
        nav_msg.heaveing = heaveing;
        nav_msg.latitude = latitude;
        nav_msg.longitude = longitude;
        nav_msg.height = height;
        nav_msg.roll = roll;
        nav_msg.pitch = pitch;
        nav_msg.yaw = yaw;
        nav_msg.nosv = nosv;
        nav_msg.recv_pos_type = recv_pos_type;
        nav_msg.recv_position_state = recv_heading_state;
        nav_msg.recv_heading_state = recv_heading_state;
        nav_msg.recv_hdop = recv_hdop;
        nav_msg.recv_nue_velocity.x = recv_nue_velocity_x;
        nav_msg.recv_nue_velocity.y = recv_nue_velocity_y;
        nav_msg.recv_nue_velocity.z = recv_nue_velocity_z;
        nav_msg.recv_latitude = recv_latitude;
        nav_msg.recv_longitude = recv_longitude;
        nav_msg.recv_height = recv_height;
        nav_msg.temperature = temperature;
        nav_msg.mag_heading = mag_heading;
        nav_msg.gnss_heading = gnss_heading;
        nav_msg.pps_time = pps_time;
        nav_msg.itow = itow;
        string utc;
        strcpy(utc_buf, utc.c_str());
        nav_msg.utc = utc;
        nav_data_[stamp] = nav_msg;
    }
    // cout << "GWNAV nav data are loaded" << endl;
    ROS_INFO("\033[1;33m----> GWNAV nav data are loaded.\033[0m");
    fclose(fp);

    // Read imu data
    fp = fopen((data_folder_path_ + "/ins_data/gwnav_imu.csv").c_str(), "r");
    sensor_msgs::Imu imu_msg;
    imu_data_.clear();
    while (fscanf(fp, "%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", &stamp,
                  &angular_velocity_x,
                  &angular_velocity_y,
                  &angular_velocity_z,
                  &linear_acceleration_x,
                  &linear_acceleration_y,
                  &linear_acceleration_z,
                  &orientation_x,
                  &orientation_y,
                  &orientation_z,
                  &orientation_w) == 11)
    {
        imu_msg.header.stamp.fromNSec(stamp);
        imu_msg.header.frame_id = "gwnav";
        imu_msg.angular_velocity.x = angular_velocity_x;
        imu_msg.angular_velocity.y = angular_velocity_y;
        imu_msg.angular_velocity.z = angular_velocity_z;
        imu_msg.linear_acceleration.x = linear_acceleration_x;
        imu_msg.linear_acceleration.y = linear_acceleration_y;
        imu_msg.linear_acceleration.z = linear_acceleration_z;
        imu_msg.orientation.x = orientation_x;
        imu_msg.orientation.y = orientation_y;
        imu_msg.orientation.z = orientation_z;
        imu_msg.orientation.w = orientation_w;

        imu_data_[stamp] = imu_msg;
    }
    // cout << "GWNAV imu data are loaded" << endl;
    ROS_INFO("\033[1;33m----> GWNAV imu data are loaded.\033[0m");
    fclose(fp);

    // Read gps data
    fp = fopen((data_folder_path_ + "/ins_data/gwnav_gps.csv").c_str(), "r");
    // double latitude, longitude, altitude;
    double altitude;
    int status, service;
    sensor_msgs::NavSatFix gps_data;
    gps_data_.clear();
    while (fscanf(fp, "%ld,%lf,%lf,%lf,%d,%d\n", &stamp,
                  &latitude,
                  &longitude,
                  &altitude,
                  &status,
                  &service) == 6)
    {
        gps_data.header.stamp.fromNSec(stamp);
        gps_data.header.frame_id = "gwnav";
        gps_data.latitude = latitude;
        gps_data.longitude = longitude;
        gps_data.altitude = altitude;
        gps_data.status.status = status;
        gps_data.status.service = service;
        gps_data_[stamp] = gps_data;
    }
    // cout << "GWNAV gps data are loaded" << endl;
    ROS_INFO("\033[1;33m----> GWNAV gps data are loaded.\033[0m");
    fclose(fp);

    if (tf_flag_ == true)
    {
        cv::Mat rot, trans;

        std::string livox_left_to_ouster_extrinsic_file = data_folder_path_ + "/calib_data/livox_left_to_ouster.yaml";
        cv::FileStorage livox_left_to_ouster_extrinsic_fs(livox_left_to_ouster_extrinsic_file, cv::FileStorage::READ);
        livox_left_to_ouster_extrinsic_fs["rotation"] >> rot;
        cv::cv2eigen(rot, livox_left_to_ouster_rot);
        livox_left_to_ouster_quat = Eigen::Quaterniond(livox_left_to_ouster_rot);
        livox_left_to_ouster_extrinsic_fs["translation"] >> trans;
        cv::cv2eigen(trans, livox_left_to_ouster_trans);
        // cout << "livox_left_to_ouster" << endl;
        // cout << livox_left_to_ouster_rot << endl;
        // cout << livox_left_to_ouster_quat.coeffs().transpose() << endl;
        // cout << livox_left_to_ouster_trans << endl;

        std::string livox_right_to_ouster_extrinsic_file = data_folder_path_ + "/calib_data/livox_right_to_ouster.yaml";
        cv::FileStorage livox_right_to_ouster_extrinsic_fs(livox_right_to_ouster_extrinsic_file, cv::FileStorage::READ);
        livox_right_to_ouster_extrinsic_fs["rotation"] >> rot;
        cv::cv2eigen(rot, livox_right_to_ouster_rot);
        livox_right_to_ouster_quat = Eigen::Quaterniond(livox_right_to_ouster_rot);
        livox_right_to_ouster_extrinsic_fs["translation"] >> trans;
        cv::cv2eigen(trans, livox_right_to_ouster_trans);
        // cout << "livox_right_to_ouster" << endl;
        // cout << livox_right_to_ouster_rot << endl;
        // cout << livox_right_to_ouster_quat.coeffs().transpose() << endl;
        // cout << livox_right_to_ouster_trans << endl;

        std::string camera_to_ouster_extrinsic_file = data_folder_path_ + "/calib_data/camera_to_ouster.yaml";
        cv::FileStorage camera_to_ouster_extrinsic_fs(camera_to_ouster_extrinsic_file, cv::FileStorage::READ);
        camera_to_ouster_extrinsic_fs["rotation"] >> rot;
        cv::cv2eigen(rot, camera_to_ouster_rot);
        camera_to_ouster_quat = Eigen::Quaterniond(camera_to_ouster_rot);
        camera_to_ouster_extrinsic_fs["translation"] >> trans;
        cv::cv2eigen(trans, camera_to_ouster_trans);
        // cout << "camera_to_ouster" << endl;
        // cout << camera_to_ouster_rot << endl;
        // cout << camera_to_ouster_quat.coeffs().transpose() << endl;
        // cout << camera_to_ouster_trans << endl;

        std::string ouster_to_gwnav_extrinsic_file = data_folder_path_ + "/calib_data/ouster_to_gwnav.yaml";
        cv::FileStorage ouster_to_gwnav_extrinsic_fs(ouster_to_gwnav_extrinsic_file, cv::FileStorage::READ);
        ouster_to_gwnav_extrinsic_fs["rotation"] >> rot;
        cv::cv2eigen(rot, ouster_to_gwnav_rot);
        ouster_to_gwnav_quat = Eigen::Quaterniond(ouster_to_gwnav_rot);
        ouster_to_gwnav_extrinsic_fs["translation"] >> trans;
        cv::cv2eigen(trans, ouster_to_gwnav_trans);
        // cout << "ouster_to_gwnav" << endl;
        // cout << ouster_to_gwnav_rot << endl;
        // cout << ouster_to_gwnav_quat.coeffs().transpose() << endl;
        // cout << ouster_to_gwnav_trans << endl;
    }

    livox_left_file_list_.clear();
    livox_right_file_list_.clear();
    ouster_file_list_.clear();
    camera_file_list_.clear();

    GetDirList(data_folder_path_ + "/lidar_data/livox_left", livox_left_file_list_);
    GetDirList(data_folder_path_ + "/lidar_data/livox_right", livox_right_file_list_);
    GetDirList(data_folder_path_ + "/lidar_data/ouster", ouster_file_list_);
    GetDirList(data_folder_path_ + "/image_data", camera_file_list_);

    // load camera info
    camera_nh_ = ros::NodeHandle(nh_, "camera");
    camera_info_manager_ = boost::shared_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(camera_nh_, "camera"));

    string camera_yaml_file_path = "file://" + data_folder_path_ + "/calib_data/camera.yaml";

    if (camera_info_manager_->validateURL(camera_yaml_file_path))
    {
        camera_info_manager_->loadCameraInfo(camera_yaml_file_path);
        // cout << "Success to load camera info" << endl;
        ROS_INFO("\033[1;33m----> Success to load camera info.\033[0m");
        camera_info_ = camera_info_manager_->getCameraInfo();
    }

    data_stamp_thread_.active_ = true;
    if (nav_flag_ == true)
    {
        nav_thread_.active_ = true;
        gps_thread_.active_ = true;
        imu_thread_.active_ = true;
    }
    if (livox_left_flag_ == true)
    {
        livox_left_thread_.active_ = true;
        livox_left_imu_thread_.active_ = true;
    }
    if (livox_right_flag_ == true)
    {
        livox_right_thread_.active_ = true;
        livox_right_imu_thread_.active_ = true;
    }
    if (ouster_flag_ == true)
    {
        ouster_thread_.active_ = true;
        ouster_imu_thread_.active_ = true;
    }
    if (camera_flag_ == true)
    {
        camera_thread_.active_ = true;
    }

    data_stamp_thread_.thread_ = std::thread(&ROSThread::DataStampThread, this);

    nav_thread_.thread_ = std::thread(&ROSThread::NavThread, this);
    gps_thread_.thread_ = std::thread(&ROSThread::GpsThread, this);
    imu_thread_.thread_ = std::thread(&ROSThread::ImuThread, this);

    livox_left_imu_thread_.thread_ = std::thread(&ROSThread::LivoxLeftImuThread, this);
    livox_right_imu_thread_.thread_ = std::thread(&ROSThread::LivoxRightImuThread, this);
    ouster_imu_thread_.thread_ = std::thread(&ROSThread::OusterImuThread, this);

    livox_left_thread_.thread_ = std::thread(&ROSThread::LivoxLeftThread, this);
    livox_right_thread_.thread_ = std::thread(&ROSThread::LivoxRightThread, this);
    ouster_thread_.thread_ = std::thread(&ROSThread::OusterThread, this);

    camera_thread_.thread_ = std::thread(&ROSThread::CameraThread, this);
}

void ROSThread::DataStampThread()
{
    auto stop_region_iter = stop_period_.begin();

    for (auto iter = data_stamp_.begin(); iter != data_stamp_.end(); iter++)
    {
        auto stamp = iter->first;

        while ((stamp > (initial_data_stamp_ + processed_stamp_)) && (data_stamp_thread_.active_ == true))
        {
            if (processed_stamp_ == 0)
            {
                iter = data_stamp_.begin();
                stop_region_iter = stop_period_.begin();
                stamp = iter->first;
            }
            usleep(1);
            if (reset_process_stamp_flag_ == true)
                break;
            // wait for data publish
        }

        if (reset_process_stamp_flag_ == true)
        {
            auto target_stamp = processed_stamp_ + initial_data_stamp_;
            // set iter
            iter = data_stamp_.lower_bound(target_stamp);
            iter = prev(iter, 1);
            // set stop region order
            auto new_stamp = iter->first;
            stop_region_iter = stop_period_.upper_bound(new_stamp);

            reset_process_stamp_flag_ = false;
            continue;
        }

        // check whether stop region or not
        if (stamp == stop_region_iter->first)
        {
            if (stop_skip_flag_ == true)
            {
                cout << "Skip stop section!!" << endl;
                iter = data_stamp_.find(stop_region_iter->second); // find stop region end
                iter = prev(iter, 1);
                processed_stamp_ = stop_region_iter->second - initial_data_stamp_;
            }
            stop_region_iter++;
            if (stop_skip_flag_ == true)
            {
                continue;
            }
        }

        if (data_stamp_thread_.active_ == false)
            return;

        if (iter->second.compare("gwnav_nav") == 0)
        {
            nav_thread_.push(stamp);
            nav_thread_.cv_.notify_all();
        }
        else if (iter->second.compare("gwnav_gps") == 0)
        {
            gps_thread_.push(stamp);
            gps_thread_.cv_.notify_all();
        }
        else if (iter->second.compare("gwnav_imu") == 0)
        {
            imu_thread_.push(stamp);
            imu_thread_.cv_.notify_all();
        }
        else if (iter->second.compare("livox_left_imu") == 0)
        {
            livox_left_imu_thread_.push(stamp);
            livox_left_imu_thread_.cv_.notify_all();
        }
        else if (iter->second.compare("livox_right_imu") == 0)
        {
            livox_right_imu_thread_.push(stamp);
            livox_right_imu_thread_.cv_.notify_all();
        }
        else if (iter->second.compare("ouster_imu") == 0)
        {
            ouster_imu_thread_.push(stamp);
            ouster_imu_thread_.cv_.notify_all();
        }
        else if (iter->second.compare("livox_left") == 0)
        {
            livox_left_thread_.push(stamp);
            livox_left_thread_.cv_.notify_all();
        }
        else if (iter->second.compare("livox_right") == 0)
        {
            livox_right_thread_.push(stamp);
            livox_right_thread_.cv_.notify_all();
        }
        else if (iter->second.compare("ouster") == 0)
        {
            ouster_thread_.push(stamp);
            ouster_thread_.cv_.notify_all();
        }
        else if (iter->second.compare("camera") == 0)
        {
            camera_thread_.push(stamp);
            camera_thread_.cv_.notify_all();
        }

        stamp_show_count_++;
        if (stamp_show_count_ > 100)
        {
            stamp_show_count_ = 0;
            emit StampShow(stamp);

            ros::Time time_stamp;
            time_stamp.fromNSec(stamp);

            static tf::TransformBroadcaster tf_broadcaster;
            static tf::Transform livox_left_to_ouster = tf::Transform(tf::Quaternion(livox_left_to_ouster_quat.x(),
                                                                                     livox_left_to_ouster_quat.y(),
                                                                                     livox_left_to_ouster_quat.z(),
                                                                                     livox_left_to_ouster_quat.w()),
                                                                      tf::Vector3(livox_left_to_ouster_trans.x(),
                                                                                  livox_left_to_ouster_trans.y(),
                                                                                  livox_left_to_ouster_trans.z()));

            static tf::Transform livox_right_to_ouster = tf::Transform(tf::Quaternion(livox_right_to_ouster_quat.x(),
                                                                                      livox_right_to_ouster_quat.y(),
                                                                                      livox_right_to_ouster_quat.z(),
                                                                                      livox_right_to_ouster_quat.w()),
                                                                       tf::Vector3(livox_right_to_ouster_trans.x(),
                                                                                   livox_right_to_ouster_trans.y(),
                                                                                   livox_right_to_ouster_trans.z()));

            static tf::Transform camera_to_ouster = tf::Transform(tf::Quaternion(camera_to_ouster_quat.x(),
                                                                                 camera_to_ouster_quat.y(),
                                                                                 camera_to_ouster_quat.z(),
                                                                                 camera_to_ouster_quat.w()),
                                                                  tf::Vector3(camera_to_ouster_trans.x(),
                                                                              camera_to_ouster_trans.y(),
                                                                              camera_to_ouster_trans.z()));

            static tf::Transform ouster_to_gwnav = tf::Transform(tf::Quaternion(ouster_to_gwnav_quat.x(),
                                                                                ouster_to_gwnav_quat.y(),
                                                                                ouster_to_gwnav_quat.z(),
                                                                                ouster_to_gwnav_quat.w()),
                                                                 tf::Vector3(ouster_to_gwnav_trans.x(),
                                                                             ouster_to_gwnav_trans.y(),
                                                                             ouster_to_gwnav_trans.z()));

            tf_broadcaster.sendTransform(tf::StampedTransform(camera_to_ouster, time_stamp, "ouster", "camera"));
            tf_broadcaster.sendTransform(tf::StampedTransform(livox_left_to_ouster, time_stamp, "ouster", "livox_left"));
            tf_broadcaster.sendTransform(tf::StampedTransform(livox_right_to_ouster, time_stamp, "ouster", "livox_right"));
            tf_broadcaster.sendTransform(tf::StampedTransform(ouster_to_gwnav, time_stamp, "gwnav", "ouster"));
        }

        if (prev_clock_stamp_ == 0 || (stamp - prev_clock_stamp_) > 10000000)
        {
            rosgraph_msgs::Clock clock;
            clock.clock.fromNSec(stamp);
            clock_pub_.publish(clock);
            prev_clock_stamp_ = stamp;
        }

        if (loop_flag_ == true && iter == prev(data_stamp_.end(), 1))
        {
            iter = data_stamp_.begin();
            stop_region_iter = stop_period_.begin();
            processed_stamp_ = 0;
        }
        if (loop_flag_ == false && iter == prev(data_stamp_.end(), 1))
        {
            play_flag_ = false;
            while (!play_flag_)
            {
                iter = data_stamp_.begin();
                stop_region_iter = stop_period_.begin();
                processed_stamp_ = 0;
                usleep(10000);
            }
        }
    }
    cout << "Data publish complete" << endl;
}

void ROSThread::NavThread()
{
    while (1)
    {
        std::unique_lock<std::mutex> ul(nav_thread_.mutex_);
        nav_thread_.cv_.wait(ul);
        if (nav_thread_.active_ == false)
            return;
        ul.unlock();

        while (!nav_thread_.data_queue_.empty())
        {
            auto data = nav_thread_.pop();
            // process
            if (nav_data_.find(data) != nav_data_.end())
            {
                nav_pub_.publish(nav_data_[data]);
            }
        }
        if (nav_thread_.active_ == false)
            return;
    }
}

void ROSThread::GpsThread()
{
    while (1)
    {
        std::unique_lock<std::mutex> ul(gps_thread_.mutex_);
        gps_thread_.cv_.wait(ul);
        if (gps_thread_.active_ == false)
            return;
        ul.unlock();

        while (!gps_thread_.data_queue_.empty())
        {
            auto data = gps_thread_.pop();
            // process
            if (gps_data_.find(data) != gps_data_.end())
            {
                gps_pub_.publish(gps_data_[data]);
            }
        }
        if (gps_thread_.active_ == false)
            return;
    }
}

void ROSThread::ImuThread()
{
    while (1)
    {
        std::unique_lock<std::mutex> ul(imu_thread_.mutex_);
        imu_thread_.cv_.wait(ul);
        if (imu_thread_.active_ == false)
            return;
        ul.unlock();

        while (!imu_thread_.data_queue_.empty())
        {
            auto data = imu_thread_.pop();
            // process
            if (imu_data_.find(data) != imu_data_.end())
            {
                imu_pub_.publish(imu_data_[data]);
            }
        }
        if (imu_thread_.active_ == false)
            return;
    }
}

void ROSThread::LivoxLeftImuThread()
{
    while (1)
    {
        std::unique_lock<std::mutex> ul(livox_left_imu_thread_.mutex_);
        livox_left_imu_thread_.cv_.wait(ul);
        if (livox_left_imu_thread_.active_ == false)
            return;
        ul.unlock();

        while (!livox_left_imu_thread_.data_queue_.empty())
        {
            auto data = livox_left_imu_thread_.pop();
            // process
            if (livox_left_imu_data_.find(data) != livox_left_imu_data_.end())
            {
                livox_left_imu_pub_.publish(livox_left_imu_data_[data]);
            }
        }
        if (livox_left_imu_thread_.active_ == false)
            return;
    }
}

void ROSThread::LivoxRightImuThread()
{
    while (1)
    {
        std::unique_lock<std::mutex> ul(livox_right_imu_thread_.mutex_);
        livox_right_imu_thread_.cv_.wait(ul);
        if (livox_right_imu_thread_.active_ == false)
            return;
        ul.unlock();

        while (!livox_right_imu_thread_.data_queue_.empty())
        {
            auto data = livox_right_imu_thread_.pop();
            // process
            if (livox_right_imu_data_.find(data) != livox_right_imu_data_.end())
            {
                livox_right_imu_pub_.publish(livox_right_imu_data_[data]);
            }
        }
        if (livox_right_imu_thread_.active_ == false)
            return;
    }
}

void ROSThread::OusterImuThread()
{
    while (1)
    {
        std::unique_lock<std::mutex> ul(ouster_imu_thread_.mutex_);
        ouster_imu_thread_.cv_.wait(ul);
        if (ouster_imu_thread_.active_ == false)
            return;
        ul.unlock();

        while (!ouster_imu_thread_.data_queue_.empty())
        {
            auto data = ouster_imu_thread_.pop();
            // process
            if (ouster_imu_data_.find(data) != ouster_imu_data_.end())
            {
                ouster_imu_pub_.publish(ouster_imu_data_[data]);
            }
        }
        if (ouster_imu_thread_.active_ == false)
            return;
    }
}

void ROSThread::TimerCallback(const ros::TimerEvent &)
{
    int64_t current_stamp = ros::Time::now().toNSec();
    if (play_flag_ == true && pause_flag_ == false)
    {
        processed_stamp_ += static_cast<int64_t>(static_cast<double>(current_stamp - pre_timer_stamp_) * play_rate_);
    }
    pre_timer_stamp_ = current_stamp;

    if (play_flag_ == false)
    {
        processed_stamp_ = 0; // reset
        prev_clock_stamp_ = 0;
    }
}
void ROSThread::LivoxLeftThread()
{
    int current_file_index = 0;
    int previous_file_index = 0;
    while (1)
    {
        std::unique_lock<std::mutex> ul(livox_left_thread_.mutex_);
        livox_left_thread_.cv_.wait(ul);
        if (livox_left_thread_.active_ == false)
            return;
        ul.unlock();

        while (!livox_left_thread_.data_queue_.empty())
        {
            auto data = livox_left_thread_.pop();

            // publish data
            if (to_string(data) + ".bin" == livox_left_next_.first)
            {
                // publish
                
                    livox_msg_left_next_.second.header.stamp.fromNSec(data);
                    livox_msg_left_next_.second.header.frame_id = "livox_left";
                    livox_msg_left_pub_.publish(livox_msg_left_next_.second);
                
                    livox_left_next_.second.header.stamp.fromNSec(data);
                    livox_left_next_.second.header.frame_id = "livox_left";
                    livox_left_pub_.publish(livox_left_next_.second);
            }
            else
            {
                //        cout << "Re-load left velodyne from path" << endl;
                // load current data
                
                pcl::PointCloud<LivoxPoint> cloud;
                cloud.clear();
                sensor_msgs::PointCloud2 publish_cloud;
                livox_ros_driver::CustomMsg livox_msg;
                
                string current_file_name = data_folder_path_ + "/lidar_data/livox_left" + "/" + to_string(data) + ".bin";
                if (find(next(livox_left_file_list_.begin(), max(0, previous_file_index - search_bound_)), livox_left_file_list_.end(), to_string(data) + ".bin") != livox_left_file_list_.end())
                {
                    ifstream file;
                    file.open(current_file_name, ios::in | ios::binary);
                    while (!file.eof())
                    {
                        LivoxPoint point;
                        file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
                        file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
                        file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
                        file.read(reinterpret_cast<char *>(&point.tag), sizeof(uint8_t));
                        file.read(reinterpret_cast<char *>(&point.line), sizeof(uint8_t));
                        file.read(reinterpret_cast<char *>(&point.reflectivity), sizeof(uint8_t));
                        file.read(reinterpret_cast<char *>(&point.offset_time), sizeof(uint32_t));
                        cloud.points.push_back(point);

                        livox_ros_driver::CustomPoint point_msg;
                        point_msg.x = point.x;
                        point_msg.y = point.y;
                        point_msg.z = point.z;
                        point_msg.tag = point.tag;
                        point_msg.line = point.line;
                        point_msg.reflectivity = point.reflectivity;
                        point_msg.offset_time = point.offset_time;
                        livox_msg.points.push_back(point_msg);
                    }
                    file.close();

                    livox_msg.points.pop_back();
                    livox_msg.point_num = livox_msg.points.size();
                    livox_msg.header.stamp.fromNSec(data);
                    livox_msg.header.frame_id = "livox_left";
                    livox_msg_left_pub_.publish(livox_msg);

                    cloud.points.pop_back();
                    pcl::toROSMsg(cloud, publish_cloud);
                    publish_cloud.header.stamp.fromNSec(data);
                    publish_cloud.header.frame_id = "livox_left";
                    livox_left_pub_.publish(publish_cloud);
                }
                previous_file_index = 0;
            }

            // load next data
            
            livox_ros_driver::CustomMsg livox_msg;
            pcl::PointCloud<LivoxPoint> cloud;
            cloud.clear();
            sensor_msgs::PointCloud2 publish_cloud;

            string current_file_name = data_folder_path_ + "/lidar_data/livox_left" + "/" + to_string(data) + ".bin";
            if (find(next(livox_left_file_list_.begin(), max(0, previous_file_index - search_bound_)), livox_left_file_list_.end(), to_string(data) + ".bin") != livox_left_file_list_.end())
            {
                ifstream file;
                file.open(current_file_name, ios::in | ios::binary);
                while (!file.eof())
                {
                    LivoxPoint point;
                    file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
                    file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
                    file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
                    file.read(reinterpret_cast<char *>(&point.tag), sizeof(uint8_t));
                    file.read(reinterpret_cast<char *>(&point.line), sizeof(uint8_t));
                    file.read(reinterpret_cast<char *>(&point.reflectivity), sizeof(uint8_t));
                    file.read(reinterpret_cast<char *>(&point.offset_time), sizeof(uint32_t));
                    cloud.points.push_back(point);

                    livox_ros_driver::CustomPoint point_msg;
                    point_msg.x = point.x;
                    point_msg.y = point.y;
                    point_msg.z = point.z;
                    point_msg.tag = point.tag;
                    point_msg.line = point.line;
                    point_msg.reflectivity = point.reflectivity;
                    point_msg.offset_time = point.offset_time;
                    livox_msg.points.push_back(point_msg);
                }
                file.close();
                livox_msg.points.pop_back();
                livox_msg.point_num = livox_msg.points.size();
                livox_msg_left_next_ = make_pair(livox_left_file_list_[current_file_index + 1], livox_msg);

                cloud.points.pop_back();
                pcl::toROSMsg(cloud, publish_cloud);
                livox_left_next_ = make_pair(livox_left_file_list_[current_file_index + 1], publish_cloud);
            }
            previous_file_index = current_file_index;
        }
        if (livox_left_thread_.active_ == false)
            return;
    }
}

void ROSThread::LivoxRightThread()
{
    int current_file_index = 0;
    int previous_file_index = 0;
    while (1)
    {
        std::unique_lock<std::mutex> ul(livox_right_thread_.mutex_);
        livox_right_thread_.cv_.wait(ul);
        if (livox_right_thread_.active_ == false)
            return;
        ul.unlock();

        while (!livox_right_thread_.data_queue_.empty())
        {
            auto data = livox_right_thread_.pop();

            // publish data
            if (to_string(data) + ".bin" == livox_right_next_.first)
            {
                // publish
                livox_msg_right_next_.second.header.stamp.fromNSec(data);
                livox_msg_right_next_.second.header.frame_id = "livox_right";
                livox_msg_right_pub_.publish(livox_msg_right_next_.second);

                livox_right_next_.second.header.stamp.fromNSec(data);
                livox_right_next_.second.header.frame_id = "livox_right";
                livox_right_pub_.publish(livox_right_next_.second);
            }
            else
            {
                //        cout << "Re-load right velodyne from path" << endl;
                // load current data
                livox_ros_driver::CustomMsg livox_msg;
                pcl::PointCloud<LivoxPoint> cloud;
                cloud.clear();
                sensor_msgs::PointCloud2 publish_cloud;
                string current_file_name = data_folder_path_ + "/lidar_data/livox_right" + "/" + to_string(data) + ".bin";
                if (find(next(livox_right_file_list_.begin(), max(0, previous_file_index - search_bound_)), livox_right_file_list_.end(), to_string(data) + ".bin") != livox_right_file_list_.end())
                {
                    ifstream file;
                    file.open(current_file_name, ios::in | ios::binary);
                    while (!file.eof())
                    {
                        LivoxPoint point;
                        file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
                        file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
                        file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
                        file.read(reinterpret_cast<char *>(&point.tag), sizeof(uint8_t));
                        file.read(reinterpret_cast<char *>(&point.line), sizeof(uint8_t));
                        file.read(reinterpret_cast<char *>(&point.reflectivity), sizeof(uint8_t));
                        file.read(reinterpret_cast<char *>(&point.offset_time), sizeof(uint32_t));
                        cloud.points.push_back(point);

                        livox_ros_driver::CustomPoint point_msg;
                        point_msg.x = point.x;
                        point_msg.y = point.y;
                        point_msg.z = point.z;
                        point_msg.tag = point.tag;
                        point_msg.line = point.line;
                        point_msg.reflectivity = point.reflectivity;
                        point_msg.offset_time = point.offset_time;
                        livox_msg.points.push_back(point_msg);
                    }
                    file.close();

                    livox_msg.points.pop_back();
                    livox_msg.point_num = livox_msg.points.size();
                    livox_msg.header.stamp.fromNSec(data);
                    livox_msg.header.frame_id = "livox_right";
                    livox_msg_right_pub_.publish(livox_msg);

                    cloud.points.pop_back();
                    pcl::toROSMsg(cloud, publish_cloud);
                    publish_cloud.header.stamp.fromNSec(data);
                    publish_cloud.header.frame_id = "livox_right";
                    livox_right_pub_.publish(publish_cloud);
                }
                previous_file_index = 0;
            }

            // load next data
            livox_ros_driver::CustomMsg livox_msg;
            pcl::PointCloud<LivoxPoint> cloud;
            cloud.clear();
            sensor_msgs::PointCloud2 publish_cloud;
            current_file_index = find(next(livox_right_file_list_.begin(), max(0, previous_file_index - search_bound_)), livox_right_file_list_.end(), to_string(data) + ".bin") - livox_right_file_list_.begin();
            if (find(next(livox_right_file_list_.begin(), max(0, previous_file_index - search_bound_)), livox_right_file_list_.end(), livox_right_file_list_[current_file_index + 1]) != livox_right_file_list_.end())
            {
                string next_file_name = data_folder_path_ + "/lidar_data/livox_right" + "/" + livox_right_file_list_[current_file_index + 1];

                ifstream file;
                file.open(next_file_name, ios::in | ios::binary);
                while (!file.eof())
                {
                    LivoxPoint point;
                    file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
                    file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
                    file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
                    file.read(reinterpret_cast<char *>(&point.tag), sizeof(uint8_t));
                    file.read(reinterpret_cast<char *>(&point.line), sizeof(uint8_t));
                    file.read(reinterpret_cast<char *>(&point.reflectivity), sizeof(uint8_t));
                    file.read(reinterpret_cast<char *>(&point.offset_time), sizeof(uint32_t));
                    cloud.points.push_back(point);

                    livox_ros_driver::CustomPoint point_msg;
                    point_msg.x = point.x;
                    point_msg.y = point.y;
                    point_msg.z = point.z;
                    point_msg.tag = point.tag;
                    point_msg.line = point.line;
                    point_msg.reflectivity = point.reflectivity;
                    point_msg.offset_time = point.offset_time;
                    livox_msg.points.push_back(point_msg);
                }
                file.close();
                livox_msg.points.pop_back();
                livox_msg.point_num = livox_msg.points.size();
                livox_msg_right_next_ = make_pair(livox_right_file_list_[current_file_index + 1], livox_msg);
                
                cloud.points.pop_back();
                pcl::toROSMsg(cloud, publish_cloud);
                livox_right_next_ = make_pair(livox_right_file_list_[current_file_index + 1], publish_cloud);
            }

            previous_file_index = current_file_index;
        }
        if (livox_right_thread_.active_ == false)
            return;
    }
}

void ROSThread::OusterThread()
{
    int current_file_index = 0;
    int previous_file_index = 0;
    while (1)
    {
        std::unique_lock<std::mutex> ul(ouster_thread_.mutex_);
        ouster_thread_.cv_.wait(ul);
        if (ouster_thread_.active_ == false)
            return;
        ul.unlock();

        while (!ouster_thread_.data_queue_.empty())
        {
            auto data = ouster_thread_.pop();

            // publish data
            if (to_string(data) + ".bin" == ouster_next_.first)
            {
                // publish
                ouster_next_.second.header.stamp.fromNSec(data);
                ouster_next_.second.header.frame_id = "ouster";
                ouster_pub_.publish(ouster_next_.second);
            }
            else
            {
                //        cout << "Re-load right velodyne from path" << endl;
                // load current data
                pcl::PointCloud<OusterPoint> cloud;
                cloud.clear();
                sensor_msgs::PointCloud2 publish_cloud;
                string current_file_name = data_folder_path_ + "/lidar_data/ouster" + "/" + to_string(data) + ".bin";
                if (find(next(ouster_file_list_.begin(), max(0, previous_file_index - search_bound_)), ouster_file_list_.end(), to_string(data) + ".bin") != ouster_file_list_.end())
                {
                    ifstream file;
                    file.open(current_file_name, ios::in | ios::binary);
                    while (!file.eof())
                    {
                        OusterPoint point;
                        file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
                        file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
                        file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
                        file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
                        file.read(reinterpret_cast<char *>(&point.t), sizeof(uint32_t));
                        file.read(reinterpret_cast<char *>(&point.reflectivity), sizeof(uint16_t));
                        file.read(reinterpret_cast<char *>(&point.ring), sizeof(uint16_t));
                        file.read(reinterpret_cast<char *>(&point.ambient), sizeof(uint16_t));
                        file.read(reinterpret_cast<char *>(&point.range), sizeof(uint32_t));
                        cloud.points.push_back(point);
                    }
                    file.close();
                    cloud.points.pop_back();
                    pcl::toROSMsg(cloud, publish_cloud);
                    publish_cloud.header.stamp.fromNSec(data);
                    publish_cloud.header.frame_id = "ouster";
                    ouster_pub_.publish(publish_cloud);
                }
                previous_file_index = 0;
            }

            // load next data
            pcl::PointCloud<OusterPoint> cloud;
            cloud.clear();
            sensor_msgs::PointCloud2 publish_cloud;
            current_file_index = find(next(ouster_file_list_.begin(), max(0, previous_file_index - search_bound_)), ouster_file_list_.end(), to_string(data) + ".bin") - ouster_file_list_.begin();
            if (find(next(ouster_file_list_.begin(), max(0, previous_file_index - search_bound_)), ouster_file_list_.end(), ouster_file_list_[current_file_index + 1]) != ouster_file_list_.end())
            {
                string next_file_name = data_folder_path_ + "/lidar_data/ouster" + "/" + ouster_file_list_[current_file_index + 1];

                ifstream file;
                file.open(next_file_name, ios::in | ios::binary);
                while (!file.eof())
                {
                    OusterPoint point;
                    file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
                    file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
                    file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
                    file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
                    file.read(reinterpret_cast<char *>(&point.t), sizeof(uint32_t));
                    file.read(reinterpret_cast<char *>(&point.reflectivity), sizeof(uint16_t));
                    file.read(reinterpret_cast<char *>(&point.ring), sizeof(uint16_t));
                    file.read(reinterpret_cast<char *>(&point.ambient), sizeof(uint16_t));
                    file.read(reinterpret_cast<char *>(&point.range), sizeof(uint32_t));
                    cloud.points.push_back(point);
                }
                file.close();
                cloud.points.pop_back();
                pcl::toROSMsg(cloud, publish_cloud);
                ouster_next_ = make_pair(ouster_file_list_[current_file_index + 1], publish_cloud);
                // std::cout << "Ouster point size: " << cloud.size() << std::endl;
                // std::cout << "Front: " << cloud.front().getVector3fMap().transpose() << std::endl;
                // std::cout << "Back: " << cloud.back().getVector3fMap().transpose() << std::endl;
            }

            previous_file_index = current_file_index;
        }
        if (ouster_thread_.active_ == false)
            return;
    }
}

void ROSThread::CameraThread()
{
    int current_img_index = 0;
    int previous_img_index = 0;

    while (1)
    {
        std::unique_lock<std::mutex> ul(camera_thread_.mutex_);
        camera_thread_.cv_.wait(ul);
        if (camera_thread_.active_ == false)
            return;
        ul.unlock();

        while (!camera_thread_.data_queue_.empty())
        {
            auto data = camera_thread_.pop();

            // publish
            if (to_string(data) + ".png" == camera_next_img_.first)
            {
                cv_bridge::CvImage camera_msg;
                camera_msg.header.stamp.fromNSec(data);
                camera_msg.header.frame_id = "camera";
                camera_msg.encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
                camera_msg.image = camera_next_img_.second;

                camera_info_.header.stamp.fromNSec(data);
                camera_info_.header.frame_id = "camera";

                camera_pub_.publish(camera_msg.toImageMsg());
                camera_info_pub_.publish(camera_info_);
            }
            else
            {
                //        cout << "Re-load camera image from image path" << endl;
                string current_camera_name = data_folder_path_ + "/image_data" + "/" + to_string(data) + ".png";
                cv::Mat current_image;
                // current_left_image = imread(current_stereo_left_name, CV_LOAD_IMAGE_ANYDEPTH);
                // current_right_image = imread(current_stereo_right_name, CV_LOAD_IMAGE_ANYDEPTH);

                if (find(next(camera_file_list_.begin(), max(0, previous_img_index - search_bound_)), camera_file_list_.end(), to_string(data) + ".bin") != camera_file_list_.end())
                {

                    current_image = imread(current_camera_name, cv::IMREAD_ANYDEPTH);

                    cv_bridge::CvImage camera_msg;
                    camera_msg.header.stamp.fromNSec(data);
                    camera_msg.header.frame_id = "camera";
                    camera_msg.encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
                    camera_msg.image = current_image;

                    camera_info_.header.stamp.fromNSec(data);
                    camera_info_.header.frame_id = "camera";

                    camera_pub_.publish(camera_msg.toImageMsg());
                    camera_info_pub_.publish(camera_info_);
                }

                previous_img_index = 0;
            }

            // load next image
            current_img_index = find(next(camera_file_list_.begin(), max(0, previous_img_index - search_bound_)), camera_file_list_.end(), to_string(data) + ".png") - camera_file_list_.begin();
            if (find(next(camera_file_list_.begin(), max(0, previous_img_index - search_bound_)), camera_file_list_.end(), camera_file_list_[current_img_index + 1]) != camera_file_list_.end())
            {

                string next_camera_name = data_folder_path_ + "/image_data" + "/" + camera_file_list_[current_img_index + 1];
                cv::Mat naxt_camera_image;
                // next_left_image = imread(next_stereo_left_name, CV_LOAD_IMAGE_ANYDEPTH);
                // next_right_image = imread(next_stereo_right_name, CV_LOAD_IMAGE_ANYDEPTH);
                naxt_camera_image = imread(next_camera_name, cv::IMREAD_ANYDEPTH);

                if (!naxt_camera_image.empty())
                {
                    camera_next_img_ = make_pair(camera_file_list_[current_img_index + 1], naxt_camera_image);
                }
            }
            previous_img_index = current_img_index;
        }
        if (camera_thread_.active_ == false)
            return;
    }
}

int ROSThread::GetDirList(string dir, vector<string> &files)
{

    vector<string> tmp_files;
    struct dirent **namelist;
    int n;
    n = scandir(dir.c_str(), &namelist, 0, alphasort);
    if (n < 0)
        perror("scandir");
    else
    {
        while (n--)
        {
            if (string(namelist[n]->d_name) != "." && string(namelist[n]->d_name) != "..")
            {
                tmp_files.push_back(string(namelist[n]->d_name));
            }
            free(namelist[n]);
        }
        free(namelist);
    }

    for (auto iter = tmp_files.rbegin(); iter != tmp_files.rend(); iter++)
    {
        files.push_back(*iter);
    }
    return 0;
}

void ROSThread::FilePlayerStart(const std_msgs::BoolConstPtr &msg)
{
    if (auto_start_flag_ == true)
    {
        cout << "File player auto start" << endl;
        usleep(1000000);
        play_flag_ = false;
        emit StartSignal();
    }
}

void ROSThread::FilePlayerStop(const std_msgs::BoolConstPtr &msg)
{
    cout << "File player auto stop" << endl;
    play_flag_ = true;
    emit StartSignal();
}
void ROSThread::ResetProcessStamp(int position)
{
    if (position > 0 && position < 10000)
    {
        processed_stamp_ = static_cast<int64_t>(static_cast<float>(last_data_stamp_ - initial_data_stamp_) * static_cast<float>(position) / static_cast<float>(10000));
        reset_process_stamp_flag_ = true;
    }
}
