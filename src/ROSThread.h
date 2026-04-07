#ifndef VIEWER_ROS_H
#define VIEWER_ROS_H

#include <fstream>
#include <iostream>
#include <sstream>
#include <QObject>
#include <QThread>
#include <QMutex>
#include <QPixmap>
#include <QVector>
#include <QVector3D>
#include <QDateTime>
#include <QReadLocker>
#include <QPainter>
#include <QLabel>
#include <algorithm>
#include <ros/ros.h>
#include <ros/time.h>
#include <rosbag/bag.h>
#include <ros/transport_hints.h>

#include <Eigen/Dense>

// #include <opencv/cv.h>
// #include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <image_transport/transport_hints.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <rosgraph_msgs/Clock.h>

#include <camera_info_manager/camera_info_manager.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <thread>
#include <mutex>
#include <condition_variable>

// pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sys/types.h>

#include <algorithm>
#include <iterator>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <gwnav_ros_driver/GWNav.h>
#include <livox_ros_driver/CustomMsg.h>
#include <livox_ros_driver/CustomPoint.h>

#include "m2cs_player/color.h"
#include "m2cs_player/datathread.h"
#include "m2cs_player/point_types.h"

using namespace std;
using namespace cv;

class ROSThread : public QThread
{
    Q_OBJECT

public:
    explicit ROSThread(QObject *parent = 0, QMutex *th_mutex = 0);
    ~ROSThread();
    void ros_initialize(ros::NodeHandle &n);
    void run();
    QMutex *mutex_;
    ros::NodeHandle nh_;
    ros::NodeHandle camera_nh_;

    boost::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

    int64_t initial_data_stamp_;
    int64_t last_data_stamp_;

    bool auto_start_flag_;
    bool tf_flag_;
    bool nav_flag_;
    bool ouster_flag_;
    bool livox_left_flag_;
    bool livox_right_flag_;
    bool camera_flag_;

    int stamp_show_count_;

    bool play_flag_;
    bool pause_flag_;
    bool loop_flag_;
    bool stop_skip_flag_;
    double play_rate_;
    string data_folder_path_;

    void Ready();
    void ResetProcessStamp(int position);

signals:
    void StampShow(quint64 stamp);
    void StartSignal();

private:
    int search_bound_;

    bool stereo_active_;
    bool omni_active_;

    ros::Subscriber start_sub_;
    ros::Subscriber stop_sub_;

    ros::Publisher nav_pub_;
    ros::Publisher gps_pub_;
    ros::Publisher imu_pub_;

    ros::Publisher livox_left_pub_;
    ros::Publisher livox_right_pub_;
    ros::Publisher livox_msg_left_pub_;
    ros::Publisher livox_msg_right_pub_;
    ros::Publisher ouster_pub_;

    ros::Publisher livox_left_imu_pub_;
    ros::Publisher livox_right_imu_pub_;
    ros::Publisher ouster_imu_pub_;

    ros::Publisher camera_pub_;
    ros::Publisher camera_info_pub_;

    ros::Publisher clock_pub_;

    int64_t prev_clock_stamp_;

    multimap<int64_t, string> data_stamp_;
    map<int64_t, gwnav_ros_driver::GWNav> nav_data_;
    map<int64_t, sensor_msgs::NavSatFix> gps_data_;
    map<int64_t, sensor_msgs::Imu> imu_data_;
    map<int64_t, sensor_msgs::Imu> livox_left_imu_data_;
    map<int64_t, sensor_msgs::Imu> livox_right_imu_data_;
    map<int64_t, sensor_msgs::Imu> ouster_imu_data_;

    DataThread<int64_t> data_stamp_thread_;
    DataThread<int64_t> nav_thread_;
    DataThread<int64_t> gps_thread_;
    DataThread<int64_t> imu_thread_;
    DataThread<int64_t> livox_left_imu_thread_;
    DataThread<int64_t> livox_right_imu_thread_;
    DataThread<int64_t> ouster_imu_thread_;
    DataThread<int64_t> livox_left_thread_;
    DataThread<int64_t> livox_right_thread_;
    DataThread<int64_t> ouster_thread_;
    DataThread<int64_t> camera_thread_;

    map<int64_t, int64_t> stop_period_; // start and stop stamp

    void DataStampThread();
    void NavThread();
    void GpsThread();
    void ImuThread();
    void LivoxLeftImuThread();
    void LivoxRightImuThread();
    void OusterImuThread();
    void LivoxLeftThread();
    void LivoxRightThread();
    void OusterThread();
    void CameraThread();

    void FilePlayerStart(const std_msgs::BoolConstPtr &msg);
    void FilePlayerStop(const std_msgs::BoolConstPtr &msg);

    vector<string> livox_left_file_list_;
    vector<string> livox_right_file_list_;
    vector<string> ouster_file_list_;
    vector<string> camera_file_list_;

    ros::Timer timer_;
    void TimerCallback(const ros::TimerEvent &);
    int64_t processed_stamp_;
    int64_t pre_timer_stamp_;

    bool reset_process_stamp_flag_;

    pair<string, livox_ros_driver::CustomMsg> livox_msg_left_next_;
    pair<string, livox_ros_driver::CustomMsg> livox_msg_right_next_;
    pair<string, sensor_msgs::PointCloud2> livox_left_next_;
    pair<string, sensor_msgs::PointCloud2> livox_right_next_;
    pair<string, sensor_msgs::PointCloud2> ouster_next_;
    pair<string, cv::Mat> camera_next_img_;
    sensor_msgs::CameraInfo camera_info_;

    Eigen::Matrix3d livox_left_to_ouster_rot;
    Eigen::Quaterniond livox_left_to_ouster_quat;
    Eigen::Vector3d livox_left_to_ouster_trans;

    Eigen::Matrix3d livox_right_to_ouster_rot;
    Eigen::Quaterniond livox_right_to_ouster_quat;
    Eigen::Vector3d livox_right_to_ouster_trans;

    Eigen::Matrix3d camera_to_ouster_rot;
    Eigen::Quaterniond camera_to_ouster_quat;
    Eigen::Vector3d camera_to_ouster_trans;

    Eigen::Matrix3d ouster_to_gwnav_rot;
    Eigen::Quaterniond ouster_to_gwnav_quat;
    Eigen::Vector3d ouster_to_gwnav_trans;

    int GetDirList(string dir, vector<string> &files);

public slots:
};

#endif // VIEWER_LCM_H
