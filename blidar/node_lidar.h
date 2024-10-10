#ifndef NODE_LIDAR_H
#define NODE_LIDAR_H

#include <stdint.h>
#include <vector>
#include <string>
#include <atomic>
#include <memory>
#include <iostream>

#include "point_cloud_optimize.h"
#include "lidar_data_processing.h"
#include "serial_port.h"
#include "locker.h"
#include "timer.h"

using namespace std;

typedef struct
{
    int version = 4;
    string port = "/dev/sc_m_c";
    int m_SerialBaudrate = 230400;
    bool m_intensities = true;
    uint64_t m_PointTime = 1e9/3800;
    uint32_t trans_delay = 0;
    string frame_id = "laser_link";
    uint16_t frequency_max = 103;
    uint16_t frequency_min = 97;

}lidar_general_info_t;

typedef struct
{
    uint16_t point_check = 0;
    uint16_t point_check_part = 0;
    uint16_t point_check_all = 0;
    int blocked_size = 0;
    int lidar_zero_count = 0;
    bool blocked_judge = true;

}lidar_block_t;

typedef struct
{
    node_package package;
    node_packages packages;
    node_package_coin package_coin;

}lidar_package_t;

typedef struct
{
    uint32_t scan_time_t = 0;
    uint64_t scan_time_current = 0;
    uint64_t scan_time_record = 0;
    uint64_t system_start_time = 0;
    uint64_t last_encry_time = 0;
    uint64_t tim_scan_start = 0;
    uint64_t tim_scan_end = 0;
    uint64_t scan_start_time = 0;
    uint64_t scan_end_time = 0;
    uint64_t lidar_frequence_abnormal_time = 0;

}lidar_time_t;

/*robot structure information*/
typedef struct
{
    float install_to_zero = 90.0;
    int ROBOT_DIAMETER_mm;
    int LIDAR_ROBOT_CENTER_DISTANCE_mm;
    int LidarCoverBarNumber = 0;
    vector<LidarCoverAngleStr> LidarCoverAngle;

}lidar_robot_info_t;

/*Operational status of lidar node*/
typedef struct
{
    bool FilterEnable = true;           // Is filtering required?
    bool GyroCompensateEnable = false;  // Whether to perform rotation angle correction
    bool isConnected = false;
    bool slam_user = false;
    bool encry_lidar = false;
    bool optimize_enable = true;
    bool lidar_cover = false;
    bool disable_encry = false;
    bool able_exposure = false;
    bool low_exposure = false;
    bool lidar_ready = false;
    bool lidar_last_status =false;
    bool close_lidar = true;
    bool lidar_trap_restart = false;    //Lidar stuck in restart state
    bool lidar_restart_try = false;
    uint8_t lidar_abnormal_state = 0;

}lidar_status_t;

struct node_lidar_t
{
    node_info *scan_node_buf;      // store information about lidar points
    uint8_t *globalRecvBuffer;     // saving received lidar data
    lidar_block_t lidar_block;     // check for lidar blocking

    lidar_package_t scan_packages; // different versions of lidar data packages

    lidar_time_t lidar_time;

    lidar_general_info_t lidar_general_info;

    lidar_robot_info_t lidar_robot_info;

    lidar_status_t lidar_status;

    size_t scan_node_count = 0;

    uint8_t recv_encryp[8];
    uint8_t lidar_version[8];

    Point_cloud_optimize optimize_lidar;
    Lidar_Data_Processing lidar_data_processing;

    std::shared_ptr<Serial_Port> serial_port;
    Event _dataEvent;
    Locker _lock;

    bool data_calibration = false;
    float range_max;

    void initialization_node_lidar();

    ~node_lidar_t();
};
extern node_lidar_t node_lidar;

bool data_handling(LaserScan &outscan);
bool initialize(); 
bool lidar_state_judgment(bool &is_lidar_closed);
void send_lidar_data(LaserScan &outscan);
result_t grabScanData(uint32_t timeout = DEFAULT_TIMEOUT);
void flushSerial();
void cleanup_lidar_resources();

#endif
