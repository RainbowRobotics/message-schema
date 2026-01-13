#ifndef AIRY_H
#define AIRY_H

// global defines
#include "slamnav_sensor_types.h"
#include "my_utils.h"

// modules
#include "config.h"
#include "logger.h"

// imu filter
#include "imu_filter.h"

// airy
#include "rs_driver/api/lidar_driver.hpp"
#include "rs_driver/msg/point_cloud_msg.hpp"
#include "rs_driver/msg/imu_data_msg.hpp"

#include <QObject>


class AIRY : public QObject
{
  Q_OBJECT
  Q_DISABLE_COPY(AIRY)

private:
  // airy msg_data type declear
  typedef PointXYZI PointT;
  typedef PointCloudT<PointT> PointCloudMsg;
  typedef robosense::lidar::ImuData ImuData;

  // common parameter declear
  struct Common
  {
    double lidar_min_range, lidar_max_range;
    double robot_size_x_min, robot_size_x_max;
    double robot_size_y_min, robot_size_y_max;
    double robot_size_z_min, robot_size_z_max;
  };

  // Created 'Client' class using the AIRY LiDAR API.
  class Client
  {
  public:
    Client(int idx, std::shared_ptr<Common> param, robosense::lidar::RSDriverParam& rs_param)
      : _idx(idx), _thread_flag(false), is_sync(false), _p(param), _rs_param(rs_param) {}

    // Checked the AIRY LiDAR driver’s callback functions and parameters.
    bool init();
    // Created Cloud & IMU thread to process data received from the AIRY LiDAR.
    void start();
    // stop thread
    void stop();

    LVX_FRM get_cur_raw();
    IMU get_cur_imu();
    std::vector<IMU> get_imu_storage();
    bool try_pop_frm_que(LVX_FRM& frm);

  public:
    // front_lidar : 0, back_lidar : 1
    int _idx;

    // lidar extrinsic parameter
    Eigen::Matrix4d pts_tf;
    Eigen::Matrix3d pts_R;
    Eigen::Vector3d pts_t;
    Eigen::Matrix4d imu_tf;
    Eigen::Matrix3d imu_R;
    Eigen::Vector3d imu_t;

    std::atomic<bool> _thread_flag;
    std::atomic<bool> is_sync;
    std::atomic<int> cur_pts_num;
    std::atomic<int> frame_que_size;
    std::atomic<double> offset_t;
    std::atomic<double> cur_frm_t;
    std::atomic<double> cur_imu_t;

  private:
    // Function to reserve the memory of point cloud data.
    std::shared_ptr<PointCloudMsg> driver_get_cloud_cb();
    // Function to save point cloud data
    void driver_return_cloud_cb(std::shared_ptr<PointCloudMsg> msg);

    // Function to reserve the memory of IMU data.
    std::shared_ptr<ImuData> driver_get_imu_cb();
    // Function to save IMU data.
    void driver_return_imu_cb(std::shared_ptr<ImuData> msg);


    // thread : process point cloud data & update storage
    void point_cloud_callback();
    // thread : process IMU data & update storage
    void imu_data_callback();

  private:
    // mutex
    std::mutex _lidar_mtx;
    std::mutex _imu_mtx;

    std::shared_ptr<Common> _p;
    robosense::lidar::RSDriverParam _rs_param;
    robosense::lidar::LidarDriver<PointCloudMsg> _driver;
    robosense::lidar::SyncQueue<std::shared_ptr<PointCloudMsg>> _free_cloud_queue;
    robosense::lidar::SyncQueue<std::shared_ptr<PointCloudMsg>> _stuffed_cloud_queue;
    robosense::lidar::SyncQueue<std::shared_ptr<ImuData>> _free_imu_queue;
    robosense::lidar::SyncQueue<std::shared_ptr<ImuData>> _stuffed_imu_queue;

    std::thread _thread_cloud;
    std::thread _thread_imu;

    LVX_FRM _cur_raw;
    tbb::concurrent_queue<LVX_FRM> _frm_que;

    IMU _cur_imu;
    std::vector<IMU> _imu_storage;
    imu_tools::ComplementaryFilter _imu_filter;
  };

public:
  // make singleton
  static AIRY* instance(QObject* parent = nullptr);

  // start airy module
  void open();

  // stop airy module
  void close();

  /***********************
   * interface funcs
   ***********************/
  QString get_info_text(int idx);
  LVX_FRM get_cur_raw(int idx);
  IMU get_cur_imu(int idx);
  std::vector<IMU> get_imu_storage(int idx);

  bool get_is_connected(int idx);
  void set_is_connected(int idx, bool val);
  bool get_is_sync(int idx);
  void set_is_sync(int idx, bool val);

  int get_time_type(int idx);
  bool try_pop_frm_que(int idx, LVX_FRM& frm);

  /***********************
   * set other modules
   ***********************/
  void set_config_module(CONFIG* _config);
  void set_logger_module(LOGGER* _logger);

private:
  explicit AIRY(QObject *parent = nullptr);
  ~AIRY();

  // other modules
  CONFIG* config;
  LOGGER* logger;

  // for multi lidar
  std::array<uint32_t, 2> airy_handles = {0, 0};

  // grab loop
  std::atomic<bool> grab_flag = {false};
  std::unique_ptr<std::thread> grab_thread;

  // params
  std::atomic<bool> is_connected = {false};

  Common client_param;
  std::vector<std::unique_ptr<Client>> client_ptr;

  int get_airy_idx(uint32_t handle);
  void grab_loop();
  void read_common_param();
  void read_airy_json(QJsonDocument& doc, robosense::lidar::RSDriverParam& rs_param, int idx);

private:
  // not used
  const double airy_frm_dt = 0.1;
};

#endif // AIRY_H
