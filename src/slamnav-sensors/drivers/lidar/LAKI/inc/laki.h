#ifndef LAKI_H
#define LAKI_H

// defines
#include "slamnav_sensor_types.h"
#include "my_utils.h"

// LAKI Lidar
#include "LakiBeamHTTP.h"
#include "LakiBeamUDP.h"

// module
#include "config.h"
#include "logger.h"
#include "mobile.h"

// qt
#include <QObject>

class LAKI : public QObject
{
  Q_OBJECT
  Q_DISABLE_COPY(LAKI)

public:
  // make singleton
  static LAKI* instance(QObject* parent = nullptr);

  // start laki module
  void open();

  // stop laki module
  void close();

  // sync between mobile, laki
  void sync(int idx);

  /***********************
   * interface funcs
   ***********************/
  QString get_info_text(int idx);
  RAW_FRAME get_cur_raw(int idx);
  bool get_is_connected(int idx);
  bool get_is_sync(int idx);
  void set_is_sync(int idx, bool val);

  bool try_pop_raw_que(int idx, RAW_FRAME& frm);

  /***********************
   * set other modules
   ***********************/
  void set_config_module(CONFIG* _config);
  void set_logger_module(LOGGER* _logger);
  void set_mobile_module(MOBILE *_mobile);

private:
  explicit LAKI(QObject *parent = nullptr);
  ~LAKI();

  // mutex
  std::shared_mutex mtx;

  // other modules
  CONFIG* config;
  LOGGER* logger;
  MOBILE* mobile;

  // LAKI lidar objects
  std::unique_ptr<LakiBeamHTTP> laki_http[2];
  std::unique_ptr<LakiBeamUDP> laki_udp[2];

  // extrinsics
  Eigen::Matrix4d pts_tf[2];

  // grab loop
  std::atomic<bool> grab_flag[2] = {false, false};
  std::array<std::unique_ptr<std::thread>, 2> grab_thread;
  void grab_loop(int idx);

  // flags
  std::atomic<bool> is_connected[2] = {false, false};
  std::atomic<bool> is_sync[2] = {false, false};
  std::atomic<bool> is_synced[2] = {false, false};

  // params
  std::atomic<double> offset_t[2] = {0, 0};
  std::atomic<double> cur_raw_t[2] = {0, 0};
  std::atomic<int> cur_pts_num[2] = {0, 0};

  // watchdog
  std::atomic<double> last_data_time[2] = {0, 0};
  std::atomic<int> reconnect_count[2] = {0, 0};

  // storage
  tbb::concurrent_queue<RAW_FRAME> raw_que[2];

  // vars
  const double angle_offset = 10.0; // LAKI:8.0, SICK:10.0
  RAW_FRAME cur_raw[2];
};

#endif // LAKI_H
