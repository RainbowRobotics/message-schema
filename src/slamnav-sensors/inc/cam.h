#ifndef CAM_H
#define CAM_H

// global defines
#include "slamnav_sensor_types.h"
#include "my_utils.h"

// modules
#include "config.h"
#include "logger.h"
#include "mobile.h"

#include "cam/ORBBEC/inc/ORBBEC.h"

#include <QObject>

class CAM : public QObject
{
  Q_OBJECT
  Q_DISABLE_COPY(CAM)

public:
  // make singleton
  static CAM* instance(QObject* parent = nullptr);

  // initialization cam module
  void init();

  // open cam module as written in config.json
  void open();

  // close cam module
  void close();

  // restart cam module
  void restart(int idx);

  /***********************
   * interface funcs
   ***********************/
  QString get_info_text();      // get all cam info
  QString get_cur_state();      // get current state
  TIME_IMG get_time_img(int idx);   // get current image (with time)
  TIME_PTS get_scan(int idx);     // get current depth (with time)

  bool get_is_param_loaded(int idx);
  CAM_INTRINSIC get_intrinsic(int idx);
  Eigen::Matrix4d get_extrinsic(int idx);

  bool get_connection(int idx);
  double get_process_time_post(int idx);

  // interface func (set)
  void set_cur_state(QString str);
  void set_sync_flag(bool flag);
  void set_is_connected(bool val);

  /***********************
   * set other modules
   ***********************/
  void set_config_module(CONFIG* _config);
  void set_logger_module(LOGGER* _logger);
  void set_mobile_module(MOBILE* _mobile);

  CAM_INTRINSIC string_to_intrinsic(QString str);

  std::vector<bool> get_rtsp_flag();

  // params
  std::atomic<bool> is_connected[max_cam_cnt];
  std::atomic<double> cam_t[max_cam_cnt];

private:
  explicit CAM(QObject *parent = nullptr);
  ~CAM();

  // mutex
  std::mutex mtx;

  // other modules
  CONFIG* config;
  LOGGER* logger;
  MOBILE* mobile;
  ORBBEC* orbbec;

  // post process loop
  std::atomic<bool> post_process_flag[max_cam_cnt];
  std::array<std::unique_ptr<std::thread>, max_cam_cnt> post_process_thread;
  void post_process_loop(int idx);

  // rtsp loop (Real Time Streaming Protocol to RRS)
  std::atomic<bool> rtsp_flag = {false};
  std::unique_ptr<std::thread> rtsp_thread;
  void rtsp_loop();

  // func
  TIME_PTS filter_radius_outlier(const TIME_PTS &tp, double radius, int min_neighbors, bool USE_ROR, bool USE_CLUSTER);
  TIME_PTS filter_simple_range(const TIME_PTS &tp);

  // params
  std::atomic<double> process_time_post[max_cam_cnt];

  TIME_IMG cur_time_img[max_cam_cnt];
  TIME_PTS cur_scan[max_cam_cnt];

  // for rrs. rtsp status check
  std::vector<bool> rtsp_cam_status;

  tbb::concurrent_queue<FRAME> merged_que;

Q_SIGNALS:

};

#endif // CAM_H
