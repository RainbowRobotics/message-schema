#ifndef SAFETY_H
#define SAFETY_H

#include "slamnav_navigation_types.h"
#include "my_utils.h"

// other modules
#include "config.h"
#include "logger.h"
#include "mobile.h"
#include "obsmap.h"

// qt
#include <QObject>
#include <QThread>
#include <QTimer>
#include <QMutex>

// standard
#include <vector>
#include <atomic>


class SAFETY : public QObject
{
  Q_OBJECT
  Q_DISABLE_COPY(SAFETY)

public:
  // make singleton
  static SAFETY* instance(QObject* parent = nullptr);

  // start safety module
  void init();

  // start safety module
  void open();

  // set other modules
  void set_config_module(CONFIG* _config);
  void set_logger_module(LOGGER* _logger);
  void set_mobile_module(MOBILE* _mobile);
  void set_obsmap_module(OBSMAP* _obsmap);

  cv::Mat get_safety_map();
  std::vector<int> get_field_collision();

  std::vector<Eigen::Matrix4d> calc_trajectory(Eigen::Vector3d cur_vel, double dt, double predict_t, Eigen::Matrix4d _cur_tf);

private:
  explicit SAFETY(QObject *parent = nullptr);
  ~SAFETY();

  // mutex
  std::shared_mutex mtx;

  // other modules
  CONFIG* config;
  LOGGER* logger;
  MOBILE* mobile;
  OBSMAP* obsmap;

  // control loop
  std::atomic<bool> safety_flag = {false};
  std::unique_ptr<std::thread> safety_thread;
  void safety_loop();

  // grid map
  cv::Mat safety_map;

  // monitoring field
  std::vector<std::vector<Eigen::Vector3d>> fields;
  std::vector<int> field_collision;

Q_SIGNALS:

};

#endif // SAFETY_H
