#ifndef POLICY_H
#define POLICY_H

// defines
#include "slamnav_navigation_types.h"
#include "my_utils.h"

// module
#include "config.h"
#include "logger.h"
#include "unimap.h"
#include "localization.h"

#include <QObject>

class POLICY : public QObject
{
  Q_OBJECT
  Q_DISABLE_COPY(POLICY)
public:
  // make singleton
  static POLICY* instance(QObject* parent = nullptr);

  // initialization policy module
  void init();

  // open policy module
  void open();

  // close policy module
  void close();

  /***********************
   * interface funcs
   ***********************/
  NODE get_cur_node();
  NODE get_cur_zone();
  LINK get_cur_link();

  /***********************
   * set other modules
   ***********************/
  void set_config_module(CONFIG* _config);
  void set_logger_module(LOGGER* _logger);
  void set_unimap_module(UNIMAP* _unimap);
  void set_localization_module(LOCALIZATION* _localization);

  std::vector<PATH> drive_policy(PATH path);
  void speed_policy(const PATH& path, std::vector<double>& ref_v);

private:
  explicit POLICY(QObject *parent = nullptr);
  ~POLICY();

  // mutex
  std::mutex mtx;

  // other modules
  CONFIG* config;
  LOGGER* logger;
  UNIMAP* unimap;
  LOCALIZATION *loc;

  // node loop
  std::atomic<bool> node_flag = {false};
  std::unique_ptr<std::thread> node_thread;
  void node_loop();

  // link loop
  std::atomic<bool> link_flag = {false};
  std::unique_ptr<std::thread> link_thread;
  void link_loop();

  // policy loop
  std::atomic<bool> policy_flag = {false};
  std::unique_ptr<std::thread> policy_thread;
  void policy_loop();

  void apply_slow(bool on);
  void apply_fast(bool on);
  void apply_warning_beep(bool on);
  void apply_ignore_2d(bool on);
  void apply_ignore_3d(bool on);
  void apply_ignore_cam(bool on);
  void apply_ignore_obs_2d(bool on);
  void apply_ignore_obs_3d(bool on);
  void apply_ignore_obs_cam(bool on);

  NODE cur_node;
  NODE cur_zone;
  LINK cur_link;

  PATH global_path;

};

#endif // POLICY_H
