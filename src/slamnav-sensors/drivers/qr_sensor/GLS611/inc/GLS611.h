#ifndef GLS611_H
#define GLS611_H

#include "slamnav_sensor_types.h"
#include "my_utils.h"

#include <QObject>
#include <QTcpSocket>
#include <QByteArray>
#include <QHostAddress>
#include <QDir>
#include <QSettings>
#include <QTimer>

// module
#include "config.h"
#include "logger.h"

class GLS611 : public QObject
{
  Q_OBJECT
public:
  explicit GLS611(QObject *parent = nullptr);
  ~GLS611();
  std::mutex mtx;

  void init();
  void load_codes();

  cv::Mat get_plot_img();
  QString get_bqr_info_str();

  TIME_POSE_ID get_cur_tpi();
  BQR_INFO get_cur_bqr();

  void process_data(QString data);

  // other modules
  CONFIG *config = NULL;
  LOGGER *logger = NULL;

  QTcpSocket socket_bqr;
  QTimer reconnect_timer;
  QTimer check_recv_timer;

  std::atomic<bool> is_connected = {false};
  std::atomic<bool> is_recv_data = {false};
  std::atomic<bool> is_init = {true};

  std::atomic<double> err_x = {0.0};
  std::atomic<double> err_y = {0.0};
  std::atomic<double> err_th = {0.0};

  std::atomic<double> cum_err_x = {0.0};
  std::atomic<double> cum_err_y = {0.0};
  std::atomic<double> cum_err_th = {0.0};

  std::atomic<double> offset_x = {0.0};
  std::atomic<double> offset_y = {0.0};
  std::atomic<double> offset_th = {0.0};

  std::atomic<double> send_code_x = {0.0};
  std::atomic<double> send_code_y = {0.0};
  std::atomic<double> send_code_th = {0.0};
  std::atomic<int> code_num = {0};

  const size_t storage_size = 10;
  std::vector<double> storage_err_x;
  std::vector<double> storage_err_y;
  std::vector<double> storage_err_th;

  BQR_INFO cur_bqr;

  cv::Mat plot_img;
  std::map<QString, cv::Vec2d> ref_bqr_codes;

  QString recv_buf = "";

  TIME_POSE_ID cur_tpi;

private Q_SLOTS:
  void connected();
  void disconnected();
  void readyread();

  void reconnect_loop();
  void check_recv_loop();
private:
};

#endif // GLS611_H
