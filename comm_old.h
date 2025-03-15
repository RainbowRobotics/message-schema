/**
 * @file comm_old.h
 * @brief 이 파일은 KAI 사천 드릴링머신의 상부 통신기능을 제공하는 헤더 파일입니다.
 * @author JaeYeol Song
 * @date 2025-02-19
 */

#ifndef COMM_OLD_H
#define COMM_OLD_H

// global defines
#include "global_defines.h"
#include "my_utils.h"

// other modules
#include "config.h"
#include "logger.h"
#include "mobile.h"
#include "lidar_2d.h"
#include "bqr_sensor.h"
#include "slam_2d.h"
#include "unimap.h"
#include "obsmap.h"
#include "autocontrol.h"
#include "dockcontrol.h"

// qt
#include <QObject>
#include <QTimer>
#include <QTcpServer>
#include <QTcpSocket>
#include <QHostAddress>
#include <QTime>
#include <QProcess>
#include <QDebug>

enum KAI_FAIL_LIST
{
    FAIL_AUTO_INVALID_PARAMS_SIZE=0,
    FAIL_AUTO_INVALID_NODE=1,
    FAIL_DOCING_INVALID_PARAMS_SIZE=2,
    FAIL_DOCKING_EXCEED_RANGE=3,
    FAIL_DOCKING_CODE_READER_NOT_CONNECTED=4,
    FAIL_DOCKING_CODE_NOT_DETECTED=5,
    FAIL_DOCKING_INVALID_NODE=6,
    FAIL_DOCKING_INVALID_PARAMS=7,
    FAIL_UNIMAP_INVALID_PATH=8,
    FAIL_UNIMAP_NO_FOUND_CLOUD_TOPO=9,
    FAIL_UNIMAP_LOAD_FAIL=10,
    FAIL_LOC_INVALID_PARAMS_SIZE=11,
    FAIL_LOC_INVALID_PATH=12,
    FAIL_LOC_NO_INIT=13,
    FAIL_RECKONING_INVALID_PARAMS_SIZE=14,
};

enum MOVE_STATE
{
    MOVE_STOP=0,
    MOVE_RUN=1,
};

class COMM_OLD : public QObject
{
    Q_OBJECT
public:
    explicit COMM_OLD(QObject *parent = nullptr);
    ~COMM_OLD();

    // mutex
    std::mutex mtx;
    std::mutex mtx_data_client;

    // other modules
    QObject *main = NULL;
    CONFIG *config = NULL;
    LOGGER *logger = NULL;
    MOBILE *mobile = NULL;
    LIDAR_2D *lidar = NULL;
    BQR_SENSOR *bqr = NULL;
    SLAM_2D *slam = NULL;
    UNIMAP *unimap = NULL;
    OBSMAP *obsmap = NULL;
    AUTOCONTROL *ctrl = NULL;
    DOCKCONTROL *dctrl = NULL;

    QTcpServer cmd_server;
    QTcpServer data_server;

    std::atomic<bool> is_cmd_connected = {false};
    std::atomic<bool> is_data_connected = {false};

    QTimer auto_timer;
    QTimer docking_timer;
    QTimer cmd_timer;

    QString last_goal_id = "";

    std::atomic<int> cmd_client_cnt = {0};
    std::atomic<int> data_client_cnt = {0};
    std::atomic<bool> is_busy = {false};

    void open();
    void log_fail(int err_code, const QString& msg);

    QString cur_msg = "";

    QString get_req_data();
    QString get_req_simple();

    QString get_kai_loc_state();
    QString get_kai_auto_state();
    QString get_kai_docking_state();
    QString get_kai_obs_state();

    void processing_command(QString msg);

    // auto init
    std::atomic<bool> auto_init_flag = {false};
    std::thread *auto_init_thread = NULL;
    void auto_init_loop();

Q_SIGNALS:
    void signal_cmd_recv(QString data_str);
    void signal_auto_init_start();

private Q_SLOTS:
    void cmd_connected();
    void data_connected();

    void cmd_disconnected();
    void data_disconnected();

    void cmd_readyRead();
    void data_readyRead();

    void slot_cmd_recv(QString str);

    void auto_loop();
    void docking_loop();
    void cmd_loop();

    void slot_semiautoinit_successed();
    void slot_semiautoinit_failed();

private:
    QTcpSocket* cmd_client = nullptr;
    tbb::concurrent_queue<QString> cmd_que;
    std::map<QString, QTcpSocket*> data_clients;
};

#endif // COMM_OLD_H
