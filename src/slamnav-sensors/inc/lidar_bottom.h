#ifndef SLAMNAV2_LIDAR_BOTTOM_H
#define SLAMNAV2_LIDAR_BOTTOM_H

// global defines
#include "slamnav_sensor_types.h"
#include "my_utils.h"

// modules
#include "config.h"
#include "logger.h"
#include "mobile.h"
#include "lidar/COIN_D4/inc/coin_d4.h"

#include <QObject>


class LIDAR_BOTTOM : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(LIDAR_BOTTOM)

public:

    // make singleton
    static LIDAR_BOTTOM* instance(QObject* parent = nullptr);

    // initialization lidar_2d module
    void init();

    // open lidar bottom module as written in config.json
    void open();

    // close lidar bottom module
    void close();

    bool get_is_connected();
    std::vector<Eigen::Vector3d> get_cur_pts();

    /***********************
     * set other modules
     ***********************/
    void set_config_module(CONFIG* _config);
    void set_logger_module(LOGGER* _logger);
    void set_mobile_module(MOBILE* _mobile);

private:
    explicit LIDAR_BOTTOM(QObject *parent = nullptr);
    ~LIDAR_BOTTOM();

    // mutex
    std::shared_mutex mtx;

    // other modules
    CONFIG* config;
    LOGGER* logger;
    MOBILE* mobile;
    COIN_D4* coin_d4;

    // params
    std::atomic<bool> is_connected = {false};

    // deskewing loop
    std::atomic<bool> deskewing_flag = {false};
    std::unique_ptr<std::thread> deskewing_thread;
    void deskewing_loop();



};

#endif //SLAMNAV2_LIDAR_BOTTOM_H
