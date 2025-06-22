#ifndef SIM_H
#define SIM_H

#include "global_defines.h"
#include "my_utils.h"

#include "config.h"
#include "logger.h"
#include "mobile.h"
#include "lidar_2d.h"
#include "lidar_3d.h"
#include "localization.h"
#include "unimap.h"

#include <QObject>

class SIM : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(SIM)

public:
    // make singleton
    static SIM* instance(QObject* parent = nullptr);

    void start();
    void stop();

    void set_cur_tf(Eigen::Matrix4d tf);
    Eigen::Matrix4d get_cur_tf();

    /***********************
     * set other modules
     ***********************/
    void set_config_module(CONFIG* _config);
    void set_logger_module(LOGGER* _logger);
    void set_unimap_module(UNIMAP* _unimap);
    void set_mobile_module(MOBILE* _mobile);
    void set_lidar_2d_module(LIDAR_2D* _lidar_2d);
    void set_lidar_3d_module(LIDAR_3D* _lidar_3d);
    void set_localization_module(LOCALIZATION* _loc);

private:
    explicit SIM(QObject *parent = nullptr);
    ~SIM();
    std::mutex mtx;

    // other modules
    CONFIG* config;
    LOGGER* logger;
    MOBILE* mobile;
    UNIMAP* unimap;
    LIDAR_2D* lidar_2d;
    LIDAR_3D* lidar_3d;
    LOCALIZATION* loc;

    double calc_limit(double v0, double v1, double v_acc_limit, double dt, double tol=0.0001);

    std::atomic<bool> simulation_flag;
    std::unique_ptr<std::thread> simulation_thread;
    void simulation_loop();

    Eigen::Matrix4d cur_tf;

};

#endif // SIM_H
