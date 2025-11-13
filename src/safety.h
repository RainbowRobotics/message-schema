#ifndef SAFETY_H
#define SAFETY_H

#include "global_defines.h"
#include "my_utils.h"

// other modules
#include "obsmap.h"
#include "localization.h"
#include "config.h"
#include "logger.h"

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

private:
    static SAFETY* m_instance;

public:
    // make singleton
    static SAFETY* instance(QObject* parent = nullptr);

    // start safety module
    void start();

    // start safety module
    void init();

    // stop safety loop
    void stop();

    // set other modules
    void set_obsmap_module(OBSMAP* _obsmap);
    void set_localization_module(LOCALIZATION* _localization);
    void set_config_module(CONFIG* _config);
    void set_logger_module(LOGGER* _logger);
    void set_mobile_module(MOBILE* _mobile);
    void set_unimap_module(UNIMAP* _unimap);

    cv::Mat get_safety_map();

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
    UNIMAP* unimap;
    OBSMAP* obsmap;
    LOCALIZATION* loc;
    
    // grid map
    cv::Mat safety_map;
    
    // control loop
    std::atomic<bool> safety_flag = {false};
    std::unique_ptr<std::thread> safety_thread;
    void safety_loop();

    // monitoring field
    std::vector<MonitoringField> safety_fields;  

Q_SIGNALS:

};

#endif // SAFETY_H
