#ifndef SIM_H
#define SIM_H

#include "global_defines.h"
#include "utils.h"

#include "config.h"
#include "logger.h"
#include "mobile.h"
#include "lidar_2d.h"
#include "slam_2d.h"
#include "unimap.h"

#include <QObject>

class SIM : public QObject
{
    Q_OBJECT
public:
    explicit SIM(QObject *parent = nullptr);
    ~SIM();
    std::mutex mtx;

    // other modules
    CONFIG *config = NULL;
    LOGGER *logger = NULL;
    MOBILE *mobile = NULL;
    LIDAR_2D *lidar = NULL;
    SLAM_2D *slam = NULL;
    UNIMAP *unimap = NULL;

    void start();
    void stop();

    std::atomic<bool> a_flag;
    std::thread* a_thread = NULL;
    void a_loop();

Q_SIGNALS:


};

#endif // SIM_H
