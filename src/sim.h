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
public:
    explicit SIM(QObject *parent = nullptr);
    ~SIM();
    std::recursive_mutex mtx;

    // other modules
    CONFIG *config = NULL;
    LOGGER *logger = NULL;
    MOBILE *mobile = NULL;
    LIDAR_2D *lidar_2d = NULL;
    LIDAR_3D *lidar_3d = NULL;
    LOCALIZATION *loc = NULL;
    UNIMAP *unimap = NULL;\

    void start();
    void stop();

    std::atomic<bool> a_flag;
    std::thread* a_thread = NULL;
    void a_loop();

    Eigen::Matrix4d cur_tf;
    void set_cur_tf(Eigen::Matrix4d tf);
    Eigen::Matrix4d get_cur_tf();

Q_SIGNALS:


};

#endif // SIM_H
