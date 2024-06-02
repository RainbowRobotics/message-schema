#ifndef OBSMAP_H
#define OBSMAP_H

#include "global_defines.h"
#include "utils.h"

#include "config.h"
#include "logger.h"

#include <QObject>

class OBSMAP : public QObject
{
    Q_OBJECT
public:
    explicit OBSMAP(QObject *parent = nullptr);
    ~OBSMAP();
    std::mutex mtx;

    CONFIG *config = NULL;
    LOGGER *logger = NULL;

    void init();
    void update_obs_map(TIME_POSE_PTS& tpp);

    Eigen::Matrix4d obs_tf;
    cv::Mat map;

    int w = 1000;
    int h = 1000;
    int cx = 500;
    int cy = 500;
    double gs = 0.05;

    const double P_hit = 0.6;
    const double P_miss = 0.4;
    const double P_min = 0.1;
    const double P_max = 0.9;

    cv::Vec2i xy_to_uv(double x, double y);
    cv::Vec2d uv_to_xy(int u, int v);

    double odds(double p);
    double odds_inv(double odd);
    double clamp(double p, double min, double max);
    double prob(double m_old, double P);

Q_SIGNALS:

};

#endif // OBSMAP_H
