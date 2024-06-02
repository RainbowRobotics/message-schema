#include "obsmap.h"

OBSMAP::OBSMAP(QObject *parent)
    : QObject{parent}
{
    obs_tf.setIdentity();
}

OBSMAP::~OBSMAP()
{

}

void OBSMAP::init()
{
    // init
    obs_tf.setIdentity();

    w = (config->OBS_MAP_RANGE/config->OBS_MAP_GRID_SIZE)*2.5;
    h = w;
    cx = w/2;
    cy = h/2;
    gs = config->OBS_MAP_GRID_SIZE;

    map = cv::Mat(h, w, CV_64F, cv::Scalar(0));
}

void OBSMAP::update_obs_map(TIME_POSE_PTS& tpp)
{
    Eigen::Matrix4d G0 = obs_tf;
    Eigen::Matrix4d G1 = tpp.tf;
    Eigen::Matrix4d dG = G1.inverse()*G0;

    mtx.lock();
    cv::Mat old_map = map.clone();
    mtx.unlock();

    // transform old map
    cv::Mat _map(h, w, CV_64F, cv::Scalar(0));
    for(int i = 0; i < h; i++)
    {
        for(int j = 0; j < w; j++)
        {
            double val = old_map.ptr<double>(i)[j];
            if(val > 0)
            {
                cv::Vec2d xy = uv_to_xy(j, i);
                Eigen::Vector3d P(xy[0], xy[1], 0);
                Eigen::Vector3d _P = dG.block(0,0,3,3)*P + dG.block(0,3,3,1);

                cv::Vec2i uv = xy_to_uv(_P[0], _P[1]);
                int u = uv[0];
                int v = uv[1];
                if(u < 0 || u >= w || v < 0 || v >= h)
                {
                    continue;
                }

                _map.ptr<double>(v)[u] = val;
            }
        }
    }

    // add new points
    cv::Mat hit_map(h, w, CV_8U, cv::Scalar(0));
    cv::Mat miss_map(h, w, CV_8U, cv::Scalar(0));
    for(size_t p = 0; p < tpp.pts.size(); p++)
    {
        cv::Vec2i uv = xy_to_uv(tpp.pts[p][0], tpp.pts[p][1]);
        int u = uv[0];
        int v = uv[1];
        if(u < 0 || u >= w || v < 0 || v >= h)
        {
            continue;
        }

        hit_map.ptr<uchar>(v)[u] = 255;
        cv::line(miss_map, cv::Point(cx,cy), cv::Point(u,v), cv::Scalar(255), 1);
    }

    for(int i = 0; i < h; i++)
    {
        for(int j = 0; j < w; j++)
        {
            if(hit_map.ptr<uchar>(i)[j] == 255)
            {
                double m_old = _map.ptr<double>(i)[j];
                double m_new = prob(m_old, P_hit);
                _map.ptr<double>(i)[j] = m_new;
            }
            else
            {
                if(miss_map.ptr<uchar>(i)[j] == 255)
                {
                    double m_old = _map.ptr<double>(i)[j];
                    double m_new = prob(m_old, P_miss);
                    _map.ptr<double>(i)[j] = m_new;
                }
            }
        }
    }

    // update map
    mtx.lock();
    map = _map;
    obs_tf = tpp.tf;
    mtx.unlock();

    cv::imshow("map", _map);
}

cv::Vec2i OBSMAP::xy_to_uv(double x, double y)
{
    //int u = std::round(x/gs) + cx;
    //int v = std::round(y/gs) + cy;

    int u = x/gs + cx;
    int v = y/gs + cy;
    return cv::Vec2i(u, v);
}

cv::Vec2d OBSMAP::uv_to_xy(int u, int v)
{
    //double x = (u-cx)*gs + gs/2;
    //double y = (v-cy)*gs + gs/2;

    double x = (u-cx)*gs;
    double y = (v-cy)*gs;
    return cv::Vec2d(x, y);
}

double OBSMAP::odds(double p)
{
    return p / (1.0 - p);
}

double OBSMAP::odds_inv(double odd)
{
    return odd/(odd+1.0);
}

double OBSMAP::clamp(double p, double min, double max)
{
    if (p > max)
    {
        p = max;
    }
    else if (p < min)
    {
        p = min;
    }
    return p;
}

double OBSMAP::prob(double m_old, double P)
{
    return clamp(odds_inv(odds(m_old)*odds(P)), P_min, P_max);
}
