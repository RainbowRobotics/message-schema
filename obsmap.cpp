#include "obsmap.h"

OBSMAP::OBSMAP(QObject *parent)
    : QObject{parent}
{
    tf.setIdentity();
}

OBSMAP::~OBSMAP()
{

}

void OBSMAP::init()
{
    // init
    tf.setIdentity();

    w = (2*config->OBS_MAP_RANGE)/config->OBS_MAP_GRID_SIZE;
    h = w;
    cx = w/2;
    cy = h/2;
    gs = config->OBS_MAP_GRID_SIZE;

    map = cv::Mat(h, w, CV_64F, cv::Scalar(0)); // obstacle is 255    
}

void OBSMAP::get_obs_map(cv::Mat& obs_map, Eigen::Matrix4d& obs_tf)
{
    mtx.lock();
    cv::Mat res(h, w, CV_8U, cv::Scalar(0));
    for(int i = 0; i < h; i++)
    {
        for(int j = 0; j < w; j++)
        {
            if(map.ptr<double>(i)[j] >= P_wall)
            {
                res.ptr<uchar>(i)[j] = 255;
            }
        }
    }
    obs_map = res;
    obs_tf = tf;
    mtx.unlock();
}

cv::Mat OBSMAP::get_plot_map()
{
    cv::Mat res(h, w, CV_8UC3, cv::Scalar(0,0,0));

    mtx.lock();
    for(int i = 0; i < h; i++)
    {
        for(int j = 0; j < w; j++)
        {
            if(map.ptr<double>(i)[j] >= P_wall)
            {
                res.ptr<cv::Vec3b>(i)[j] = cv::Vec3b(255, 255, 255);
            }
        }
    }
    mtx.unlock();

    return res;
}

void OBSMAP::draw_robot(cv::Mat& img, Eigen::Matrix4d robot_tf)
{
    mtx.lock();
    Eigen::Matrix4d G = tf.inverse()*robot_tf;
    mtx.unlock();

    // draw rect
    const double x_min = config->ROBOT_SIZE_X[0];
    const double x_max = config->ROBOT_SIZE_X[1];
    const double y_min = config->ROBOT_SIZE_Y[0];
    const double y_max = config->ROBOT_SIZE_Y[1];

    Eigen::Vector3d P0(x_min, y_min, 0);
    Eigen::Vector3d P1(x_min, y_max, 0);
    Eigen::Vector3d P2(x_max, y_max, 0);
    Eigen::Vector3d P3(x_max, y_min, 0);

    Eigen::Vector3d _P0 = G.block(0,0,3,3)*P0 + G.block(0,3,3,1);
    Eigen::Vector3d _P1 = G.block(0,0,3,3)*P1 + G.block(0,3,3,1);
    Eigen::Vector3d _P2 = G.block(0,0,3,3)*P2 + G.block(0,3,3,1);
    Eigen::Vector3d _P3 = G.block(0,0,3,3)*P3 + G.block(0,3,3,1);

    cv::Vec2i uv0 = xy_uv(_P0[0], _P0[1]);
    cv::Vec2i uv1 = xy_uv(_P1[0], _P1[1]);
    cv::Vec2i uv2 = xy_uv(_P2[0], _P2[1]);
    cv::Vec2i uv3 = xy_uv(_P3[0], _P3[1]);

    std::vector<std::vector<cv::Point>> pts(1);
    pts[0].push_back(cv::Point(uv0[0], uv0[1]));
    pts[0].push_back(cv::Point(uv1[0], uv1[1]));
    pts[0].push_back(cv::Point(uv2[0], uv2[1]));
    pts[0].push_back(cv::Point(uv3[0], uv3[1]));

    cv::fillPoly(img, pts, cv::Scalar(255, 127, 0), cv::LINE_AA);

    // draw axis
    Eigen::Vector3d P_c(0, 0, 0);
    Eigen::Vector3d P_x(x_max, 0, 0);
    Eigen::Vector3d P_y(0, y_max, 0);

    Eigen::Vector3d _P_c = G.block(0,0,3,3)*P_c + G.block(0,3,3,1);
    Eigen::Vector3d _P_x = G.block(0,0,3,3)*P_x + G.block(0,3,3,1);
    Eigen::Vector3d _P_y = G.block(0,0,3,3)*P_y + G.block(0,3,3,1);

    cv::Vec2i uv_c = xy_uv(_P_c[0], _P_c[1]);
    cv::Vec2i uv_x = xy_uv(_P_x[0], _P_x[1]);
    cv::Vec2i uv_y = xy_uv(_P_y[0], _P_y[1]);

    cv::line(img, cv::Point(uv_c[0], uv_c[1]), cv::Point(uv_y[0], uv_y[1]), cv::Scalar(0,255,0), 1, cv::LINE_AA);
    cv::line(img, cv::Point(uv_c[0], uv_c[1]), cv::Point(uv_x[0], uv_x[1]), cv::Scalar(0,0,255), 1, cv::LINE_AA);
}

bool OBSMAP::is_collision(Eigen::Matrix4d robot_tf)
{
    // get obs map
    cv::Mat _obs_map;
    Eigen::Matrix4d _obs_tf;
    get_obs_map(_obs_map, _obs_tf);
    Eigen::Matrix4d G = _obs_tf.inverse()*robot_tf;

    // draw rect
    const double x_min = config->ROBOT_SIZE_X[0];
    const double x_max = config->ROBOT_SIZE_X[1];
    const double y_min = config->ROBOT_SIZE_Y[0];
    const double y_max = config->ROBOT_SIZE_Y[1];

    Eigen::Vector3d P0(x_min, y_min, 0);
    Eigen::Vector3d P1(x_min, y_max, 0);
    Eigen::Vector3d P2(x_max, y_max, 0);
    Eigen::Vector3d P3(x_max, y_min, 0);

    Eigen::Vector3d _P0 = G.block(0,0,3,3)*P0 + G.block(0,3,3,1);
    Eigen::Vector3d _P1 = G.block(0,0,3,3)*P1 + G.block(0,3,3,1);
    Eigen::Vector3d _P2 = G.block(0,0,3,3)*P2 + G.block(0,3,3,1);
    Eigen::Vector3d _P3 = G.block(0,0,3,3)*P3 + G.block(0,3,3,1);

    cv::Vec2i uv0 = xy_uv(_P0[0], _P0[1]);
    if(uv0[0] < 0 || uv0[0] >= w || uv0[1] < 0 || uv0[1] >= h)
    {
        return true;
    }

    cv::Vec2i uv1 = xy_uv(_P1[0], _P1[1]);
    if(uv1[0] < 0 || uv1[0] >= w || uv1[1] < 0 || uv1[1] >= h)
    {
        return true;
    }

    cv::Vec2i uv2 = xy_uv(_P2[0], _P2[1]);
    if(uv2[0] < 0 || uv2[0] >= w || uv2[1] < 0 || uv2[1] >= h)
    {
        return true;
    }

    cv::Vec2i uv3 = xy_uv(_P3[0], _P3[1]);
    if(uv3[0] < 0 || uv3[0] >= w || uv3[1] < 0 || uv3[1] >= h)
    {
        return true;
    }

    std::vector<std::vector<cv::Point>> pts(1);
    pts[0].push_back(cv::Point(uv0[0], uv0[1]));
    pts[0].push_back(cv::Point(uv1[0], uv1[1]));
    pts[0].push_back(cv::Point(uv2[0], uv2[1]));
    pts[0].push_back(cv::Point(uv3[0], uv3[1]));

    cv::Mat mask(h, w, CV_8U, cv::Scalar(0));
    cv::fillPoly(mask, pts, cv::Scalar(255));

    for(int i = 0; i < h; i++)
    {
        for(int j = 0; j < w; j++)
        {
            if(mask.ptr<uchar>(i)[j] == 255 && _obs_map.ptr<uchar>(i)[j] == 255)
            {
                return true;
            }
        }
    }

    // non collision
    return false;
}

bool OBSMAP::is_collision(std::vector<Eigen::Matrix4d> robot_tfs)
{
    // get obs map
    cv::Mat _obs_map;
    Eigen::Matrix4d _obs_tf;
    get_obs_map(_obs_map, _obs_tf);

    const double x_min = config->ROBOT_SIZE_X[0];
    const double x_max = config->ROBOT_SIZE_X[1];
    const double y_min = config->ROBOT_SIZE_Y[0];
    const double y_max = config->ROBOT_SIZE_Y[1];

    Eigen::Vector3d P0(x_min, y_min, 0);
    Eigen::Vector3d P1(x_min, y_max, 0);
    Eigen::Vector3d P2(x_max, y_max, 0);
    Eigen::Vector3d P3(x_max, y_min, 0);

    std::vector<std::vector<cv::Point>> pts(robot_tfs.size());
    for(size_t p = 0; p < robot_tfs.size(); p++)
    {
        Eigen::Matrix4d G = _obs_tf.inverse()*robot_tfs[p];

        Eigen::Vector3d _P0 = G.block(0,0,3,3)*P0 + G.block(0,3,3,1);
        Eigen::Vector3d _P1 = G.block(0,0,3,3)*P1 + G.block(0,3,3,1);
        Eigen::Vector3d _P2 = G.block(0,0,3,3)*P2 + G.block(0,3,3,1);
        Eigen::Vector3d _P3 = G.block(0,0,3,3)*P3 + G.block(0,3,3,1);

        cv::Vec2i uv0 = xy_uv(_P0[0], _P0[1]);
        if(uv0[0] < 0 || uv0[0] >= w || uv0[1] < 0 || uv0[1] >= h)
        {
            return true;
        }

        cv::Vec2i uv1 = xy_uv(_P1[0], _P1[1]);
        if(uv1[0] < 0 || uv1[0] >= w || uv1[1] < 0 || uv1[1] >= h)
        {
            return true;
        }

        cv::Vec2i uv2 = xy_uv(_P2[0], _P2[1]);
        if(uv2[0] < 0 || uv2[0] >= w || uv2[1] < 0 || uv2[1] >= h)
        {
            return true;
        }

        cv::Vec2i uv3 = xy_uv(_P3[0], _P3[1]);
        if(uv3[0] < 0 || uv3[0] >= w || uv3[1] < 0 || uv3[1] >= h)
        {
            return true;
        }

        pts[p].push_back(cv::Point(uv0[0], uv0[1]));
        pts[p].push_back(cv::Point(uv1[0], uv1[1]));
        pts[p].push_back(cv::Point(uv2[0], uv2[1]));
        pts[p].push_back(cv::Point(uv3[0], uv3[1]));
    }

    cv::Mat mask(h, w, CV_8U, cv::Scalar(0));
    cv::fillPoly(mask, pts, cv::Scalar(255));

    for(int i = 0; i < h; i++)
    {
        for(int j = 0; j < w; j++)
        {
            if(mask.ptr<uchar>(i)[j] == 255 && _obs_map.ptr<uchar>(i)[j] == 255)
            {
                return true;
            }
        }
    }

    return false;
}

bool OBSMAP::is_collision(cv::Mat obs_map, Eigen::Matrix4d obs_tf, Eigen::Matrix4d robot_tf, cv::Mat avoid_area)
{
    // collision check for ompl

    // calc tf
    Eigen::Matrix4d G = obs_tf.inverse()*robot_tf;

    // draw rect
    const double x_min = config->ROBOT_SIZE_X[0];
    const double x_max = config->ROBOT_SIZE_X[1];
    const double y_min = config->ROBOT_SIZE_Y[0];
    const double y_max = config->ROBOT_SIZE_Y[1];

    Eigen::Vector3d P0(x_min, y_min, 0);
    Eigen::Vector3d P1(x_min, y_max, 0);
    Eigen::Vector3d P2(x_max, y_max, 0);
    Eigen::Vector3d P3(x_max, y_min, 0);

    Eigen::Vector3d _P0 = G.block(0,0,3,3)*P0 + G.block(0,3,3,1);
    Eigen::Vector3d _P1 = G.block(0,0,3,3)*P1 + G.block(0,3,3,1);
    Eigen::Vector3d _P2 = G.block(0,0,3,3)*P2 + G.block(0,3,3,1);
    Eigen::Vector3d _P3 = G.block(0,0,3,3)*P3 + G.block(0,3,3,1);

    cv::Vec2i uv0 = xy_uv(_P0[0], _P0[1]);
    if(uv0[0] < 0 || uv0[0] >= w || uv0[1] < 0 || uv0[1] >= h)
    {
        return true;
    }

    cv::Vec2i uv1 = xy_uv(_P1[0], _P1[1]);
    if(uv1[0] < 0 || uv1[0] >= w || uv1[1] < 0 || uv1[1] >= h)
    {
        return true;
    }

    cv::Vec2i uv2 = xy_uv(_P2[0], _P2[1]);
    if(uv2[0] < 0 || uv2[0] >= w || uv2[1] < 0 || uv2[1] >= h)
    {
        return true;
    }

    cv::Vec2i uv3 = xy_uv(_P3[0], _P3[1]);
    if(uv3[0] < 0 || uv3[0] >= w || uv3[1] < 0 || uv3[1] >= h)
    {
        return true;
    }

    std::vector<std::vector<cv::Point>> pts(1);
    pts[0].push_back(cv::Point(uv0[0], uv0[1]));
    pts[0].push_back(cv::Point(uv1[0], uv1[1]));
    pts[0].push_back(cv::Point(uv2[0], uv2[1]));
    pts[0].push_back(cv::Point(uv3[0], uv3[1]));

    cv::Mat mask(h, w, CV_8U, cv::Scalar(0));
    cv::fillPoly(mask, pts, cv::Scalar(255));

    for(int i = 0; i < h; i++)
    {
        for(int j = 0; j < w; j++)
        {
            if(mask.ptr<uchar>(i)[j] == 255 && obs_map.ptr<uchar>(i)[j] == 255)
            {
                // collision
                return true;
            }

            if(mask.ptr<uchar>(i)[j] == 255 && avoid_area.ptr<uchar>(i)[j] == 0)
            {
                // collision
                return true;
            }
        }
    }

    // non collision
    return false;
}

bool OBSMAP::is_pivot_collision(cv::Mat obs_map, Eigen::Matrix4d obs_tf, Eigen::Matrix4d robot_tf, cv::Mat avoid_area)
{
    // collision check for ompl

    // calc tf
    Eigen::Matrix4d G = obs_tf.inverse()*robot_tf;

    // draw circle
    const double robot_radius = config->ROBOT_RADIUS;
    int r = std::ceil(robot_radius/gs);

    cv::Vec2i uv = xy_uv(G(0,3), G(1,3));
    if(uv[0] < 0 || uv[0] >= w || uv[1] < 0 || uv[1] >= h)
    {
        return true;
    }

    cv::Mat mask(h, w, CV_8U, cv::Scalar(0));
    cv::circle(mask, cv::Point(uv[0], uv[1]), r, cv::Scalar(255), -1);

    for(int i = 0; i < h; i++)
    {
        for(int j = 0; j < w; j++)
        {
            if(mask.ptr<uchar>(i)[j] == 255 && obs_map.ptr<uchar>(i)[j] == 255)
            {
                // collision
                return true;
            }

            if(mask.ptr<uchar>(i)[j] == 255 && avoid_area.ptr<uchar>(i)[j] == 0)
            {
                // collision
                return true;
            }
        }
    }

    // non collision
    return false;
}

void OBSMAP::update_obs_map(TIME_POSE_PTS& tpp)
{
    cv::Mat _map(h, w, CV_64F, cv::Scalar(0));
    for(size_t p = 0; p < tpp_storage.size(); p++)
    {
        Eigen::Matrix4d G = tpp.tf.inverse()*tpp_storage[p].tf;

        // update old points
        cv::Mat hit_map(h, w, CV_8U, cv::Scalar(0));
        cv::Mat miss_map(h, w, CV_8U, cv::Scalar(0));
        for(size_t q = 0; q < tpp_storage[p].pts.size(); q++)
        {
            Eigen::Vector3d P = tpp_storage[p].pts[q];
            Eigen::Vector3d _P = G.block(0,0,3,3)*P + G.block(0,3,3,1);

            cv::Vec2i uv = xy_uv(_P[0], _P[1]);
            int u = uv[0];
            int v = uv[1];
            if(u < 0 || u >= w || v < 0 || v >= h)
            {
                continue;
            }

            cv::Vec2i uv0 = xy_uv(G(0,3), G(1,3));

            hit_map.ptr<uchar>(v)[u] = 255;
            cv::line(miss_map, cv::Point(uv0[0], uv0[1]), cv::Point(u, v), cv::Scalar(255), 1);
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
    }

    // update new points
    cv::Mat hit_map(h, w, CV_8U, cv::Scalar(0));
    cv::Mat miss_map(h, w, CV_8U, cv::Scalar(0));
    for(size_t p = 0; p < tpp.pts.size(); p++)
    {
        Eigen::Vector3d P = tpp.pts[p];

        cv::Vec2i uv = xy_uv(P[0], P[1]);
        int u = uv[0];
        int v = uv[1];
        if(u < 0 || u >= w || v < 0 || v >= h)
        {
            continue;
        }

        hit_map.ptr<uchar>(v)[u] = 255;
        cv::line(miss_map, cv::Point(cx, cy), cv::Point(u, v), cv::Scalar(255), 1);
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

    // update storage
    tpp_storage.push_back(tpp);
    if(tpp_storage.size() > 30)
    {
        tpp_storage.erase(tpp_storage.begin());
    }

    // add margin
    cv::dilate(_map, _map, cv::Mat());

    // update map
    mtx.lock();
    map = _map;
    tf = tpp.tf;
    mtx.unlock();
}

cv::Vec2i OBSMAP::xy_uv(double x, double y)
{
    // y axis flip
    int u = x/gs + cx;
    int v = -y/gs + cy;
    return cv::Vec2i(u, v);
}

cv::Vec2d OBSMAP::uv_xy(int u, int v)
{
    double x = (u-cx)*gs;
    double y = -(v-cy)*gs;
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

void OBSMAP::shift_grid_map(cv::Mat& src, double x, double y, double th)
{
    cv::Mat R = cv::getRotationMatrix2D(cv::Point(cx, cy), th*R2D, 1.0);
    cv::Mat rotated;
    cv::warpAffine(src, rotated, R, src.size(), cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));

    cv::Mat T = (cv::Mat_<double>(2,3) << 1, 0, x/gs, 0, 1, -y/gs);
    cv::Mat shifted;
    cv::warpAffine(rotated, shifted, T, src.size(), cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));

    src = shifted;

    cv::imshow("src", src);
}
