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

    prob_map = cv::Mat(h, w, CV_64F, cv::Scalar(0));
    wall_map = cv::Mat(h, w, CV_8U, cv::Scalar(0));

    octree = new octomap::OcTree(config->OBS_MAP_GRID_SIZE);
    octree->setProbHit(0.8);
    octree->setProbMiss(0.48);
}

void OBSMAP::clear()
{
    mtx.lock();
    octree->clear();
    mtx.unlock();
}

void OBSMAP::get_obs_map(cv::Mat& obs_map, Eigen::Matrix4d& obs_tf)
{
    mtx.lock();    
    obs_map = wall_map.clone();
    obs_tf = tf;
    mtx.unlock();
}

cv::Mat OBSMAP::calc_avoid_area(const std::vector<Eigen::Matrix4d>& path, const Eigen::Matrix4d& robot_tf)
{
    mtx.lock();
    cv::Mat obs_map = wall_map.clone();
    Eigen::Matrix4d obs_tf = tf;
    mtx.unlock();

    cv::Mat avoid_area = cv::Mat(h, w, CV_8U, cv::Scalar(0));
    int r = 2*(config->ROBOT_RADIUS/gs) + 1;

    Eigen::Matrix4d obs_tf_inv = obs_tf.inverse();

    // draw robot boundary
    {
        Eigen::Vector3d P(robot_tf(0,3), robot_tf(1,3), robot_tf(2,3));
        Eigen::Vector3d _P = obs_tf_inv.block(0,0,3,3)*P + obs_tf_inv.block(0,3,3,1);

        cv::Vec2i uv = xy_uv(_P[0], _P[1]);
        cv::circle(avoid_area, cv::Point(uv[0], uv[1]), r, cv::Scalar(255), -1);
    }

    // draw path boundary
    for(size_t p = 0; p < path.size(); p++)
    {
        Eigen::Vector3d P(path[p](0,3), path[p](1,3), path[p](2,3));
        Eigen::Vector3d _P = obs_tf_inv.block(0,0,3,3)*P + obs_tf_inv.block(0,3,3,1);

        cv::Vec2i uv = xy_uv(_P[0], _P[1]);
        cv::circle(avoid_area, cv::Point(uv[0], uv[1]), r, cv::Scalar(255), -1);
    }

    // draw obs boundary
    cv::Mat avoid_area2 = avoid_area.clone();
    for(int i = 0; i < h; i++)
    {
        for(int j = 0; j < w; j++)
        {
            if(avoid_area.ptr<uchar>(i)[j] == 255 && obs_map.ptr<uchar>(i)[j] == 255)
            {
                cv::circle(avoid_area2, cv::Point(j, i), r, cv::Scalar(255), -1);
            }
        }
    }

    cv::dilate(avoid_area2, avoid_area2, cv::Mat(), cv::Point(-1,-1), 2);
    return avoid_area2;
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

    Eigen::Vector3d P0(x_max, y_max, 0);
    Eigen::Vector3d P1(x_max, y_min, 0);
    Eigen::Vector3d P2(x_min, y_min, 0);
    Eigen::Vector3d P3(x_min, y_max, 0);

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

bool OBSMAP::is_pos_collision(const Eigen::Vector3d& pos, double radius)
{
    // get obs map
    cv::Mat _obs_map;
    Eigen::Matrix4d _obs_tf;
    get_obs_map(_obs_map, _obs_tf);
    Eigen::Matrix4d G = _obs_tf.inverse();

    Eigen::Vector3d P = G.block(0,0,3,3)*pos + G.block(0,3,3,1);
    cv::Vec2i uv = xy_uv(P[0], P[1]);
    if(uv[0] < 0 || uv[0] >= w || uv[1] < 0 || uv[1] >= h)
    {
        return true;
    }

    cv::Mat mask(h, w, CV_8U, cv::Scalar(0));

    int r = std::ceil(radius/gs);
    cv::circle(mask, cv::Point(uv[0], uv[1]), r, cv::Scalar(255), -1);

    for(int i = 0; i < h; i++)
    {
        for(int j = 0; j < w; j++)
        {
            if(mask.ptr<uchar>(i)[j] == 255 && _obs_map.ptr<uchar>(i)[j] == 255)
            {
                // collision
                return true;
            }
        }
    }

    // non collision
    return false;
}

bool OBSMAP::is_pivot_collision(const Eigen::Matrix4d& robot_tf)
{
    // get obs map
    cv::Mat _obs_map;
    Eigen::Matrix4d _obs_tf;
    get_obs_map(_obs_map, _obs_tf);
    Eigen::Matrix4d G = _obs_tf.inverse()*robot_tf;

    // draw circle
    cv::Vec2i uv = xy_uv(G(0,3), G(1,3));
    if(uv[0] < 0 || uv[0] >= w || uv[1] < 0 || uv[1] >= h)
    {
        return true;
    }

    cv::Mat mask(h, w, CV_8U, cv::Scalar(0));

    int r = std::ceil(config->ROBOT_RADIUS/gs);
    cv::circle(mask, cv::Point(uv[0], uv[1]), r, cv::Scalar(255), -1);

    for(int i = 0; i < h; i++)
    {
        for(int j = 0; j < w; j++)
        {
            if(mask.ptr<uchar>(i)[j] == 255 && _obs_map.ptr<uchar>(i)[j] == 255)
            {
                // collision
                return true;
            }
        }
    }

    // non collision
    return false;
}

bool OBSMAP::is_tf_collision(const Eigen::Matrix4d& robot_tf, double margin_x, double margin_y)
{
    // get obs map
    cv::Mat _obs_map;
    Eigen::Matrix4d _obs_tf;
    get_obs_map(_obs_map, _obs_tf);
    Eigen::Matrix4d G = _obs_tf.inverse()*robot_tf;

    // draw rect
    const double x_min = config->ROBOT_SIZE_X[0] - margin_x;
    const double x_max = config->ROBOT_SIZE_X[1] + margin_x;
    const double y_min = config->ROBOT_SIZE_Y[0] - margin_y;
    const double y_max = config->ROBOT_SIZE_Y[1] + margin_y;

    Eigen::Vector3d P0(x_max, y_max, 0);
    Eigen::Vector3d P1(x_max, y_min, 0);
    Eigen::Vector3d P2(x_min, y_min, 0);
    Eigen::Vector3d P3(x_min, y_max, 0);

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

bool OBSMAP::is_tf_collision_with_area(const Eigen::Matrix4d& robot_tf, const cv::Mat& area, double margin_x, double margin_y)
{
    // get obs map
    cv::Mat _obs_map;
    Eigen::Matrix4d _obs_tf;
    get_obs_map(_obs_map, _obs_tf);
    Eigen::Matrix4d G = _obs_tf.inverse()*robot_tf;

    // draw rect
    const double x_min = config->ROBOT_SIZE_X[0] - margin_x;
    const double x_max = config->ROBOT_SIZE_X[1] + margin_x;
    const double y_min = config->ROBOT_SIZE_Y[0] - margin_y;
    const double y_max = config->ROBOT_SIZE_Y[1] + margin_y;

    Eigen::Vector3d P0(x_max, y_max, 0);
    Eigen::Vector3d P1(x_max, y_min, 0);
    Eigen::Vector3d P2(x_min, y_min, 0);
    Eigen::Vector3d P3(x_min, y_max, 0);

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

            if(mask.ptr<uchar>(i)[j] == 255 && area.ptr<uchar>(i)[j] == 0)
            {
                return true;
            }
        }
    }

    // non collision
    return false;
}

bool OBSMAP::is_path_collision(const std::vector<Eigen::Matrix4d>& robot_tfs, int st_idx, int idx_step)
{
    // get obs map
    cv::Mat _obs_map;
    Eigen::Matrix4d _obs_tf;
    get_obs_map(_obs_map, _obs_tf);

    const double x_min = config->ROBOT_SIZE_X[0];
    const double x_max = config->ROBOT_SIZE_X[1];
    const double y_min = config->ROBOT_SIZE_Y[0];
    const double y_max = config->ROBOT_SIZE_Y[1];

    Eigen::Vector3d P0(x_max, y_max, 0);
    Eigen::Vector3d P1(x_max, y_min, 0);
    Eigen::Vector3d P2(x_min, y_min, 0);
    Eigen::Vector3d P3(x_min, y_max, 0);

    cv::Mat mask(h, w, CV_8U, cv::Scalar(0));
    for(size_t p = st_idx; p < robot_tfs.size(); p += idx_step)
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

        std::vector<std::vector<cv::Point>> pts(1);
        pts[0].push_back(cv::Point(uv0[0], uv0[1]));
        pts[0].push_back(cv::Point(uv1[0], uv1[1]));
        pts[0].push_back(cv::Point(uv2[0], uv2[1]));
        pts[0].push_back(cv::Point(uv3[0], uv3[1]));

        cv::fillPoly(mask, pts, cv::Scalar(255));
    }

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

bool OBSMAP::get_tf_collision_cnt(const Eigen::Matrix4d& robot_tf, double margin_x, double margin_y, double chk_range, int& cnt0, int& cnt1)
{
    // get obs map
    cv::Mat _obs_map;
    Eigen::Matrix4d _obs_tf;
    get_obs_map(_obs_map, _obs_tf);
    Eigen::Matrix4d G = _obs_tf.inverse()*robot_tf;

    // draw rect
    const double x_min = config->ROBOT_SIZE_X[0] - margin_x;
    const double x_max = config->ROBOT_SIZE_X[1] + margin_x;
    const double y_min = config->ROBOT_SIZE_Y[0] - margin_y;
    const double y_max = config->ROBOT_SIZE_Y[1] + margin_y;

    Eigen::Vector3d P0(x_max, y_max, 0);
    Eigen::Vector3d P1(x_max, y_min, 0);
    Eigen::Vector3d P2(x_min, y_min, 0);
    Eigen::Vector3d P3(x_min, y_max, 0);

    Eigen::Vector3d _P0 = G.block(0,0,3,3)*P0 + G.block(0,3,3,1);
    Eigen::Vector3d _P1 = G.block(0,0,3,3)*P1 + G.block(0,3,3,1);
    Eigen::Vector3d _P2 = G.block(0,0,3,3)*P2 + G.block(0,3,3,1);
    Eigen::Vector3d _P3 = G.block(0,0,3,3)*P3 + G.block(0,3,3,1);

    cv::Vec2i uv0 = xy_uv(_P0[0], _P0[1]);
    if(uv0[0] < 0 || uv0[0] >= w || uv0[1] < 0 || uv0[1] >= h)
    {
        return false;
    }

    cv::Vec2i uv1 = xy_uv(_P1[0], _P1[1]);
    if(uv1[0] < 0 || uv1[0] >= w || uv1[1] < 0 || uv1[1] >= h)
    {
        return false;
    }

    cv::Vec2i uv2 = xy_uv(_P2[0], _P2[1]);
    if(uv2[0] < 0 || uv2[0] >= w || uv2[1] < 0 || uv2[1] >= h)
    {
        return false;
    }

    cv::Vec2i uv3 = xy_uv(_P3[0], _P3[1]);
    if(uv3[0] < 0 || uv3[0] >= w || uv3[1] < 0 || uv3[1] >= h)
    {
        return false;
    }

    std::vector<std::vector<cv::Point>> pts(1);
    pts[0].push_back(cv::Point(uv0[0], uv0[1]));
    pts[0].push_back(cv::Point(uv1[0], uv1[1]));
    pts[0].push_back(cv::Point(uv2[0], uv2[1]));
    pts[0].push_back(cv::Point(uv3[0], uv3[1]));

    cv::Mat mask(h, w, CV_8U, cv::Scalar(0));
    cv::fillPoly(mask, pts, cv::Scalar(255));

    // circle range
    int r = chk_range/gs;
    cv::Vec2i uv = xy_uv(G(0,3), G(1,3));
    cv::Mat mask2(h, w, CV_8U, cv::Scalar(0));
    cv::circle(mask2, cv::Point(uv[0], uv[1]), r, cv::Scalar(255), -1);

    // collision counting
    int _cnt0 = 0;
    int _cnt1 = 0;
    for(int i = 0; i < h; i++)
    {
        for(int j = 0; j < w; j++)
        {
            if(mask.ptr<uchar>(i)[j] == 255 && _obs_map.ptr<uchar>(i)[j] == 255)
            {
                _cnt0++;
            }

            if(mask2.ptr<uchar>(i)[j] == 255 && _obs_map.ptr<uchar>(i)[j] == 255)
            {
                _cnt1++;
            }
        }
    }

    cnt0 = _cnt0;
    cnt1 = _cnt1;

    return true;
}

Eigen::Vector3d OBSMAP::get_obs_force(const Eigen::Vector3d& center, const double max_r)
{
    // get obs map
    cv::Mat _obs_map;
    Eigen::Matrix4d _obs_tf;
    get_obs_map(_obs_map, _obs_tf);

    // global to local
    Eigen::Matrix4d _obs_tf_inv = _obs_tf.inverse();
    Eigen::Vector3d _center = _obs_tf_inv.block(0,0,3,3)*center + _obs_tf_inv.block(0,3,3,1);

    // calc average force
    int search_r = max_r/gs + 1;
    cv::Vec2i center_uv = xy_uv(_center[0], _center[1]);
    std::vector<cv::Vec2i> circle = filled_circle_iterator(center_uv, search_r);

    int num = 0;
    Eigen::Vector3d sum_f(0, 0, 0);
    for(size_t p = 0; p < circle.size(); p++)
    {
        int u = circle[p][0];
        int v = circle[p][1];
        if(u < 0 || u >= w || v < 0 || v >= h)
        {
            continue;
        }

        if(_obs_map.ptr<uchar>(v)[u] == 255)
        {
            // local to global
            cv::Vec2d xy = uv_xy(u, v);
            Eigen::Vector3d P(xy[0], xy[1], 0);
            Eigen::Vector3d obs_pos = _obs_tf.block(0,0,3,3)*P + _obs_tf.block(0,3,3,1);

            double dx = center[0] - obs_pos[0];
            double dy = center[1] - obs_pos[1];
            double d = std::sqrt(dx*dx + dy*dy);
            if(d > max_r)
            {
                continue;
            }

            //double mag = (max_r-d)/(d+0.01);
            double mag = (max_r-d);
            Eigen::Vector3d dir(dx, dy, 0);

            // repulsion force
            Eigen::Vector3d f = mag*dir.normalized();
            sum_f += f;            
            num++;
        }
    }

    if(num > 0)
    {
        return sum_f/num;
    }
    else
    {
        return sum_f;
    }
}

void OBSMAP::update_obs_map(TIME_POSE_PTS& tpp)
{
    Eigen::Matrix4d cur_tf = tpp.tf;
    Eigen::Matrix4d cur_tf_inv = cur_tf.inverse();

    // update octomap
    octomap::Pointcloud cloud;
    for(size_t p = 0; p < tpp.pts.size(); p++)
    {
        // local to global
        Eigen::Vector3d P = tpp.pts[p];
        Eigen::Vector3d _P = cur_tf.block(0,0,3,3)*P + cur_tf.block(0,3,3,1);
        cloud.push_back(_P[0], _P[1], _P[2]);
    }

    // update
    mtx.lock();
    octree->insertPointCloud(cloud, octomap::point3d(cur_tf(0,3), cur_tf(1,3), cur_tf(2,3)), config->OBS_MAP_RANGE);

    // obsmap boundary
    octomap::point3d bbx_min(cur_tf(0,3) - config->OBS_MAP_RANGE, cur_tf(1,3) - config->OBS_MAP_RANGE, cur_tf(2,3) + config->OBS_MAP_MIN_Z);
    octomap::point3d bbx_max(cur_tf(0,3) + config->OBS_MAP_RANGE, cur_tf(1,3) + config->OBS_MAP_RANGE, cur_tf(2,3) + config->OBS_MAP_MAX_Z);

    cv::Mat _map(h, w, CV_64F, cv::Scalar(0));
    for(octomap::OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(bbx_min, bbx_max, 16); it != octree->end_leafs_bbx(); it++)
    {
        double x = it.getX();
        double y = it.getY();
        double z = it.getZ();
        double prob = it->getOccupancy();

        // global to local
        Eigen::Vector3d P(x,y,z);
        Eigen::Vector3d _P = cur_tf_inv.block(0,0,3,3)*P + cur_tf_inv.block(0,3,3,1);

        cv::Vec2i uv = xy_uv(_P[0], _P[1]);
        int u = uv[0];
        int v = uv[1];
        if(u < 0 || u >= w || v < 0 || v >= h)
        {
            continue;
        }

        if(_map.ptr<double>(v)[u] == 0 || prob > _map.ptr<double>(v)[u])
        {
            _map.ptr<double>(v)[u] = prob;
        }
    }

    // make wall map
    cv::Mat wall(h, w, CV_8U, cv::Scalar(0));
    for(int i = 0; i < h; i++)
    {
        for(int j = 0; j < w; j++)
        {
            if(_map.ptr<double>(i)[j] >= 0.7)
            {
                wall.ptr<uchar>(i)[j] = 255;
            }
        }
    }

    // add static map
    double query_pt[3] = {cur_tf(0,3), cur_tf(1,3), cur_tf(2,3)};
    double sq_radius = config->OBS_MAP_RANGE*config->OBS_MAP_RANGE;
    std::vector<nanoflann::ResultItem<unsigned int, double>> res_idxs;
    nanoflann::SearchParameters params;
    unimap->kdtree_index->radiusSearch(&query_pt[0], sq_radius, res_idxs, params);

    for(size_t p = 0; p < res_idxs.size(); p++)
    {
        int idx = res_idxs[p].first;
        double x = unimap->kdtree_cloud.pts[idx].x;
        double y = unimap->kdtree_cloud.pts[idx].y;
        double z = unimap->kdtree_cloud.pts[idx].z;
        if(z < cur_tf(2,3) + config->OBS_MAP_MIN_Z || z > cur_tf(2,3) + config->OBS_MAP_MAX_Z)
        {
            continue;
        }

        // global to local
        Eigen::Vector3d P(x,y,z);
        Eigen::Vector3d _P = cur_tf_inv.block(0,0,3,3)*P + cur_tf_inv.block(0,3,3,1);

        cv::Vec2i uv = xy_uv(_P[0], _P[1]);
        int u = uv[0];
        int v = uv[1];
        if(u < 0 || u >= w || v < 0 || v >= h)
        {
            continue;
        }

        wall.ptr<uchar>(v)[u] = 255;
    }

    wall_map = wall;
    prob_map = _map;
    tf = cur_tf;
    mtx.unlock();

    // signal for redrawing
    Q_EMIT obs_updated();
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
