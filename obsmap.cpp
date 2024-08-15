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
    static_map = cv::Mat(h, w, CV_8U, cv::Scalar(0));
    dynamic_map = cv::Mat(h, w, CV_8U, cv::Scalar(0));

    octree = new octomap::OcTree(config->OBS_MAP_GRID_SIZE);
    octree->setProbHit(P_HIT);
    octree->setProbMiss(P_MISS);
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

void OBSMAP::get_dyn_map(cv::Mat& dyn_map, Eigen::Matrix4d& obs_tf)
{
    mtx.lock();
    dyn_map = dynamic_map.clone();
    obs_tf = tf;
    mtx.unlock();
}

std::vector<Eigen::Vector4d> OBSMAP::get_obs_pts()
{
    mtx.lock();
    std::vector<Eigen::Vector4d> res = obs_pts;
    mtx.unlock();

    return res;
}

cv::Mat OBSMAP::calc_avoid_area(const std::vector<Eigen::Matrix4d>& path, const Eigen::Matrix4d& robot_tf0, const Eigen::Matrix4d& robot_tf1, double margin_x, double margin_y)
{
    mtx.lock();
    cv::Mat obs_map = wall_map.clone();
    Eigen::Matrix4d obs_tf = tf;
    mtx.unlock();

    cv::Mat avoid_area = cv::Mat(h, w, CV_8U, cv::Scalar(0));
    double dx = std::max<double>(std::abs(config->ROBOT_SIZE_X[0]), std::abs(config->ROBOT_SIZE_X[1])) + margin_x;
    double dy = std::max<double>(std::abs(config->ROBOT_SIZE_Y[0]), std::abs(config->ROBOT_SIZE_Y[1])) + margin_y;
    double _r = std::sqrt(dx*dx + dy*dy);
    int r = 2*(_r/gs) + 1;

    Eigen::Matrix4d obs_tf_inv = obs_tf.inverse();

    // calc approach points
    int idx0 = 0;
    int idx1 = path.size()-1;
    double min_d0 = 99999999;
    double min_d1 = 99999999;
    Eigen::Vector3d pos0 = robot_tf0.block(0,3,3,1);
    Eigen::Vector3d pos1 = robot_tf1.block(0,3,3,1);
    for(size_t p = 0; p < path.size(); p++)
    {
        Eigen::Vector3d pos = path[p].block(0,3,3,1);
        double d0 = calc_dist_2d(pos - pos0);
        if(d0 < min_d0)
        {
            min_d0 = d0;
            idx0 = p;
        }

        double d1 = calc_dist_2d(pos - pos1);
        if(d1 < min_d1)
        {
            min_d1 = d1;
            idx1 = p;
        }
    }

    // draw robot boundary, st
    {
        Eigen::Vector3d P0(robot_tf0(0,3), robot_tf0(1,3), robot_tf0(2,3));
        Eigen::Vector3d _P0 = obs_tf_inv.block(0,0,3,3)*P0 + obs_tf_inv.block(0,3,3,1);

        Eigen::Vector3d P1(path[idx0](0,3), path[idx0](1,3), path[idx0](2,3));
        Eigen::Vector3d _P1 = obs_tf_inv.block(0,0,3,3)*P1 + obs_tf_inv.block(0,3,3,1);

        cv::Vec2i uv0 = xy_uv(_P0[0], _P0[1]);
        cv::Vec2i uv1 = xy_uv(_P1[0], _P1[1]);
        cv::line(avoid_area, cv::Point(uv0[0], uv0[1]), cv::Point(uv1[0], uv1[1]), cv::Scalar(255), r);
    }

    // draw robot boundary, ed
    {
        Eigen::Vector3d P0(robot_tf1(0,3), robot_tf1(1,3), robot_tf1(2,3));
        Eigen::Vector3d _P0 = obs_tf_inv.block(0,0,3,3)*P0 + obs_tf_inv.block(0,3,3,1);

        Eigen::Vector3d P1(path[idx1](0,3), path[idx1](1,3), path[idx1](2,3));
        Eigen::Vector3d _P1 = obs_tf_inv.block(0,0,3,3)*P1 + obs_tf_inv.block(0,3,3,1);

        cv::Vec2i uv0 = xy_uv(_P0[0], _P0[1]);
        cv::Vec2i uv1 = xy_uv(_P1[0], _P1[1]);
        cv::line(avoid_area, cv::Point(uv0[0], uv0[1]), cv::Point(uv1[0], uv1[1]), cv::Scalar(255), r);
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
    //cv::imshow("avoid_area", avoid_area2);

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

bool OBSMAP::is_tf_collision_dynamic(const Eigen::Matrix4d& robot_tf, double margin_x, double margin_y)
{
    // get obs map
    cv::Mat _dyn_map;
    Eigen::Matrix4d _obs_tf;
    get_dyn_map(_dyn_map, _obs_tf);

    // calc tf
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
            if(mask.ptr<uchar>(i)[j] == 255 && _dyn_map.ptr<uchar>(i)[j] == 255)
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

bool OBSMAP::is_path_collision(const std::vector<Eigen::Matrix4d>& robot_tfs, int st_idx, int idx_step, double margin_x, double margin_y)
{
    // get obs map
    cv::Mat _obs_map;
    Eigen::Matrix4d _obs_tf;
    get_obs_map(_obs_map, _obs_tf);

    const double x_min = config->ROBOT_SIZE_X[0] - margin_x;
    const double x_max = config->ROBOT_SIZE_X[1] + margin_x;
    const double y_min = config->ROBOT_SIZE_Y[0] - margin_y;
    const double y_max = config->ROBOT_SIZE_Y[1] + margin_y;

    Eigen::Vector3d P0(x_max, y_max, 0);
    Eigen::Vector3d P1(x_max, y_min, 0);
    Eigen::Vector3d P2(x_min, y_min, 0);
    Eigen::Vector3d P3(x_min, y_max, 0);

    cv::Mat mask(h, w, CV_8U, cv::Scalar(0));
    for(int p = st_idx; p < (int)robot_tfs.size(); p++)
    {
        if(p == st_idx || p == (int)robot_tfs.size()-1 || p%idx_step == 0)
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
    }

    /*
    cv::Mat debug_img(h, w, CV_8UC3, cv::Scalar(0));
    for(int i = 0; i < h; i++)
    {
        for(int j = 0; j < w; j++)
        {
            debug_img.ptr<cv::Vec3b>(i)[j] = cv::Vec3b(_obs_map.ptr<uchar>(i)[j], mask.ptr<uchar>(i)[j], 0);
        }
    }
    */

    for(int i = 0; i < h; i++)
    {
        for(int j = 0; j < w; j++)
        {
            if(mask.ptr<uchar>(i)[j] == 255 && _obs_map.ptr<uchar>(i)[j] == 255)
            {
                //debug_img.ptr<cv::Vec3b>(i)[j] = cv::Vec3b(0, 0, 255);
                //cv::imshow("debug_img", debug_img);
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
                _cnt0+=10;
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

    // raw data to range data
    double step = 0.25*D2R;
    std::vector<double> range_data((2*M_PI)/step, config->LIDAR_MAX_RANGE);
    for(size_t p = 0; p < tpp.pts.size(); p++)
    {
        double x = tpp.pts[p][0];
        double y = tpp.pts[p][1];
        double z = tpp.pts[p][2];
        if(z < config->OBS_MAP_MIN_Z || z > config->OBS_MAP_MAX_Z)
        {
            continue;
        }

        double th = std::atan2(y, x) + M_PI;

        int idx = th/step;
        if(idx > (int)range_data.size()-1)
        {
            continue;
        }

        double d = std::sqrt(x*x + y*y);
        if(d < range_data[idx])
        {
            range_data[idx] = d;
        }
    }

    std::vector<Eigen::Vector3d> pts;
    for(size_t p = 0; p < range_data.size(); p++)
    {
        double d = range_data[p];
        double th = p*step - M_PI;
        double x = d*std::cos(th);
        double y = d*std::sin(th);
        pts.push_back(Eigen::Vector3d(x,y,0));
    }

    // update octomap
    octomap::Pointcloud cloud;
    for(size_t p = 0; p < pts.size(); p++)
    {
        // local to global
        Eigen::Vector3d P = pts[p];
        Eigen::Vector3d _P = cur_tf.block(0,0,3,3)*P + cur_tf.block(0,3,3,1);
        cloud.push_back(_P[0], _P[1], 0);
    }

    mtx.lock();
    octree->insertPointCloud(cloud, octomap::point3d(cur_tf(0,3), cur_tf(1,3), 0), config->OBS_MAP_RANGE, false, true);

    // calc grid map
    octomap::point3d bbx_min(cur_tf(0,3) - config->OBS_MAP_RANGE, cur_tf(1,3) - config->OBS_MAP_RANGE, cur_tf(2,3) + config->OBS_MAP_MIN_Z);
    octomap::point3d bbx_max(cur_tf(0,3) + config->OBS_MAP_RANGE, cur_tf(1,3) + config->OBS_MAP_RANGE, cur_tf(2,3) + config->OBS_MAP_MAX_Z);

    std::vector<Eigen::Vector4d> _obs_pts;
    cv::Mat _map(h, w, CV_64F, cv::Scalar(0));
    for(octomap::OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(bbx_min, bbx_max, 16); it != octree->end_leafs_bbx(); it++)
    {
        double x = it.getX();
        double y = it.getY();
        double z = it.getZ();
        double prob = it->getOccupancy();

        // for plot
        _obs_pts.push_back(Eigen::Vector4d(x, y, z, prob));

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

    mtx.unlock();

    // make wall map
    cv::Mat _wall_map(h, w, CV_8U, cv::Scalar(0));
    for(int i = 0; i < h; i++)
    {
        for(int j = 0; j < w; j++)
        {
            if(_map.ptr<double>(i)[j] >= P_HIT)
            {
                _wall_map.ptr<uchar>(i)[j] = 255;
            }
        }
    }

    // add static map
    double query_pt[3] = {cur_tf(0,3), cur_tf(1,3), cur_tf(2,3)};
    double sq_radius = config->OBS_MAP_RANGE*config->OBS_MAP_RANGE;
    std::vector<nanoflann::ResultItem<unsigned int, double>> res_idxs;
    nanoflann::SearchParameters params;
    unimap->kdtree_index->radiusSearch(&query_pt[0], sq_radius, res_idxs, params);

    cv::Mat _static_map(h, w, CV_8U, cv::Scalar(0));
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

        _wall_map.ptr<uchar>(v)[u] = 255;
        _static_map.ptr<uchar>(v)[u] = 255;
    }

    // add obs nodes
    std::vector<QString> obs_nodes = unimap->get_nodes("OBS");
    if(obs_nodes.size() > 0)
    {
        for(size_t p = 0; p < obs_nodes.size(); p++)
        {
            QString id = obs_nodes[p];
            NODE* node = unimap->get_node_by_id(id);
            if(node != NULL)
            {
                QString info = node->info;
                Eigen::Matrix4d tf = node->tf;

                NODE_INFO res;
                if(parse_info(info, "SIZE", res))
                {
                    Eigen::Vector3d P0( res.sz[0]/2,  res.sz[1]/2, -res.sz[2]/2);
                    Eigen::Vector3d P1( res.sz[0]/2, -res.sz[1]/2,  res.sz[2]/2); // for z range check
                    Eigen::Vector3d P2(-res.sz[0]/2, -res.sz[1]/2, -res.sz[2]/2);
                    Eigen::Vector3d P3(-res.sz[0]/2,  res.sz[1]/2, -res.sz[2]/2);

                    Eigen::Vector3d _P0 = tf.block(0,0,3,3)*P0 + tf.block(0,3,3,1);
                    Eigen::Vector3d _P1 = tf.block(0,0,3,3)*P1 + tf.block(0,3,3,1);
                    Eigen::Vector3d _P2 = tf.block(0,0,3,3)*P2 + tf.block(0,3,3,1);
                    Eigen::Vector3d _P3 = tf.block(0,0,3,3)*P3 + tf.block(0,3,3,1);

                    Eigen::Vector3d l_P0 = cur_tf_inv.block(0,0,3,3)*_P0 + cur_tf_inv.block(0,3,3,1);
                    Eigen::Vector3d l_P1 = cur_tf_inv.block(0,0,3,3)*_P1 + cur_tf_inv.block(0,3,3,1);
                    Eigen::Vector3d l_P2 = cur_tf_inv.block(0,0,3,3)*_P2 + cur_tf_inv.block(0,3,3,1);
                    Eigen::Vector3d l_P3 = cur_tf_inv.block(0,0,3,3)*_P3 + cur_tf_inv.block(0,3,3,1);

                    // check z overlap
                    if(l_P1[2] < config->OBS_MAP_MIN_Z || l_P0[2] > config->OBS_MAP_MAX_Z)
                    {
                        // no overlap
                        continue;
                    }

                    cv::Vec2i uv0 = xy_uv(l_P0[0], l_P0[1]);
                    cv::Vec2i uv1 = xy_uv(l_P1[0], l_P1[1]);
                    cv::Vec2i uv2 = xy_uv(l_P2[0], l_P2[1]);
                    cv::Vec2i uv3 = xy_uv(l_P3[0], l_P3[1]);

                    std::vector<std::vector<cv::Point>> pts(1);
                    pts[0].push_back(cv::Point(uv0[0], uv0[1]));
                    pts[0].push_back(cv::Point(uv1[0], uv1[1]));
                    pts[0].push_back(cv::Point(uv2[0], uv2[1]));
                    pts[0].push_back(cv::Point(uv3[0], uv3[1]));

                    cv::fillPoly(_wall_map, pts, cv::Scalar(255));
                    cv::fillPoly(_static_map, pts, cv::Scalar(255));
                }
            }
        }
    }

    // subtract static map for dynamic obstacle
    cv::Mat _static_map2;
    cv::dilate(_static_map, _static_map2, cv::Mat(), cv::Point(-1,-1), 2);

    cv::Mat _dynamic_map;
    cv::subtract(_wall_map, _static_map2, _dynamic_map);

    // area filtering
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(_dynamic_map, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point());

    cv::Mat _dynamic_map2(h, w, CV_8U, cv::Scalar(0));
    for(size_t p = 0; p < contours.size(); p++)
    {
        if (contours[p].size() >= 2)
        {
            cv::drawContours(_dynamic_map2, contours, p, cv::Scalar(255), cv::FILLED, 8);
        }
    }

    // update
    mtx.lock();
    obs_pts = _obs_pts;    
    tf = cur_tf;
    prob_map = _map;
    wall_map = _wall_map;
    static_map = _static_map;
    dynamic_map = _dynamic_map2;
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
