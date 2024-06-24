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

bool OBSMAP::is_collision(const Eigen::Matrix4d& robot_tf)
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

bool OBSMAP::is_collision(const std::vector<Eigen::Matrix4d>& robot_tfs)
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

bool OBSMAP::is_collision(const cv::Mat& obs_map, const Eigen::Matrix4d& obs_tf, const Eigen::Matrix4d& robot_tf, const cv::Mat& avoid_area)
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

bool OBSMAP::is_collision(const cv::Mat& obs_map, const Eigen::Matrix4d& obs_tf, const std::vector<Eigen::Matrix4d>& robot_tfs, const cv::Mat& avoid_area)
{
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
        Eigen::Matrix4d G = obs_tf.inverse()*robot_tfs[p];

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
            if(mask.ptr<uchar>(i)[j] == 255 && obs_map.ptr<uchar>(i)[j] == 255)
            {
                return true;
            }

            if(mask.ptr<uchar>(i)[j] == 255 && avoid_area.ptr<uchar>(i)[j] == 0)
            {
                return true;
            }
        }
    }

    return false;
}

bool OBSMAP::is_pivot_collision(const cv::Mat& obs_map, const Eigen::Matrix4d& obs_tf, const Eigen::Matrix4d& robot_tf, const cv::Mat& avoid_area)
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

int OBSMAP::get_conflict_idx(const cv::Mat& obs_map, const Eigen::Matrix4d& obs_tf, const std::vector<Eigen::Matrix4d>& robot_tfs, const cv::Mat& avoid_area, const int idx0)
{
    const double x_min = config->ROBOT_SIZE_X[0];
    const double x_max = config->ROBOT_SIZE_X[1];
    const double y_min = config->ROBOT_SIZE_Y[0];
    const double y_max = config->ROBOT_SIZE_Y[1];

    Eigen::Vector3d P0(x_min, y_min, 0);
    Eigen::Vector3d P1(x_min, y_max, 0);
    Eigen::Vector3d P2(x_max, y_max, 0);
    Eigen::Vector3d P3(x_max, y_min, 0);

    int conflict_idx = idx0;
    for(size_t p = idx0; p < robot_tfs.size(); p++)
    {
        Eigen::Matrix4d G = obs_tf.inverse()*robot_tfs[p];

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
                    // return previous non confilict index
                    return conflict_idx;
                }

                if(mask.ptr<uchar>(i)[j] == 255 && avoid_area.ptr<uchar>(i)[j] == 0)
                {
                    // return previous non confilict index
                    return conflict_idx;
                }
            }
        }

        // update
        conflict_idx = p;
    }

    // idx0 or last index of pre local path
    return conflict_idx;
}

void OBSMAP::update_obs_map(TIME_POSE_PTS& tpp)
{
    Eigen::Matrix4d cur_tf = tpp.tf;
    Eigen::Matrix4d cur_tf_inv = cur_tf.inverse();

    octomap::Pointcloud cloud;
    for(size_t p = 0; p < tpp.pts.size(); p++)
    {
        Eigen::Vector3d P = tpp.pts[p];

        // range filtering
        double d = calc_dist_2d(P);
        if(d > config->OBS_MAP_RANGE)
        {
            continue;
        }

        Eigen::Vector3d _P = cur_tf.block(0,0,3,3)*P + cur_tf.block(0,3,3,1);
        cloud.push_back(_P[0], _P[1], _P[2]);
    }

    unimap->octree->insertPointCloud(cloud, octomap::point3d(cur_tf(0,3), cur_tf(1,3), cur_tf(2,3)));

    // set local grid map
    cv::Mat _map(h, w, CV_64F, cv::Scalar(0));

    // obsmap boundary
    octomap::point3d bbx_min(cur_tf(0,3) - config->OBS_MAP_RANGE, cur_tf(1,3) - config->OBS_MAP_RANGE, cur_tf(2,3) + config->OBS_MAP_MIN_Z);
    octomap::point3d bbx_max(cur_tf(0,3) + config->OBS_MAP_RANGE, cur_tf(1,3) + config->OBS_MAP_RANGE, cur_tf(2,3) + config->OBS_MAP_MAX_Z);
    for(octomap::OcTree::leaf_bbx_iterator it = unimap->octree->begin_leafs_bbx(bbx_min, bbx_max, 16); it != unimap->octree->end_leafs_bbx(); it++)
    {
        double x = it.getX();
        double y = it.getY();
        double z = it.getZ();
        double prob = it->getOccupancy();

        Eigen::Vector3d P(x,y,z);
        Eigen::Vector3d _P = cur_tf_inv.block(0,0,3,3)*P + cur_tf_inv.block(0,3,3,1);

        cv::Vec2i uv = xy_uv(_P[0], _P[1]);
        int u = uv[0];
        int v = uv[1];
        if(u < 0 || u >= w || v < 0 || v >= h)
        {
            continue;
        }

        _map.ptr<double>(v)[u] = prob;
    }

    // update map
    mtx.lock();
    map = _map;
    tf = cur_tf;
    mtx.unlock();

    // signal for redrawing
    Q_EMIT obs_updated();
}

/*
void OBSMAP::update_obs_map(TIME_POSE_PTS& tpp)
{
    // update storage
    tpp_storage.push_back(tpp);
    if(tpp_storage.size() > 30)
    {
        tpp_storage.erase(tpp_storage.begin());
    }

    // build obs map
    cv::Mat _map(h, w, CV_64F, cv::Scalar(0));
    for(size_t p = 0; p < tpp_storage.size(); p+=2)
    {
        Eigen::Matrix4d G = tpp.tf.inverse()*tpp_storage[p].tf;

        cv::Mat hit_map(h, w, CV_8U, cv::Scalar(0));
        cv::Mat miss_map(h, w, CV_8U, cv::Scalar(0));
        for(size_t q = 0; q < tpp_storage[p].pts.size(); q++)
        {
            Eigen::Vector3d P = tpp_storage[p].pts[q];
            if(P[0] >= config->ROBOT_SIZE_X[0] && P[0] <= config->ROBOT_SIZE_X[1] &&
               P[1] >= config->ROBOT_SIZE_Y[0] && P[1] <= config->ROBOT_SIZE_Y[1])
            {
                continue;
            }

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

    // update map
    mtx.lock();
    map = _map;
    tf = tpp.tf;
    mtx.unlock();
}
*/

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
