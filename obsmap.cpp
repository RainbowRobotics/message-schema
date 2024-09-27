#include "obsmap.h"

OBSMAP::OBSMAP(QObject *parent)
    : QObject{parent}
{
    map_tf.setIdentity();
}

OBSMAP::~OBSMAP()
{

}

void OBSMAP::init()
{
    // init
    map_tf.setIdentity();

    w = (2*config->OBS_MAP_RANGE)/config->OBS_MAP_GRID_SIZE;
    h = (2*config->OBS_MAP_RANGE)/config->OBS_MAP_GRID_SIZE;
    cx = w/2;
    cy = h/2;
    gs = config->OBS_MAP_GRID_SIZE;

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
    octree->insertPointCloud(cloud, octomap::point3d(cur_tf(0,3), cur_tf(1,3), 0), config->OBS_MAP_RANGE, false, false);

    // calc grid map
    octomap::point3d bbx_min(cur_tf(0,3) - config->OBS_MAP_RANGE, cur_tf(1,3) - config->OBS_MAP_RANGE, -config->OBS_MAP_RANGE);
    octomap::point3d bbx_max(cur_tf(0,3) + config->OBS_MAP_RANGE, cur_tf(1,3) + config->OBS_MAP_RANGE, config->OBS_MAP_RANGE);

    std::vector<Eigen::Vector4d> _obs_pts;
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

        // for plot
        _obs_pts.push_back(Eigen::Vector4d(x, y, cur_tf(2,3), prob));

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
            if(_map.ptr<double>(i)[j] >= P_OBS)
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

        // global to local
        Eigen::Vector3d P(x,y,z);
        Eigen::Vector3d _P = cur_tf_inv.block(0,0,3,3)*P + cur_tf_inv.block(0,3,3,1);
        if(_P[2] < config->OBS_MAP_MIN_Z*0.5 || _P[2] > config->OBS_MAP_MAX_Z)
        {
            continue;
        }

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

    // add virtual obs nodes
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

    // add sensing boundary
    {
        cv::Vec2i uv = xy_uv(0, 0);
        int r = config->OBS_MAP_RANGE/gs;

        cv::Mat mask(h, w, CV_8U, cv::Scalar(255));
        cv::circle(mask, cv::Point(uv[0], uv[1]), r, cv::Scalar(0), -1);

        cv::bitwise_or(_wall_map, mask, _wall_map);
        cv::bitwise_or(_static_map, mask, _static_map);
    }

    // subtract static map for dynamic obstacle
    cv::Mat _static_map2;
    cv::dilate(_static_map, _static_map2, cv::Mat(), cv::Point(-1,-1), 3);
    cv::erode(_static_map2, _static_map2, cv::Mat(), cv::Point(-1,-1), 2);

    cv::Mat _dynamic_map;
    cv::subtract(_wall_map, _static_map2, _dynamic_map);
    cv::erode(_static_map2, _static_map2, cv::Mat(), cv::Point(-1,-1), 1);

    // area filtering
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(_dynamic_map, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point());

    cv::Mat _dynamic_map2(h, w, CV_8U, cv::Scalar(0));
    for(size_t p = 0; p < contours.size(); p++)
    {
        if (contours[p].size() >= 3)
        {
            cv::drawContours(_dynamic_map2, contours, p, cv::Scalar(255), cv::FILLED, 8);
        }
    }
    _dynamic_map = _dynamic_map2;

    // single point filtering
    for(int i = 1; i < h-1; i++)
    {
        for(int j = 1; j < w-1; j++)
        {
            int cnt0 = 0;
            int cnt1 = 0;
            int cnt2 = 0;
            for(int my = -1; my <= 1; my++)
            {
                for(int mx = -1; mx <= 1; mx++)
                {
                    int ii = i + my;
                    int jj = j + mx;

                    if(_wall_map.ptr<uchar>(ii)[jj] == 255)
                    {
                        cnt0++;
                    }

                    if(_static_map.ptr<uchar>(ii)[jj] == 255)
                    {
                        cnt1++;
                    }

                    if(_dynamic_map.ptr<uchar>(ii)[jj] == 255)
                    {
                        cnt2++;
                    }
                }
            }

            if(cnt0 == 1)
            {
                _wall_map.ptr<uchar>(i)[j] = 0;
            }

            if(cnt1 == 1)
            {
                _static_map.ptr<uchar>(i)[j] = 0;
            }

            if(cnt2 == 1)
            {
                _dynamic_map.ptr<uchar>(i)[j] = 0;
            }
        }
    }

    // update
    mtx.lock();
    obs_pts = _obs_pts;
    map_tf = cur_tf;
    wall_map = _wall_map;
    static_map = _static_map;
    dynamic_map = _dynamic_map2;
    mtx.unlock();

    // signal for redrawing
    Q_EMIT obs_updated();
}

void OBSMAP::get_obs_map(cv::Mat& map, Eigen::Matrix4d& tf)
{
    mtx.lock();    
    map = wall_map.clone();
    tf = map_tf;
    mtx.unlock();
}

void OBSMAP::get_dyn_map(cv::Mat& map, Eigen::Matrix4d& tf)
{
    mtx.lock();
    map = dynamic_map.clone();
    tf = map_tf;
    mtx.unlock();
}

std::vector<Eigen::Vector4d> OBSMAP::get_obs_pts()
{
    mtx.lock();
    std::vector<Eigen::Vector4d> res = obs_pts;
    mtx.unlock();

    return res;
}

void OBSMAP::draw_robot(cv::Mat& img, Eigen::Matrix4d robot_tf)
{
    mtx.lock();
    Eigen::Matrix4d G = map_tf.inverse()*robot_tf;
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

bool OBSMAP::is_pos_collision(const Eigen::Vector3d& pos, double radius, bool is_dyn)
{
    // get map
    cv::Mat _map;
    Eigen::Matrix4d _map_tf;

    if(is_dyn)
    {
        // dynamic map
        get_dyn_map(_map, _map_tf);
    }
    else
    {
        // static + dynamic map
        get_obs_map(_map, _map_tf);
    }

    Eigen::Matrix4d _map_tf_inv = _map_tf.inverse();

    // global to local
    Eigen::Vector3d P = _map_tf_inv.block(0,0,3,3)*pos + _map_tf_inv.block(0,3,3,1);
    cv::Vec2i uv = xy_uv(P[0], P[1]);

    // draw radius
    int r = std::ceil(radius/gs);
    cv::Mat mask(h, w, CV_8U, cv::Scalar(0));
    cv::circle(mask, cv::Point(uv[0], uv[1]), r, cv::Scalar(255), -1);

    for(int i = 0; i < h; i++)
    {
        for(int j = 0; j < w; j++)
        {
            if(mask.ptr<uchar>(i)[j] == 255 && _map.ptr<uchar>(i)[j] == 255)
            {
                // collision
                return true;
            }
        }
    }

    // non collision
    return false;
}

bool OBSMAP::is_tf_collision(const Eigen::Matrix4d& robot_tf, double margin_x, double margin_y, bool is_dyn)
{
    // get map
    cv::Mat _map;
    Eigen::Matrix4d _map_tf;

    if(is_dyn)
    {
        // dynamic map
        get_dyn_map(_map, _map_tf);
    }
    else
    {
        // static + dynamic map
        get_obs_map(_map, _map_tf);
    }

    Eigen::Matrix4d _map_tf_inv = _map_tf.inverse();

    // draw rect
    const double x_min = config->ROBOT_SIZE_X[0] - margin_x;
    const double x_max = config->ROBOT_SIZE_X[1] + margin_x;
    const double y_min = config->ROBOT_SIZE_Y[0] - margin_y;
    const double y_max = config->ROBOT_SIZE_Y[1] + margin_y;

    Eigen::Vector3d P0(x_max, y_max, 0);
    Eigen::Vector3d P1(x_max, y_min, 0);
    Eigen::Vector3d P2(x_min, y_min, 0);
    Eigen::Vector3d P3(x_min, y_max, 0);

    Eigen::Vector3d _P0 = _map_tf_inv.block(0,0,3,3)*P0 + _map_tf_inv.block(0,3,3,1);
    Eigen::Vector3d _P1 = _map_tf_inv.block(0,0,3,3)*P1 + _map_tf_inv.block(0,3,3,1);
    Eigen::Vector3d _P2 = _map_tf_inv.block(0,0,3,3)*P2 + _map_tf_inv.block(0,3,3,1);
    Eigen::Vector3d _P3 = _map_tf_inv.block(0,0,3,3)*P3 + _map_tf_inv.block(0,3,3,1);

    cv::Vec2i uv0 = xy_uv(_P0[0], _P0[1]);
    cv::Vec2i uv1 = xy_uv(_P1[0], _P1[1]);    
    cv::Vec2i uv2 = xy_uv(_P2[0], _P2[1]);
    cv::Vec2i uv3 = xy_uv(_P3[0], _P3[1]);

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
            if(mask.ptr<uchar>(i)[j] == 255 && _map.ptr<uchar>(i)[j] == 255)
            {
                return true;
            }
        }
    }

    // non collision
    return false;
}

bool OBSMAP::is_path_collision(const std::vector<Eigen::Matrix4d>& robot_tfs, double margin_x, double margin_y, int st_idx, int idx_step, bool is_dyn)
{
    // get map
    cv::Mat _map;
    Eigen::Matrix4d _map_tf;

    if(is_dyn)
    {
        // dynamic map
        get_dyn_map(_map, _map_tf);
    }
    else
    {
        // static + dynamic map
        get_obs_map(_map, _map_tf);
    }

    Eigen::Matrix4d _map_tf_inv = _map_tf.inverse();

    // draw trajectory
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
            Eigen::Matrix4d G = _map_tf_inv*robot_tfs[p];

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
            if(mask.ptr<uchar>(i)[j] == 255 && _map.ptr<uchar>(i)[j] == 255)
            {
                //debug_img.ptr<cv::Vec3b>(i)[j] = cv::Vec3b(0, 0, 255);
                //cv::imshow("debug_img", debug_img);
                return true;
            }
        }
    }

    return false;
}

// for avoid path
double OBSMAP::calc_clearance(const cv::Mat& map, const Eigen::Matrix4d& robot_tf, double radius)
{
    // in this function, robot_tf should be local tf referenced by map_tf

    // draw rect
    const double x_min = config->ROBOT_SIZE_X[0];
    const double x_max = config->ROBOT_SIZE_X[1];
    const double y_min = config->ROBOT_SIZE_Y[0];
    const double y_max = config->ROBOT_SIZE_Y[1];

    Eigen::Vector3d P0(x_max, y_max, 0);
    Eigen::Vector3d P1(x_max, y_min, 0);
    Eigen::Vector3d P2(x_min, y_min, 0);
    Eigen::Vector3d P3(x_min, y_max, 0);

    Eigen::Vector3d P = (P0 + P1 + P2 + P3)/4;
    Eigen::Vector3d _P = robot_tf.block(0,0,3,3)*P + robot_tf.block(0,3,3,1);

    cv::Vec2i uv = xy_uv(_P[0], _P[1]);

    cv::Mat mask(h, w, CV_8U, cv::Scalar(0));

    int r = std::ceil(radius/gs);
    cv::circle(mask, cv::Point(uv[0], uv[1]), r, cv::Scalar(255), -1);

    double min_d = radius;
    for(int i = 0; i < h; i++)
    {
        for(int j = 0; j < w; j++)
        {
            if(mask.ptr<uchar>(i)[j] == 255 && map.ptr<uchar>(i)[j] == 255)
            {
                double dx = (j - uv[0])*gs;
                double dy = (i - uv[1])*gs;
                double d = std::sqrt(dx*dx + dy*dy);
                if(d < min_d)
                {
                    min_d = d;
                }
            }
        }
    }

    return saturation(min_d, 0.1, radius);
}

bool OBSMAP::is_collision(const cv::Mat& map, const std::vector<Eigen::Matrix4d>& robot_tfs, double margin_x, double margin_y)
{
    // in this function, robot_tfs should be local trajctory referenced by map_tf

    const double x_min = config->ROBOT_SIZE_X[0] - margin_x;
    const double x_max = config->ROBOT_SIZE_X[1] + margin_x;
    const double y_min = config->ROBOT_SIZE_Y[0] - margin_y;
    const double y_max = config->ROBOT_SIZE_Y[1] + margin_y;

    Eigen::Vector3d P0(x_max, y_max, 0);
    Eigen::Vector3d P1(x_max, y_min, 0);
    Eigen::Vector3d P2(x_min, y_min, 0);
    Eigen::Vector3d P3(x_min, y_max, 0);

    cv::Mat mask(h, w, CV_8U, cv::Scalar(0));
    for(int p = 0; p < (int)robot_tfs.size(); p++)
    {
        Eigen::Matrix4d G = robot_tfs[p];

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

        cv::fillPoly(mask, pts, cv::Scalar(255));
    }

    for(int i = 0; i < h; i++)
    {
        for(int j = 0; j < w; j++)
        {
            if(mask.ptr<uchar>(i)[j] == 255 && map.ptr<uchar>(i)[j] == 255)
            {
                return true;
            }
        }
    }

    return false;
}

std::vector<Eigen::Matrix4d> OBSMAP::calc_path(Eigen::Matrix4d st_tf, Eigen::Matrix4d ed_tf)
{
    // params
    const double margin_x = config->OBS_PATH_MARGIN_X;
    const double margin_y = config->OBS_PATH_MARGIN_Y;

    const double chk_dt = 0.05;
    const double chk_dr = 5.0*D2R;

    const double sampling_dt = 0.2;
    const double sampling_dr = 10.0*D2R;

    // get obs map
    cv::Mat _obs_map;
    Eigen::Matrix4d _obs_tf;
    get_obs_map(_obs_map, _obs_tf);

    // for additional costing
    cv::Mat thin_map;
    //cv::ximgproc::thinning(~_obs_map, thin_map, cv::ximgproc::THINNING_GUOHALL);
    cv::ximgproc::thinning(~_obs_map, thin_map, cv::ximgproc::THINNING_ZHANGSUEN);

    // for debug
    cv::Mat debug_map;
    if(config->SIM_MODE == 1)
    {
        cv::cvtColor(_obs_map, debug_map, cv::COLOR_GRAY2BGR);

        for(int i = 0; i < h; i++)
        {
            for(int j = 0; j < w; j++)
            {
                if(_obs_map.ptr<uchar>(i)[j] == 0 && thin_map.ptr<uchar>(i)[j] == 255)
                {
                    debug_map.ptr<cv::Vec3b>(i)[j] = cv::Vec3b(196, 196, 196);
                }
            }
        }
    }

    // st, ed global to local
    Eigen::Matrix4d _obs_tf_inv = _obs_tf.inverse();
    Eigen::Matrix4d _st_tf = _obs_tf_inv*st_tf;
    Eigen::Matrix4d _ed_tf = _obs_tf_inv*ed_tf;
    Eigen::Vector2d dtdr0 = dTdR(_st_tf, _ed_tf);

    // search algorithm
    std::vector<ASTAR_NODE*> open_set;
    std::vector<ASTAR_NODE*> close_set;

    // set st, ed
    ASTAR_NODE *ed = new ASTAR_NODE();
    ed->tf = _ed_tf;

    ASTAR_NODE *st = new ASTAR_NODE();
    st->tf = _st_tf;
    st->g = 0;
    st->h = dtdr0[0] + std::abs(dtdr0[1]);
    st->f = st->g + st->h;
    open_set.push_back(st);

    // set first flag
    bool is_first = true;

    // search loop
    double st_time = get_time();
    while(open_set.size() > 0)
    {
        // get best node
        int cur_idx = 0;
        ASTAR_NODE *cur = open_set.front();
        for(size_t p = 0; p < open_set.size(); p++)
        {
            if(open_set[p]->f < cur->f)
            {
                cur = open_set[p];
                cur_idx = p;
            }
        }

        // pop open_set, push close_set
        open_set.erase(open_set.begin()+cur_idx);
        close_set.push_back(cur);

        // for debug
        if(config->SIM_MODE == 1)
        {
            cv::Vec2i uv = xy_uv(cur->tf(0,3), cur->tf(1,3));
            debug_map.ptr<cv::Vec3b>(uv[1])[uv[0]] = cv::Vec3b(0,255,0);

            cv::Vec2i uv_st = xy_uv(st->tf(0,3), st->tf(1,3));
            cv::Vec2i uv_ed = xy_uv(ed->tf(0,3), ed->tf(1,3));

            cv::circle(debug_map, cv::Point(uv_st[0], uv_st[1]), 3, cv::Scalar(0,0,255), 2);
            cv::circle(debug_map, cv::Point(uv_ed[0], uv_ed[1]), 3, cv::Scalar(0,255,255), 2);

            cv::Mat resized_debug_map;
            cv::resize(debug_map, resized_debug_map, cv::Size(debug_map.cols*2, debug_map.rows*2));
            cv::imshow("resized_debug_map", resized_debug_map);
        }

        // found goal
        Eigen::Matrix4d goal_tf0 = calc_tf(cur->tf.block(0,3,3,1), ed->tf.block(0,3,3,1));
        Eigen::Matrix4d goal_tf1 = goal_tf0;
        goal_tf0.block(0,3,3,1) = cur->tf.block(0,3,3,1);
        goal_tf1.block(0,3,3,1) = ed->tf.block(0,3,3,1);

        std::vector<Eigen::Matrix4d> traj_goal0 = intp_tf(cur->tf, goal_tf0, sampling_dt, sampling_dr);
        std::vector<Eigen::Matrix4d> traj_goal1 = intp_tf(goal_tf0, goal_tf1, sampling_dt, sampling_dr);

        std::vector<Eigen::Matrix4d> traj_goal = traj_goal0;
        traj_goal.insert(traj_goal.end(), traj_goal1.begin(), traj_goal1.end());

        if(!is_collision(_obs_map, traj_goal, margin_x, margin_y))
        {
            std::vector<Eigen::Matrix4d> res;

            ASTAR_NODE* _cur = cur;
            while(_cur != NULL)
            {
                res.push_back(_obs_tf*_cur->tf);
                _cur = _cur->parent;
            }

            std::reverse(res.begin(), res.end());

            // set final pose
            for(size_t p = 0; p < traj_goal.size(); p++)
            {
                res.push_back(_obs_tf*traj_goal[p]);
            }

            printf("[OBSMAP] path_finding complete, num:%d\n", (int)res.size());
            return res;
        }

        // expand nodes for differential drive
        double step = 0.2;
        std::vector<Eigen::Matrix4d> around;
        for(int i = -1; i <= 1; i++)
        {
            for(int j = -1; j <= 1; j++)
            {
                if(j == 0 && i == 0)
                {
                    continue;
                }

                double offset_x = j*step;
                double offset_y = i*step;
                double offset_th = std::atan2(offset_y, offset_x);

                Eigen::Vector3d offset_xi(offset_x, offset_y, offset_th);
                Eigen::Matrix4d offset_tf = se2_to_TF(offset_xi);

                Eigen::Matrix4d tf0 = cur->tf;
                Eigen::Matrix4d tf1 = offset_tf;

                tf1(0,3) += tf0(0,3);
                tf1(1,3) += tf0(1,3);
                tf1(2,3) += tf0(2,3);

                // check range
                if(calc_dist_2d(tf1.block(0,3,3,1)) > config->OBS_MAP_RANGE)
                {
                    continue;
                }

                // check close set
                bool is_close_set = false;
                for(size_t p = 0; p < close_set.size(); p++)
                {
                    Eigen::Vector2d dtdr = dTdR(close_set[p]->tf, tf1);
                    if(dtdr[0] < chk_dt && std::abs(dtdr[1]) < chk_dr)
                    {
                        is_close_set = true;
                        break;
                    }
                }
                if(is_close_set)
                {
                    continue;
                }

                std::vector<Eigen::Matrix4d> traj;
                if(is_first && std::abs(i) <= 1 && std::abs(j) <= 1)
                {
                    traj.push_back(tf1);
                }
                else
                {
                    traj = intp_tf(tf0, tf1, sampling_dt, sampling_dr);
                }

                // check collision
                if(is_collision(_obs_map, traj, margin_x, margin_y))
                {
                    continue;
                }

                around.push_back(tf1);
            }
        }

        // calc child node        
        for(size_t p = 0; p < around.size(); p++)
        {
            // calc heuristics
            double w = calc_clearance(thin_map, around[p], 1.5);

            Eigen::Vector2d dtdr1 = dTdR(cur->tf, around[p]);
            Eigen::Vector2d dtdr2 = dTdR(around[p], ed->tf);
            double g = cur->g + dtdr1[0]
                              + std::abs(dtdr1[1]) * 1.0/M_PI
                              + 1.0/calc_clearance(_obs_map, around[p], 1.5) * 0.1
                              + w;

            double h = dtdr2[0] + std::abs(dtdr2[0]) + w;
            double f = g + 10*h;

            // check open set
            bool is_open_set = false;
            ASTAR_NODE* open_node = NULL;
            for(size_t q = 0; q < open_set.size(); q++)
            {
                Eigen::Vector2d dtdr = dTdR(open_set[q]->tf, around[p]);
                if(dtdr[0] < chk_dt && std::abs(dtdr[1]) < chk_dr)
                {
                    is_open_set = true;
                    open_node = open_set[q];
                    break;
                }
            }

            if(is_open_set)
            {
                if(g < open_node->g)
                {
                    open_node->parent = cur;
                    open_node->g = g;
                    open_node->h = h;
                    open_node->f = f;
                }
                continue;
            }

            // add new child to open set
            ASTAR_NODE *child = new ASTAR_NODE();
            child->parent = cur;
            child->tf = around[p];
            child->g = g;
            child->h = h;
            child->f = f;
            open_set.push_back(child);
        }

        double timeout = get_time() - st_time;
        if(timeout > 20.0)
        {
            printf("[OBSMAP] timeout\n");
            break;
        }

        // clear first flag
        is_first = false;
    }

    std::vector<Eigen::Matrix4d> res;
    return res;
}

// util
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
