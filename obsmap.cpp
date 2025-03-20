#include "obsmap.h"

OBSMAP::OBSMAP(QObject *parent)
    : QObject{parent}
{
    map_tf.setIdentity();        
}

OBSMAP::~OBSMAP()
{
    if(octree != NULL)
    {
        octree->clear();
        delete octree;
        octree= NULL;
    }
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
    virtual_map = cv::Mat(h, w, CV_8U, cv::Scalar(0));

    octree = new octomap::OcTree(config->OBS_MAP_GRID_SIZE);
    octree->setProbHit(P_HIT);
    octree->setProbMiss(P_MISS);
}

void OBSMAP::clear()
{
    mtx.lock();

    if(octree != NULL)
    {
        octree->clear();
        delete octree;
        octree= NULL;
    }

    octree = new octomap::OcTree(config->OBS_MAP_GRID_SIZE);
    octree->setProbHit(P_HIT);
    octree->setProbMiss(P_MISS);

    obs_pts_static.clear();
    obs_pts_dynamic.clear();
    obs_pts_virtual.clear();
    closure_pts_virtual.clear();
    plot_pts.clear();

    vobs_list_robots.clear();
    vobs_list_closures.clear();

    wall_map = cv::Mat(h, w, CV_8U, cv::Scalar(0));
    static_map = cv::Mat(h, w, CV_8U, cv::Scalar(0));
    dynamic_map = cv::Mat(h, w, CV_8U, cv::Scalar(0));
    virtual_map = cv::Mat(h, w, CV_8U, cv::Scalar(0));
    map_tf = Eigen::Matrix4d::Identity();

    mtx.unlock();
}

void OBSMAP::update_obs_map_sim(Eigen::Matrix4d tf)
{
    mtx.lock();

    Eigen::Matrix4d cur_tf = tf;
    Eigen::Matrix4d cur_tf_inv = cur_tf.inverse();

    // add static map
    double query_pt[3] = {cur_tf(0,3), cur_tf(1,3), cur_tf(2,3)};
    double sq_radius = config->OBS_MAP_RANGE*config->OBS_MAP_RANGE;
    std::vector<nanoflann::ResultItem<unsigned int, double>> res_idxs;
    nanoflann::SearchParameters params;
    unimap->kdtree_index->radiusSearch(&query_pt[0], sq_radius, res_idxs, params);

    cv::Mat _wall_map(h, w, CV_8U, cv::Scalar(0));
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
    }

    map_tf = cur_tf;
    wall_map = _wall_map;
    mtx.unlock();

    // signal for redrawing
    Q_EMIT obs_updated();
}

void OBSMAP::update_obs_map(TIME_POSE_PTS& tpp)
{
    mtx.lock();    
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

    octree->insertPointCloud(cloud, octomap::point3d(cur_tf(0,3), cur_tf(1,3), 0), config->OBS_MAP_RANGE, false, false);

    // calc grid map
    octomap::point3d bbx_min(cur_tf(0,3) - config->OBS_MAP_RANGE, cur_tf(1,3) - config->OBS_MAP_RANGE, -config->OBS_MAP_RANGE);
    octomap::point3d bbx_max(cur_tf(0,3) + config->OBS_MAP_RANGE, cur_tf(1,3) + config->OBS_MAP_RANGE, config->OBS_MAP_RANGE);

    std::vector<Eigen::Vector4d> global_obs_pts;
    std::vector<Eigen::Vector2d> local_obs_pts;
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
        global_obs_pts.push_back(Eigen::Vector4d(x, y, cur_tf(2,3), prob));
        local_obs_pts.push_back(Eigen::Vector2d(_P[0], _P[1]));

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

                    if(config->USE_SIM == 0)
                    {
                        cv::fillPoly(_static_map, pts, cv::Scalar(255));
                    }
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
    {
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
    }

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

    // set denoised obs_pts
    std::vector<Eigen::Vector3d> _obs_pts;
    std::vector<Eigen::Vector3d> _dyn_pts;
    std::vector<Eigen::Vector4d> _plot_pts;
    for(size_t p = 0; p < local_obs_pts.size(); p++)
    {
        double x = local_obs_pts[p][0];
        double y = local_obs_pts[p][1];

        cv::Vec2i uv = xy_uv(x,y);
        if(uv[0] < 0 || uv[0] >= w || uv[1] < 0 || uv[1] >= h)
        {
            continue;
        }

        if(_wall_map.ptr<uchar>(uv[1])[uv[0]] == 255)
        {
            _plot_pts.push_back(global_obs_pts[p]);
            _obs_pts.push_back(Eigen::Vector3d(global_obs_pts[p][0], global_obs_pts[p][1], global_obs_pts[p][2]));
        }

        if(_dynamic_map.ptr<uchar>(uv[1])[uv[0]] == 255)
        {
            _dyn_pts.push_back(Eigen::Vector3d(global_obs_pts[p][0], global_obs_pts[p][1], global_obs_pts[p][2]));
        }
    }

    // update    
    obs_pts_static = _obs_pts;
    obs_pts_dynamic = _dyn_pts;
    plot_pts = _plot_pts;
    map_tf = cur_tf;
    wall_map = _wall_map;
    static_map = _static_map;
    dynamic_map = _dynamic_map;
    mtx.unlock();

    // signal for redrawing
    Q_EMIT obs_updated();
}

void OBSMAP::update_vobs_map()
{
    mtx.lock();
    Eigen::Matrix4d cur_tf = map_tf;
    Eigen::Matrix4d cur_tf_inv = cur_tf.inverse();

    // add vobs robots from fms
    std::vector<Eigen::Vector3d> _vir_pts;    
    cv::Mat _virtual_map(h, w, CV_8U, cv::Scalar(0));
    for(size_t p = 0; p < vobs_list_robots.size(); p++)
    {
        // add global points
        Eigen::Vector3d center = vobs_list_robots[p];

        std::vector<Eigen::Vector3d> pts = circle_iterator_3d(center, config->ROBOT_SIZE_Y[1]);
        pts.push_back(center);

        for(size_t q = 0; q < pts.size(); q++)
        {
            Eigen::Vector3d P = pts[q];
            _vir_pts.push_back(P);

            // add local obsmap
            Eigen::Vector3d _P = cur_tf_inv.block(0,0,3,3)*P + cur_tf_inv.block(0,3,3,1);

            double x = _P[0];
            double y = _P[1];

            cv::Vec2i uv = xy_uv(x,y);
            if(uv[0] < 0 || uv[0] >= w || uv[1] < 0 || uv[1] >= h)
            {
                continue;
            }
            _virtual_map.ptr<uchar>(uv[1])[uv[0]] = 255;
        }
    }

    // add vobs closures from fms
    std::vector<Eigen::Vector3d> _vir_closure_pts;
    for(size_t p = 0; p < vobs_list_closures.size(); p++)
    {
        // add global points
        Eigen::Vector3d center = vobs_list_closures[p];
        _vir_closure_pts.push_back(center);

        //std::vector<Eigen::Vector3d> pts = circle_iterator_3d(center, 0.05);
        std::vector<Eigen::Vector3d> pts = circle_iterator_3d(center, config->ROBOT_SIZE_Y[1]);
        pts.push_back(center);

        for(size_t q = 0; q < pts.size(); q++)
        {
            Eigen::Vector3d P = pts[q];
            _vir_pts.push_back(P);

            // add local obsmap
            Eigen::Vector3d _P = cur_tf_inv.block(0,0,3,3)*P + cur_tf_inv.block(0,3,3,1);

            double x = _P[0];
            double y = _P[1];

            cv::Vec2i uv = xy_uv(x,y);
            if(uv[0] < 0 || uv[0] >= w || uv[1] < 0 || uv[1] >= h)
            {
                continue;
            }
            _virtual_map.ptr<uchar>(uv[1])[uv[0]] = 255;
        }
    }

    obs_pts_virtual = _vir_pts;
    closure_pts_virtual = _vir_closure_pts;
    virtual_map = _virtual_map;

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

void OBSMAP::get_vir_map(cv::Mat& map, Eigen::Matrix4d& tf)
{
    mtx.lock();
    map = virtual_map.clone();
    tf = map_tf;
    mtx.unlock();
}

std::vector<Eigen::Vector3d> OBSMAP::get_obs_pts()
{
    mtx.lock();
    std::vector<Eigen::Vector3d> res = obs_pts_static;
    mtx.unlock();

    return res;
}

std::vector<Eigen::Vector3d> OBSMAP::get_dyn_pts()
{
    mtx.lock();
    std::vector<Eigen::Vector3d> res = obs_pts_dynamic;
    mtx.unlock();

    return res;
}

std::vector<Eigen::Vector3d> OBSMAP::get_vir_pts()
{
    mtx.lock();
    std::vector<Eigen::Vector3d> res = obs_pts_virtual;
    mtx.unlock();

    return res;
}

std::vector<Eigen::Vector3d> OBSMAP::get_vir_closure_pts()
{
    mtx.lock();
    std::vector<Eigen::Vector3d> res = closure_pts_virtual;
    mtx.unlock();

    return res;
}

std::vector<Eigen::Vector4d> OBSMAP::get_plot_pts()
{
    mtx.lock();
    std::vector<Eigen::Vector4d> res = plot_pts;
    mtx.unlock();

    return res;
}

void OBSMAP::draw_robot(cv::Mat& img)
{
    // draw rect
    const double x_min = config->ROBOT_SIZE_X[0];
    const double x_max = config->ROBOT_SIZE_X[1];
    const double y_min = config->ROBOT_SIZE_Y[0];
    const double y_max = config->ROBOT_SIZE_Y[1];

    Eigen::Vector3d P0(x_max, y_max, 0);
    Eigen::Vector3d P1(x_max, y_min, 0);
    Eigen::Vector3d P2(x_min, y_min, 0);
    Eigen::Vector3d P3(x_min, y_max, 0);

    cv::Vec2i uv0 = xy_uv(P0[0], P0[1]);
    cv::Vec2i uv1 = xy_uv(P1[0], P1[1]);
    cv::Vec2i uv2 = xy_uv(P2[0], P2[1]);
    cv::Vec2i uv3 = xy_uv(P3[0], P3[1]);

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

    cv::Vec2i uv_c = xy_uv(P_c[0], P_c[1]);
    cv::Vec2i uv_x = xy_uv(P_x[0], P_x[1]);
    cv::Vec2i uv_y = xy_uv(P_y[0], P_y[1]);

    cv::line(img, cv::Point(uv_c[0], uv_c[1]), cv::Point(uv_y[0], uv_y[1]), cv::Scalar(0,255,0), 1, cv::LINE_AA);
    cv::line(img, cv::Point(uv_c[0], uv_c[1]), cv::Point(uv_x[0], uv_x[1]), cv::Scalar(0,0,255), 1, cv::LINE_AA);
}

int OBSMAP::is_tf_collision(const Eigen::Matrix4d& robot_tf, double margin_x, double margin_y)
{
    // get
    mtx.lock();
    std::vector<Eigen::Vector3d> pts = obs_pts_static;
    mtx.unlock();

    // robot pts
    const double x_min = config->ROBOT_SIZE_X[0] - margin_x;
    const double x_max = config->ROBOT_SIZE_X[1] + margin_x;
    const double y_min = config->ROBOT_SIZE_Y[0] - margin_y;
    const double y_max = config->ROBOT_SIZE_Y[1] + margin_y;

    Eigen::Vector3d P0(x_max, y_max, 0);
    Eigen::Vector3d P1(x_max, y_min, 0);
    Eigen::Vector3d P2(x_min, y_min, 0);
    Eigen::Vector3d P3(x_min, y_max, 0);

    std::vector<Eigen::Vector3d> robot_pts;
    robot_pts.push_back(robot_tf.block(0,0,3,3)*P0 + robot_tf.block(0,3,3,1));
    robot_pts.push_back(robot_tf.block(0,0,3,3)*P1 + robot_tf.block(0,3,3,1));
    robot_pts.push_back(robot_tf.block(0,0,3,3)*P2 + robot_tf.block(0,3,3,1));
    robot_pts.push_back(robot_tf.block(0,0,3,3)*P3 + robot_tf.block(0,3,3,1));

    // check collision (static obs)
    for(size_t p = 0; p < pts.size(); p++)
    {
        Eigen::Vector3d P = Eigen::Vector3d(pts[p][0], pts[p][1], pts[p][2]);

        bool is_collision = false;
        for(size_t q = 0; q < robot_pts.size(); q++)
        {
            Eigen::Vector3d pt = Eigen::Vector3d(P[0], P[1], P[2]);
            Eigen::Vector3d pt0 = robot_pts[q];
            Eigen::Vector3d pt1 = robot_pts[(q+1) % robot_pts.size()];

            if (((pt0[1] >= pt[1] && pt1[1] < pt[1]) || (pt0[1] < pt[1] && pt1[1] >= pt[1])) &&
                 (pt[0] < (pt1[0]-pt0[0])*(pt[1]-pt0[1])/(pt1[1]-pt0[1]) + pt0[0]))
            {
                is_collision = !is_collision;
            }
        }

        // collision
        if(is_collision)
        {
            return OBS_DETECT_STATIC;
        }
    }

    // no collision
    return OBS_DETECT_NONE;
}

int OBSMAP::is_tf_collision_dyn(const Eigen::Matrix4d& robot_tf, double margin_x, double margin_y)
{
    // get
    mtx.lock();
    std::vector<Eigen::Vector3d> pts_dyn = obs_pts_dynamic;
    std::vector<Eigen::Vector3d> pts_vir = obs_pts_virtual;
    mtx.unlock();

    // robot pts
    const double x_min = config->ROBOT_SIZE_X[0] - margin_x;
    const double x_max = config->ROBOT_SIZE_X[1] + margin_x;
    const double y_min = config->ROBOT_SIZE_Y[0] - margin_y;
    const double y_max = config->ROBOT_SIZE_Y[1] + margin_y;

    Eigen::Vector3d P0(x_max, y_max, 0);
    Eigen::Vector3d P1(x_max, y_min, 0);
    Eigen::Vector3d P2(x_min, y_min, 0);
    Eigen::Vector3d P3(x_min, y_max, 0);

    std::vector<Eigen::Vector3d> robot_pts;
    robot_pts.push_back(robot_tf.block(0,0,3,3)*P0 + robot_tf.block(0,3,3,1));
    robot_pts.push_back(robot_tf.block(0,0,3,3)*P1 + robot_tf.block(0,3,3,1));
    robot_pts.push_back(robot_tf.block(0,0,3,3)*P2 + robot_tf.block(0,3,3,1));
    robot_pts.push_back(robot_tf.block(0,0,3,3)*P3 + robot_tf.block(0,3,3,1));

    // check collision (dynamic)
    for(size_t p = 0; p < pts_dyn.size(); p++)
    {
        Eigen::Vector3d P = Eigen::Vector3d(pts_dyn[p][0], pts_dyn[p][1], pts_dyn[p][2]);

        bool is_collision = false;
        for(size_t q = 0; q < robot_pts.size(); q++)
        {
            Eigen::Vector3d pt = Eigen::Vector3d(P[0], P[1], P[2]);
            Eigen::Vector3d pt0 = robot_pts[q];
            Eigen::Vector3d pt1 = robot_pts[(q+1) % robot_pts.size()];

            if (((pt0[1] >= pt[1] && pt1[1] < pt[1]) || (pt0[1] < pt[1] && pt1[1] >= pt[1])) &&
                 (pt[0] < (pt1[0]-pt0[0])*(pt[1]-pt0[1])/(pt1[1]-pt0[1]) + pt0[0]))
            {
                is_collision = !is_collision;
            }
        }

        // collision
        if(is_collision)
        {
            return OBS_DETECT_DYNAMIC;
        }
    }

    // check collision (virtual)
    for(size_t p = 0; p < pts_vir.size(); p++)
    {
        Eigen::Vector3d P = Eigen::Vector3d(pts_vir[p][0], pts_vir[p][1], pts_vir[p][2]);

        bool is_collision = false;
        for(size_t q = 0; q < robot_pts.size(); q++)
        {
            Eigen::Vector3d pt = Eigen::Vector3d(P[0], P[1], P[2]);
            Eigen::Vector3d pt0 = robot_pts[q];
            Eigen::Vector3d pt1 = robot_pts[(q+1) % robot_pts.size()];

            if (((pt0[1] >= pt[1] && pt1[1] < pt[1]) || (pt0[1] < pt[1] && pt1[1] >= pt[1])) &&
                 (pt[0] < (pt1[0]-pt0[0])*(pt[1]-pt0[1])/(pt1[1]-pt0[1]) + pt0[0]))
            {
                is_collision = !is_collision;
            }
        }

        // collision
        if(is_collision)
        {
            return OBS_DETECT_VIRTUAL;
        }
    }

    // no collision
    return OBS_DETECT_NONE;
}

int OBSMAP::is_path_collision(const std::vector<Eigen::Matrix4d>& robot_tfs, double margin_x, double margin_y, int st_idx, int idx_step)
{
    // get
    mtx.lock();
    std::vector<Eigen::Vector3d> pts = obs_pts_static;
    mtx.unlock();

    // draw trajectory
    const double x_min = config->ROBOT_SIZE_X[0] - margin_x;
    const double x_max = config->ROBOT_SIZE_X[1] + margin_x;
    const double y_min = config->ROBOT_SIZE_Y[0] - margin_y;
    const double y_max = config->ROBOT_SIZE_Y[1] + margin_y;

    Eigen::Vector3d P0(x_max, y_max, 0);
    Eigen::Vector3d P1(x_max, y_min, 0);
    Eigen::Vector3d P2(x_min, y_min, 0);
    Eigen::Vector3d P3(x_min, y_max, 0);

    // check collision (static obs)
    for(int i = st_idx; i < (int)robot_tfs.size(); i++)
    {
        if(i == st_idx || i == (int)robot_tfs.size()-1 || i%idx_step == 0)
        {
            Eigen::Matrix4d G = robot_tfs[i];

            std::vector<Eigen::Vector3d> robot_pts;
            robot_pts.push_back(G.block(0,0,3,3)*P0 + G.block(0,3,3,1));
            robot_pts.push_back(G.block(0,0,3,3)*P1 + G.block(0,3,3,1));
            robot_pts.push_back(G.block(0,0,3,3)*P2 + G.block(0,3,3,1));
            robot_pts.push_back(G.block(0,0,3,3)*P3 + G.block(0,3,3,1));

            for(size_t p = 0; p < pts.size(); p++)
            {
                bool is_collision = false;
                for(size_t q = 0; q < robot_pts.size(); q++)
                {
                    Eigen::Vector3d pt = pts[p];
                    Eigen::Vector3d pt0 = robot_pts[q];
                    Eigen::Vector3d pt1 = robot_pts[(q+1) % robot_pts.size()];

                    if (((pt0[1] >= pt[1] && pt1[1] < pt[1]) || (pt0[1] < pt[1] && pt1[1] >= pt[1])) &&
                         (pt[0] < (pt1[0]-pt0[0])*(pt[1]-pt0[1])/(pt1[1]-pt0[1]) + pt0[0]))
                    {
                        is_collision = !is_collision;
                    }
                }

                // collision
                if(is_collision)
                {
                    return OBS_DETECT_STATIC;
                }
            }
        }
    }

    // no collision
    return OBS_DETECT_NONE;
}

int OBSMAP::is_path_collision_dyn(const std::vector<Eigen::Matrix4d>& robot_tfs_dyn, const std::vector<Eigen::Matrix4d>& robot_tfs_vir, double margin_x, double margin_y, int st_idx, int idx_step)
{
    // get
    std::vector<Eigen::Vector3d> pts_dyn;
    std::vector<Eigen::Vector3d> pts_vir;
    mtx.lock();
    pts_dyn = obs_pts_dynamic;
    pts_vir = obs_pts_virtual;
    mtx.unlock();

    // draw trajectory
    const double x_min = config->ROBOT_SIZE_X[0] - margin_x;
    const double x_max = config->ROBOT_SIZE_X[1] + margin_x;
    const double y_min = config->ROBOT_SIZE_Y[0] - margin_y;
    const double y_max = config->ROBOT_SIZE_Y[1] + margin_y;

    Eigen::Vector3d P0(x_max, y_max, 0);
    Eigen::Vector3d P1(x_max, y_min, 0);
    Eigen::Vector3d P2(x_min, y_min, 0);
    Eigen::Vector3d P3(x_min, y_max, 0);

    // check collision (dynamic obs)
    for(int i = st_idx; i < (int)robot_tfs_dyn.size(); i++)
    {
        if(i == st_idx || i == (int)robot_tfs_dyn.size()-1 || i%idx_step == 0)
        {
            Eigen::Matrix4d G = robot_tfs_dyn[i];

            std::vector<Eigen::Vector3d> robot_pts;
            robot_pts.push_back(G.block(0,0,3,3)*P0 + G.block(0,3,3,1));
            robot_pts.push_back(G.block(0,0,3,3)*P1 + G.block(0,3,3,1));
            robot_pts.push_back(G.block(0,0,3,3)*P2 + G.block(0,3,3,1));
            robot_pts.push_back(G.block(0,0,3,3)*P3 + G.block(0,3,3,1));

            for(size_t p = 0; p < pts_dyn.size(); p++)
            {
                bool is_collision = false;
                for(size_t q = 0; q < robot_pts.size(); q++)
                {
                    Eigen::Vector3d pt = pts_dyn[p];
                    Eigen::Vector3d pt0 = robot_pts[q];
                    Eigen::Vector3d pt1 = robot_pts[(q+1) % robot_pts.size()];

                    if(((pt0[1] >= pt[1] && pt1[1] < pt[1]) || (pt0[1] < pt[1] && pt1[1] >= pt[1])) &&
                         (pt[0] < (pt1[0]-pt0[0])*(pt[1]-pt0[1])/(pt1[1]-pt0[1]) + pt0[0]))
                    {
                        is_collision = !is_collision;
                    }
                }

                // collision
                if(is_collision)
                {
                    return OBS_DETECT_DYNAMIC;
                }
            }
        }
    }

    // check collision (virtual obs)
    for(int i = st_idx; i < (int)robot_tfs_vir.size(); i++)
    {
        if(i == st_idx || i == (int)robot_tfs_vir.size()-1 || i%idx_step == 0)
        {
            Eigen::Matrix4d G = robot_tfs_vir[i];

            std::vector<Eigen::Vector3d> robot_pts;
            robot_pts.push_back(G.block(0,0,3,3)*P0 + G.block(0,3,3,1));
            robot_pts.push_back(G.block(0,0,3,3)*P1 + G.block(0,3,3,1));
            robot_pts.push_back(G.block(0,0,3,3)*P2 + G.block(0,3,3,1));
            robot_pts.push_back(G.block(0,0,3,3)*P3 + G.block(0,3,3,1));

            for(size_t p = 0; p < pts_vir.size(); p++)
            {
                bool is_collision = false;
                for(size_t q = 0; q < robot_pts.size(); q++)
                {
                    Eigen::Vector3d pt = pts_vir[p];
                    Eigen::Vector3d pt0 = robot_pts[q];
                    Eigen::Vector3d pt1 = robot_pts[(q+1) % robot_pts.size()];

                    if(((pt0[1] >= pt[1] && pt1[1] < pt[1]) || (pt0[1] < pt[1] && pt1[1] >= pt[1])) &&
                         (pt[0] < (pt1[0]-pt0[0])*(pt[1]-pt0[1])/(pt1[1]-pt0[1]) + pt0[0]))
                    {
                        is_collision = !is_collision;
                    }
                }

                // collision
                if(is_collision)
                {
                    return OBS_DETECT_VIRTUAL;
                }
            }
        }
    }

    // no collision
    return OBS_DETECT_NONE;
}

int OBSMAP::is_undock_path_collision(const std::vector<Eigen::Matrix4d>& robot_tfs, double margin_x, double margin_y, int st_idx, int idx_step)
{
    // get
    mtx.lock();
    std::vector<Eigen::Vector3d> pts = obs_pts_static;
    mtx.unlock();

    // draw trajectory
    const double x_min = config->ROBOT_SIZE_X[0] - margin_x;
    const double x_max = config->ROBOT_SIZE_X[1] - 0.1;      // excluding forward charging station detection
    const double y_min = config->ROBOT_SIZE_Y[0] - margin_y;
    const double y_max = config->ROBOT_SIZE_Y[1] + margin_y;

    Eigen::Vector3d P0(x_max, y_max, 0);
    Eigen::Vector3d P1(x_max, y_min, 0);
    Eigen::Vector3d P2(x_min, y_min, 0);
    Eigen::Vector3d P3(x_min, y_max, 0);

    // check collision
    for(int i = st_idx; i < (int)robot_tfs.size(); i++)
    {
        if(i == st_idx || i == (int)robot_tfs.size()-1 || i%idx_step == 0)
        {
            Eigen::Matrix4d G = robot_tfs[i];

            std::vector<Eigen::Vector3d> robot_pts;
            robot_pts.push_back(G.block(0,0,3,3)*P0 + G.block(0,3,3,1));
            robot_pts.push_back(G.block(0,0,3,3)*P1 + G.block(0,3,3,1));
            robot_pts.push_back(G.block(0,0,3,3)*P2 + G.block(0,3,3,1));
            robot_pts.push_back(G.block(0,0,3,3)*P3 + G.block(0,3,3,1));

            for(size_t p = 0; p < pts.size(); p++)
            {
                bool is_collision = false;
                for(size_t q = 0; q < robot_pts.size(); q++)
                {
                    Eigen::Vector3d pt = pts[p];
                    Eigen::Vector3d pt0 = robot_pts[q];
                    Eigen::Vector3d pt1 = robot_pts[(q+1) % robot_pts.size()];

                    if (((pt0[1] >= pt[1] && pt1[1] < pt[1]) || (pt0[1] < pt[1] && pt1[1] >= pt[1])) &&
                         (pt[0] < (pt1[0]-pt0[0])*(pt[1]-pt0[1])/(pt1[1]-pt0[1]) + pt0[0]))
                    {
                        is_collision = !is_collision;
                    }
                }

                if(is_collision)
                {
                    // collision
                    return OBS_DETECT_STATIC;
                }
            }
        }
    }

    return OBS_DETECT_NONE;
}

int OBSMAP::is_undock_path_collision_dyn(const std::vector<Eigen::Matrix4d>& robot_tfs_dyn, const std::vector<Eigen::Matrix4d>& robot_tfs_vir, double margin_x, double margin_y, int st_idx, int idx_step)
{
    // get
    mtx.lock();
    std::vector<Eigen::Vector3d> pts_dyn = obs_pts_dynamic;
    std::vector<Eigen::Vector3d> pts_vir = obs_pts_virtual;
    mtx.unlock();

    // draw trajectory
    const double x_min = config->ROBOT_SIZE_X[0] - margin_x;
    const double x_max = config->ROBOT_SIZE_X[1] - 0.1;      // excluding forward charging station detection
    const double y_min = config->ROBOT_SIZE_Y[0] - margin_y;
    const double y_max = config->ROBOT_SIZE_Y[1] + margin_y;

    Eigen::Vector3d P0(x_max, y_max, 0);
    Eigen::Vector3d P1(x_max, y_min, 0);
    Eigen::Vector3d P2(x_min, y_min, 0);
    Eigen::Vector3d P3(x_min, y_max, 0);

    // check collision (dynamic obs)
    for(int i = st_idx; i < (int)robot_tfs_dyn.size(); i++)
    {
        if(i == st_idx || i == (int)robot_tfs_dyn.size()-1 || i%idx_step == 0)
        {
            Eigen::Matrix4d G = robot_tfs_dyn[i];

            std::vector<Eigen::Vector3d> robot_pts;
            robot_pts.push_back(G.block(0,0,3,3)*P0 + G.block(0,3,3,1));
            robot_pts.push_back(G.block(0,0,3,3)*P1 + G.block(0,3,3,1));
            robot_pts.push_back(G.block(0,0,3,3)*P2 + G.block(0,3,3,1));
            robot_pts.push_back(G.block(0,0,3,3)*P3 + G.block(0,3,3,1));

            for(size_t p = 0; p < pts_dyn.size(); p++)
            {
                bool is_collision = false;
                for(size_t q = 0; q < robot_pts.size(); q++)
                {
                    Eigen::Vector3d pt = pts_dyn[p];
                    Eigen::Vector3d pt0 = robot_pts[q];
                    Eigen::Vector3d pt1 = robot_pts[(q+1) % robot_pts.size()];

                    if (((pt0[1] >= pt[1] && pt1[1] < pt[1]) || (pt0[1] < pt[1] && pt1[1] >= pt[1])) &&
                         (pt[0] < (pt1[0]-pt0[0])*(pt[1]-pt0[1])/(pt1[1]-pt0[1]) + pt0[0]))
                    {
                        is_collision = !is_collision;
                    }
                }

                if(is_collision)
                {
                    // collision
                    return OBS_DETECT_DYNAMIC;
                }
            }
        }
    }

    // check collision (virtual obs)
    for(int i = st_idx; i < (int)robot_tfs_vir.size(); i++)
    {
        if(i == st_idx || i == (int)robot_tfs_vir.size()-1 || i%idx_step == 0)
        {
            Eigen::Matrix4d G = robot_tfs_vir[i];

            std::vector<Eigen::Vector3d> robot_pts;
            robot_pts.push_back(G.block(0,0,3,3)*P0 + G.block(0,3,3,1));
            robot_pts.push_back(G.block(0,0,3,3)*P1 + G.block(0,3,3,1));
            robot_pts.push_back(G.block(0,0,3,3)*P2 + G.block(0,3,3,1));
            robot_pts.push_back(G.block(0,0,3,3)*P3 + G.block(0,3,3,1));

            for(size_t p = 0; p < pts_vir.size(); p++)
            {
                bool is_collision = false;
                for(size_t q = 0; q < robot_pts.size(); q++)
                {
                    Eigen::Vector3d pt = pts_vir[p];
                    Eigen::Vector3d pt0 = robot_pts[q];
                    Eigen::Vector3d pt1 = robot_pts[(q+1) % robot_pts.size()];

                    if (((pt0[1] >= pt[1] && pt1[1] < pt[1]) || (pt0[1] < pt[1] && pt1[1] >= pt[1])) &&
                         (pt[0] < (pt1[0]-pt0[0])*(pt[1]-pt0[1])/(pt1[1]-pt0[1]) + pt0[0]))
                    {
                        is_collision = !is_collision;
                    }
                }

                if(is_collision)
                {
                    // collision
                    return OBS_DETECT_DYNAMIC;
                }
            }
        }
    }

    // no collision
    return OBS_DETECT_NONE;
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

    int r = std::ceil(radius/gs);

    std::vector<cv::Vec2i> circle = circle_iterator(uv, r);

    double min_d = radius;
    for(size_t p = 0; p < circle.size(); p++)
    {
        int u = circle[p][0];
        int v = circle[p][1];
        if(u < 0 || u >= w || v < 0 || v >= h)
        {
            continue;
        }

        if(map.ptr<uchar>(v)[u] == 255)
        {
            double dx = (u - uv[0])*gs;
            double dy = (v - uv[1])*gs;
            double d = std::sqrt(dx*dx + dy*dy);
            if(d < min_d)
            {
                min_d = d;
            }
        }
    }

    return saturation(min_d, 0.2, radius);
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

cv::Mat OBSMAP::calc_flowfield(const cv::Mat& map, cv::Vec2i ed)
{
    ed[0] = saturation(ed[0], 0, w-1);
    ed[1] = saturation(ed[1], 0, h-1);

    cv::Mat res(h, w, CV_32F, cv::Scalar(-1));

    std::vector<cv::Vec2i> directions = { {0, 1}, {1, 0}, {0, -1}, {-1, 0} };

    std::queue<cv::Vec2i> q;
    q.push(ed);
    res.ptr<float>(ed[1])[ed[0]] = 0;

    while(!q.empty())
    {
        cv::Vec2i current = q.front();
        q.pop();

        float current_cost = res.ptr<float>(current[1])[current[0]];

        for(const auto& dir : directions)
        {
            cv::Vec2i neighbor = current + dir;
            if (neighbor[0] < 0 || neighbor[0] >= w || neighbor[1] < 0 || neighbor[1] >= h)
            {
                continue;
            }

            if (map.ptr<uchar>(neighbor[1])[neighbor[0]] == 255)
            {
                continue;
            }

            if (res.ptr<float>(neighbor[1])[neighbor[0]] == -1)
            {
                res.ptr<float>(neighbor[1])[neighbor[0]] = current_cost + 1;
                q.push(neighbor);
            }
        }
    }

    return res;
}

std::vector<Eigen::Matrix4d> OBSMAP::calc_path(Eigen::Matrix4d st_tf, Eigen::Matrix4d ed_tf)
{
    // params
    const double margin_x = config->OBS_PATH_MARGIN_X;
    const double margin_y = config->OBS_PATH_MARGIN_Y;

    const double chk_dt = 0.03;
    const double chk_dr = 3.0*D2R;

    const double sampling_dt = 0.05;
    const double sampling_dr = 5.0*D2R;

    // get maps
    mtx.lock();
    Eigen::Matrix4d obs_tf = map_tf;
    Eigen::Matrix4d _obs_tf_inv = obs_tf.inverse();
    cv::Mat obs_map = wall_map.clone();
    mtx.unlock();

    // for debug
    cv::Mat debug_img;
    if(config->USE_SIM == 1)
    {
        cv::cvtColor(obs_map, debug_img, cv::COLOR_GRAY2BGR);
    }

    // st, ed global to local
    Eigen::Matrix4d _st_tf = _obs_tf_inv*st_tf;
    Eigen::Matrix4d _ed_tf = _obs_tf_inv*ed_tf;

    cv::Vec2i st_uv = xy_uv(_st_tf(0,3), _st_tf(1,3));
    cv::Vec2i ed_uv = xy_uv(_ed_tf(0,3), _ed_tf(1,3));

    // calc flowfield
    cv::Mat obs_map2;
    //int robot_head_r = std::ceil((config->ROBOT_SIZE_Y[1]+margin_y)/gs);
    int robot_head_r = (config->ROBOT_SIZE_Y[1]+margin_y)/gs;
    cv::dilate(obs_map, obs_map2, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2*robot_head_r+1, 2*robot_head_r+1)));

    cv::Mat flow_field = calc_flowfield(obs_map2, ed_uv);
    double cost0 = flow_field.ptr<float>(st_uv[1])[st_uv[0]];

    // search algorithm
    std::vector<ASTAR_NODE*> open_set;
    std::vector<std::vector<ASTAR_NODE*>> close_set(w*h); // close set maybe too big

    // set st, ed
    ASTAR_NODE *ed = new ASTAR_NODE();
    ed->tf = _ed_tf;

    ASTAR_NODE *st = new ASTAR_NODE();
    st->tf = _st_tf;
    st->g = 1.0/calc_clearance(obs_map, st->tf, 2.0) * 0.1;
    st->h = cost0*gs;
    st->f = st->g + 5*st->h;
    open_set.push_back(st);

    // search loop    
    int iter = 0;
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
        cv::Vec2i uv0 = xy_uv(cur->tf(0,3), cur->tf(1,3));
        if(uv0[0] < 0 || uv0[0] >= w || uv0[1] < 0 || uv0[1] >= h)
        {
            continue;
        }

        close_set[uv0[1]*w + uv0[0]].push_back(cur);

        // for debug
        if(config->USE_SIM == 1)
        {
            debug_img.ptr<cv::Vec3b>(uv0[1])[uv0[0]] = cv::Vec3b(0,255,0);

            cv::circle(debug_img, cv::Point(st_uv[0], st_uv[1]), 3, cv::Scalar(0,0,255), 2);
            cv::circle(debug_img, cv::Point(ed_uv[0], ed_uv[1]), 3, cv::Scalar(0,255,255), 2);

            cv::Mat debug_map2;
            cv::resize(debug_img, debug_map2, cv::Size(debug_img.cols*2, debug_img.rows*2));
            cv::imshow("calc_path_debug", debug_map2);
        }

        // found goal
        if(cur->h < st->h*0.5)
        {
            Eigen::Matrix4d goal_tf0 = calc_tf(cur->tf.block(0,3,3,1), ed->tf.block(0,3,3,1));
            Eigen::Matrix4d goal_tf1 = goal_tf0;
            goal_tf0.block(0,3,3,1) = cur->tf.block(0,3,3,1);
            goal_tf1.block(0,3,3,1) = ed->tf.block(0,3,3,1);

            std::vector<Eigen::Matrix4d> traj_goal0 = intp_tf(cur->tf, goal_tf0, sampling_dt, sampling_dr);
            std::vector<Eigen::Matrix4d> traj_goal1 = intp_tf(goal_tf0, goal_tf1, sampling_dt, sampling_dr);

            std::vector<Eigen::Matrix4d> traj_goal = traj_goal0;
            traj_goal.insert(traj_goal.end(), traj_goal1.begin(), traj_goal1.end());

            if(!is_collision(obs_map, traj_goal, margin_x, margin_y))
            {
                std::vector<Eigen::Matrix4d> res;

                ASTAR_NODE* _cur = cur;
                while(_cur != NULL)
                {
                    res.push_back(obs_tf*_cur->tf);
                    _cur = _cur->parent;
                }

                std::reverse(res.begin(), res.end());

                // set final pose
                for(size_t p = 0; p < traj_goal.size(); p++)
                {
                    res.push_back(obs_tf*traj_goal[p]);
                }

                printf("[OBSMAP] path_finding complete, num:%d, iter:%d\n", (int)res.size(), iter);
                return res;
            }
        }

        // expand nodes for differential drive        
        double step = gs;
        std::vector<Eigen::Matrix4d> around;
        for(int i = -2; i <= 2; i++)
        {
            for(int j = -2; j <= 2; j++)
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

                // check collision
                cv::Vec2i uv1 = xy_uv(tf1(0,3), tf1(1,3));
                if(uv1[0] < 0 || uv1[0] >= w || uv1[1] < 0 || uv1[1] >= h)
                {
                    continue;
                }

                if(flow_field.ptr<float>(uv1[1])[uv1[0]] < 0)
                {
                    continue;
                }

                // check close set
                bool is_close_set = false;
                for(size_t p = 0; p < close_set[uv1[1]*w + uv1[0]].size(); p++)
                {
                    double dth = calc_dth(close_set[uv1[1]*w + uv1[0]][p]->tf, tf1);
                    if(dth < chk_dr)
                    {
                        is_close_set = true;
                        break;
                    }
                }
                if(is_close_set)
                {
                    continue;
                }

                // check collision                
                std::vector<Eigen::Matrix4d> traj = intp_tf(tf0, tf1, sampling_dt, sampling_dr);
                if(is_collision(obs_map, traj, margin_x, margin_y))
                {
                    continue;
                }

                around.push_back(tf1);
            }
        }

        // calc child node        
        for(size_t p = 0; p < around.size(); p++)
        {
            cv::Vec2i uv = xy_uv(around[p](0,3), around[p](1,3));
            if(uv[0] < 0 || uv[0] >= w || uv[1] < 0 || uv[1] >= h)
            {
                continue;
            }

            double cost1 = flow_field.ptr<float>(uv[1])[uv[0]];
            if(cost1 < 0)
            {
                continue;
            }

            // calc heuristics
            Eigen::Vector2d dtdr1 = dTdR(cur->tf, around[p]);
            double g = cur->g + dtdr1[0]
                              + dtdr1[1] * 0.1
                              + 1.0/calc_clearance(obs_map, around[p], 2.0) * 0.1;

            double h = cost1*gs;
            double f = g + 5*h;

            // check open set
            bool is_open_set = false;
            ASTAR_NODE* open_node = NULL;
            for(size_t q = 0; q < open_set.size(); q++)
            {
                Eigen::Vector2d dtdr = dTdR(open_set[q]->tf, around[p]);
                if(dtdr[0] < chk_dt && dtdr[1] < chk_dr)
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
        if(timeout > 1.5)
        {
            printf("[OBSMAP] timeout, iter:%d\n", iter);
            break;
        }

        iter++;
    }

    std::vector<Eigen::Matrix4d> res;
    return res;
}

// util
cv::Vec2i OBSMAP::xy_uv(double x, double y)
{
    // y axis flip
    int u = std::floor(x/gs) + cx;
    int v = std::floor(-y/gs) + cy;
    return cv::Vec2i(u, v);
}

cv::Vec2d OBSMAP::uv_xy(int u, int v)
{
    double x = (u-cx)*gs;
    double y = -(v-cy)*gs;
    return cv::Vec2d(x, y);
}
