#include "obsmap.h"

namespace 
{
  const char* MODULE_NAME = "OBSMAP";
}

OBSMAP* OBSMAP::instance(QObject* parent)
{
  static OBSMAP* inst = nullptr;
  if(!inst && parent)
  {
    inst = new OBSMAP(parent);
  }
  else if(inst && parent && inst->parent() == nullptr)
  {
    inst->setParent(parent);
  }
  return inst;
}

OBSMAP::OBSMAP(QObject *parent) : QObject{parent},
  config(nullptr),
  logger(nullptr),
  unimap(nullptr)
{
  current_snapshot = std::make_shared<ObsMapSnapshot>(h, w);
  // map_tf.setIdentity();

  // wall_map = cv::Mat(h, w, CV_8U, cv::Scalar(0));
  // static_map = cv::Mat(h, w, CV_8U, cv::Scalar(0));
  // dynamic_map = cv::Mat(h, w, CV_8U, cv::Scalar(0));
  // virtual_map = cv::Mat(h, w, CV_8U, cv::Scalar(0));
}

OBSMAP::~OBSMAP()
{
  std::lock_guard<std::mutex> oct_lock(octree_mtx);
  if(octree)
  {
    octree->clear();
    octree.reset();
  }
}

void OBSMAP::init()
{
  // init
  // map_tf.setIdentity();

  reset_obs_box();

  double obsmap_grid_size = config->get_obs_map_grid_size();
  if(obsmap_grid_size == 0)
  {
    //printf("[OBSMAP] Critical error occured...\n");
    spdlog::error("[OBSMAP] Critical error occured...");
    return;
  }

  double obsmap_range = config->get_obs_map_range();

  w = (2*obsmap_range)/obsmap_grid_size;
  h = (2*obsmap_range)/obsmap_grid_size;
  cx = w/2;
  cy = h/2;
  gs = obsmap_grid_size;

  // wall_map = cv::Mat(h, w, CV_8U, cv::Scalar(0));
  // static_map = cv::Mat(h, w, CV_8U, cv::Scalar(0));
  // dynamic_map = cv::Mat(h, w, CV_8U, cv::Scalar(0));
  // virtual_map = cv::Mat(h, w, CV_8U, cv::Scalar(0));
  // 새 스냅샷으로 초기화
  auto new_snapshot = std::make_shared<ObsMapSnapshot>(h, w);

  {
    std::unique_lock<std::shared_mutex> lock(mtx);
    current_snapshot = new_snapshot;
  }

  {
    std::lock_guard<std::mutex> oct_lock(octree_mtx);
    octree = std::make_unique<octomap::OcTree>(obsmap_grid_size);
    octree->setProbHit(P_HIT);
    octree->setProbMiss(P_MISS);
  }

  // octree = std::make_unique<octomap::OcTree>(obsmap_grid_size);
  // octree->setProbHit(P_HIT);
  // octree->setProbMiss(P_MISS);
}

void OBSMAP::clear()
{
  
  // std::unique_lock<std::shared_mutex> lock(mtx);
  {
    std::lock_guard<std::mutex> oct_lock(octree_mtx);
    if(octree)
    {
      octree->clear();
      octree.reset();
    }
    octree = std::make_unique<octomap::OcTree>(config->get_obs_map_grid_size());
    octree->setProbHit(P_HIT);
    octree->setProbMiss(P_MISS);
  }

  auto new_snapshot = std::make_shared<ObsMapSnapshot>(h, w);
  {
    std::unique_lock<std::shared_mutex> lock(mtx);
    current_snapshot = new_snapshot;
    vobs_list_robots.clear();
    vobs_list_closures.clear();
  }

  

  // if(octree)
  // {
  //     octree->clear();
  //     octree.reset();
  // }

  // octree = std::make_unique<octomap::OcTree>(config->get_obs_map_grid_size());
  // octree->setProbHit(P_HIT);
  // octree->setProbMiss(P_MISS);

  // obs_pts.clear();
  // dyn_pts.clear();
  // vir_pts.clear();
  // vir_closure_pts.clear();
  // plot_pts.clear();

  // vobs_list_robots.clear();
  // vobs_list_closures.clear();

  // wall_map = cv::Mat(h, w, CV_8U, cv::Scalar(0));
  // static_map = cv::Mat(h, w, CV_8U, cv::Scalar(0));
  // dynamic_map = cv::Mat(h, w, CV_8U, cv::Scalar(0));
  // virtual_map = cv::Mat(h, w, CV_8U, cv::Scalar(0));
  // map_tf = Eigen::Matrix4d::Identity();

  reset_obs_box();
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

void OBSMAP::set_config_module(CONFIG* _config)
{
  config = _config;
}

void OBSMAP::set_logger_module(LOGGER* _logger)
{
  logger = _logger;
}

void OBSMAP::set_unimap_module(UNIMAP* _unimap)
{
  unimap = _unimap;
}

void OBSMAP::set_obs_box_xy(double min_x, double max_x, double min_y, double max_y)
{
  obs_box_min_x.store(min_x);
  obs_box_max_x.store(max_x);
  obs_box_min_y.store(min_y);
  obs_box_max_y.store(max_y);
}

void OBSMAP::set_obs_box_z(double min_z, double max_z)
{
  obs_box_min_z.store(min_z);
  obs_box_max_z.store(max_z);
}

void OBSMAP::set_obs_box_range(double range)
{
  obs_box_range.store(range);
}

void OBSMAP::reset_obs_box()
{
  obs_box_min_x.store(config->get_robot_size_x_min());
  obs_box_max_x.store(config->get_robot_size_x_max());
  obs_box_min_y.store(config->get_robot_size_y_min());
  obs_box_max_y.store(config->get_robot_size_y_max());
  obs_box_min_z.store(config->get_robot_size_z_min());
  obs_box_max_z.store(config->get_robot_size_z_max());
  obs_box_range.store(config->get_obs_map_range());
}

double OBSMAP::get_obs_box_min_x()
{
   return obs_box_min_x.load();
}

double OBSMAP::get_obs_box_max_x()
{
  return obs_box_max_x.load();
}

double OBSMAP::get_obs_box_min_y()
{
   return obs_box_min_y.load();
}

double OBSMAP::get_obs_box_max_y()
{
  return obs_box_max_y.load();
}

double OBSMAP::get_obs_box_min_z()
{
   return obs_box_min_z.load();
}

double OBSMAP::get_obs_box_max_z()
{
  return obs_box_max_z.load();
}

double OBSMAP::get_obs_box_range()
{
  return obs_box_range.load();
}

void OBSMAP::set_vobs_list_robots(const std::vector<Eigen::Vector3d>& _vobs_r_list)
{
  std::unique_lock<std::shared_mutex> lock(mtx);
  vobs_list_robots = _vobs_r_list;
}

void OBSMAP::set_vobs_list_closures(const std::vector<Eigen::Vector3d>& _vobs_c_list)
{
  std::unique_lock<std::shared_mutex> lock(mtx);
  vobs_list_closures = _vobs_c_list;
}

std::vector<Eigen::Vector3d> OBSMAP::get_vobs_list_robots()
{
  std::shared_lock<std::shared_mutex> lock(mtx);
  return vobs_list_robots;
}

std::vector<Eigen::Vector3d> OBSMAP::get_vobs_list_closures()
{
  std::shared_lock<std::shared_mutex> lock(mtx);
  return vobs_list_closures;
}

std::shared_ptr<ObsMapSnapshot> OBSMAP::get_snapshot()
{
  std::shared_lock<std::shared_mutex> lock(mtx);
  return current_snapshot;
}

void OBSMAP::update_obs_map(TIME_POSE_PTS& tpp)
{
  // std::unique_lock<std::shared_mutex> lock(mtx);
  double start_time = get_time();
  
  Eigen::Matrix4d cur_tf = tpp.tf;
  Eigen::Matrix4d cur_tf_inv = cur_tf.inverse();

  double step = 0.25*D2R;
  double lidar_max_range = (config->get_loc_mode() == "2D") ? config->get_lidar_2d_max_range() : config->get_lidar_3d_max_range();
  size_t range_data_size = (size_t)((2*M_PI)/step);
  
  std::vector<double> range_data(range_data_size, lidar_max_range);
  
  const double robot_x_min = config->get_robot_size_x_min();
  const double robot_x_max = config->get_robot_size_x_max();
  const double robot_y_min = config->get_robot_size_y_min();
  const double robot_y_max = config->get_robot_size_y_max();
  const double robot_z_min = config->get_robot_size_z_min();
  const double robot_z_max = config->get_robot_size_z_max();
  // const double robot_add_x = config->get_robot_size_add_x();
  // const double robot_add_y = config->get_robot_size_add_y();
  
  // const double total_x_min = robot_x_min - robot_add_x;
  // const double total_x_max = robot_x_max + robot_add_x;
  // const double total_y_min = robot_y_min - robot_add_y;
  // const double total_y_max = robot_y_max + robot_add_y;
  
  size_t pts_size = tpp.pts.size();
  for(size_t p = 0; p < pts_size; p++)
  {
    Eigen::Vector3d pt = tpp.pts[p];
    double z = pt[2];
    if(z < config->get_obs_map_min_z() || z > config->get_obs_map_max_z())
    {
      continue;
    }

    double x = pt[0];
    double y = pt[1];

    // self collision filter
    if(check_self_collision(x, y, z, robot_x_min, robot_x_max, robot_y_min, robot_y_max, robot_z_min, robot_z_max))
    {
      continue;
    }
    
    double th = std::atan2(y, x) + M_PI;
    int idx = (int)(th/step);
    if(idx >= 0 && idx < (int)range_data_size)
    {
      double d = std::sqrt(x*x + y*y);
      if(d < range_data[idx])
      {
        range_data[idx] = d;
      }
    }
  }

  // 2D Grid based obstacle map (replaced OctoMap for performance)
  const double obsmap_range = config->get_obs_map_range();

  std::vector<Eigen::Vector4d> global_obs_pts;
  std::vector<Eigen::Vector2d> local_obs_pts;

  // directly write to wall_map from range data - O(N)
  cv::Mat _wall_map(h, w, CV_8U, cv::Scalar(0));
  for(size_t p = 0; p < range_data_size; p++)
  {
    double d = range_data[p];
    if(d >= lidar_max_range)
    {
      continue;  // no obstacle detected in this direction
    }

    double th = p*step - M_PI;
    double x = d*std::cos(th);
    double y = d*std::sin(th);

    // for plot (local coords)
    local_obs_pts.push_back(Eigen::Vector2d(x, y));

    // local to global for plot_pts
    Eigen::Vector3d P(x, y, 0);
    Eigen::Vector3d _P = cur_tf.block(0,0,3,3)*P + cur_tf.block(0,3,3,1);
    global_obs_pts.push_back(Eigen::Vector4d(_P[0], _P[1], cur_tf(2,3), 1.0));

    // write directly to grid
    cv::Vec2i uv = xy_uv(x, y);
    int u = uv[0];
    int v = uv[1];
    if(u >= 0 && u < w && v >= 0 && v < h)
    {
      _wall_map.ptr<uchar>(v)[u] = 255;
    }
  }

  // add static map
  double query_pt[3] = {cur_tf(0,3), cur_tf(1,3), cur_tf(2,3)};
  double sq_radius = obsmap_range*obsmap_range;
  std::vector<nanoflann::ResultItem<unsigned int, double>> res_idxs;
  nanoflann::SearchParameters params;
  unimap->radius_search_kdtree_idx(&query_pt[0], sq_radius, res_idxs, params);

  cv::Mat _static_map(h, w, CV_8U, cv::Scalar(0));
  std::shared_ptr<XYZR_CLOUD> kdtree_cloud = unimap->get_kdtree_cloud();
  for(size_t p = 0; p < res_idxs.size(); p++)
  {
    int idx = res_idxs[p].first;
    if(idx < 0 || idx >= (int)kdtree_cloud->pts.size())
    {
      continue;
    }

    double x = kdtree_cloud->pts[idx].x;
    double y = kdtree_cloud->pts[idx].y;
    double z = kdtree_cloud->pts[idx].z;

    // global to local
    Eigen::Vector3d P(x,y,z);
    Eigen::Vector3d _P = cur_tf_inv.block(0,0,3,3)*P + cur_tf_inv.block(0,3,3,1);
    //if(_P[2] < obs_map_min_z * 0.5 || _P[2] > obs_map_max_z)
    if(_P[2] < config->get_obs_map_min_z() || _P[2] > config->get_obs_map_max_z())
    {
      continue;
    }

    // self collision filter
    if(check_self_collision(_P[0], _P[1], _P[2], robot_x_min, robot_x_max, robot_y_min, robot_y_max, robot_z_min, robot_z_max))
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
      if(node != nullptr)
      {
        Eigen::Matrix4d tf = node->tf;

        Eigen::Vector3d P0( node->size[0]/2,  node->size[1]/2, -node->size[2]/2);
        Eigen::Vector3d P1( node->size[0]/2, -node->size[1]/2,  node->size[2]/2); // for z range check
        Eigen::Vector3d P2(-node->size[0]/2, -node->size[1]/2, -node->size[2]/2);
        Eigen::Vector3d P3(-node->size[0]/2,  node->size[1]/2, -node->size[2]/2);

        Eigen::Vector3d _P0 = tf.block(0,0,3,3)*P0 + tf.block(0,3,3,1);
        Eigen::Vector3d _P1 = tf.block(0,0,3,3)*P1 + tf.block(0,3,3,1);
        Eigen::Vector3d _P2 = tf.block(0,0,3,3)*P2 + tf.block(0,3,3,1);
        Eigen::Vector3d _P3 = tf.block(0,0,3,3)*P3 + tf.block(0,3,3,1);

        Eigen::Vector3d l_P0 = cur_tf_inv.block(0,0,3,3)*_P0 + cur_tf_inv.block(0,3,3,1);
        Eigen::Vector3d l_P1 = cur_tf_inv.block(0,0,3,3)*_P1 + cur_tf_inv.block(0,3,3,1);
        Eigen::Vector3d l_P2 = cur_tf_inv.block(0,0,3,3)*_P2 + cur_tf_inv.block(0,3,3,1);
        Eigen::Vector3d l_P3 = cur_tf_inv.block(0,0,3,3)*_P3 + cur_tf_inv.block(0,3,3,1);

        // check z overlap
        if(l_P1[2] < config->get_obs_map_min_z() || l_P0[2] > config->get_obs_map_max_z())
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

  // add sensing boundary
  {
    cv::Vec2i uv = xy_uv(0, 0);
    int r = config->get_obs_map_range()/gs;

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
      if(contours[p].size() >= 3)
      {
        cv::drawContours(_dynamic_map2, contours, p, cv::Scalar(255), cv::FILLED, 8);
      }
    }
    _dynamic_map = _dynamic_map2;
  }

  // single point filtering
  {
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    
    cv::Mat wall_eroded, static_eroded, dynamic_eroded;
    cv::Mat wall_dilated, static_dilated, dynamic_dilated;
    
    cv::erode(_wall_map, wall_eroded, kernel);
    cv::erode(_static_map, static_eroded, kernel);
    cv::erode(_dynamic_map, dynamic_eroded, kernel);
    
    cv::dilate(_wall_map, wall_dilated, kernel);
    cv::dilate(_static_map, static_dilated, kernel);
    cv::dilate(_dynamic_map, dynamic_dilated, kernel);
    
    cv::Mat wall_single_mask, static_single_mask, dynamic_single_mask;
    cv::compare(wall_eroded, wall_dilated, wall_single_mask, cv::CMP_EQ);
    cv::compare(static_eroded, static_dilated, static_single_mask, cv::CMP_EQ);
    cv::compare(dynamic_eroded, dynamic_dilated, dynamic_single_mask, cv::CMP_EQ);
    
    cv::Mat wall_single_pixels, static_single_pixels, dynamic_single_pixels;
    cv::bitwise_and(_wall_map, wall_single_mask, wall_single_pixels);
    cv::bitwise_and(_static_map, static_single_mask, static_single_pixels);
    cv::bitwise_and(_dynamic_map, dynamic_single_mask, dynamic_single_pixels);
    
    cv::subtract(_wall_map, wall_single_pixels, _wall_map);
    cv::subtract(_static_map, static_single_pixels, _static_map);
    cv::subtract(_dynamic_map, dynamic_single_pixels, _dynamic_map);
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
  // obs_pts = _obs_pts;
  // dyn_pts = _dyn_pts;
  // plot_pts = _plot_pts;
  // map_tf = cur_tf;
  // wall_map = _wall_map;
  // static_map = _static_map;
  // dynamic_map = _dynamic_map;

  auto new_snapshot = std::make_shared<ObsMapSnapshot>();
  new_snapshot->map_tf = cur_tf;
  new_snapshot->wall_map = std::move(_wall_map);
  new_snapshot->static_map = std::move(_static_map);
  new_snapshot->dynamic_map = std::move(_dynamic_map);
  new_snapshot->obs_pts = std::move(_obs_pts);
  new_snapshot->dyn_pts = std::move(_dyn_pts);
  new_snapshot->plot_pts = std::move(_plot_pts);

  // Atomic swap - 기존 virtual 데이터 보존
  {
    std::unique_lock<std::shared_mutex> lock(mtx);
    // 기존 스냅샷에서 virtual 관련 데이터 복사
    new_snapshot->vir_pts = current_snapshot->vir_pts;
    new_snapshot->vir_closure_pts = current_snapshot->vir_closure_pts;
    new_snapshot->virtual_map = current_snapshot->virtual_map.clone();
    current_snapshot = new_snapshot;
  }

  // signal for redrawing
  Q_EMIT obs_updated();

  double end_time = get_time();
  last_obs_update_time = end_time - start_time;
  spdlog::debug("[{}] update_obs_map completed in {:.2f}ms", MODULE_NAME, (end_time - start_time) * 1000);
}

void OBSMAP::update_obs_map_sim(Eigen::Matrix4d tf)
{
  // Simulation용 - 동일한 최적화 패턴 적용
  Eigen::Matrix4d cur_tf = tf;
  Eigen::Matrix4d cur_tf_inv = cur_tf.inverse();

  double query_pt[3] = {cur_tf(0,3), cur_tf(1,3), cur_tf(2,3)};
  double sq_radius0 = config->get_obs_map_range();
  double sq_radius = sq_radius0*sq_radius0;

  double obs_map_min_z = config->get_obs_map_min_z() + obs_box_min_z.load();
  double obs_map_max_z = config->get_obs_map_max_z() + obs_box_max_z.load();

  std::vector<nanoflann::ResultItem<unsigned int, double>> res_idxs;
  nanoflann::SearchParameters params;
  unimap->radius_search_kdtree_idx(&query_pt[0], sq_radius, res_idxs, params);

  std::shared_ptr<XYZR_CLOUD> kdtree_cloud = unimap->get_kdtree_cloud();

  cv::Mat _wall_map(h, w, CV_8U, cv::Scalar(0));
  cv::Mat _static_map(h, w, CV_8U, cv::Scalar(0));
  std::vector<Eigen::Vector3d> _obs_pts;
  std::vector<Eigen::Vector3d> _vir_pts;

  for(size_t p = 0; p < res_idxs.size(); p++)
  {
    int idx = res_idxs[p].first;
    if(idx < 0 || idx >= (int)kdtree_cloud->pts.size())
    {
      continue;
    }

    double x = kdtree_cloud->pts[idx].x;
    double y = kdtree_cloud->pts[idx].y;
    double z = kdtree_cloud->pts[idx].z;

    Eigen::Vector3d P(x,y,z);
    Eigen::Vector3d _P = cur_tf_inv.block(0,0,3,3)*P + cur_tf_inv.block(0,3,3,1);
    if(_P[2] < obs_map_min_z || _P[2] > obs_map_max_z)
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
    _obs_pts.push_back(P);
  }

  // Virtual obs nodes
  std::vector<QString> obs_nodes = unimap->get_nodes("OBS");
  for(size_t i = 0; i < obs_nodes.size(); i++)
  {
    NODE* node = unimap->get_node_by_id(obs_nodes[i]);
    if(node == nullptr) continue;

    Eigen::Matrix4d tf_node = node->tf;
    double sx = node->size[0];
    double sy = node->size[1];
    double sz = node->size[2];
    double step = 0.1;

    for(double x = -sx/2.0; x <= sx/2.0; x += step)
    {
      for(double y = -sy/2.0; y <= sy/2.0; y += step)
      {
        for(double z = -sz/2.0; z <= sz/2.0; z += step)
        {
          Eigen::Vector3d P = tf_node.block(0,0,3,3)*Eigen::Vector3d(x, y, z) + tf_node.block(0,3,3,1);
          Eigen::Vector3d _P = cur_tf_inv.block(0,0,3,3)*P + cur_tf_inv.block(0,3,3,1);
          if(_P[2] < obs_map_min_z || _P[2] > obs_map_max_z)
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
          _vir_pts.push_back(P);
        }
      }
    }
  }

  // 새 스냅샷으로 교체
  auto new_snapshot = std::make_shared<ObsMapSnapshot>();
  new_snapshot->map_tf = cur_tf;
  new_snapshot->wall_map = std::move(_wall_map);
  new_snapshot->static_map = std::move(_static_map);
  new_snapshot->obs_pts = std::move(_obs_pts);
  new_snapshot->vir_pts = std::move(_vir_pts);

  {
    std::unique_lock<std::shared_mutex> lock(mtx);
    // 기존 스냅샷에서 다른 데이터 보존
    new_snapshot->dynamic_map = current_snapshot->dynamic_map.clone();
    new_snapshot->virtual_map = current_snapshot->virtual_map.clone();
    new_snapshot->dyn_pts = current_snapshot->dyn_pts;
    new_snapshot->plot_pts = current_snapshot->plot_pts;
    new_snapshot->vir_closure_pts = current_snapshot->vir_closure_pts;
    current_snapshot = new_snapshot;
  }

  Q_EMIT obs_updated();
}

void OBSMAP::update_vobs_map()
{
  double start_time = get_time();
  // Eigen::Matrix4d cur_tf, cur_tf_inv;
  // {
  //     std::shared_lock<std::shared_mutex> lock(mtx);
  //     cur_tf = map_tf;
  // }
  // cur_tf_inv = cur_tf.inverse();

  Eigen::Matrix4d cur_tf;
  std::vector<Eigen::Vector3d> vobs_list_robots_copy, vobs_list_closures_copy;
  {
    std::shared_lock<std::shared_mutex> lock(mtx);
    cur_tf = current_snapshot->map_tf;
    vobs_list_robots_copy = vobs_list_robots;
    vobs_list_closures_copy = vobs_list_closures;
  }
  Eigen::Matrix4d cur_tf_inv = cur_tf.inverse();

  size_t total_robots_size = vobs_list_robots_copy.size();
  size_t total_closures_size = vobs_list_closures_copy.size();
  
  double y_max = config->get_robot_size_y_max();
  size_t estimated_pts_per_robot = static_cast<size_t>(M_PI * y_max * y_max * 4);
  
  std::vector<Eigen::Vector3d> _vir_pts;
  std::vector<Eigen::Vector3d> _vir_closure_pts;
  _vir_pts.reserve(total_robots_size * estimated_pts_per_robot);
  _vir_closure_pts.reserve(total_closures_size * estimated_pts_per_robot);
  
  cv::Mat _virtual_map(h, w, CV_8U, cv::Scalar(0));

  auto process_vobs = [&](const std::vector<Eigen::Vector3d>& list, bool is_closure)
  {
    if(list.empty()) return;
    
    const int num_threads = 3;
    std::vector<std::vector<Eigen::Vector3d>> local_pts(num_threads);
    std::vector<std::vector<Eigen::Vector3d>> local_closure_pts(num_threads);
    std::vector<std::vector<cv::Point>> local_pixels(num_threads);
    
    size_t pts_per_thread = list.size() / num_threads + 1;
    for(int tid = 0; tid < num_threads; ++tid)
    {
      local_pts[tid].reserve(pts_per_thread * estimated_pts_per_robot);
      if(is_closure)
      {
        local_closure_pts[tid].reserve(pts_per_thread * estimated_pts_per_robot);
      }
      local_pixels[tid].reserve(pts_per_thread * estimated_pts_per_robot);
    }

    const Eigen::Matrix3d R_inv = cur_tf_inv.block<3,3>(0,0);
    const Eigen::Vector3d t_inv = cur_tf_inv.block<3,1>(0,3);

    #pragma omp parallel for num_threads(num_threads) schedule(dynamic, 1)
    for(int i = 0; i < static_cast<int>(list.size()); i++)
    {
      int tid = omp_get_thread_num();
      if(tid >= 0 && tid < num_threads)
      {
        const Eigen::Vector3d& center = list[i];

        auto pts = circle_iterator_3d(center, y_max);
        pts.push_back(center);

        local_pts[tid].reserve(local_pts[tid].size() + pts.size());
        if(is_closure)
        {
          local_closure_pts[tid].reserve(local_closure_pts[tid].size() + pts.size());
        }

        for(const auto& pt : pts)
        {
          local_pts[tid].push_back(pt);

          if(is_closure)
          {
            local_closure_pts[tid].push_back(pt);
          }

          const Eigen::Vector3d _P = R_inv * pt + t_inv;
          const cv::Vec2i uv = xy_uv(_P[0], _P[1]);
          if(uv[0] >= 0 && uv[0] < w && uv[1] >= 0 && uv[1] < h)
          {
            local_pixels[tid].push_back(uv);
          }
        }
      }
    }

    // thread-local → main vector merge - move semantics 사용
    for(int tid = 0; tid < num_threads; ++tid)
    {
      _vir_pts.insert(_vir_pts.end(), 
              std::make_move_iterator(local_pts[tid].begin()), 
              std::make_move_iterator(local_pts[tid].end()));

      if(is_closure)
      {
        _vir_closure_pts.insert(_vir_closure_pts.end(), 
                    std::make_move_iterator(local_closure_pts[tid].begin()), 
                    std::make_move_iterator(local_closure_pts[tid].end()));
      }

      for(const auto& px : local_pixels[tid])
      {
        _virtual_map.ptr<uchar>(px.y)[px.x] = 255;
      }
    }
  };

  if(!vobs_list_robots_copy.empty())
  {
    process_vobs(vobs_list_robots_copy, false);
  }
  
  if(!vobs_list_closures_copy.empty())
  {
    process_vobs(vobs_list_closures_copy, true);
  }

  // {
  //     std::unique_lock<std::shared_mutex> lock(mtx);
  //     vir_pts = std::move(_vir_pts);
  //     vir_closure_pts = std::move(_vir_closure_pts);
  //     virtual_map = std::move(_virtual_map);
  // }
  {
    std::unique_lock<std::shared_mutex> lock(mtx);
    // 기존 스냅샷 복사 후 virtual 데이터만 업데이트
    auto new_snapshot = std::make_shared<ObsMapSnapshot>(*current_snapshot);
    new_snapshot->vir_pts = std::move(_vir_pts);
    new_snapshot->vir_closure_pts = std::move(_vir_closure_pts);
    new_snapshot->virtual_map = std::move(_virtual_map);
    current_snapshot = new_snapshot;
  }

  // signal for redrawing
  Q_EMIT obs_updated();

  double end_time = get_time();
  last_vobs_update_time = end_time - start_time;
}

void OBSMAP::update_vobs_list_robots(const std::vector<Eigen::Vector3d>& vobs_r)
{
  std::unique_lock<std::shared_mutex> lock(mtx);
  vobs_list_robots = vobs_r;
}

void OBSMAP::update_vobs_list_closures(const std::vector<Eigen::Vector3d>& vobs_c)
{
  std::unique_lock<std::shared_mutex> lock(mtx);
  vobs_list_closures = vobs_c;
}

// 
void OBSMAP::get_obs_map(cv::Mat& map, Eigen::Matrix4d& tf)
{
  auto snapshot = get_snapshot();
  map = snapshot->wall_map.clone();
  tf = snapshot->map_tf;
}

void OBSMAP::get_dyn_map(cv::Mat& map, Eigen::Matrix4d& tf)
{
  auto snapshot = get_snapshot();
  map = snapshot->dynamic_map.clone();
  tf = snapshot->map_tf;
}

void OBSMAP::get_vir_map(cv::Mat& map, Eigen::Matrix4d& tf)
{
  auto snapshot = get_snapshot();
  map = snapshot->virtual_map.clone();
  tf = snapshot->map_tf;
}

double OBSMAP::get_last_obs_update_time()
{
  return last_obs_update_time.load();
}

double OBSMAP::get_last_vobs_update_time()
{
  return last_vobs_update_time.load();
}

cv::Mat OBSMAP::get_obs_map()
{
  return get_snapshot()->wall_map.clone();
}

cv::Mat OBSMAP::get_dyn_map()
{
  return get_snapshot()->dynamic_map.clone();
}

cv::Mat OBSMAP::get_static_map()
{
  return get_snapshot()->static_map.clone();
}

cv::Mat OBSMAP::get_vir_map()
{
  return get_snapshot()->virtual_map.clone();
}

Eigen::Matrix4d OBSMAP::get_map_tf()
{
  return get_snapshot()->map_tf;
}

std::vector<Eigen::Vector3d> OBSMAP::get_obs_pts()
{
  return get_snapshot()->obs_pts;
}

std::vector<Eigen::Vector3d> OBSMAP::get_dyn_pts()
{
  return get_snapshot()->dyn_pts;
}

std::vector<Eigen::Vector3d> OBSMAP::get_vir_pts()
{
  return get_snapshot()->vir_pts;
}

std::vector<Eigen::Vector3d> OBSMAP::get_vir_closure_pts()
{
  return get_snapshot()->vir_closure_pts;
}

std::vector<Eigen::Vector4d> OBSMAP::get_plot_pts()
{
  return get_snapshot()->plot_pts;
}

void OBSMAP::draw_robot(cv::Mat& img)
{
  // draw rect
  double x_min = config->get_robot_size_x_min();
  double x_max = config->get_robot_size_x_max();
  double y_min = config->get_robot_size_y_min();
  double y_max = config->get_robot_size_y_max();

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

void OBSMAP::draw_robot_outline(cv::Mat& img)
{
  // draw rect
  // double x_min = config->get_robot_size_x_min();
  // double x_max = config->get_robot_size_x_max();
  // double y_min = config->get_robot_size_y_min();
  // double y_max = config->get_robot_size_y_max();

  double x_min = obs_box_min_x.load();
  double x_max = obs_box_max_x.load();
  double y_min = obs_box_min_y.load();
  double y_max = obs_box_max_y.load();

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

  cv::polylines(img, pts, true, cv::Scalar(0, 140, 255), 1, cv::LINE_AA);

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

int OBSMAP::is_tf_collision(const Eigen::Matrix4d& robot_tf, bool is_dyn, double margin_x, double margin_y)
{
  // get
  std::vector<Eigen::Vector3d> pts;
  std::vector<Eigen::Vector3d> pts_vir;
  // {
  //     std::shared_lock<std::shared_mutex> lock(mtx);
  //     if(is_dyn)
  //     {
  //       pts = dyn_pts;
  //       pts_vir = vir_pts;
  //     }
  //     else
  //     {
  //       pts = obs_pts;
  //     }
  // }
  {
    auto snapshot = get_snapshot();
    if(is_dyn)
    {
      pts = snapshot->dyn_pts;
      pts_vir = snapshot->vir_pts;
    }
    else
    {
      pts = snapshot->obs_pts;
    }
  }

  // robot pts
  // double x_min = config->get_robot_size_x_min() - margin_x;
  // double x_max = config->get_robot_size_x_max() + margin_x;
  // double y_min = config->get_robot_size_y_min() - margin_y;
  // double y_max = config->get_robot_size_y_max() + margin_y;

  double x_min = obs_box_min_x.load() - margin_x;
  double x_max = obs_box_max_x.load() + margin_x;
  double y_min = obs_box_min_y.load() - margin_y;
  double y_max = obs_box_max_y.load() + margin_y;

  Eigen::Vector3d P0(x_max, y_max, 0);
  Eigen::Vector3d P1(x_max, y_min, 0);
  Eigen::Vector3d P2(x_min, y_min, 0);
  Eigen::Vector3d P3(x_min, y_max, 0);

  std::vector<Eigen::Vector3d> robot_pts;
  robot_pts.push_back(robot_tf.block(0,0,3,3)*P0 + robot_tf.block(0,3,3,1));
  robot_pts.push_back(robot_tf.block(0,0,3,3)*P1 + robot_tf.block(0,3,3,1));
  robot_pts.push_back(robot_tf.block(0,0,3,3)*P2 + robot_tf.block(0,3,3,1));
  robot_pts.push_back(robot_tf.block(0,0,3,3)*P3 + robot_tf.block(0,3,3,1));

  // check collision
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

    if(is_collision)
    {
      // collision
      return OBS_DYN;
    }
  }

  // check multi-robot collision
  if(is_dyn)
  {
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

      if(is_collision)
      {
        // collision
        return OBS_VIR;
      }
    }
  }

  // no collision
  return OBS_NONE;
}

int OBSMAP::is_path_collision(const std::vector<Eigen::Matrix4d>& robot_tfs, bool is_dyn, double margin_x, double margin_y, int st_idx, int idx_step)
{
  // get
  std::vector<Eigen::Vector3d> pts;
  std::vector<Eigen::Vector3d> pts_vir;
  // {
  //     std::shared_lock<std::shared_mutex> lock(mtx);
  //     if(is_dyn)
  //     {
  //       pts = dyn_pts;
  //       pts_vir = vir_pts;
  //     }
  //     else
  //     {
  //       pts = obs_pts;
  //     }
  // }
  {
    auto snapshot = get_snapshot();
    if(is_dyn)
    {
      pts = snapshot->dyn_pts;
      pts_vir = snapshot->vir_pts;
    }
    else
    {
      pts = snapshot->obs_pts;
    }
  }

  // robot pts
  double x_min = config->get_robot_size_x_min() - margin_x;
  double x_max = config->get_robot_size_x_max() + margin_x;
  double y_min = config->get_robot_size_y_min() - margin_y;
  double y_max = config->get_robot_size_y_max() + margin_y;

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

        // collision
        if(is_collision)
        {
          return OBS_DYN;
        }
      }
    }
  }

  // check multi-robot collision
  if(is_dyn)
  {
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

          // collision
          if(is_collision)
          {
            return OBS_VIR;
          }
        }
      }
    }
  }

  // no collision
  return OBS_NONE;
}

// for avoid path
double OBSMAP::calc_clearance(const cv::Mat& map, const Eigen::Matrix4d& robot_tf, double radius)
{
  // in this function, robot_tf should be local tf referenced by map_tf

  // draw rect
  const double x_min = config->get_robot_size_x_min();
  const double x_max = config->get_robot_size_x_max();
  const double y_min = config->get_robot_size_y_min();
  const double y_max = config->get_robot_size_y_max();

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
  double x_min = config->get_robot_size_x_min() - margin_x;
  double x_max = config->get_robot_size_x_max() + margin_x;
  double y_min = config->get_robot_size_y_min() - margin_y;
  double y_max = config->get_robot_size_y_max() + margin_y;

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
  double margin_x = config->get_obs_path_margin_x();
  double margin_y = config->get_obs_path_margin_y();

  const double chk_dt = 0.03;
  const double chk_dr = 3.0*D2R;

  const double sampling_dt = 0.05;
  const double sampling_dr = 5.0*D2R;

  // get maps
  Eigen::Matrix4d obs_tf;
  Eigen::Matrix4d _obs_tf_inv;
  cv::Mat obs_map;
  // {
  //     std::shared_lock<std::shared_mutex> lock(mtx);
  //     obs_tf = map_tf;
  //     _obs_tf_inv = obs_tf.inverse();
  //     obs_map = wall_map.clone();
  // }
  {
    auto snapshot = get_snapshot();
    obs_tf = snapshot->map_tf;
    obs_map = snapshot->wall_map.clone();
  }

  _obs_tf_inv = obs_tf.inverse();

  // for debug
  cv::Mat debug_img;
  if(config->get_use_sim())
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
  int robot_head_r = (config->get_robot_size_y_max()+margin_y)/gs;
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
  
  while(!open_set.empty())
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
      delete cur;
      continue;
    }

    close_set[uv0[1]*w + uv0[0]].push_back(cur);

    // for debug
    if(config->get_use_sim())
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

        //printf("[OBSMAP] path_finding complete, num:%d, iter:%d\n", (int)res.size(), iter);
        spdlog::info("[OBSMAP] path_finding complete, num:{}, iter:{}", (int)res.size(), iter);

        for(auto& row : close_set)
        {
          for(auto* node : row)
          {
            delete node;
          }
        }
        for(auto* node : open_set)
        {
          delete node;
        }
        delete ed;

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
        if(calc_dist_2d(tf1.block(0,3,3,1)) > config->get_obs_map_range())
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
      //printf("[OBSMAP] timeout, iter:%d\n", iter);
      spdlog::warn("[OBSMAP] timeout, iter:{}", iter);
      break;
    }

    iter++;
  }

  for(auto& row : close_set)
  {
    for(auto* node : row)
    {
      delete node;
    }
  }
  for(auto* node : open_set)
  {
    delete node;
  }
  delete ed;

  std::vector<Eigen::Matrix4d> res;
  return res;
}
