#include "cam.h"
namespace 
{
  const char* MODULE_NAME = "CAM";
}


CAM* CAM::instance(QObject* parent)
{
  static CAM* inst = nullptr;
  if(!inst && parent)
  {
    inst = new CAM(parent);
  }
  else if(inst && parent && inst->parent() == nullptr)
  {
    inst->setParent(parent);
  }
  return inst;
}

CAM::CAM(QObject *parent) : QObject{parent},
  config(nullptr),
  logger(nullptr),
  mobile(nullptr),
  orbbec(nullptr)
{
  // init
  for(int p = 0; p < max_cam_cnt; p++)
  {
    is_connected[p] = false;
    post_process_flag[p] = false;
    process_time_post[p] = 0.0;
    cam_t[p] = 0.0;
  }
}

void CAM::close()
{
  log_info("close");
  int cam_num = config->get_cam_num();
  for(int p = 0; p < cam_num; p++)
  {
    is_connected[p] = false;
  }

  if(config->get_cam_type() == "ORBBEC" && orbbec)
  {
    for(int p = 0; p < config->get_cam_num(); p++)
    {
      post_process_flag[p] = false;
      if(post_process_thread[p] && post_process_thread[p]->joinable())
      {
        post_process_thread[p]->join();
      }
      post_process_thread[p].reset();
    }
  }
}

void CAM::restart(int idx)
{
  if(config->get_cam_type() == "ORBBEC" && orbbec)
  {
    orbbec->restart(idx);
  }
}

CAM::~CAM()
{
  int cam_num = config->get_cam_num();
  for(int p = 0; p < cam_num; p++)
  {
    is_connected[p] = false;
  }

  // close cam
  if(orbbec != nullptr)
  {
    orbbec->close();
  }

  close();
}

void CAM::init()
{
  if(config->get_cam_type() == "ORBBEC")
  {
    if(!orbbec)
    {
      orbbec = ORBBEC::instance(this);
      orbbec->set_config_module(this->config);
      orbbec->set_logger_module(this->logger);
      orbbec->set_mobile_module(this->mobile);
      orbbec->init();
      orbbec->open();
    }
  }
}

void CAM::open()
{
  if(config->get_cam_type() == "ORBBEC" && orbbec)
  {
    int cam_num = config->get_cam_num();

    for(int p = 0; p < cam_num; p++)
    {
      post_process_flag[p] = true;
      post_process_thread[p] = std::make_unique<std::thread>(&CAM::post_process_loop, this, p);
    }
  }

  if(config->get_use_rtsp())
  {
    rtsp_flag = true;
    rtsp_thread = std::make_unique<std::thread>(&CAM::rtsp_loop, this);
  }

  //if(!config->get_use_cam() || !config->get_use_cam_depth() || !config->get_use_cam_rgb())
  if(!config->get_use_cam() && !config->get_use_cam_depth() && !config->get_use_cam_rgb())
  {
    log_info("CAM disabled (all flags false) -> skip open");
    return;
  }
  log_info("CAM enabled: USE_CAM={}, USE_CAM_DEPTH={}, USE_CAM_RGB={}",
       config->get_use_cam(),
       config->get_use_cam_depth(),
       config->get_use_cam_rgb());
}

bool CAM::get_connection(int idx)
{
  return is_connected[idx];
}

double CAM::get_process_time_post(int idx)
{
  return (double)process_time_post[idx].load();
}

TIME_IMG CAM::get_time_img(int idx)
{
  std::lock_guard<std::mutex> lock(mtx);
  return cur_time_img[idx];
}

TIME_PTS CAM::get_scan(int idx)
{
  std::lock_guard<std::mutex> lock(mtx);
  return cur_scan[idx];
}

bool CAM::get_is_param_loaded(int idx)
{
  if(config->get_cam_type() == "ORBBEC" && orbbec)
  {
    return orbbec->get_is_param_loaded(idx);
  }
  return false;
}

CAM_INTRINSIC CAM::get_intrinsic(int idx)
{
  if(config->get_cam_type() == "ORBBEC" && orbbec)
  {
    return orbbec->get_intrinsic(idx);
  }
  return CAM_INTRINSIC();
}

Eigen::Matrix4d CAM::get_extrinsic(int idx)
{
  if(config->get_cam_type() == "ORBBEC" && orbbec)
  {
    return orbbec->get_extrinsic(idx);
  }
  return Eigen::Matrix4d::Identity();
}

TIME_PTS CAM::filter_radius_outlier(const TIME_PTS &tp, double radius, int min_neighbors, bool USE_ROR, bool USE_CLUSTER)
{
  double t0 = get_time();

  TIME_PTS res;
  res.t = tp.t;

  if(tp.pts.size() == 0)
  {
    return res;
  }

  // get filter range
  double min_z = config->get_obs_map_min_z();
  double max_z = config->get_obs_map_max_z();
  double max_range = config->get_obs_map_range();

  // keep
  std::vector<Eigen::Vector3d> keep_pts;
  keep_pts.reserve(tp.pts.size());

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->reserve(tp.pts.size());

  // split
  for(size_t i = 0; i < tp.pts.size(); i++)
  {
    const Eigen::Vector3d &p = tp.pts[i];

    // check nan
    if(!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2]))
    {
      continue;
    }

    bool in_z_range = (p[2] >= min_z && p[2] <= max_z);
    bool in_xy_range = (p[0]*p[0] + p[1]*p[1] <= max_range*max_range);

    if(in_z_range && in_xy_range)
    {
      cloud->push_back(pcl::PointXYZ((float)p.x(), (float)p.y(), (float)p.z()));
    }
    else
    {
      keep_pts.push_back(p);
    }
  }
  if(cloud->size() == 0)
  {
    res.pts = keep_pts;
    return res;
  }

  // voxel downsample
  pcl::PointCloud<pcl::PointXYZ>::Ptr ds(new pcl::PointCloud<pcl::PointXYZ>);
  {
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    float leaf = (float)0.03;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(leaf, leaf, leaf);
    voxel_grid.filter(*ds);
  }
  if(ds->size() == 0)
  {
    res.pts = keep_pts;
    return res;
  }

  // radius outlier removal
  pcl::PointCloud<pcl::PointXYZ> cloud_ror;
  if(USE_ROR)
  {
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    ror.setInputCloud(ds);
    ror.setRadiusSearch(radius);
    ror.setMinNeighborsInRadius(min_neighbors);
    ror.filter(cloud_ror);
  }
  else
  {
    cloud_ror = *ds;
  }

  if(cloud_ror.size() == 0)
  {
    res.pts = keep_pts;
    return res;
  }

  // euclidean clustering
  pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
  if(USE_CLUSTER)
  {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_ror.makeShared());

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> extractor;
    extractor.setClusterTolerance((float)(radius * 1.5));
    extractor.setMinClusterSize(config->get_cam_filter_cluster_min_size());
    extractor.setMaxClusterSize(500000);
    extractor.setSearchMethod(tree);
    extractor.setInputCloud(cloud_ror.makeShared());
    extractor.extract(cluster_indices);

    for(size_t ci = 0; ci < cluster_indices.size(); ci++)
    {
      const pcl::PointIndices &inds = cluster_indices[ci];
      for(size_t k = 0; k < inds.indices.size(); k++)
      {
        int idx = inds.indices[k];
        cloud_filtered.push_back(cloud_ror[idx]);
      }
    }
  }
  else
  {
    cloud_filtered = cloud_ror;
  }

  if(cloud_filtered.size() == 0)
  {
    res.pts = keep_pts;
    return res;
  }

  // merge
  res.pts.clear();
  res.pts.reserve(keep_pts.size() + cloud_filtered.size());
  for(size_t i = 0; i < keep_pts.size(); i++)
  {
    res.pts.push_back(keep_pts[i]);
  }

  for(size_t i = 0; i < cloud_filtered.size(); i++)
  {
    const pcl::PointXYZ &p = cloud_filtered[i];
    res.pts.emplace_back((double)p.x, (double)p.y, (double)p.z);
  }

  // debug
  // log_info("(filtering) dt:{}, DS/ROR/CL raw:{}, keep:{}, cloud:{} → ds:{} → ror:{} → filtered:{}, (USE_ROR:{} USE_CLUSTER:{}) (r:{:.2f} nmin:{})",
  //      get_time()-t0,
  //      (int)tp.pts.size(),
  //      (int)keep_pts.size(),
  //      (int)cloud->size(),
  //      (int)ds->size(),
  //      (int)cloud_ror.size(),
  //      (int)cloud_filtered.size(),
  //      (int)USE_ROR, (int)USE_CLUSTER,
  //      radius, min_neighbors);

  return res;
}

void CAM::post_process_loop(int idx)
{
  const double dt = 0.1; // 10hz

  log_info("post_process_loop start");
  while(post_process_flag[idx])
  {
    // =======================
    // ORBBEC queue processing
    // =======================
    double pre_loop_time  = get_time();

    if(config->get_cam_type() == "ORBBEC" && orbbec)
    {
      is_connected[idx].store(orbbec->is_connected[idx].load());
      cam_t[idx].store(orbbec->cam_t[idx].load());

      try
      {
        // use depth only
        if(config->get_use_cam_depth())
        {
          // std::this_thread::sleep_for(std::chrono::milliseconds(100));
          TIME_PTS tp;
          if(orbbec->try_pop_depth_que(idx, tp))
          {
            if(config->get_use_cam_filter())
            {
              TIME_PTS filtered_tp = filter_radius_outlier(tp, config->get_cam_filter_ror_radius(), config->get_cam_filter_ror_min_neighbors(), true, true);
              std::lock_guard<std::mutex> lock(mtx);
              cur_scan[idx] = filtered_tp;
            }
            else
            {
              std::lock_guard<std::mutex> lock(mtx);
              cur_scan[idx] = tp;
            }
          }
        }
        
        // use rgb only
        //if(config->get_use_cam_rgb() || config->get_use_cam())
        if(config->get_use_cam_rgb())
        {
          TIME_IMG ti;
          if(orbbec->try_pop_img_que(idx, ti))
          {
            RobotModel robot_model = config->get_robot_model();
            if(robot_model == RobotModel::S100)
            {
              //log_debug("S100 image flip processing");
              // left camera
              if(idx == 0)
              {
                cv::flip(ti.img, ti.img, -1);
              }
            }

            std::lock_guard<std::mutex> lock(mtx);
            cur_time_img[idx] = ti;
          }
        }
      }
      catch(const ob::Error &e)
      {
        // If an SDK exception occurs, log it and set the connection status to false
        logger->write_log(QString("[ORBBEC] Camera %1 error: %2").arg(idx).arg(e.getMessage()));
        log_error("Camera {} error: {}", idx, e.getMessage());
        is_connected[idx] = false;
      }
    }

    // =======================
    // loop time check, sleep
    // =======================

    double cur_loop_time = get_time();
    double delta_loop_time = cur_loop_time - pre_loop_time;
    if(delta_loop_time < dt)
    {
      int sleep_ms = (dt - delta_loop_time) * 1000;
      std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    }
    else
    {
      // logger->write_log(QString("[CAM] loop time drift, dt:%1").arg(delta_loop_time));
      log_warn("cam_{} loop time drift, dt:{}", idx, delta_loop_time);
    }

    process_time_post[idx] = delta_loop_time;
  }
}

void CAM::rtsp_loop()
{
  const int send_w = 320;
  const int send_h = 160;

  int cam_num = config->get_cam_num();

  std::vector<cv::VideoWriter> writer(cam_num);
  std::vector<std::string> pipeline(cam_num);

  for (int p = 0; p < cam_num; p++)
  {
    std::stringstream ss;
    ss << "appsrc ! queue max-size-buffers=1 leaky=downstream ! "
       << "videoconvert ! video/x-raw,format=I420 ! "
       << "x264enc tune=zerolatency speed-preset=ultrafast bitrate=600 "
       << "key-int-max=30 bframes=0 ! "
       << "video/x-h264,profile=baseline ! "
       << "rtspclientsink location=rtsp://localhost:8554/cam" << p
       << " protocols=udp latency=0";
    pipeline[p] = ss.str();
  }

  rtsp_cam_status.resize(cam_num, false);

  for(int p = 0; p < cam_num; p++)
  {
    bool is_try_open = false;
    if(!writer[p].isOpened())
    {
      is_try_open = writer[p].open(pipeline[p], cv::CAP_GSTREAMER, 10.0, cv::Size(send_w, send_h), true);
      log_info("RTSP cam{} writer open, pipeline: {}", p, pipeline[p]);

    }

    if(!is_try_open)
    {
      logger->write_log(QString("[RTSP] cam%1 rtsp writer open failed").arg(p));
      log_error("[RTSP] cam{} rtsp writer open failed", p);
      rtsp_cam_status[p] = false;
    }
    else
    {
      rtsp_cam_status[p] = true;
    }

  }

  // All camera statuses must be true for rtsp_flag to be true
  rtsp_flag = std::all_of(rtsp_cam_status.begin(), rtsp_cam_status.end(), [](bool status)
  {
    return status;
  });

  while(rtsp_flag)
  {
    for(int p=0; p<config->get_cam_num(); p++)
    {
      if(!is_connected[p])
      {
        continue;
      }

      if(!writer[p].isOpened())
      {
        continue;
      }

      TIME_IMG timg = get_time_img(p);
      cv::Mat img = timg.img.clone();
      if(!img.empty())
      {
        if(img.cols != send_w || img.rows != send_h)
        {
          cv::Mat _img;
          cv::resize(img, _img, cv::Size(send_w, send_h));
          writer[p].write(_img);
        }
        else
        {
          writer[p].write(img);
        }
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}

CAM_INTRINSIC CAM::string_to_intrinsic(QString str)
{
  // w, h, fx, fy, cx, cy, k1, k2, p1, p2
  QStringList str_list = str.split(",");
  if(str_list.size() != 10)
  {
    return CAM_INTRINSIC();
  }

  CAM_INTRINSIC res;
  res.w = str_list[0].toDouble();
  res.h = str_list[1].toDouble();
  res.fx = str_list[2].toDouble();
  res.fy = str_list[3].toDouble();
  res.cx = str_list[4].toDouble();
  res.cy = str_list[5].toDouble();
  res.k1 = str_list[6].toDouble();
  res.k2 = str_list[7].toDouble();
  res.p1 = str_list[8].toDouble();
  res.p2 = str_list[9].toDouble();
  return res;
}

std::vector<bool> CAM::get_rtsp_flag()
{
   std::lock_guard<std::mutex> lock(mtx);
   return rtsp_cam_status;
}

void CAM::set_config_module(CONFIG* _config)
{
  config = _config;
}

void CAM::set_logger_module(LOGGER* _logger)
{
  logger = _logger;
}

void CAM::set_mobile_module(MOBILE *_mobile)
{
  mobile = _mobile;
}
