 #include "rp_lidar.h"

 namespace 
{
  const char* MODULE_NAME = "RP_LIDAR";
}

RP_LIDAR* RP_LIDAR::instance(QObject* parent)
{
  static RP_LIDAR* inst = nullptr;
  if(!inst && parent)
  {
    inst = new RP_LIDAR(parent);
  }
  else if(inst && parent && inst->parent() == nullptr)
  {
    inst->setParent(parent);
  }
  return inst;
}

RP_LIDAR::RP_LIDAR(QObject *parent) : QObject{parent}
{

}

RP_LIDAR::~RP_LIDAR()
{
  for(int idx = 0; idx < config->get_lidar_2d_num(); idx++)
  {
    is_connected[idx] = false;
  }

  close();
}

void RP_LIDAR::open()
{
  printf("[RP_LIDAR] open\n");

  // stop first
  close();

  for(int i = 0; i < config->get_lidar_2d_num(); i++)
  {
    pts_tf[i] = string_to_TF(config->get_lidar_2d_tf(i));
  }

  printf("[RP_LIDAR] init\n");

  // loop start
  for(int idx = 0; idx < config->get_lidar_2d_num(); idx++)
  {
    if(grab_thread[idx] == nullptr)
    {
      grab_flag[idx] = true;
      grab_thread[idx] = std::make_unique<std::thread>(&RP_LIDAR::grab_loop, this, idx);
      printf("[RP_LIDAR] start grab loop idx:%d\n", idx);
    }
  }
}

void RP_LIDAR::close()
{
  for(int idx = 0; idx < config->get_lidar_2d_num(); idx++)
  {
    is_connected[idx] = false;

    grab_flag[idx] = false;
    if(grab_thread[idx] && grab_thread[idx]->joinable())
    {
      grab_thread[idx]->join();
    }
    grab_thread[idx].reset();
  }
}

QString RP_LIDAR::get_info_text(int idx)
{
  QString res;
  res += QString("[RP %1]\npts_t: %2 (%3)\n")
    .arg(idx)
    .arg(cur_raw_t[idx].load(), 0, 'f', 3)
    .arg(cur_pts_num[idx].load());
  res += QString("fq: %1,").arg((int)raw_que[idx].unsafe_size());

  return res;
}

RAW_FRAME RP_LIDAR::get_cur_raw(int idx)
{
  std::lock_guard<std::mutex> lock(mtx);
  RAW_FRAME res = cur_raw[idx];
  return res;
}

bool RP_LIDAR::get_is_connected(int idx)
{
  return (bool)is_connected[idx].load();
}

bool RP_LIDAR::get_is_sync(int idx)
{
  return (bool)is_sync[idx].load();
}

void RP_LIDAR::set_is_sync(int idx, bool val)
{
  is_sync[idx].store(val);
}

bool RP_LIDAR::try_pop_raw_que(int idx, RAW_FRAME& frm)
{
  if(raw_que[idx].try_pop(frm))
  {
    return true;
  }
  return false;
}

void RP_LIDAR::sync(int idx)
{
  is_sync[idx] = true;
  printf("[RP_LIDAR] time sync\n");
}

void RP_LIDAR::grab_loop(int idx)
{
  logger->write_log("[RP_LIDAR] start grab loop", "Green", true, false);

  sl::ILidarDriver* drv = *sl::createLidarDriver();
  if(!drv)
  {
    logger->write_log("[RP_LIDAR] driver init failed", "Red", true, false);
    return;
  }
  logger->write_log("[RP_LIDAR] driver init success", "Green", true, false);

  std::string device = config->get_lidar_2d_dev(idx).toStdString();
  if(device == "")
  {
    logger->write_log("[RP_LIDAR] invalid device name", "Red", true, false);
    return;
  }

  sl::IChannel* channel = (*sl::createSerialPortChannel(device, 256000));
  if(!channel->open())
  {
    logger->write_log("[RP_LIDAR] port open failed", "Red", true, false);
    return;
  }
  else
  {
    logger->write_log("[RP_LIDAR] channel init success", "Green", true, false);
    channel->close();
  }

  if(drv->connect(channel) != SL_RESULT_OK)
  {
    logger->write_log("[RP_LIDAR] connection failed", "Red", true, false);
    return;
  }

  logger->write_log("[RP_LIDAR] connect success", "Green", true, false);

  std::vector<sl::LidarScanMode> modes;
  drv->getAllSupportedScanModes(modes);
  if(modes.size() == 0)
  {
    logger->write_log("[RP_LIDAR] no mode failed", "Red", true, false);
    return;
  }

  sl::LidarScanMode mode;
  double per_sample = modes[0].us_per_sample;
  for(size_t p = 0; p < modes.size(); p++)
  {
    printf("%s[%d] us_per_sample:%f\n", modes[p].scan_mode, modes[p].id, modes[p].us_per_sample);
    if(modes[p].us_per_sample < per_sample)
    {
      per_sample = modes[p].us_per_sample;
      mode = modes[p];
    }
  }
  per_sample *= U2S;

  if(drv->setMotorSpeed(DEFAULT_MOTOR_SPEED) != SL_RESULT_OK)
  {
    logger->write_log("[RP_LIDAR] lidar set motor speed failed", "Red", true, false);
    return;
  }

  if(drv->startScanExpress(0, mode.id, 0, &mode) != SL_RESULT_OK)
  {
    logger->write_log("[RP_LIDAR] start scan failed", "Red", true, false);
    return;
  }

  logger->write_log(QString("[RP_LIDAR] lidar scan start, MODE :%1").arg(mode.scan_mode), "Green", true, false);

  is_connected[idx] = true;

  int drop_cnt = 10;
  logger->write_log(QString("[RP_LIDAR] start grab loop"), "Green", true, false);

  Eigen::Matrix4d tf_lidar = ZYX_to_TF(config->get_lidar_2d_tf(idx));

  while(grab_flag[idx])
  {
    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = _countof(nodes);

    sl_u64 timeStamp = 0;
    if(drv->grabScanDataHqWithTimeStamp(nodes, count, timeStamp) == SL_RESULT_OK)
    {
      drv->ascendScanData(nodes, count);

      // initial drop
      if(drop_cnt > 0)
      {
        drop_cnt--;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }

      // time sync
      double pc_t = get_time();
      double lidar_t = (double)timeStamp*U2S;

      if(is_sync[idx])
      {
        is_sync[idx] = false;
        offset_t[idx] = pc_t - lidar_t;

        is_synced[idx] = true;
        QString str = QString("[RP_LIDAR] sync. lidar :%1, lidar_t_f: %2, offset_t_f: %3").arg(idx).arg(lidar_t).arg((double)offset_t[idx]);
        printf("%s\n", str.toLocal8Bit().data());
      }

      // check lidar, mobile sync
      if(!is_synced[idx].load() || !mobile->get_is_synced())
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }

      // lidar frame synced ref time
      double t0 = lidar_t + offset_t[idx];
      double t1 = lidar_t + (count*per_sample) + offset_t[idx];

      // check
      if(mobile->get_pose_storage_size() != MO_STORAGE_NUM)
      {
        // drop
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }

      // wait
      while(t1 > mobile->get_last_pose_t() && grab_flag[idx].load())
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }

      // get mobile poses
      std::vector<MOBILE_POSE> pose_storage = mobile->get_pose_storage();

      // get lidar raw data
      std::vector<double> times;
      std::vector<double> reflects;
      std::vector<Eigen::Vector3d> pts;

      const int step = 1;
      for(size_t p = 0; p < count; p+=step)
      {
        if(nodes[p].dist_mm_q2 == 0)
        {
          continue;
        }

        double t  = t0 + p*per_sample;
        double deg  = (nodes[p].angle_z_q14 * 90.0)/RP_LIDAR_INFO::deg_resolution;
        double dist = (nodes[p].dist_mm_q2 / 4.0)/RP_LIDAR_INFO::dist_resolution;
        double rssi = (double)nodes[p].quality;

        // dist filter
        if(dist < 0.05 || dist > config->get_lidar_2d_max_range())
        {
          continue;
        }

        double x = dist * std::cos(deg*D2R);
        double y = dist * std::sin(deg*D2R);
        if(!isfinite(x) || !isfinite(y))
        {
          continue;
        }

        times.push_back(t);
        reflects.push_back(rssi);
        pts.push_back(Eigen::Vector3d(x, y, 0));
      }

      // check invalid lidar frame
      if(pts.size() < RP_LIDAR_INFO::minimum_lidar_pts)
      {
        // drop
        logger->write_log("[RP_LIDAR] not enough points, drop");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }

      std::vector<Eigen::Vector3d> pts_transformed;
      for(size_t p=0; p<pts.size(); p++)
      {
        Eigen::Vector3d P = tf_lidar.block(0,0,3,3) * pts[p] + tf_lidar.block(0,3,3,1);
        if(P[0] > config->get_robot_size_x_min() && P[0] < config->get_robot_size_x_max() &&
           P[1] > config->get_robot_size_y_min() && P[1] < config->get_robot_size_y_max())
        {
          continue;
        }

        pts_transformed.push_back(P);
      }
      pts.swap(pts_transformed);

      Eigen::Vector3d min_pose = pose_storage.back().pose;
      double min_dt = std::numeric_limits<double>::max();
      for(size_t p = 0; p < pose_storage.size(); p++)
      {
        double dt = std::abs(pose_storage[p].t - t1);
        if(dt < min_dt)
        {
          min_dt = dt;
          min_pose = pose_storage[p].pose;
        }
      }

      cur_pts_num[idx] = (int)pts.size();
      cur_raw_t[idx] = t0;

      MOBILE_POSE mo;
      mo.t = t1;
      mo.pose = min_pose;

      RAW_FRAME frm;
      frm.t0 = t0;
      frm.t1 = t1;
      frm.times = times;
      frm.reflects = reflects;
      frm.pts = pts;
      frm.mo = mo;
      raw_que[idx].push(frm);

      {
        std::lock_guard<std::mutex> lock(mtx);
        cur_raw[idx] = frm;
      }

      // que overflow control
      if(raw_que[idx].unsafe_size() > 50)
      {
        RAW_FRAME temp;
        raw_que[idx].try_pop(temp);
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  logger->write_log("[RP_LIDAR] stop grab loop", "Green", true, false);
}

void RP_LIDAR::set_config_module(CONFIG* _config)
{
  config = _config;
}

void RP_LIDAR::set_logger_module(LOGGER* _logger)
{
  logger = _logger;
}

void RP_LIDAR::set_mobile_module(MOBILE *_mobile)
{
  mobile = _mobile;
}
