#include "lidar_3d.h"
namespace 
{
  const char* MODULE_NAME = "LIDAR_3D";
}
LIDAR_3D* LIDAR_3D::instance(QObject* parent)
{
  static LIDAR_3D* inst = nullptr;
  if(!inst && parent)
  {
    inst = new LIDAR_3D(parent);
  }
  else if(inst && parent && inst->parent() == nullptr)
  {
    inst->setParent(parent);
  }
  return inst;
}

LIDAR_3D::LIDAR_3D(QObject *parent) : QObject(parent),
  config(nullptr),
  logger(nullptr),
  mobile(nullptr),
  livox(nullptr)
{
  connect(this, SIGNAL(signal_set_on(std::vector<int>)), this, SLOT(slot_set_on(std::vector<int>)));
  connect(this, SIGNAL(signal_set_off(std::vector<int>)), this, SLOT(slot_set_off(std::vector<int>)));
}

LIDAR_3D::~LIDAR_3D()
{
  is_connected = false;

  // close lidar
  if(livox != nullptr)
  {
    livox->close();
  }

  close();
}

void LIDAR_3D::init()
{
  // check simulation mode
  if(config->get_use_sim())
  {
    //printf("[LIDAR_3D] simulation mode\n");
    spdlog::info("[LIDAR_3D] simulation mode init");

    return;
  }

  if(config->get_lidar_3d_type() == "LIVOX")
  {
    if(livox == nullptr)
    {
      LIVOX::instance(this);
      livox = LIVOX::instance();
      livox->set_config_module(this->config);
      livox->set_logger_module(this->logger);
      livox->open();
    }
  }
  else if(config->get_lidar_3d_type() == "AIRY")
  {
    if(airy == nullptr)
    {
      AIRY::instance(this);
      airy = AIRY::instance();
      airy->set_config_module(this->config);
      airy->set_logger_module(this->logger);
      airy->open();
    }
  }

  spdlog::info("[LIDAR_3D] init");
}

void LIDAR_3D::open()
{
  // check simulation mode
  if(config->get_use_sim())
  {
    //printf("[LIDAR_3D] simulation mode\n");
    spdlog::info("[LIDAR_3D] simulation mode open");
    return;
  }

  //printf("[LIDAR_3D] open\n");
  spdlog::info("[LIDAR_3D] open");

  // stop first
  close();

  // deskewing loop start
  int lidar_num = config->get_lidar_3d_num();
  for(int idx = 0; idx < lidar_num; idx++)
  {
    deskewing_flag[idx] = true;
    deskewing_thread[idx] = std::make_unique<std::thread>(&LIDAR_3D::deskewing_loop, this, idx);
  }

  // merge loop start
  merge_flag = true;
  merge_thread = std::make_unique<std::thread>(&LIDAR_3D::merge_loop, this);
}

void LIDAR_3D::close()
{
  is_connected = false;

  int lidar_num = config->get_lidar_3d_num();
  for(int idx = 0; idx < lidar_num; idx++)
  {
    deskewing_flag[idx] = false;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  merge_flag = false;
  if(merge_thread && merge_thread->joinable())
  {
    merge_thread->join();
  }
  merge_thread.reset();

  for(int idx = 0; idx < lidar_num; idx++)
  {
    if(deskewing_thread[idx] && deskewing_thread[idx]->joinable())
    {
      deskewing_thread[idx]->join();
    }
    deskewing_thread[idx].reset();
  }
}

LVX_FRM LIDAR_3D::get_cur_raw(int idx)
{
  LVX_FRM res;
  if(config->get_lidar_3d_type() == "LIVOX" && livox != nullptr)
  {
    res = livox->get_cur_raw(idx);
  }
  else if(config->get_lidar_3d_type() == "AIRY" && airy != nullptr)
  {
    res = airy->get_cur_raw(idx);
  }

  return res;
}

TIME_PTS LIDAR_3D::get_cur_frm()
{
  std::lock_guard<std::mutex> lock(mtx);
  return cur_frm;
}

bool LIDAR_3D::get_is_connected()
{
  return (bool)is_connected.load();
}

bool LIDAR_3D::get_is_sync()
{
  return (bool)is_sync.load();
}

double LIDAR_3D::get_process_time_deskewing(int idx)
{
  return (double)process_time_deskewing[idx].load();
}

double LIDAR_3D::get_process_time_merge()
{
  return (double)process_time_merge.load();
}

IMU LIDAR_3D::get_cur_imu(int idx)
{
  IMU res;
  if(config->get_lidar_3d_type() == "LIVOX" && livox != nullptr)
  {
    res = livox->get_cur_imu(idx);
  }
  else if(config->get_lidar_3d_type() == "AIRY" && airy != nullptr)
  {
    res = airy->get_cur_imu(idx);
  }

  return res;
}

IMU LIDAR_3D::get_best_imu(double ref_t)
{
  int lidar_num = config->get_lidar_3d_num();
  std::vector<std::vector<IMU>> imu_storage(lidar_num);
//    if(config->get_lidar_3d_type() == "LIVOX" && livox != nullptr)
  if(true)
  {
    for(int p = 0; p < lidar_num; p++)
    {
      imu_storage[p] = livox->get_imu_storage(p);
    }
  }
  else
  {
    //printf("[LIDAR_3D] unknown LIDAR type or lidar not initialized\n");
    spdlog::info("[LIDAR_3D] unknown LIDAR type or lidar not initialized");
    return IMU();
  }

  IMU res;
  double min_dt = std::numeric_limits<double>::max();
  for(int idx = 0; idx < lidar_num; idx++)
  {
    const std::vector<IMU>& list = imu_storage[idx];
    for(size_t p = 0; p < list.size(); p++)
    {
      double dt = std::abs(list[p].t - ref_t);
      if(dt < min_dt)
      {
        min_dt = dt;
        res = list[p];
      }
    }
  }

  return res;
}

IMU LIDAR_3D::get_best_imu(double ref_t, int idx)
{
  std::vector<IMU> imu_storage;
//    if(config->get_lidar_3d_type() == "LIVOX" && livox != nullptr)
  if(true)
  {
    imu_storage = livox->get_imu_storage(idx);
  }
  else
  {
    //printf("[LIDAR_3D] unknown LIDAR type or lidar not initialized\n");
    spdlog::info("[LIDAR_3D] unknown LIDAR type or lidar not initialized");
    return IMU();
  }

  IMU res = imu_storage.back();
  double min_dt = std::numeric_limits<double>::max();
  for(size_t p = 0; p < imu_storage.size(); p++)
  {
    double dt = std::abs(imu_storage[p].t - ref_t);
    if(dt < min_dt)
    {
      min_dt = dt;
      res = imu_storage[p];
    }
  }

  return res;
}

QString LIDAR_3D::get_cur_state()
{
  std::lock_guard<std::mutex> lock(mtx);
  QString res = cur_state;
  return res;
}

void LIDAR_3D::set_cur_state(QString str)
{
  std::lock_guard<std::mutex> lock(mtx);
  cur_state = str;
}

QString LIDAR_3D::get_info_text()
{
  QString res;
  int lidar_num = config->get_lidar_3d_num();
  if(config->get_lidar_3d_type() == "LIVOX" && livox != nullptr)
  {
    for(int idx = 0; idx < lidar_num; idx++)
    {
      res += livox->get_info_text(idx);
      res += QString("dq: %1\n\n").arg((int)deskewing_que[idx].unsafe_size());
    }
  }
  else if(config->get_lidar_3d_type() == "AIRY" && airy != nullptr)
  {
    for(int idx = 0; idx < lidar_num; idx++)
    {
      res += airy->get_info_text(idx);
      res += QString("dq: %1\n\n").arg((int)deskewing_que[idx].unsafe_size());
    }
  }

  res += QString("mq: %1 (%2)").arg((int)merged_que.unsafe_size()).arg(cur_merged_num.load());
  return res;
}

void LIDAR_3D::set_is_sync(bool flag)
{
  is_sync = flag;

  QString lidar_type = config->get_lidar_3d_type();
  int lidar_num = config->get_lidar_3d_num();
  for(int idx = 0; idx < lidar_num; idx++)
  {
    if(lidar_type == "LIVOX" && livox != nullptr)
    {
      livox->set_is_sync(idx, flag);
      //printf("[LIDAR_3D] set livox->is_sync[%d] = %d\n",idx, flag);
      spdlog::info("[LIDAR_3D] set livox->is_sync[{}] = {}",idx, flag);
    }
  }
}

void LIDAR_3D::clear_merged_queue()
{
  merged_que.clear();
}

bool LIDAR_3D::try_pop_merged_queue(TIME_PTS& frm)
{
  if(merged_que.try_pop(frm))
  {
    return true;
  }

  return false;
}

void LIDAR_3D::deskewing_loop(int idx)
{
  IMU _pre_imu;

  double pre_loop_time = get_time();

  //printf("[LIDAR_3D] dsk_loop[%d] start\n", idx);
  spdlog::info("[LIDAR_3D] dsk_loop[{}] start", idx);

  while(deskewing_flag[idx])
  {
    if(!is_connected)
    {
      is_connected = true;
      livox->set_is_connected(idx, true);
    }

    LVX_FRM frm;
    if(livox->try_pop_frm_que(idx, frm))
    {
      // imu
      IMU _cur_imu = get_best_imu(frm.t, idx);

      // deskewing
      std::vector<Eigen::Vector3d> dsk;
      {
        IMU imu0 = get_best_imu(frm.pts.front().t, idx);
        IMU imu1 = get_best_imu(frm.pts.back().t, idx);

        Eigen::Matrix3d R0 = Sophus::SO3d::exp(Sophus::Vector3d(imu0.rx, imu0.ry, imu0.rz)).matrix();
        Eigen::Matrix3d R1 = Sophus::SO3d::exp(Sophus::Vector3d(imu1.rx, imu1.ry, imu1.rz)).matrix();
        Eigen::Matrix3d dR = R0.inverse()*R1;
        Eigen::Matrix4d dG = Eigen::Matrix4d::Identity();
        dG.block(0,0,3,3) = dR;

        dsk.resize(frm.pts.size());
        #pragma omp parallel for
        for(size_t p = 0; p < frm.pts.size(); p++)
        {
          Eigen::Matrix4d G_i = intp_tf(frm.pts[p].alpha, Eigen::Matrix4d::Identity(), dG);
          Eigen::Vector3d P(frm.pts[p].x, frm.pts[p].y, frm.pts[p].z);
          Eigen::Vector3d _P = G_i.block(0,0,3,3)*P + G_i.block(0,3,3,1);
          dsk[p] = _P;
        }
      }

      // make frame
      TIME_PTS dsk_frm;
      dsk_frm.t = frm.t;
      dsk_frm.pts = dsk;
      // printf("dsk_frm[%d] t:%f, pts:%zu\n", idx, dsk_frm.t, dsk_frm.pts.size());

      // set queue
      deskewing_que[idx].push(dsk_frm);

      // for queue overflow
      if(deskewing_que[idx].unsafe_size() > 10)
      {
        TIME_PTS tmp;
        deskewing_que[idx].try_pop(tmp);
      }

      // for next
      _pre_imu = _cur_imu;

      process_time_deskewing[idx] = (double)(get_time() - pre_loop_time);
      pre_loop_time = get_time();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  //printf("[LIDAR_3D] dsk_loop[%d] stop\n", idx);
  spdlog::info("[LIDAR_3D] dsk_loop[{}] stop", idx);

}

void LIDAR_3D::merge_loop()
{
  // 10ms
  const int lidar_3d_num = config->get_lidar_3d_num();
  double timeout = 0.05;
  double wait_timeout = 0.15;
  double last_recv_t[lidar_3d_num];

  std::vector<TIME_PTS> storage[2];

  double pre_loop_time = get_time();

  //printf("[LIDAR_3D] a_loop start\n");
  spdlog::info("[LIDAR_3D] a_loop start");
  while(merge_flag)
  {
    TIME_PTS merge_frm;
    merge_frm.pts.clear();

    // set storage
    if(lidar_3d_num == 1)
    {
      TIME_PTS frm;
      if(deskewing_que[0].try_pop(frm))
      {
        merged_que.push(frm);
        cur_merged_frm_t = frm.t;
        cur_merged_num = frm.pts.size();

        // update
        if(merged_que.unsafe_size() > 10)
        {
          TIME_PTS tmp;
          merged_que.try_pop(tmp);
        }

        std::lock_guard<std::mutex> lock(mtx);
        cur_frm = frm;
      }

      process_time_merge = (double)(get_time() - pre_loop_time);
      pre_loop_time = get_time();

      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    else if((livox->get_time_type(0) == 1 && livox->get_time_type(0) == 1))
    {
      for(int idx = 0; idx < lidar_3d_num; idx++)
      {
        TIME_PTS frm;
        if(deskewing_que[idx].try_pop(frm))
        {
          storage[idx].push_back(frm);
          last_recv_t[idx] = frm.t;

          if(storage[idx].size() > 10)
          {
            storage[idx].erase(storage[idx].begin());
          }
        }
      }
    }
    else
    {
      TIME_PTS frm;
      if(deskewing_que[0].try_pop(frm))
      {
        storage[0].push_back(frm);
        last_recv_t[0] = frm.t;

        if(storage[0].size() > 10)
        {
          storage[0].erase(storage[0].begin());
        }
      }
    }

    /*
    // find the closest pair
    bool matched = false;
    TIME_PTS best0, best1;
    double min_dt = 9999;
    size_t best_i = 0, best_j = 0;

    for(size_t i = 0; i < storage[0].size(); i++)
    {
      for(size_t j = 0; j < storage[1].size(); j++)
      {
        double dt = std::abs(storage[0][i].t - storage[1][j].t);
        if(dt < timeout && dt < min_dt)
        {
          matched = true;
          min_dt = dt;
          best0 = storage[0][i];
          best1 = storage[1][j];
          best_i = i;
          best_j = j;
        }
      }
    }
    */

    bool matched = false;
    TIME_PTS best0, best1;
    double min_dt = 9999;
    size_t ref_i = 0;
    size_t best_j = 0;

    if(!storage[0].empty() && !storage[1].empty())
    {
      ref_i = storage[0].size() - 1;

      for(size_t j = 0; j < storage[1].size(); j++)
      {
        double dt = std::abs(storage[0][ref_i].t - storage[1][j].t);
        if(dt < timeout && dt < min_dt)
        {
          min_dt = dt;
          best0 = storage[0][ref_i];
          best1 = storage[1][j];
          best_j = j;
          matched = true;
        }
      }
    }

    if(matched)
    {
      // merge
      merge_frm.t = std::min(best0.t, best1.t);
      merge_frm.pts.insert(merge_frm.pts.end(), best0.pts.begin(), best0.pts.end());
      merge_frm.pts.insert(merge_frm.pts.end(), best1.pts.begin(), best1.pts.end());

      // erase
      // storage[0].erase(storage[0].begin(), storage[0].begin() + best_i + 1);
      storage[0].erase(storage[0].begin(), storage[0].begin() + ref_i + 1);
      storage[1].erase(storage[1].begin(), storage[1].begin() + best_j + 1);
      // printf("[LIDAR] paired t=%.6f, idx: (%zu, %zu), pts0_t=%.6f, pts1_t=%.6f, total=%zu (storage: %zu, %zu))\n", merge_frm.t, ref_i, best_j, best0.t, best1.t, merge_frm.pts.size(), storage[0].size(), storage[1].size());
      spdlog::debug("[LIDAR] paired t={:.6f}, idx: ({}, {}), pts0_t={:.6f}, pts1_t={:.6f}, total={} (storage: {}, {}))", merge_frm.t, ref_i, best_j, best0.t, best1.t, merge_frm.pts.size(), storage[0].size(), storage[1].size());
    }
    else
    {
      // use single frame only if too old to wait for match
      int use_idx = -1;

      if(!storage[0].empty() && storage[1].empty())
      {
        if(storage[0].back().t - last_recv_t[1] > wait_timeout)
        {
          use_idx = 0;
          // printf("[LIDAR] 0-time: %f - %f = %f\n", storage[0].back().t, last_recv_t[1], storage[0].back().t - last_recv_t[1]);
        }
      }
      else if(storage[0].empty() && !storage[1].empty())
      {
        if(storage[1].back().t - last_recv_t[0] > wait_timeout)
        {
          use_idx = 1;
          // printf("[LIDAR] 1-time: %f - %f = %f\n", storage[1].back().t, last_recv_t[0], storage[1].back().t - last_recv_t[0]);
        }
      }
      else if(!storage[0].empty() && !storage[1].empty())
      {
        if(std::abs(storage[0].back().t - storage[1].back().t) > wait_timeout)
        {
          // use_idx = (t0 < t1) ? 0 : 1;
          use_idx = 0;
        }
      }

      if(use_idx >= 0)
      {
        merge_frm = storage[use_idx].back();
        storage[use_idx].pop_back();

        // printf("[LIDAR] single  t=%.6f, from=%d, pts=%zu (storage: %zu, %zu)\n", merge_frm.t, use_idx, merge_frm.pts.size(), storage[0].size(), storage[1].size());
      }
    }

    // save result
    if((int)merge_frm.pts.size() > 0)
    {
      // update
      merged_que.push(merge_frm);

      // watchdog
      cur_merged_frm_t = merge_frm.t;
      cur_merged_num = merge_frm.pts.size();

      if(merged_que.unsafe_size() > 10)
      {
        TIME_PTS tmp;
        merged_que.try_pop(tmp);
      }

      std::lock_guard<std::mutex> lock(mtx);
      cur_frm = merge_frm;
    }

    process_time_merge = (double)(get_time() - pre_loop_time);
    pre_loop_time = get_time();

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  //printf("[LIDAR_3D] a_loop stop\n");
  spdlog::info("[LIDAR_3D] a_loop stop");
}

void LIDAR_3D::set_config_module(CONFIG* _config)
{
  config = _config;
}

void LIDAR_3D::set_logger_module(LOGGER* _logger)
{
  logger = _logger;
}

void LIDAR_3D::set_mobile_module(MOBILE *_mobile)
{
  mobile = _mobile;
}

bool LIDAR_3D::sensor_on(int idx)
{
  return livox->sensor_on(idx);
}

bool LIDAR_3D::sensor_off(int idx)
{
  return livox->sensor_off(idx);
}

void LIDAR_3D::slot_set_on(std::vector<int> indexs)
{
  for(auto& idx : indexs)
  {
    livox->sensor_on(idx);
    spdlog::info("[LIDAR_3D] slot set on, idx: {}", idx);
  }
}

void LIDAR_3D::slot_set_off(std::vector<int> indexs)
{
  for(auto& idx : indexs)
  {
    livox->sensor_off(idx);
    spdlog::info("[LIDAR_3D] slot set off, idx: {}", idx);
  }
}
