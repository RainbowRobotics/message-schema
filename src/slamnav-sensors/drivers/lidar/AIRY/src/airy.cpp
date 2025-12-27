#include "airy.h"
namespace 
{
    const char* MODULE_NAME = "AIRY";
}

AIRY* AIRY::instance(QObject* parent)
{
    static AIRY* inst = nullptr;
    if(!inst && parent)
    {
        inst = new AIRY(parent);
    }
    else if(inst && parent && inst->parent() == nullptr)
    {
        inst->setParent(parent);
    }
    return inst;
}

AIRY::AIRY(QObject *parent) : QObject{parent}
{

}

AIRY::~AIRY()
{
    is_connected = false;
    close();
}

bool AIRY::Client::init()
{
    _driver.regPointCloudCallback(std::bind(&Client::driver_get_cloud_cb, this),
                   std::bind(&Client::driver_return_cloud_cb, this, std::placeholders::_1));
    _driver.regImuDataCallback(std::bind(&Client::driver_get_imu_cb, this),
                   std::bind(&Client::driver_return_imu_cb, this, std::placeholders::_1));

    if(!_driver.init(_rs_param))
    {
        return false;
    }
    return true;
}

void AIRY::Client::start()
{
    _thread_flag = true;
    _thread_cloud = std::thread(std::bind(&Client::point_cloud_callback, this));
    _thread_imu = std::thread(std::bind(&Client::imu_data_callback, this));
    _driver.start();
}

void AIRY::Client::stop()
{
    _driver.stop();
    _thread_flag = false;
    _thread_cloud.join();
    _thread_imu.join();
}

LVX_FRM AIRY::Client::get_cur_raw()
{
    std::lock_guard<std::mutex> lock(_lidar_mtx);
    LVX_FRM res = _cur_raw;
    return res;
}

IMU AIRY::Client::get_cur_imu()
{
    std::lock_guard<std::mutex> lock(_imu_mtx);
    IMU res = _cur_imu;
    return res;
}

std::vector<IMU> AIRY::Client::get_imu_storage()
{
    std::lock_guard<std::mutex> lock(_imu_mtx);
    std::vector<IMU> res = _imu_storage;
    return res;
}

bool AIRY::Client::try_pop_frm_que(LVX_FRM& frm)
{
    if(_frm_que.try_pop(frm))
    {
        return true;
    }
    return false;
}

std::shared_ptr<AIRY::PointCloudMsg> AIRY::Client::driver_get_cloud_cb()
{
    std::shared_ptr<PointCloudMsg> msg = _free_cloud_queue.pop();
    if(msg.get() != NULL)
    {
        return msg;
    }
    return std::make_shared<PointCloudMsg>();
}

void AIRY::Client::driver_return_cloud_cb(std::shared_ptr<PointCloudMsg> msg)
{
    _stuffed_cloud_queue.push(msg);
}

std::shared_ptr<AIRY::ImuData> AIRY::Client::driver_get_imu_cb()
{
    std::shared_ptr<ImuData> msg = _free_imu_queue.pop();
    if(msg.get() != NULL)
    {
        return msg;
    }
    return std::make_shared<ImuData>();
}

void AIRY::Client::driver_return_imu_cb(std::shared_ptr<ImuData> msg)
{
    _stuffed_imu_queue.push(msg);
}

void AIRY::Client::point_cloud_callback()
{
    while(_thread_flag)
    {
        std::shared_ptr<PointCloudMsg> msg = _stuffed_cloud_queue.popWait();
        if(msg.get() == NULL)
        {
            continue;
        }

        size_t size = msg->points.size();
        if(size < 2)
        {
            continue;
        }

        // airy API does not provide time_interval between frames
        // so time_interval between frames defined 100ms
        uint64_t time_base = static_cast<const uint64_t>(msg->timestamp * 1e9); // nanosec
        uint16_t time_interval = static_cast<const uint16_t>(1e8 / size); // (0.1(sec) / size)
        double alpha0 = 1 / (size - 1);

        std::vector<LVX_PT> pts;
        pts.reserve(size);

        for(size_t i = 0; i < size; ++i)
        {
            auto& p = msg->points[i];
            if(p.intensity == 0)
            {
                continue;
            }

            Eigen::Vector3d P(p.x, p.y, p.z);

            Eigen::Vector3d _P = pts_R * P + pts_t;

            double d = _P.norm();
            if(d < _p->lidar_min_range || d > _p->lidar_max_range)
            {
                continue;
            }

            if(check_self_collision(_P(0), _P(1), _P(2), _p->robot_size_x_min, _p->robot_size_x_max,
                                    _p->robot_size_y_min, _p->robot_size_y_max,
                                    _p->robot_size_z_min, _p->robot_size_z_max))
            {
                continue;
            }

            LVX_PT pt;
            pt.t = (time_base + i * time_interval) * N2S + offset_t; // nanosec to sec
            pt.x = _P(0);
            pt.y = _P(1);
            pt.z = _P(2);

            // check scale
            pt.reflect = p.intensity;
            pt.alpha = alpha0 * i;
            pts.push_back(pt);
        }

        if(pts.size() < 2)
        {
            continue;
        }

        // make frame
        LVX_FRM frm;
        frm.t = pts[0].t;
        frm.pts.swap(pts);
        _frm_que.push(frm);

        frame_que_size = _frm_que.unsafe_size();
        if(frame_que_size > 5)
        {
            LVX_FRM tmp;
            _frm_que.try_pop(tmp);
        }

        {
            std::lock_guard<std::mutex> lock(_lidar_mtx);
            _cur_raw = frm;
            cur_frm_t = frm.t;
            cur_pts_num = frm.pts.size();
        }

        // for memory reuse
        _free_cloud_queue.push(msg);
    }
}

void AIRY::Client::imu_data_callback()
{
    while(_thread_flag)
    {
        std::shared_ptr<ImuData> msg = _stuffed_imu_queue.popWait();
        if(msg.get() == NULL) continue;

        double time_base = msg->timestamp; // sec
        // time sync
        if(offset_t == 0 || is_sync)
        {
            is_sync = false;
            offset_t = get_time() - time_base;
        }

        Eigen::Vector3d acc_vec(msg->linear_acceleration_x, msg->linear_acceleration_y, msg->linear_acceleration_z);
        Eigen::Vector3d gyr_vec(msg->angular_velocity_x, msg->angular_velocity_y, msg->angular_velocity_z);

        Eigen::Vector3d _acc_vec = imu_R * acc_vec;
        Eigen::Vector3d _gyr_vec = imu_R * gyr_vec;

        // considering centripetal acceleration
        _acc_vec = _acc_vec - _gyr_vec.cross(_gyr_vec.cross(imu_t));

        IMU imu;
        imu.t = time_base + offset_t;
        imu.acc_x = _acc_vec[0] * ACC_G;
        imu.acc_y = _acc_vec[1] * ACC_G;
        imu.acc_z = _acc_vec[2] * ACC_G;
        imu.gyr_x = _gyr_vec[0];
        imu.gyr_y = _gyr_vec[1];
        imu.gyr_z = _gyr_vec[2];

        // get orientation using complementary filter
        double q0, q1, q2, q3;
        _imu_filter.update(_acc_vec[0], _acc_vec[1], _acc_vec[2], _gyr_vec[0], _gyr_vec[1], _gyr_vec[2], 0.005); // 200hz
        _imu_filter.getOrientation(q0, q1, q2, q3);
        Eigen::Matrix3d R = Eigen::Quaterniond(q0, q1, q2, q3).normalized().toRotationMatrix();
        Eigen::Vector3d r = Sophus::SO3d::fitToSO3(R).log();

        imu.rx = r[0];
        imu.ry = r[1];
        imu.rz = r[2];

        {
            std::lock_guard<std::mutex> lock(_imu_mtx);
            {
                cur_imu_t = imu.t;
                _cur_imu = imu;
                _imu_storage.push_back(imu);
            }

            if(_imu_storage.size() > 50)
            {
                _imu_storage.erase(_imu_storage.begin());
            }
        }

        // for memory reuse
        _free_imu_queue.push(msg);
    }
}

void AIRY::open()
{
    spdlog::info("[AIRY] open");
    close();

    read_common_param();

    QString path = QCoreApplication::applicationDirPath() + "/config/" + "airy.json";
    QFile file(path);

    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        spdlog::error("[AIRY] config file not found or unreadable: {}", path.toStdString());
        return;
    }

    QByteArray data = file.readAll();
    file.close();

    QJsonParseError err;
    QJsonDocument doc = QJsonDocument::fromJson(data, &err);
    if(err.error != QJsonParseError::NoError)
    {
        spdlog::error("[AIRY] json parse error: {}", err.errorString().toStdString());
        return;
    }

    auto common_param = std::make_shared<Common>(client_param);
    for(int i = 0; i < config->get_lidar_3d_num(); ++i)
    {
        // set airy config
        robosense::lidar::RSDriverParam rs_param;
        read_airy_json(doc, rs_param, i);
        client_ptr.push_back(std::make_unique<Client>(i, common_param, rs_param));
        if(!client_ptr[i]->init())
        {
            spdlog::error("[AIRY] {} Airy Client init fail", i);
            return;
        }
    }

    spdlog::info("[AIRY] initialized");

    // loop start
    grab_flag = true;
    grab_thread = std::make_unique<std::thread>(&AIRY::grab_loop, this);
}

void AIRY::close()
{
    is_connected = false;

    grab_flag = false;
    if(grab_thread && grab_thread->joinable())
    {
        for(auto& ptr : client_ptr) ptr->stop();
        client_ptr.clear();

        grab_thread->join();
    }
    grab_thread.reset();
}

QString AIRY::get_info_text(int idx)
{
    if(is_connected)
    {
        return QString();
    }

    auto& client = client_ptr[idx];

    IMU imu = client->get_cur_imu();
    int cur_pts_num = client->cur_pts_num;
    int frame_que_size = client->frame_que_size;
    double cur_imu_t = client->cur_imu_t;
    double cur_frm_t = client->cur_frm_t;

    QString res;
    res += QString().sprintf("[LIDAR %d]\nimu_t: %.3f, pts_t: %.3f (%d)\n", idx, cur_imu_t, cur_frm_t, cur_pts_num);
    res += QString().sprintf("acc: %.2f, %.2f, %.2f\n", imu.acc_x, imu.acc_y, imu.acc_z);
    res += QString().sprintf("gyr: %.2f, %.2f, %.2f\n", imu.gyr_x * R2D, imu.gyr_y * R2D, imu.gyr_z * R2D);
    res += QString().sprintf("so3: %.1f, %.1f, %.1f\n", imu.rx * R2D, imu.ry * R2D, imu.rz * R2D);
    // time_type = 1
    res += QString().sprintf("time_sync_type: %d\n fq: %d,", 1, frame_que_size);

    return res;
}


LVX_FRM AIRY::get_cur_raw(int idx)
{
    if(!is_connected)
    {
        return LVX_FRM();
    }
    return client_ptr[idx]->get_cur_raw();
}

IMU AIRY::get_cur_imu(int idx)
{
    if(!is_connected)
    {
        return IMU();
    }
    return client_ptr[idx]->get_cur_imu();
}

std::vector<IMU> AIRY::get_imu_storage(int idx)
{
    if(!is_connected)
    {
        return std::vector<IMU>();
    }
    return client_ptr[idx]->get_imu_storage();
}

bool AIRY::get_is_connected(int idx)
{
    // return (bool)is_connected[idx].load();
    return true;
}

void AIRY::set_is_connected(int idx, bool val)
{
    //is_connected[idx].store(val);
}

bool AIRY::get_is_sync(int idx)
{
    if(!is_connected)
    {
        return false;
    }
    return client_ptr[idx]->is_sync.load();
}

void AIRY::set_is_sync(int idx, bool val)
{
    if(!is_connected)
    {
        return;
    }
    client_ptr[idx]->is_sync.store(val);
}

// ptp time.
int AIRY::get_time_type(int idx)
{
    return 1;
}

bool AIRY::try_pop_frm_que(int idx, LVX_FRM& frm)
{
    if(!is_connected)
    {
        return false;
    }

    if(client_ptr[idx]->try_pop_frm_que(frm))
    {
        return true;
    }
    return false;
}

void AIRY::set_config_module(CONFIG* _config)
{
    config = _config;
}

void AIRY::set_logger_module(LOGGER* _logger)
{
    logger = _logger;
}

// handle is not used
int AIRY::get_airy_idx(uint32_t handle)
{
//    for(size_t i = 0; i < airy_handles.size(); i++)
//    {
//        if(airy_handles[i] == handle)
//        {
//            return i;
//        }
//    }
    return -1;
}

void AIRY::grab_loop()
{
    spdlog::info("[AIRY] grab_loop start");
    is_connected = true;
    for(auto& ptr : client_ptr)
    {
        // read config info
        QString tf_str = config->get_lidar_3d_tf(ptr->_idx);
        ptr->pts_tf = string_to_TF(tf_str);
        ptr->pts_R = ptr->pts_tf.block(0,0,3,3);
        ptr->pts_t = ptr->pts_tf.block(0,3,3,1);
        // imu_tf[i] = string_to_TF(config->LIDAR_IMU_TF[i]);
        ptr->imu_tf = string_to_TF(tf_str);
        ptr->imu_R = ptr->pts_tf.block(0,0,3,3);
        ptr->imu_t = ptr->pts_tf.block(0,3,3,1);

        ptr->start();
    }

    while(grab_flag)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(90));

//        // capture pcd
//        for(auto& ptr : client_ptr)
//        {
//            auto pts = ptr->get_cur_raw();

//            if(pts.pts.size() < 1000) continue;

//            pcl::PointCloud<pcl::PointXYZ> pcl_pts;

//            for(auto& p : pts.pts)
//            {
//                pcl_pts.emplace_back(p.x, p.y, p.z);
//            }

//            auto time = get_time();
//            QString time_str = QString::number(time);
//            QString file = QString("/home/rainbow/ptss/" + time_str + "pts_file.pcd");
//            pcl::io::savePCDFileBinary(file.toStdString(), pcl_pts);
//        }
    }

    spdlog::info("[AIRY] grab_loop stop");
    close();
}

void AIRY::read_common_param()
{
    client_param.lidar_min_range = config->get_lidar_3d_min_range();
    client_param.lidar_max_range = config->get_lidar_3d_max_range();

    client_param.robot_size_x_min = config->get_robot_size_x_min();
    client_param.robot_size_x_max = config->get_robot_size_x_max();
    client_param.robot_size_y_min = config->get_robot_size_y_min();
    client_param.robot_size_y_max = config->get_robot_size_y_max();
    client_param.robot_size_z_min = config->get_robot_size_z_min();
    client_param.robot_size_z_max = config->get_robot_size_z_max();
}

void AIRY::read_airy_json(QJsonDocument& doc, robosense::lidar::RSDriverParam& rs_param, int idx)
{
    QJsonObject root = doc.object();

    {
        rs_param.lidar_type = robosense::lidar::LidarType::RSAIRY;
        rs_param.input_type = robosense::lidar::InputType::ONLINE_LIDAR;
    }

    {
        // read config info
        rs_param.input_param.host_address = config->get_lidar_3d_ip(idx).toStdString();

        // read airy json
        QJsonObject input = root["input"].toObject();
        QJsonArray ports = input["input_port"].toArray();
        QJsonObject port = ports[idx].toObject();

        rs_param.frame_id = port["frame_id"].toString().toStdString();
        rs_param.input_param.msop_port = port["msop_port"].toInt();
        rs_param.input_param.difop_port = port["difop_port"].toInt();
        rs_param.input_param.imu_port = port["imu_port"].toInt();
    }

    {
        // read config info
//        rs_param.decoder_param.min_distance = config->get_lidar_3d_min_range();
//        rs_param.decoder_param.max_distance = config->get_lidar_3d_max_range();

        // read airy json
        QJsonObject common = root["common"].toObject();
        QJsonObject decoder = common["decoder"].toObject();

        rs_param.decoder_param.use_lidar_clock = decoder["use_lidar_clock"].toBool();
        rs_param.decoder_param.dense_points = decoder["dense_points"].toBool();
        rs_param.decoder_param.ts_first_point = decoder["ts_first_point"].toBool();
        rs_param.decoder_param.wait_for_difop = decoder["wait_for_difop"].toBool();
        rs_param.decoder_param.start_angle = decoder["start_angle"].toDouble();
        rs_param.decoder_param.end_angle = decoder["end_angle"].toDouble();
    }

    if(root["print_param"].toBool())
    {
        rs_param.print();
    }
}
