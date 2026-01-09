#include "coin_d4.h"

namespace
{
    const char* MODULE_NAME = "COIN_D4";
}

COIN_D4* COIN_D4::instance(QObject *parent)
{
    static COIN_D4* inst = nullptr;
    if(!inst && parent)
    {
        inst = new COIN_D4(parent);
    }
    else if(inst && parent && inst->parent() == nullptr)
    {
        inst->setParent(parent);
    }
    return inst;
}

COIN_D4::COIN_D4(QObject *parent) : QObject{parent},
    config(nullptr),
    logger(nullptr)
{
}

COIN_D4::~COIN_D4()
{
    recv_flag = false;
    if(recv_thread && recv_thread->joinable())
    {
        recv_thread->join();
    }
    recv_thread.reset();

    grab_flag = false;
    if(grab_thread && grab_thread->joinable())
    {
        grab_thread->join();
    }
    grab_thread.reset();

    if(is_connected.load())
    {
        logger->write_log("[COIN_D4] Destructor cleanup: turning off laser.");
        laser.turnOff();           // mottor off
        laser.disconnecting();     // lidar disconnect
        is_connected.store(false);
    }
}

void COIN_D4::open()
{
    close();

    if(recv_thread == nullptr)
    {
        recv_flag = true;
        recv_thread = make_unique<std::thread>(&COIN_D4::recv_loop, this);
    }

    if(grab_thread == nullptr)
    {
        grab_flag = true;
        grab_thread = make_unique<std::thread>(&COIN_D4::grab_loop, this);
    }
}

void COIN_D4::close()
{
    is_connected.store(false);

    recv_flag = false;
    if(recv_thread && recv_thread->joinable())
    {
        recv_thread->join();
    }
    recv_thread.reset();

    grab_flag = false;
    if(grab_thread && grab_thread->joinable())
    {
        grab_thread->join();
    }
    grab_thread.reset();
}

void COIN_D4::set_raw_scan(const LaserScan& scan)
{
    std::unique_lock<std::shared_mutex> lock(mtx);
    raw_scan = scan;
}

LaserScan COIN_D4::get_raw_scan()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return raw_scan;
}

std::vector<Eigen::Vector3d> COIN_D4::get_cur_pts()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return cur_pts;
}

TIME_PTS COIN_D4::get_cur_tp()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return cur_tp;
}

void COIN_D4::grab_loop()
{
    Eigen::Vector2d circle_xy(0.3, 0.15);
    double circle_radius = 0.05;

    double MASKING_SIZE_X[2] = {0, 0.4};
    double MASKING_SIZE_Y[2] = {-20, 20};

    Eigen::Matrix4d tf = ZYX_to_TF(0.22, 0, 0, 0, 0, 180*D2R);

    is_connected.store(true);

    log_info("grab_loop start");
    while(grab_flag)
    {
        LaserScan local_scan = get_raw_scan();
        if(local_scan.system_time_stamp == last_time.load() || local_scan.data.empty())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        std::vector<Eigen::Vector3d> raw_pts;
        std::vector<double> reflects;

        double max_dist = std::numeric_limits<double>::min();
        for(int i = 0; i < local_scan.data.size(); i++)
        {
            double deg = local_scan.data[i].angle;
            double dist = local_scan.data[i].distance;

            // set angle view
            if(deg < set_min_angle || deg > set_max_angle)
            {
                continue;
            }

            if(dist < set_min_dist || dist > set_max_dist)
            {
                continue;
            }

            double x = dist * std::cos(deg * D2R);
            double y = dist * std::sin(deg * D2R);

            if(dist > max_dist)
            {
                max_dist = dist;
            }

            double rssi = local_scan.data[i].intensity;
            reflects.push_back(rssi);

            Eigen::Vector3d P = tf.block(0,0,3,3) * Eigen::Vector3d(x,y,0) + tf.block(0,3,3,1);

            if(P[0] > config->get_robot_size_x_min() && P[0] < config->get_robot_size_x_max() &&
               P[1] > config->get_robot_size_y_min() && P[1] < config->get_robot_size_y_max())
            {
                continue;
            }

            if(P[0] > MASKING_SIZE_X[0] && P[0] < MASKING_SIZE_X[1] &&
               P[1] > MASKING_SIZE_Y[0] && P[1] < MASKING_SIZE_Y[1])
            {
                continue;
            }

            double dx0 = P[0] - circle_xy[0];
            double dy0 = P[1] - circle_xy[1];
            double masking_dist0 = std::sqrt((dx0*dx0) + (dy0*dy0));

            double dx1 = P[0] - circle_xy[0];
            double dy1 = P[1] - (-circle_xy[1]);
            double masking_dist1 = std::sqrt((dx1*dx1) + (dy1*dy1));

            if(masking_dist0 < circle_radius || masking_dist1 < circle_radius)
            {
                continue;
            }

            raw_pts.push_back(P);
        }

        {
            std::unique_lock<std::shared_mutex> lock(mtx);
            cur_pts = raw_pts;
            max_dist_blidar = max_dist;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    log_info("grab_loop stop");
}

void COIN_D4::recv_loop()
{
    std::vector<float> ignore_array;
    bool hardError;

    LidarHealthInfo.status = 0;
    LidarHealthInfo.error_code = 0;
    ignore_array.clear();

    laser.setIgnoreArray(ignore_array);

    bool ret = false;
    while(recv_flag)
    {
        if(!is_connected)
        {
            logger->write_log("[COIN_D4] Attempting to (re)connect...");
            laser.turnOff();
            laser.disconnecting();

            laser.lidarModelConfiguration(CSPC_LIDAR_COIN_D4, port);
            ret = laser.initialize();

            if(ret)
            {
                ret = laser.turnOn();
                if(!ret)
                {
                    if(laser.getSdkErrorInfo() == e_CspcNoError)
                    {
                        printf("[C_CSPC_Lidar INFO] No Error\n");
                    }
                    else if(laser.getSdkErrorInfo() == e_BlockError)
                    {
                        printf("[C_CSPC_Lidar INFO] Lidar Block Error\n");
                    }
                    else if(laser.getSdkErrorInfo() == e_CommunicationOrPowerError)
                    {
                        printf("[C_CSPC_Lidar INFO] Communication or Power Error\n");
                    }
                    else if(laser.getSdkErrorInfo() == e_LaserOrCcdFailureError)
                    {
                        printf("[C_CSPC_Lidar INFO] Laser or CCD Failure\n");
                    }
                    else if(laser.getSdkErrorInfo() == e_LidarCalibrationCrcError)
                    {
                        printf("[C_CSPC_Lidar INFO] Calibration CRC Error\n");
                    }
                    else if(laser.getSdkErrorInfo() == e_LidarSncodeStorageError)
                    {
                        printf("[C_CSPC_Lidar INFO] SN Code Storage Error\n");
                    }
                    else
                    {
                        printf("[C_CSPC_Lidar INFO] Other SDK Error\n");
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                    continue;
                }
                else
                {
                    is_connected = true;
                    failure_count = 0;
                    logger->write_log("[COIN_D4] LIDAR connected successfully.");
                }
            }
            else
            {
                logger->write_log("[COIN_D4] LIDAR initialize() failed. Retrying...");
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                continue;
            }
        }

        LaserScan scan;
        bool ret1 = laser.doProcessSimple(scan, hardError);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if(!ret1)
        {
            failure_count++;
            if(failure_count >= max_failures)
            {
                logger->write_log("[COIN_D4] Data stream failed multiple times. Disconnecting.");
                is_connected = false;
                continue;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        set_raw_scan(scan);

        failure_count = 0;
    }

    // wrap up grab_flag loop
    laser.turnOff();
    laser.disconnecting();
    logger->write_log("[COIN_D4] recv_loop terminated.");
}

void COIN_D4::set_config_module(CONFIG* _config)
{
    config = _config;
}

void COIN_D4::set_logger_module(LOGGER* _logger)
{
    logger = _logger;
}
