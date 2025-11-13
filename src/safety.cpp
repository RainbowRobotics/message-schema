#include "safety.h"
namespace 
{
    const char* MODULE_NAME = "SAFETY";
}

SAFETY* SAFETY::instance(QObject* parent)
{
    static SAFETY* inst = nullptr;
    if(!inst && parent)
    {
        inst = new SAFETY(parent);
    }
    else if(inst && parent && inst->parent() == nullptr)
    {
        inst->setParent(parent);
    }
    return inst;
}

SAFETY::SAFETY(QObject *parent) : QObject{parent},
    config(nullptr),
    logger(nullptr),
    mobile(nullptr),
    unimap(nullptr),
    obsmap(nullptr),
    loc(nullptr)
{

}

SAFETY::~SAFETY()
{
    safety_flag = false;
    if(safety_thread && safety_thread->joinable())
    {
        safety_thread->join();
    }
    safety_thread.reset();
}

void SAFETY::init()
{
    if(config)
    {
        safety_fields = config->get_monitoring_field();
        for(int i = 0; i < safety_fields.size(); i++)
        {
            logger->write_log("[SAFETY] safety_fields[" + QString::number(i) + "].monitor_id: " + QString::number(safety_fields[i].monitor_id), "Green");
            logger->write_log("[SAFETY] safety_fields[" + QString::number(i) + "].min_x: " + QString::number(safety_fields[i].min_x), "Green");
            logger->write_log("[SAFETY] safety_fields[" + QString::number(i) + "].max_x: " + QString::number(safety_fields[i].max_x), "Green");
            logger->write_log("[SAFETY] safety_fields[" + QString::number(i) + "].min_y: " + QString::number(safety_fields[i].min_y), "Green");
            logger->write_log("[SAFETY] safety_fields[" + QString::number(i) + "].max_y: " + QString::number(safety_fields[i].max_y), "Green");

            spdlog::info("[SAFETY] safety_fields[{}].monitor_id: {}", i, safety_fields[i].monitor_id);
            spdlog::info("[SAFETY] safety_fields[{}].min_x: {}", i, safety_fields[i].min_x);
            spdlog::info("[SAFETY] safety_fields[{}].max_x: {}", i, safety_fields[i].max_x);
            spdlog::info("[SAFETY] safety_fields[{}].min_y: {}", i, safety_fields[i].min_y);
            spdlog::info("[SAFETY] safety_fields[{}].max_y: {}", i, safety_fields[i].max_y);
        }
        logger->write_log("[SAFETY] safety_fields size: " + QString::number(safety_fields.size()), "Green");
        spdlog::info("[SAFETY] safety_fields size: {}", safety_fields.size());

    }
    else
    {
        //To do: log error message
    }
}

void SAFETY::start()
{
    safety_flag = true;
    safety_thread = std::make_unique<std::thread>(&SAFETY::safety_loop, this);
}

void SAFETY::stop()
{
    safety_flag = false;
    if(safety_thread && safety_thread->joinable())
    {
        safety_thread->join();
    }
    safety_thread.reset();
}

void SAFETY::set_obsmap_module(OBSMAP* _obsmap)
{
    obsmap = _obsmap;
}

void SAFETY::set_localization_module(LOCALIZATION* _loc)
{
    loc = _loc;
}

void SAFETY::set_config_module(CONFIG* _config)
{
    config = _config;
}

void SAFETY::set_logger_module(LOGGER* _logger)
{
    logger = _logger;
}

void SAFETY::set_mobile_module(MOBILE* _mobile)
{
    mobile = _mobile;
}

void SAFETY::set_unimap_module(UNIMAP* _unimap)
{
    unimap = _unimap;
}

cv::Mat SAFETY::get_safety_map()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return safety_map;
}

std::vector<Eigen::Matrix4d> SAFETY::calc_trajectory(Eigen::Vector3d cur_vel, double dt, double predict_t, Eigen::Matrix4d _cur_tf)
{
    std::vector<Eigen::Matrix4d> res;

    double vx = cur_vel[0];
    double vy = cur_vel[1];
    double wz = cur_vel[2];

    double pre_x = 0;
    double pre_y = 0;
    double pre_th = 0;

    for(double t = 0; t <= predict_t; t += dt)
    {
        pre_th = pre_th + wz*dt;

        Eigen::Matrix3d pre_rot = Eigen::Matrix3d::Zero();

        pre_rot << std::cos(pre_th), -std::sin(pre_th), 0,
                std::sin(pre_th),  std::cos(pre_th), 0,
                0,                 0, 1;

        Eigen::Vector3d local_vel = Eigen::Vector3d(vx, vy, 0);
        Eigen::Vector3d vel = pre_rot*local_vel;

        pre_x += vel[0]*dt;
        pre_y += vel[1]*dt;

        Eigen::Matrix4d G = se2_to_TF(Eigen::Vector3d(pre_x, pre_y, pre_th));
        Eigen::Matrix4d predict_G = _cur_tf*G;
        res.push_back(predict_G);
    }

    return res;
}

void SAFETY::safety_loop()
{
    const double dt = 0.02; // 50hz
    double pre_loop_time = get_time();

    double predict_t = 5;
    int stride = 5;

    spdlog::info("[SAFETY] safety_loop start");
    while(safety_flag)
    {
        cv::Mat static_map = obsmap->get_static_map();
        if(static_map.empty())
        {
            continue;
        }

        cv::Mat canvas;
        cv::cvtColor(static_map, canvas, cv::COLOR_GRAY2BGR);

        Eigen::Vector3d cur_vel = mobile->get_pose().vel;
        std::vector<Eigen::Matrix4d> traj = calc_trajectory(cur_vel, dt, predict_t, Eigen::Matrix4d::Identity());

        for(size_t k = 0; k < traj.size(); k++)
        {
            const Eigen::Matrix4d &G = traj[k];

            double x = G(0,3);
            double y = G(1,3);

            cv::Vec2i uv = obsmap->xy_uv(x, y);
            int u = uv[0];
            int v = uv[1];
            if(u >= 0 && u < canvas.cols && v >= 0 && v < canvas.rows)
            {
                canvas.ptr<cv::Vec3b>(v)[u] = cv::Vec3b(0,255,0);
            }

            if((int)k % stride == 0)
            {
                double x_min = config->get_robot_size_x_min();
                double x_max = config->get_robot_size_x_max();
                double y_min = config->get_robot_size_y_min();
                double y_max = config->get_robot_size_y_max();

                Eigen::Vector3d P0(x_max, y_max, 0);
                Eigen::Vector3d P1(x_max, y_min, 0);
                Eigen::Vector3d P2(x_min, y_min, 0);
                Eigen::Vector3d P3(x_min, y_max, 0);

                Eigen::Vector3d _P0 = G.block(0,0,3,3)*P0 + G.block(0,3,3,1);
                Eigen::Vector3d _P1 = G.block(0,0,3,3)*P1 + G.block(0,3,3,1);
                Eigen::Vector3d _P2 = G.block(0,0,3,3)*P2 + G.block(0,3,3,1);
                Eigen::Vector3d _P3 = G.block(0,0,3,3)*P3 + G.block(0,3,3,1);

                cv::Vec2i uv0 = obsmap->xy_uv(_P0[0], _P0[1]);
                cv::Vec2i uv1 = obsmap->xy_uv(_P1[0], _P1[1]);
                cv::Vec2i uv2 = obsmap->xy_uv(_P2[0], _P2[1]);
                cv::Vec2i uv3 = obsmap->xy_uv(_P3[0], _P3[1]);

                if(uv0[0] >= 0 && uv1[0] >= 0 && uv2[0] >= 0 && uv3[0] >= 0 &&
                        uv0[0] < canvas.cols && uv1[0] < canvas.cols &&
                        uv2[0] < canvas.cols && uv3[0] < canvas.cols &&
                        uv0[1] >= 0 && uv1[1] >= 0 && uv2[1] >= 0 && uv3[1] >= 0 &&
                        uv0[1] < canvas.rows && uv1[1] < canvas.rows &&
                        uv2[1] < canvas.rows && uv3[1] < canvas.rows)
                {
                    std::vector<std::vector<cv::Point> > poly(1);
                    poly[0].push_back(cv::Point(uv0[0], uv0[1]));
                    poly[0].push_back(cv::Point(uv1[0], uv1[1]));
                    poly[0].push_back(cv::Point(uv2[0], uv2[1]));
                    poly[0].push_back(cv::Point(uv3[0], uv3[1]));
                    cv::polylines(canvas, poly, true, cv::Scalar(0,255,255), 1, cv::LINE_8);
                }
            }
        }

        // obsmap->draw_robot_outline(canvas);

        // update
        {
            std::unique_lock<std::shared_mutex> lock(mtx);
            safety_map = canvas.clone();
        }

        // for real time loop
        double cur_loop_time = get_time();
        double delta_loop_time = cur_loop_time - pre_loop_time;

        if(delta_loop_time < dt)
        {
            int sleep_ms = (dt-delta_loop_time)*1000;
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
        else
        {
            // printf("[SAFETY] safety_loop time drift, dt:%f\n", delta_loop_time);
        }
        pre_loop_time = get_time();
    }
    spdlog::info("[SAFETY] safety_loop stop");
}
