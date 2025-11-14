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
    obsmap(nullptr)
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
    std::vector<double> min_x = config->get_field_size_min_x();
    std::vector<double> max_x = config->get_field_size_max_x();
    std::vector<double> min_y = config->get_field_size_min_y();
    std::vector<double> max_y = config->get_field_size_max_y();

    fields.resize(config->get_monitoring_field_num());

    for(size_t i = 0; i < fields.size(); i++)
    {
        double x_min = min_x[i];
        double x_max = max_x[i];
        double y_min = min_y[i];
        double y_max = max_y[i];

        fields[i].push_back(Eigen::Vector3d(x_max, y_max, 0));
        fields[i].push_back(Eigen::Vector3d(x_max, y_min, 0));
        fields[i].push_back(Eigen::Vector3d(x_min, y_min, 0));
        fields[i].push_back(Eigen::Vector3d(x_min, y_max, 0));

        spdlog::info("[SAFETY] field[{}] set: x[{:.3f},{:.3f}] y[{:.3f},{:.3f}]", i, x_min, x_max, y_min, y_max);
    }
}

void SAFETY::open()
{
    safety_flag = true;
    safety_thread = std::make_unique<std::thread>(&SAFETY::safety_loop, this);
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

void SAFETY::set_obsmap_module(OBSMAP* _obsmap)
{
    obsmap = _obsmap;
}

cv::Mat SAFETY::get_safety_map()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return safety_map;
}

std::vector<int> SAFETY::get_field_collision()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return field_collision;
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

    const size_t num_fields = fields.size();

    spdlog::info("[SAFETY] safety_loop start");
    while(safety_flag)
    {
        cv::Mat static_map = obsmap->get_static_map();
        if(static_map.empty())
        {
            continue;
        }

        cv::Mat _safety_map;
        cv::cvtColor(static_map, _safety_map, cv::COLOR_GRAY2BGR);

        // clear
        std::vector<int> collision_flags;
        collision_flags.resize(num_fields);
        for(size_t j = 0; j < collision_flags.size(); j++)
        {
            collision_flags[j] = 0;
        }

        Eigen::Vector3d cur_vel = mobile->get_pose().vel;
        std::vector<Eigen::Matrix4d> traj = calc_trajectory(cur_vel, 1, 5, Eigen::Matrix4d::Identity());

        for(size_t i = 0; i < traj.size(); i++)
        {
            const Eigen::Matrix4d &G = traj[i];

            // drawing field traj
            for(size_t j = 0; j < num_fields; j++)
            {
                std::vector<cv::Point> field_pixel;
                for(int k = 0; k < 4; k++)
                {
                    Eigen::Vector3d pt = fields[j][k];
                    Eigen::Vector3d _pt = G.block(0,0,3,3)*pt + G.block(0,3,3,1);

                    cv::Vec2i uv = obsmap->xy_uv(_pt[0], _pt[1]);

                    field_pixel.push_back(cv::Point(uv[0], uv[1]));
                }
                cv::polylines(_safety_map, field_pixel, true, cv::Scalar(0,255,255), 1, cv::LINE_8);

                // collision check
                bool is_collision = false;
                {
                    std::vector<cv::Point> poly;
                    poly = field_pixel;

                    cv::Mat field_mask(_safety_map.rows, _safety_map.cols, CV_8U, cv::Scalar(0));
                    cv::fillPoly(field_mask, poly, cv::Scalar(255));

                    cv::Mat mask;
                    cv::bitwise_and(field_mask, static_map, mask);

                    if(cv::countNonZero(mask) > 0)
                    {
                        is_collision = true;

                        cv::Mat red(_safety_map.size(), _safety_map.type(), cv::Scalar(0,0,255));
                        red.copyTo(_safety_map, mask);
                    }
                }
                if(is_collision)
                {
                    collision_flags[j] = 1;
                }
            }

            // drawing robot traj
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

            std::vector<cv::Point> robot_pixel;
            robot_pixel.push_back(cv::Point(uv0[0], uv0[1]));
            robot_pixel.push_back(cv::Point(uv1[0], uv1[1]));
            robot_pixel.push_back(cv::Point(uv2[0], uv2[1]));
            robot_pixel.push_back(cv::Point(uv3[0], uv3[1]));

            cv::polylines(_safety_map, robot_pixel, true, cv::Scalar(0,255,0), 1, cv::LINE_8);
        }

        obsmap->draw_robot_outline(_safety_map);

        // update
        {
            std::unique_lock<std::shared_mutex> lock(mtx);
            safety_map = _safety_map.clone();

            field_collision.clear();
            for(size_t i = 0; i < num_fields; i++)
            {
                if(collision_flags[i] == 1)
                {
                    field_collision.push_back((int)i);
                }
            }
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
