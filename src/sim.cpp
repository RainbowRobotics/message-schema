#include "sim.h"

SIM* SIM::instance(QObject* parent)
{
    static SIM* inst = nullptr;
    if(!inst && parent)
    {
        inst = new SIM(parent);
    }
    else if(inst && parent && inst->parent() == nullptr)
    {
        inst->setParent(parent);
    }
    return inst;
}

SIM::SIM(QObject *parent) : QObject{parent},
    config(nullptr),
    logger(nullptr),
    mobile(nullptr),
    unimap(nullptr),
    lidar_2d(nullptr),
    lidar_3d(nullptr),
    loc(nullptr)
{
    cur_tf = Eigen::Matrix4d::Identity();
}

SIM::~SIM()
{
    simulation_flag = false;
    if(simulation_thread && simulation_thread->joinable())
    {
        simulation_thread->join();
    }
    simulation_thread.reset();
}

void SIM::set_cur_tf(Eigen::Matrix4d tf)
{
    std::lock_guard<std::mutex> lock(mtx);
    cur_tf = tf;
}

Eigen::Matrix4d SIM::get_cur_tf()
{
    std::lock_guard<std::mutex> lock(mtx);
    Eigen::Matrix4d res = cur_tf;
    return res;
}

void SIM::start()
{
    simulation_flag = true;
    simulation_thread = std::make_unique<std::thread>(&SIM::simulation_loop, this);
}

void SIM::stop()
{
    simulation_flag = false;
    if(simulation_thread && simulation_thread->joinable())
    {
        simulation_thread->join();
    }
    simulation_thread.reset();
}

void SIM::simulation_loop()
{
    const double dt = 0.02; // 50hz
    double pre_loop_time = get_time();

    double vx0 = 0;
    double vy0 = 0;
    double wz0 = 0;

    Eigen::Matrix4d _cur_tf = get_cur_tf();

    printf("[SIM] simultation_loop start\n");

    const double motor_limit_v = config->get_motor_limit_v();
    const double motor_limit_w = config->get_motor_limit_w()*D2R;

    const double motor_limit_v_acc = config->get_motor_limit_v_acc();
    const double motor_limit_w_acc = config->get_motor_limit_w_acc()*D2R;

    int cnt = 0;
    while(simulation_flag)
    {
        cnt++;
        double sim_t = get_time();

        Eigen::Vector3d control_input = mobile->get_control_input();

        // get input
        double vx1 = saturation(control_input[0], -motor_limit_v, motor_limit_v);
        double vy1 = saturation(control_input[1], -motor_limit_v, motor_limit_v);
        double wz1 = saturation(control_input[2], -motor_limit_w, motor_limit_w);

        // calc vx, vy, wz
        double vx = calc_limit(vx0, vx1, motor_limit_v_acc, dt);
        double vy = calc_limit(vy0, vy1, motor_limit_v_acc, dt);
        double wz = calc_limit(wz0, wz1, motor_limit_w_acc, dt);

        // calc delta pose
        double dth = wz*dt;
        double dx = vx*std::cos(dth)*dt - vy*std::sin(dth)*dt;
        double dy = vx*std::sin(dth)*dt + vy*std::cos(dth)*dt;
        Eigen::Matrix4d dG = se2_to_TF(Eigen::Vector3d(dx, dy, dth));

        // update pose
        _cur_tf = _cur_tf*dG;
        Eigen::Vector3d cur_xi = TF_to_se2(_cur_tf);
        Eigen::Matrix4d cur_tf_inv = _cur_tf.inverse();

        // update vel
        vx0 = vx;
        vy0 = vy;
        wz0 = wz;

        // set pose
        MOBILE_POSE pose;
        pose.t = sim_t;
        pose.pose = cur_xi;

        distance = vx0*dt;

        pose.vel = Eigen::Vector3d(vx, vy, wz);

        MOBILE_STATUS status;        
        status.t = sim_t;
        status.connection_m0 = 1;
        status.connection_m1 = 1;
        status.status_m0 = 1;
        status.status_m1 = 1;
        status.charge_state = 0;
        status.motor_stop_state = 1;
        status.power_state = 1;
        status.remote_state = 1;

        // storing
        mobile->set_cur_pose(pose);
        mobile->set_cur_status(status);

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
            //printf("[SIM] a_loop loop time drift, dt:%f\n", delta_loop_time);
        }
        pre_loop_time = get_time();
    }
    printf("[SIM] simulation_loop stop\n");
}

double SIM::calc_limit(double v0, double v1, double v_acc_limit, double dt, double tol)
{
    if(v1 - v0 > tol)
    {
        double dv = v_acc_limit * dt;
        double v = v0 + dv;
        if(v > v1)
        {
            v = v1;
        }
        return v;
    }
    else if(v1 - v0 < -tol)
    {
        double dv = -v_acc_limit * dt;
        double v = v0 + dv;
        if(v < v1)
        {
            v = v1;
        }
        return v;
    }
    else
    {
        return v1;
    }

    return 0.0;
}

void SIM::set_config_module(CONFIG* _config)
{
    config = _config;
}

void SIM::set_logger_module(LOGGER* _logger)
{
    logger = _logger;
}

void SIM::set_mobile_module(MOBILE* _mobile)
{
    mobile = _mobile;
}

void SIM::set_unimap_module(UNIMAP* _unimap)
{
    unimap = _unimap;
}

void SIM::set_lidar_2d_module(LIDAR_2D *_lidar_2d)
{
    lidar_2d = _lidar_2d;
}

void SIM::set_lidar_3d_module(LIDAR_3D *_lidar_3d)
{
    lidar_3d = _lidar_3d;
}

void SIM::set_localization_module(LOCALIZATION *_loc)
{
    loc = _loc;
}
