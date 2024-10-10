#include "dockingcontrol.h"

DOCKINGCONTROL::DOCKINGCONTROL(QObject *parent)
    : QObject{parent}
{
    last_cur_pos.setZero();
}

DOCKINGCONTROL::~DOCKINGCONTROL()
{
    // control loop stop
    if(a_thread != NULL)
    {
        a_flag = false;
        a_thread->join();
        a_thread = NULL;
    }
    mobile->move(0, 0, 0);
    is_moving = false;
}

void DOCKINGCONTROL::init()
{

}

void DOCKINGCONTROL::stop()
{
    // control loop stop
    if(a_thread != NULL)
    {
        a_flag = false;
        a_thread->join();
        a_thread = NULL;
    }
    mobile->move(0, 0, 0);
    is_moving = false;
    is_pause = false;
}

void DOCKINGCONTROL::move(Eigen::Matrix4d goal_tf, double p_gain, double d_gain, double off_x, double off_y, double off_t)
{
    // stop first
    stop();

    // obs clear
    obsmap->clear();

    ox = off_x;
    oy = off_y;
    ot = off_t;

    p_gain_ratio = std::clamp(p_gain, 0.0, 2.0);
    d_gain_ratio = std::clamp(d_gain, 0.0, 2.0);

    // start control loop
    a_flag = true;
    a_thread = new std::thread(&DOCKINGCONTROL::a_loop, this, goal_tf);
}

void DOCKINGCONTROL::a_loop(Eigen::Matrix4d goal_tf)
{
    // set flag
    is_moving = true;
    if(code->is_connected == false)
    {
        logger->write_log("[DOCKING] code reader not connected.", "Red", true, false);
        return;
    }

    if(code->is_recv_data == false)
    {
        logger->write_log("[DOCKING] code not detected.", "Red", true, false);
        return;
    }

    // loop params
    const double dt = 0.01; // 100hz
    double pre_loop_time = get_time();

    // control params
    double extend_dt = 0;

    // for obs
    double obs_wait_st_time = 0;

    double pre_err_th = 0;
    double pre_goal_err_d = 0;

    fsm_state = DOCKING_FSM_DRIVING;
    printf("[DOCKING] a_loop start\n");
    while(a_flag)
    {
        if(is_pause)
        {
            printf("[DOCKING] PAUSE\n");
            mobile->move(0, 0, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        int is_good_everything = is_everything_fine();
        if(is_good_everything == DRIVING_FAILED)
        {
            mobile->move(0, 0, 0);
            is_moving = false;

            printf("[DOCKING] something wrong (failed)\n");
            break;
        }

        else if(is_good_everything == DRIVING_NOT_READY)
        {
            mobile->move(0, 0, 0);
            is_moving = false;

            printf("[DOCKING] something wrong (not ready)\n");
            break;
        }

        // get current status
        Eigen::Vector3d cur_vel(mobile->vx0, mobile->vy0, mobile->wz0);
        Eigen::Matrix4d cur_tf = slam->get_cur_tf();

        // code reader error
        double code_err_x = code->err_x - ox * 0.001;
        double code_err_y = code->err_y - oy * 0.001;

        if(fsm_state == DOCKING_FSM_DRIVING)
        {
            if(obsmap->is_tf_collision(cur_tf, 0.1, 0.1))
            {
                obs_wait_st_time = get_time();
                mobile->move(0,0,0);
                fsm_state = DOCKING_FSM_OBS;

                printf("[DOCKING] DRIVING -> OBS_WAIT\n");
                continue;
            }

            // calc error and ref vel
            double goal_err_d = std::sqrt(code_err_x*code_err_x + code_err_y*code_err_y);
            double goal_err_th = code->err_th - ot * D2R;

            double dir_x = code_err_x/goal_err_d;
            double dir_y = code_err_y/goal_err_d;
            double ref_v = p_gain_ratio*config->DOCKING_GAIN_KP*goal_err_d + d_gain_ratio*config->DOCKING_GAIN_KD*(goal_err_d- pre_goal_err_d)/dt;

            double kp_w = 2.0;
            double kd_w = 0.5;

            kp_w *= p_gain_ratio;
            kd_w *= d_gain_ratio;

            double vx = dir_x*ref_v;
            double vy = dir_y*ref_v;
            double wz = kp_w*goal_err_th + kd_w*(deltaRad(goal_err_th,pre_err_th))/dt;
            pre_err_th = goal_err_th;

            double ref_v0 = std::sqrt(cur_vel[0]*cur_vel[0] + cur_vel[1]*cur_vel[1]);
            ref_v = saturation(ref_v, ref_v0 - 0.5*dt, ref_v0 + 0.5*dt);

            wz = saturation(wz, cur_vel[2] - (config->DOCKING_LIMIT_W_ACC*D2R)*dt, cur_vel[2] + (config->DOCKING_LIMIT_W_ACC*D2R)*dt);
            wz = saturation(wz, -config->DOCKING_LIMIT_W*D2R, config->DOCKING_LIMIT_W*D2R);

            // scaling
            double scale_v = 1.0 - config->DOCKING_DRIVE_T*std::abs(wz/(config->DOCKING_LIMIT_W*D2R));
            double scale_w = 1.0 - config->DOCKING_DRIVE_T*std::abs(ref_v/config->DOCKING_LIMIT_V);

            vx *= scale_v;
            vy *= scale_v;
            wz *= scale_w;

            // goal check
            if(std::abs(goal_err_d) < config->DOCKING_GOAL_D && std::abs(goal_err_th) < config->DOCKING_GOAL_TH*D2R)
            {
                extend_dt += dt;
                if(extend_dt > config->DOCKING_EXTENDED_CONTROL_TIME)
                {
                    mobile->move(0, 0, 0);
                    is_moving = false;

                    fsm_state = DOCKING_FSM_COMPLETE;
                    printf("[DOCKING] DOCKING COMPLETE, goal_err: %.3f, %.3f\n", goal_err_d, goal_err_th*R2D);
                    return;
                }
            }

            mobile->move(vx, vy, wz);

            pre_goal_err_d = goal_err_d;
        }

        else if(fsm_state == DOCKING_FSM_OBS)
        {
            mobile->move(0,0,0);
            if(get_time() - obs_wait_st_time > 1.0)
            {
                pre_err_th = 0;
                fsm_state = DOCKING_FSM_DRIVING;
                printf("[DOCKING] OBS_WAIT -> DRIVING\n");
                continue;
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
            printf("[AUTO] loop time drift, dt:%f\n", delta_loop_time);
        }
        pre_loop_time = get_time();
    }
}

// for local path planning
std::vector<Eigen::Matrix4d> DOCKINGCONTROL::calc_trajectory(Eigen::Vector3d cur_vel, double dt, double predict_t, Eigen::Matrix4d G0)
{
    std::vector<Eigen::Matrix4d> res;

    float vx=cur_vel[0];
    float vy=cur_vel[1];
    float wz=cur_vel[2];

    double pre_x = 0;
    double pre_y = 0;
    double pre_th = 0;
    for(double t = dt; t <= predict_t; t += dt) // maybe relexation obs stuck condition..?
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
        Eigen::Matrix4d predict_G = G0*G;
        res.push_back(predict_G);
    }

    res.erase(res.begin());
    return res;
}

// check condition
int DOCKINGCONTROL::is_everything_fine()
{
    if(code->is_connected == false)
    {
        logger->write_log("[DOCKING] failed (not connected code)", "Red", true, false);
        return DRIVING_FAILED;
    }

    if(code->is_recv_data == false)
    {
        logger->write_log("[DOCKING] failed (not received code)", "Red", true, false);
        return DRIVING_FAILED;
    }

    MOBILE_STATUS ms = mobile->get_status();
    if(ms.connection_m0 != 1 || ms.connection_m1 != 1)
    {
        logger->write_log("[DOCKING] failed (motor not connected)", "Red", true, false);
        return DRIVING_FAILED;
    }

    if(ms.status_m0 > 1 || ms.status_m1 > 1)
    {
        int motor_err_code = ms.status_m0 > 1 ? ms.status_m0 : ms.status_m1;
        if(motor_err_code == MOTOR_ERR_MOD)
        {
            logger->write_log("[DOCKING] failed (motor error MOD, 2)", "Red", true, false);
        }
        else if(motor_err_code == MOTOR_ERR_JAM)
        {
            logger->write_log("[DOCKING] failed (motor error JAM, 4)", "Red", true, false);
        }
        else if(motor_err_code == MOTOR_ERR_CUR)
        {
            logger->write_log("[DOCKING] failed (motor error CUR, 8)", "Red", true, false);
        }
        else if(motor_err_code == MOTOR_ERR_BIG)
        {
            logger->write_log("[DOCKING] failed (motor error BIG, 16)", "Red", true, false);
        }
        else if(motor_err_code == MOTOR_ERR_IN)
        {
            logger->write_log("[DOCKING] failed (motor error IN, 32)", "Red", true, false);
        }
        else if(motor_err_code == MOTOR_ERR_PSI)
        {
            logger->write_log("[DOCKING] failed (motor error:PS1|2, 64)", "Red", true, false);
        }
        else if(motor_err_code == MOTOR_ERR_NON)
        {
            logger->write_log("[DOCKING] failed (motor error NON, 128)", "Red", true, false);
        }
        return DRIVING_FAILED;
    }

    if(ms.charge_state == 1)
    {
        logger->write_log("[DOCKING] failed (robot charging)", "Red", true, false);
        return DRIVING_FAILED;
    }

    if(ms.emo_state == 0)
    {
        logger->write_log("[DOCKING] not ready (emo pushed)", "Orange", true, false);
        return DRIVING_NOT_READY;
    }

    if(ms.status_m0 == 0 && ms.status_m1 == 0)
    {
        logger->write_log("[DOCKING] not ready (motor lock offed)", "Orange", true, false);
        return DRIVING_NOT_READY;
    }

    return DRIVING_FINE;
}
