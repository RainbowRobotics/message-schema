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

void DOCKINGCONTROL::move(Eigen::Matrix4d goal_tf)
{
    // stop first
    stop();

    // obs clear
    obsmap->clear();

    // start control loop
    a_flag = true;
    a_thread = new std::thread(&DOCKINGCONTROL::a_loop, this, goal_tf);
}

void DOCKINGCONTROL::a_loop(Eigen::Matrix4d goal_tf)
{
    // set flag
    is_moving = true;

    // global goal
    Eigen::Vector3d goal_pos = goal_tf.block(0,3,3,1);
    Eigen::Vector3d goal_xi = TF_to_se2(goal_tf);

    // check goal
    Eigen::Vector2d dtdr = dTdR(slam->get_cur_tf(), goal_tf);
    if(dtdr[0] > config->DRIVE_GOAL_D)
    {
        if(std::abs(dtdr[1]) > config->DRIVE_GOAL_TH*D2R)
        {
            // robot is not in goal
            mobile->move(0, 0, 0);
            is_moving = false;

            logger->PrintLog("[DOCKING] not in goal.", "Red", true, false);
            return;
        }
    }

    if(code->is_connected == false)
    {
        logger->PrintLog("[DOCKING] code reader not connected.", "Red", true, false);
        return;
    }

    // loop params
    const double dt = 0.05; // 20hz
    double pre_loop_time = get_time();

    // control params
    double extend_dt = 0;

    // for obs
    double obs_wait_st_time = 0;

    double docking_goal_d = 0.01;
    double docking_goal_th = 0.5;

    double docking_limit_w_acc = 0.5;

    double docking_limit_v = 0.05;
    double docking_limit_w = 1.0;

    double drive_t = 0.15;

    double pre_err_th = 0;

    printf("[DOCKING] a_loop start\n");
    while(a_flag)
    {
        if(is_pause)
        {
            mobile->move(0, 0, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        if(is_everything_fine() == false)
        {
            mobile->move(0, 0, 0);
            is_moving = false;

            printf("[DOCKING] something wrong\n");
            break;
        }

        // get current status
        Eigen::Vector3d cur_vel(mobile->vx0, mobile->vy0, mobile->wz0);
        Eigen::Matrix4d cur_tf = slam->get_cur_tf();
        Eigen::Vector3d cur_xi = TF_to_se2(cur_tf);

        // code reader error
        double code_err_x = code->err_x;
        double code_err_y = code->err_y;

        if(fsm_state == DOCKING_FSM_DRIVING)
        {
            if(obsmap->is_tf_collision(cur_tf, 0.1, 0.1))
            {
                mobile->move(0,0,0);
                fsm_state = DOCKING_FSM_OBS;

                printf("[DOCKING] OBS_WAIT -> DRIVING\n");
                continue;
            }

            // calc error and ref vel
            Eigen::Matrix4d cur_tf_inv = cur_tf.inverse();
            Eigen::Vector3d tgt_pos = cur_tf_inv.block(0,0,3,3)*goal_pos + cur_tf_inv.block(0,3,3,1);
            double tgt_th = goal_xi[2];

            double err_x = tgt_pos[0];
            double err_y = tgt_pos[1];
            double err_th = deltaRad(tgt_th, cur_xi[2]);

            double err_d = std::sqrt(err_x*err_x + err_y*err_y);
            double dir_x = err_x/err_d;
            double dir_y = err_y/err_d;
            double ref_v = 0.05;

            //double goal_err_d = calc_dist_2d(goal_pos - cur_pos);
            //double goal_err_th = deltaRad(goal_xi[2], cur_xi[2]);
            double goal_err_d = std::sqrt(code_err_x*code_err_x + code_err_y*code_err_y);
            double goal_err_th = code->err_th;
            double goal_v = goal_err_d*0.3;

            // calc control input
            double ref_v0 = std::sqrt(cur_vel[0]*cur_vel[0] + cur_vel[1]*cur_vel[1]);
            ref_v = saturation(ref_v, ref_v0 - 0.05*dt, ref_v0 + 0.05*dt);
            ref_v = saturation(ref_v, 0, goal_v);

            double kp_w = 2.0;
            double kd_w = 0.1;

            double vx = dir_x*ref_v;
            double vy = dir_y*ref_v;
            double wz = kp_w*err_th + kd_w*(err_th - pre_err_th)/dt;
            pre_err_th = err_th;

            wz = saturation(wz, cur_vel[2] - (docking_limit_w_acc*D2R)*dt, cur_vel[2] + (docking_limit_w_acc*D2R)*dt);
            wz = saturation(wz, -docking_limit_w*D2R, docking_limit_w*D2R);

            // scaling
            double scale_v = 1.0 - drive_t*std::abs(wz/(docking_limit_w*D2R));
            double scale_w = 1.0 - drive_t*std::abs(ref_v/docking_limit_v);

            vx *= scale_v;
            vy *= scale_v;
            wz *= scale_w;

            // goal check
            if(std::abs(goal_err_d) < docking_goal_d && std::abs(goal_err_th) < docking_goal_th*D2R)
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

bool DOCKINGCONTROL::is_everything_fine()
{
    if(code->is_recv_data == false)
    {
        return false;
    }
    if(slam->get_cur_loc_state() == "none" || slam->get_cur_loc_state() == "fail")
    {
        return false;
    }

    return true;
}
