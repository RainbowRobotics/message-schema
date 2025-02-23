#include "dockcontrol.h"

DOCKCONTROL::DOCKCONTROL(QObject *parent)
    : QObject{parent}
{
}

DOCKCONTROL::~DOCKCONTROL()
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

void DOCKCONTROL::init()
{

}

void DOCKCONTROL::stop()
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

void DOCKCONTROL::move(double p_gain, double d_gain, double off_x, double off_y, double off_t)
{
    // stop first
    stop();

    // load preset
    params = load_preset(0);

    ox = std::clamp(off_x, -0.03, 0.03);
    oy = std::clamp(off_y, -0.03, 0.03);
    ot = off_t*D2R;

    bqr->offset_x = ox;
    bqr->offset_y = oy;
    bqr->offset_th = ot;

    p_gain_ratio = std::clamp(p_gain, 0.0, 2.0);
    d_gain_ratio = std::clamp(d_gain, 0.0, 2.0);

    // start control loop
    a_flag = true;
    a_thread = new std::thread(&DOCKCONTROL::a_loop, this);
}

void DOCKCONTROL::a_loop()
{
    // set flag
    if(bqr->is_connected == false)
    {
        is_moving = false;
        logger->write_log("[DOCKING] code reader not connected.", "Red", true, false);
        return;
    }

    if(bqr->is_recv_data == false)
    {
        is_moving = false;
        logger->write_log("[DOCKING] code not detected.", "Red", true, false);
        return;
    }

    is_moving = true;

    // loop params
    const double dt = 0.05; // 20hz
    double pre_loop_time = get_time();

    // control params
    double extend_dt = 0;

    double pre_err_th = 0;
    double pre_goal_err_d = 0;
    fsm_state = DOCK_FSM_DRIVING;

    logger->write_log("[DOCK] start a_loop", "Green");
    while(a_flag)
    {
        if(is_pause)
        {
            logger->write_log("[DOCK] pause", "Green");
            mobile->move(0, 0, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        bool is_station_docking = (fsm_state != DOCK_FSM_DRIVING_FOR_CHRGE) && (fsm_state != DOCK_FSM_WAIT_FOR_CHRGE);
        if(is_everything_fine() == false && (is_station_docking == true))
        {
            mobile->move(0, 0, 0);

            is_moving = false;
            fsm_state = DOCK_FSM_FAILED;
            logger->write_log("[DOCK] something wrong. docking fail", "Red");
            return;
        }

        // code reader error
        double code_err_x = bqr->err_x;
        double code_err_y = bqr->err_y;
        double code_err_th = bqr->err_th;

        if(fsm_state == DOCK_FSM_DRIVING)
        {
            // gain ratio
            double kp = p_gain_ratio;
            double kd = d_gain_ratio;

            // calc error and ref vel
            double goal_err_d = std::sqrt(code_err_x*code_err_x + code_err_y*code_err_y) + 0.000001;
            double goal_err_th = code_err_th;

            double dir_x = code_err_x/goal_err_d;
            double dir_y = code_err_y/goal_err_d;
            double ref_v = kp*params.GAIN_P*goal_err_d + kd*params.GAIN_D*(goal_err_d - pre_goal_err_d)/dt;
            ref_v = saturation(ref_v, -params.LIMIT_V, params.LIMIT_V);

            double vx = dir_x*ref_v;
            double vy = dir_y*ref_v;

            double wz = kp*params.GAIN_P*goal_err_th + kd*params.GAIN_D*(deltaRad(goal_err_th, pre_err_th))/dt;
            wz = saturation(wz, -params.LIMIT_W*D2R, params.LIMIT_W*D2R);

            pre_err_th = goal_err_th;

            // goal check
            if(std::abs(goal_err_d) < config->DOCKING_GOAL_D && std::abs(goal_err_th) < config->DOCKING_GOAL_TH*D2R)
            {
                extend_dt += dt;
                if(extend_dt > config->DOCK_EXTENDED_CONTROL_TIME)
                {
                    mobile->move(0, 0, 0);
                    if(docking_node_type == "GOAL")
                    {
                        is_moving = false;
                        fsm_state = DOCK_FSM_COMPLETE;

                        QString str;
                        str.sprintf("[DOCK] COMPLETE, goal_err: %.3f, %.3f\n", goal_err_d, goal_err_th*R2D);
                        logger->write_log(str, "Green");
                        return;
                    }
                    else if(docking_node_type == "STATION")
                    {
                        fsm_state = DOCK_FSM_DRIVING_FOR_CHRGE;

                        QString str;
                        str.sprintf("[DOCK] COMPLETE -> MOVE FOR CHRGE, goal_err: %.3f, %.3f\n", goal_err_d, goal_err_th*R2D);
                        logger->write_log(str, "Green");
                        continue;
                    }
                }
            }

            mobile->move(vx, vy, wz);
            pre_goal_err_d = goal_err_d;
            {
                //std::cout << "vx: " << vx << ", vy: " << vy << ", w:" << wz*R2D <<
                //            ", goal_err_d: " << goal_err_d << ", code_err_th:" << code_err_th*R2D << ", goal_err_th:" << goal_err_th*R2D << std::endl;
            }
        }
        else if(fsm_state == DOCK_FSM_DRIVING_FOR_CHRGE)
        {
            double t = station_linear_move_d / (station_linear_move_v + 1e-6);
            mobile->move_linear(station_linear_move_d, station_linear_move_v);

            fsm_state = DOCK_FSM_WAIT_FOR_CHRGE;
            int wait_t = int((t+1.5)*1000);
            QString str;
            str.sprintf("[DOCK] DRIVING_FOR_CHRGE -> WAIT_FOR_CHRGE, wait_time: %.3f (s)", wait_t);
            logger->write_log(str, "Green");

            std::this_thread::sleep_for(std::chrono::milliseconds(wait_t));
            continue;
        }
        else if(fsm_state == DOCK_FSM_WAIT_FOR_CHRGE)
        {
            MOBILE_STATUS ms = mobile->get_status();
            if((int)ms.charge_state == CHARGING_STATION_CHARGING)
            {
                is_moving = false;
                fsm_state = DOCK_FSM_COMPLETE;
                logger->write_log("[DOCK] DOCKING & CHARGING COMPLETE", "Green");
                return;
            }
            else
            {
                is_moving = false;
                fsm_state = DOCK_FSM_FAILED;
                logger->write_log("[DOCK] CHARGING FAILED. check this", "Red");
                return;
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
            printf("[DOCK] loop time drift, dt:%f\n", delta_loop_time);
        }
        pre_loop_time = get_time();
    }

    mobile->move(0, 0, 0);
    is_moving = false;
    fsm_state = DOCK_FSM_COMPLETE;

    logger->write_log("[DOCK] stop a_loop", "Green");
}

// for local path planning
std::vector<Eigen::Matrix4d> DOCKCONTROL::calc_trajectory(Eigen::Vector3d cur_vel, double dt, double predict_t, Eigen::Matrix4d G0)
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

bool DOCKCONTROL::is_everything_fine()
{
    if(bqr->is_recv_data == false)
    {
        return false;
    }

    MOBILE_STATUS mobile_status = mobile->get_status();
    if(mobile_status.charge_state == 1)
    {
        logger->write_log("[DOCK] dock failed (charging)", "Red", true, false);
        return false;
    }

    if(mobile_status.status_m0 > 1 || mobile_status.status_m1 > 1 || mobile_status.status_m2 > 1 || mobile_status.status_m3 > 1)
    {
        logger->write_log("[DOCK] dock failed (motor error)", "Red", true, false);
        return false;
    }

    if(mobile_status.connection_m0 != 1 || mobile_status.connection_m1 != 1)
    {
        logger->write_log("[DOCK] dock failed (motor not connected)", "Red", true, false);
        return false;
    }

    if(mobile_status.motor_stop_state == 0)
    {
        logger->write_log("[DOCK] dock failed (emo pushed)", "Red", true, false);
        return false;
    }

    if(mobile_status.status_m0 == 0 || mobile_status.status_m1 == 0 || mobile_status.status_m2 == 0 || mobile_status.status_m3 == 0)
    {
        logger->write_log("[DOCK] dock failed (motor lock offed)", "Red", true, false);
        return false;
    }

    return true;
}

void DOCKCONTROL::slot_check_docking()
{
    //printf("[DOCKING] slot check docking\n");

    //if(node_type == "GOAL" || node_type == "INIT")
    //{
    //    docking_node = "GOAL";
    //}
    //else if(node_type == "STATION")
    //{
    //    docking_node = "STATION";
    //}
    //else
    //{
    //    printf("[DOCKING] not support node type.\n");
    //    return;
    //}

    //double offset_x = offset_val[0];
    //double offset_y = offset_val[1];
    //double offset_th = offset_val[2];

    //move(1, 0, offset_x, offset_y, offset_th);
}

DCTRL_PARAM DOCKCONTROL::load_preset(int preset)
{
    DCTRL_PARAM res;

    // read
    QString preset_path = "";

    // config module init
    #ifdef USE_SRV
    preset_path = QCoreApplication::applicationDirPath() + "/config/SRV/" + "preset_" + QString::number(preset) + ".json";
    #endif

    #ifdef USE_AMR_400
    preset_path = QCoreApplication::applicationDirPath() + "/config/AMR_400/" + "preset_" + QString::number(preset) + ".json";
    #endif

    #ifdef USE_AMR_400_LAKI
    preset_path = QCoreApplication::applicationDirPath() + "/config/AMR_400_LAKI/" + "preset_" + QString::number(preset) + ".json";
    #endif

    #ifdef USE_MECANUM_OLD
    preset_path = QCoreApplication::applicationDirPath() + "/preset/" + "preset_" + QString::number(preset) + ".json";
    #endif

    #ifdef USE_MECANUM
    preset_path = QCoreApplication::applicationDirPath() + "/config/MECANUM/" + "preset_" + QString::number(preset) + ".json";
    #endif

    QFileInfo info(preset_path);
    if(info.exists() && info.isFile())
    {
        QFile file(preset_path);
        if(file.open(QIODevice::ReadOnly))
        {
            QByteArray data = file.readAll();
            QJsonDocument doc = QJsonDocument::fromJson(data);
            QJsonObject obj = doc.object();

            // param load
            printf("[PRESET] %s load\n", preset_path.toLocal8Bit().data());

            res.GAIN_P = obj["GAIN_P"].toString().toDouble();
            printf("[PRESET] GAIN_P :%f\n", res.GAIN_P);

            res.GAIN_D = obj["GAIN_D"].toString().toDouble();
            printf("[PRESET] GAIN_D :%f\n", res.GAIN_D);

            res.LIMIT_V = obj["LIMIT_V"].toString().toDouble();
            printf("[PRESET] LIMIT_V :%f\n", res.LIMIT_V);

            res.LIMIT_W = obj["LIMIT_W"].toString().toDouble();
            printf("[PRESET] LIMIT_W :%f\n", res.LIMIT_W);

            res.LIMIT_V_ACC = obj["LIMIT_V_ACC"].toString().toDouble();
            printf("[PRESET] LIMIT_V_ACC :%f\n", res.LIMIT_V_ACC);

            res.LIMIT_W_ACC = obj["LIMIT_W_ACC"].toString().toDouble();
            printf("[PRESET] LIMIT_W_ACC :%f\n", res.LIMIT_W_ACC);

            file.close();
        }
    }
    else
    {
        printf("[DOCK] invalid preset path\n");
    }

    return res;
}

void DOCKCONTROL::set_node_type(QString type)
{
    mtx.lock();
    docking_node_type = type;
    mtx.unlock();
}

void DOCKCONTROL::set_dock_offset(Eigen::Vector3d offset_val)
{
    mtx.lock();
    ox = offset_val[0];
    oy = offset_val[1];
    ot = offset_val[2];
    mtx.unlock();
}
