#include "sim.h"

SIM::SIM(QObject *parent)
    : QObject{parent}
{
    cur_tf = Eigen::Matrix4d::Identity();
}

SIM::~SIM()
{
    if(a_thread != NULL)
    {
        a_flag = false;
        a_thread->join();
        a_thread = NULL;
    }
}

void SIM::start()
{
    if(a_thread == NULL)
    {
        a_flag = true;
        a_thread = new std::thread(&SIM::a_loop, this);
    }
}

void SIM::stop()
{
    if(a_thread != NULL)
    {
        a_flag = false;
        a_thread->join();
        a_thread = NULL;
    }
}

void SIM::a_loop()
{
    const double dt = 0.01; // 50hz
    double pre_loop_time = get_time();

    double vx0 = 0;
    double vy0 = 0;
    double wz0 = 0;

    double last_lidar_t = 0;

    printf("[SIM] a_loop start\n");
    while(a_flag)
    {
        double sim_t = get_time();

        // get input
        double vx1 = saturation(mobile->vx0, -config->MOTOR_LIMIT_V, config->MOTOR_LIMIT_V);
        double vy1 = saturation(mobile->vy0, -config->MOTOR_LIMIT_V, config->MOTOR_LIMIT_V);
        double wz1 = saturation(mobile->wz0, -config->MOTOR_LIMIT_W*D2R, config->MOTOR_LIMIT_W*D2R);

        // calc vx
        double vx = 0;
        if(vx1-vx0 > 0.0001)
        {
            double dvx = config->MOTOR_LIMIT_V_ACC*dt;
            vx = vx0 + dvx;
            if(vx > vx1)
            {
                vx = vx1;
            }
        }
        else if(vx1-vx0 < -0.0001)
        {
            double dvx = -config->MOTOR_LIMIT_V_ACC*dt;
            vx = vx0 + dvx;
            if(vx < vx1)
            {
                vx = vx1;
            }
        }
        else
        {
            vx = vx1;
        }

        // calc vy
        double vy = 0;
        if(vy1-vy0 > 0.0001)
        {
            double dvy = config->MOTOR_LIMIT_V_ACC*dt;
            vy = vy0 + dvy;
            if(vy > vy1)
            {
                vy = vy1;
            }
        }
        else if(vy1-vy0 < -0.0001)
        {
            double dvy = -config->MOTOR_LIMIT_V_ACC*dt;
            vy = vy0 + dvy;
            if(vy < vy1)
            {
                vy = vy1;
            }
        }
        else
        {
            vy = vy1;
        }

        // calc wz
        double wz = 0;
        if(wz1-wz0 > 0.0001)
        {
            double dwz = (config->MOTOR_LIMIT_W_ACC*D2R)*dt;
            wz = wz0 + dwz;
            if(wz > wz1)
            {
                wz = wz1;
            }
        }
        else if(wz1-wz0 < -0.0001)
        {
            double dwz = -(config->MOTOR_LIMIT_W_ACC*D2R)*dt;
            wz = wz0 + dwz;
            if(wz < wz1)
            {
                wz = wz1;
            }
        }
        else
        {
            wz = wz1;
        }

        // calc delta pose
        double dth = wz*dt;
        double dx = vx*std::cos(dth)*dt - vy*std::sin(dth)*dt;
        double dy = vx*std::sin(dth)*dt + vy*std::cos(dth)*dt;
        Eigen::Matrix4d dG = se2_to_TF(Eigen::Vector3d(dx, dy, dth));

        // update tf
        cur_tf = cur_tf*dG;
        Eigen::Vector3d cur_xi = TF_to_se2(cur_tf);

        // update vel
        vx0 = vx;
        vy0 = vy;
        wz0 = wz;

        // set pose
        MOBILE_POSE pose;
        pose.t = sim_t;
        pose.pose = cur_xi;
        pose.vel = Eigen::Vector3d(vx, vy, wz);

        MOBILE_STATUS status;
        #if defined (USE_SRV) || defined (USE_AMR_400) || (USE_AMR_400_LAKI)
        status.t = sim_t;
        status.connection_m0 = 1;
        status.connection_m1 = 1;
        status.status_m0 = 1;
        status.status_m1 = 1;
        status.charge_state = 0;
        status.emo_state = 1;
        status.power_state = 1;
        status.remote_state = 1;
        #endif

        // storing
        mobile->mtx.lock();
        mobile->cur_pose = pose;
        mobile->cur_status = status;
        mobile->mtx.unlock();

        if(unimap->is_loaded && sim_t - last_lidar_t > 0.1)
        {
            // generate lidar frame
            double query_pt[3] = {cur_xi[0], cur_xi[1], 0};
            double sq_radius = config->LIDAR_MAX_RANGE*config->LIDAR_MAX_RANGE;
            std::vector<nanoflann::ResultItem<unsigned int, double>> res_idxs;
            nanoflann::SearchParameters params;
            unimap->kdtree_index->radiusSearch(&query_pt[0], sq_radius, res_idxs, params);

            Eigen::Matrix4d cur_tf_inv = cur_tf.inverse();
            std::vector<double> reflects;
            std::vector<Eigen::Vector3d> pts;            
            for(size_t p = 0; p < res_idxs.size(); p+=10)
            {
                unsigned int idx = res_idxs[p].first;
                double x = unimap->kdtree_cloud.pts[idx].x;
                double y = unimap->kdtree_cloud.pts[idx].y;
                double z = unimap->kdtree_cloud.pts[idx].z;
                double r = unimap->kdtree_cloud.pts[idx].r;

                Eigen::Vector3d P(x,y,z);
                Eigen::Vector3d _P = cur_tf_inv.block(0,0,3,3)*P + cur_tf_inv.block(0,3,3,1);
                pts.push_back(_P);
                reflects.push_back(r);

                if(pts.size() >= 360)
                {
                    break;
                }
            }

            // update
            FRAME frm;
            frm.t = sim_t;
            frm.mo = pose;
            frm.pts = pts;
            frm.reflects = reflects;

            lidar->mtx.lock();
            lidar->scan_que.push(frm);
            if(lidar->scan_que.unsafe_size() > 50)
            {
                FRAME dummy;
                lidar->scan_que.try_pop(dummy);
            }

            lidar->cur_scan = pts;
            lidar->cur_scan_f = pts;
            lidar->cur_scan_b = pts;
            lidar->mtx.unlock();

            last_lidar_t = sim_t;
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
            printf("[SIM] a_loop loop time drift, dt:%f\n", delta_loop_time);
        }
        pre_loop_time = get_time();
    }
    printf("[SIM] a_loop stop\n");
}
