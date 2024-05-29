#include "lidar_2d.h"

LIDAR_2D::LIDAR_2D(QObject *parent) : QObject{parent}
{
}

LIDAR_2D::~LIDAR_2D()
{
    if(grab_thread_f != NULL)
    {
        grab_flag_f = false;
        grab_thread_f->join();
        grab_thread_f = NULL;
    }

    if(grab_thread_b != NULL)
    {
        grab_flag_b = false;
        grab_thread_b->join();
        grab_thread_b = NULL;
    }

    if(a_thread != NULL)
    {
        a_flag = false;
        a_thread->join();
        a_thread = NULL;
    }
}

void LIDAR_2D::open(MOBILE *_mobile)
{
    mobile = _mobile;

    // start grab loop
    if(grab_thread_f == NULL)
    {
        grab_flag_f = true;
        grab_thread_f = new std::thread(&LIDAR_2D::grab_loop_f, this);
    }

    if(grab_thread_b == NULL)
    {
        grab_flag_b = true;
        grab_thread_b = new std::thread(&LIDAR_2D::grab_loop_b, this);
    }

    if(a_thread == NULL)
    {
        a_flag = true;
        a_thread = new std::thread(&LIDAR_2D::a_loop, this);
    }
}

void LIDAR_2D::sync_f()
{
    is_sync_f = true;
    printf("[LIDAR] front time sync\n");
}

void LIDAR_2D::sync_b()
{
    is_sync_b = true;
    printf("[LIDAR] back time sync\n");
}

std::vector<Eigen::Vector3d> LIDAR_2D::get_cur_scan_f()
{
    mtx.lock();
    std::vector<Eigen::Vector3d> res = cur_scan_f;
    mtx.unlock();

    return res;
}

std::vector<Eigen::Vector3d> LIDAR_2D::get_cur_scan_b()
{
    mtx.lock();
    std::vector<Eigen::Vector3d> res = cur_scan_b;
    mtx.unlock();

    return res;
}

std::vector<Eigen::Vector3d> LIDAR_2D::get_cur_scan()
{
    mtx.lock();
    std::vector<Eigen::Vector3d> res = cur_scan;
    mtx.unlock();

    return res;
}

void LIDAR_2D::grab_loop_f()
{
    printf("[LIDAR] start grab loop, front\n");

    // init lidar (host pc should be set ip 192.168.2.2)
    std::unique_ptr<LakiBeamUDP> lidar(new LakiBeamUDP("192.168.2.2", "2367", "192.168.2.10", "8888"));
    is_connected_f = true;

    int drop_cnt = 100;
    while(grab_flag_f)
    {
        repark_t temp_pack;
        if(lidar->get_repackedpack(temp_pack))
        {
            // for sim
            if(is_sim)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // initial drop
            if(drop_cnt > 0)
            {
                drop_cnt--;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // time sync
            double pc_t = get_time();
            double lidar_t = temp_pack.dotcloud[0].timestamp * U2S;

            if(is_sync_f)
            {
                is_sync_f = false;
                offset_t_f = pc_t - lidar_t;
                raw_que_f.clear();

                is_synced_f = true;
                printf("[LIDAR] sync, offset_t_f: %f\n", (double)offset_t_f);
            }

            // check lidar, mobile sync
            if(is_synced_f == false || mobile->is_synced == false)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // lidar frame synced ref time
            double t0 = lidar_t + offset_t_f;
            double t1 = (temp_pack.dotcloud[temp_pack.maxdots-1].timestamp * U2S) + offset_t_f;

            // check
            if(mobile->get_pose_storage_size() != MO_STORAGE_NUM)
            {
                // drop
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // wait
            while(t1 > mobile->last_pose_t && grab_flag_f)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            // get mobile poses
            std::vector<MOBILE_POSE> pose_storage = mobile->get_pose_storage();

            // get lidar raw data
            std::vector<double> times;
            std::vector<double> reflects;
            std::vector<Eigen::Vector3d> raw_pts;
            for (int p = 0; p < temp_pack.maxdots; p++)
            {
                double t = temp_pack.dotcloud[p].timestamp*U2S + offset_t_f;
                double deg = (double)temp_pack.dotcloud[p].angle/100; // deg, 0~360
                double dist = ((double)temp_pack.dotcloud[p].distance)/1000; // meter
                double rssi = (double)temp_pack.dotcloud[p].rssi;

                // dist filter
                if(dist < 0.05 || dist > config->LIDAR_MAX_RANGE)
                {
                    continue;
                }

                // angle filter
                if((deg < 45.0 + angle_offset) || (deg > 315.0 - angle_offset))
                {
                    continue;
                }

                double x = dist * std::cos(deg*D2R);
                double y = dist * std::sin(deg*D2R);
                if(!isfinite(x) || !isfinite(y))
                {
                    continue;
                }

                times.push_back(t);
                reflects.push_back(rssi);
                raw_pts.push_back(Eigen::Vector3d(x, y, 0));
            }

            // get boundary mobile pose
            int idx0 = -1;
            for(size_t p = pose_storage.size()-1; p >= 0; p--)
            {
                if(pose_storage[p].t < t0)
                {
                    idx0 = p;
                    break;
                }
            }

            int idx1 = -1;
            for(size_t p = 0; p < pose_storage.size(); p++)
            {
                if(pose_storage[p].t > t1)
                {
                    idx1 = p;
                    break;
                }
            }

            // check
            if(idx0 == -1 || idx1 == -1)
            {
                // drop
                printf("[LIDAR] front lidar, invalid mobile poses\n");
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // precise deskewing
            double min_t = 0;
            double min_dt = 99999999;
            Eigen::Matrix4d min_tf = Eigen::Matrix4d::Identity();
            for(size_t p = 0; p < pose_storage.size(); p++)
            {
                double dt = std::abs(pose_storage[p].t - t1);
                if(dt < min_dt)
                {
                    min_t = pose_storage[p].t;
                    min_dt = dt;
                    min_tf = se2_to_TF(pose_storage[p].pose);
                }
            }
            Eigen::Matrix4d min_tf_inv = min_tf.inverse();

            std::vector<Eigen::Vector3d> dsk_pts(raw_pts.size());
            for(size_t p = 0; p < raw_pts.size(); p++)
            {
                double t = times[p];

                int i1 = 0;
                for(int i = idx0; i <= idx1; i++)
                {
                    if(pose_storage[i].t > t)
                    {
                        i1 = i;
                        break;
                    }
                }

                int i0 = i1 - 1;
                if(i0 < 0)
                {
                    i0 = idx0;
                    i1 = idx1;
                }

                // get bound_t
                double _mo_t0 = pose_storage[i0].t;
                double _mo_t1 = pose_storage[i1].t;

                Eigen::Matrix4d _mo_tf0 = se2_to_TF(pose_storage[i0].pose);
                Eigen::Matrix4d _mo_tf1 = se2_to_TF(pose_storage[i1].pose);

                double alpha = (t-_mo_t0)/(_mo_t1-_mo_t0);
                Eigen::Matrix4d tf = intp_tf(alpha, _mo_tf0, _mo_tf1);

                dsk_pts[p] = tf.block(0,0,3,3)*raw_pts[p] + tf.block(0,3,3,1);
                dsk_pts[p] = min_tf_inv.block(0,0,3,3)*dsk_pts[p] + min_tf_inv.block(0,3,3,1);
            }

            MOBILE_POSE mo;
            mo.t = min_t;
            mo.pose = TF_to_se2(min_tf);

            RAW_FRAME frm;
            frm.t0 = t0;
            frm.t1 = t1;
            frm.times = times;
            frm.reflects = reflects;
            frm.dsk = dsk_pts;
            frm.mo = mo;
            raw_que_f.push(frm);

            last_t_f = min_t;

            // que overflow control
            if(raw_que_f.unsafe_size() > 50)
            {
                RAW_FRAME temp;
                raw_que_f.try_pop(temp);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    printf("[LIDAR] stop, front\n");
}

void LIDAR_2D::grab_loop_b()
{
    printf("[LIDAR] start grab loop, back\n");

    // init lidar
    std::unique_ptr<LakiBeamUDP> lidar(new LakiBeamUDP("192.168.2.2", "2368", "192.168.2.11", "8888"));
    is_connected_b = true;

    int drop_cnt = 100;
    while(grab_flag_b)
    {
        repark_t temp_pack;
        if(lidar->get_repackedpack(temp_pack))
        {
            // for sim
            if(is_sim)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // initial drop
            if(drop_cnt > 0)
            {
                drop_cnt--;
                continue;
            }
            \
            // time sync
            double pc_t = get_time();
            double lidar_t = temp_pack.dotcloud[0].timestamp * U2S;

            if(is_sync_b)
            {
                is_sync_b = false;
                offset_t_b = pc_t - lidar_t;
                raw_que_b.clear();

                is_synced_b = true;
                printf("[LIDAR] sync, offset_t_b: %f\n", (double)offset_t_b);
            }

            // check lidar, mobile sync
            if(is_synced_b == false || mobile->is_synced == false)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // lidar frame synced ref time
            double t0 = lidar_t + offset_t_b;
            double t1 = (temp_pack.dotcloud[temp_pack.maxdots-1].timestamp * U2S) + offset_t_b;

            // check
            if(mobile->get_pose_storage_size() != MO_STORAGE_NUM)
            {
                // drop
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // wait
            while(t1 > mobile->last_pose_t && grab_flag_b)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            // get mobile poses
            std::vector<MOBILE_POSE> pose_storage = mobile->get_pose_storage();

            // get lidar raw data
            std::vector<double> times;
            std::vector<double> reflects;
            std::vector<Eigen::Vector3d> raw_pts;
            for (int p = 0; p < temp_pack.maxdots; p++)
            {
                double t = temp_pack.dotcloud[p].timestamp*U2S + offset_t_b;
                double deg = (double)temp_pack.dotcloud[p].angle/100; // deg, 0~360
                double dist = ((double)temp_pack.dotcloud[p].distance)/1000; // meter
                double rssi = (double)temp_pack.dotcloud[p].rssi;

                // dist filter
                if(dist < 0.05 || dist > config->LIDAR_MAX_RANGE)
                {
                    continue;
                }

                // angle filter
                if((deg < 45.0 + angle_offset) || (deg > 315.0 - angle_offset))
                {
                    continue;
                }

                double x = dist * std::cos(deg*D2R);
                double y = dist * std::sin(deg*D2R);
                if(!isfinite(x) || !isfinite(y))
                {
                    continue;
                }

                times.push_back(t);
                reflects.push_back(rssi);
                raw_pts.push_back(Eigen::Vector3d(x, y, 0));
            }

            // get boundary mobile pose
            int idx0 = -1;
            for(size_t p = pose_storage.size()-1; p >= 0; p--)
            {
                if(pose_storage[p].t < t0)
                {
                    idx0 = p;
                    break;
                }
            }

            int idx1 = -1;
            for(size_t p = 0; p < pose_storage.size(); p++)
            {
                if(pose_storage[p].t > t1)
                {
                    idx1 = p;
                    break;
                }
            }

            // check
            if(idx0 == -1 || idx1 == -1)
            {
                // drop
                printf("[LIDAR] back lidar, invalid mobile poses\n");
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // precise deskewing
            double min_t = 0;
            double min_dt = 99999999;
            Eigen::Matrix4d min_tf = Eigen::Matrix4d::Identity();
            for(size_t p = 0; p < pose_storage.size(); p++)
            {
                double dt = std::abs(pose_storage[p].t - t1);
                if(dt < min_dt)
                {
                    min_t = pose_storage[p].t;
                    min_dt = dt;
                    min_tf = se2_to_TF(pose_storage[p].pose);
                }
            }
            Eigen::Matrix4d min_tf_inv = min_tf.inverse();

            std::vector<Eigen::Vector3d> dsk_pts(raw_pts.size());
            for(size_t p = 0; p < raw_pts.size(); p++)
            {
                double t = times[p];

                int i1 = 0;
                for(int i = idx0; i <= idx1; i++)
                {
                    if(pose_storage[i].t > t)
                    {
                        i1 = i;
                        break;
                    }
                }

                int i0 = i1 - 1;
                if(i0 < 0)
                {
                    i0 = idx0;
                    i1 = idx1;
                }

                // get bound_t
                double _mo_t0 = pose_storage[i0].t;
                double _mo_t1 = pose_storage[i1].t;

                Eigen::Matrix4d _mo_tf0 = se2_to_TF(pose_storage[i0].pose);
                Eigen::Matrix4d _mo_tf1 = se2_to_TF(pose_storage[i1].pose);

                double alpha = (t-_mo_t0)/(_mo_t1-_mo_t0);
                Eigen::Matrix4d tf = intp_tf(alpha, _mo_tf0, _mo_tf1);

                dsk_pts[p] = tf.block(0,0,3,3)*raw_pts[p] + tf.block(0,3,3,1);
                dsk_pts[p] = min_tf_inv.block(0,0,3,3)*dsk_pts[p] + min_tf_inv.block(0,3,3,1);
            }

            MOBILE_POSE mo;
            mo.t = min_t;
            mo.pose = TF_to_se2(min_tf);

            RAW_FRAME frm;
            frm.t0 = t0;
            frm.t1 = t1;
            frm.times = times;
            frm.reflects = reflects;
            frm.dsk = dsk_pts;
            frm.mo = mo;
            raw_que_b.push(frm);

            last_t_b = min_t;

            // que overflow control
            if(raw_que_b.unsafe_size() > 50)
            {
                RAW_FRAME temp;
                raw_que_b.try_pop(temp);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    printf("[LIDAR] stop, back\n");
}

void LIDAR_2D::a_loop()
{
    std::vector<RAW_FRAME> storage;

    printf("[LIDAR] start a loop\n");
    while(a_flag)
    {
        RAW_FRAME frm0;
        if(raw_que_f.try_pop(frm0))
        {
            // for sim
            if(is_sim)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // wait
            while(frm0.t0 > last_t_b && a_flag)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            // find best pair
            RAW_FRAME frm;
            while(raw_que_b.try_pop(frm) && a_flag)
            {
                storage.push_back(frm);
            }

            int min_idx = -1;
            double min_dt = 999999;
            for(size_t p = 0; p < storage.size(); p++)
            {
                double dt = std::abs(storage[p].t0 - frm0.t0);
                if(dt < min_dt)
                {
                    min_dt = dt;
                    min_idx = p;
                }
            }

            // merge two lidar frame
            Eigen::Matrix4d tf_f = ZYX_to_TF(config->LIDAR_TF_F);
            Eigen::Matrix4d tf_b = ZYX_to_TF(config->LIDAR_TF_B);

            if(min_idx >= 0)
            {
                RAW_FRAME frm1 = storage[min_idx];

                std::vector<double> reflects;
                std::vector<Eigen::Vector3d> pts;
                std::vector<Eigen::Vector3d> pts_f;
                for(size_t p = 0; p < frm0.dsk.size(); p++)
                {
                    Eigen::Vector3d P = tf_f.block(0,0,3,3)*frm0.dsk[p] + tf_f.block(0,3,3,1);

                    if(P[0] > config->ROBOT_SIZE_X[0] && P[0] < config->ROBOT_SIZE_X[1] &&
                       P[1] > config->ROBOT_SIZE_Y[0] && P[1] < config->ROBOT_SIZE_Y[1])
                    {
                        continue;
                    }

                    pts.push_back(P);
                    pts_f.push_back(P);
                    reflects.push_back(frm0.reflects[p]);
                }

                Eigen::Matrix4d dtf = se2_to_TF(frm0.mo.pose).inverse()*se2_to_TF(frm1.mo.pose);

                std::vector<Eigen::Vector3d> pts_b;
                for(size_t p = 0; p < frm1.dsk.size(); p++)
                {
                    Eigen::Vector3d _P = tf_b.block(0,0,3,3)*frm1.dsk[p] + tf_b.block(0,3,3,1);
                    Eigen::Vector3d P = dtf.block(0,0,3,3)*_P + dtf.block(0,3,3,1);

                    if(P[0] > config->ROBOT_SIZE_X[0] && P[0] < config->ROBOT_SIZE_X[1] &&
                       P[1] > config->ROBOT_SIZE_Y[0] && P[1] < config->ROBOT_SIZE_Y[1])
                    {
                        continue;
                    }

                    pts.push_back(P);
                    pts_b.push_back(P);
                    reflects.push_back(frm1.reflects[p]);
                }

                // update
                mtx.lock();
                cur_scan = pts;
                cur_scan_f = pts_f;
                cur_scan_b = pts_b;
                mtx.unlock();

                FRAME merge_frm;
                merge_frm.t = frm0.t0;
                merge_frm.mo = frm0.mo;
                merge_frm.reflects = reflects;
                merge_frm.pts = pts;
                scan_que.push(merge_frm);

                // for que overflow
                if(scan_que.unsafe_size() > 50)
                {
                    FRAME temp;
                    scan_que.try_pop(temp);
                }

                storage.erase(storage.begin(), storage.begin()+min_idx);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    printf("[LIDAR] stop a loop\n");
}

