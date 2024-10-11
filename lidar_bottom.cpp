#include "lidar_bottom.h"

node_lidar_t node_lidar;

LIDAR_BOTTOM::LIDAR_BOTTOM(QObject *parent)
    : QObject{parent}
{

}

LIDAR_BOTTOM::~LIDAR_BOTTOM()
{
    if(recv_thread != NULL)
    {
        recv_flag = false;
        recv_thread->join();
        recv_thread = NULL;
    }

    if(grab_thread != NULL)
    {
        grab_flag = false;
        grab_thread->join();
        grab_thread = NULL;
    }

    flushSerial();
    node_lidar.initialization_node_lidar();
}

void LIDAR_BOTTOM::open()
{
    // check simulation mode
    if(config->SIM_MODE == 1)
    {
        printf("[LIDAR] simulation mode\n");
        return;
    }

    if(recv_thread == NULL)
    {
        recv_flag = true;
        recv_thread = new std::thread(&LIDAR_BOTTOM::recv_loop, this);
    }

    if(grab_thread == NULL)
    {
        grab_flag = true;
        grab_thread = new std::thread(&LIDAR_BOTTOM::grab_loop, this);
    }
}

TIME_PTS LIDAR_BOTTOM::get_cur_scan()
{
    mtx.lock();
    TIME_PTS _cur_scan = cur_scan;
    mtx.unlock();

    return _cur_scan;
}

void LIDAR_BOTTOM::grab_loop()
{
    //sudo adduser $USER dialout
    //return;

    logger->PrintLog("[BLIDAR] start grab loop", "Green", true, false);

    Eigen::Vector2d circle_xy(0.3, 0.15);
    double circle_radius = 0.05;

    double MASKING_SIZE_X[2] = {0, 0.4};
    double MASKING_SIZE_Y[2] = {-20, 20};

    Eigen::Matrix4d tf = ZYX_to_TF(0.22, 0, 0, 0, 0, 180*D2R);
    while(grab_flag)
    {
        LaserScan scan;
        if(data_handling(scan))
        {
            //double min_dist = 9999999;
            //double min_x, min_y;
            std::vector<Eigen::Vector3d> pts;
            for(size_t p = 0; p < scan.points.size(); p++)
            {
                double th = scan.points[p].angle*D2R;
                double dist = scan.points[p].range;
                if(dist > 0.5)
                {
                    continue;
                }

                if((th < 90*D2R) || (th > 270*D2R))
                {
                    continue;
                }

                double x = dist * std::cos(th);
                double y = -1*(dist * std::sin(th));
                if(!isfinite(x) || !isfinite(y))
                {
                    continue;
                }

                Eigen::Vector3d P = tf.block(0,0,3,3) * Eigen::Vector3d(x,y,0) + tf.block(0,3,3,1);
                if(P[0] > config->ROBOT_SIZE_X[0] && P[0] < config->ROBOT_SIZE_X[1] &&
                   P[1] > config->ROBOT_SIZE_Y[0] && P[1] < config->ROBOT_SIZE_Y[1])
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

                /*
                if(dist < min_dist)
                {
                    min_dist = dist;
                    min_x = P[0];
                    min_y = P[1];
                }
                */

                pts.push_back(P);
            }

            //std::cout << "minxy " << min_x << ", " << min_y << std::endl;
            double _time = get_time();

            // update scan
            TIME_PTS scan;
            scan.t = _time;
            scan.pts = pts;

            mtx.lock();
            time = _time;
            cur_scan = scan;
            mtx.unlock();

            grab_fail_cnt = 0;
        }
        else
        {
            grab_fail_cnt++;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void LIDAR_BOTTOM::recv_loop()
{
    constexpr size_t localBufSize = 128;
    constexpr size_t localScanSize = 1000;

    node_info  local_buf[localBufSize];
    node_info  local_scan[localScanSize];
    size_t     scan_count = 0;
    result_t   ans = RESULT_FAIL;

    node_lidar.lidar_general_info.port = std::string("/dev/ttyBL0");

    if(!initialize())
    {
        logger->PrintLog("[BLIDAR] init failed", "Red", true, false);
        return;
    }

    node_lidar.lidar_status.lidar_ready = true;
    memset(local_scan, 0, sizeof(local_scan));
    node_lidar.lidar_time.scan_time_record = current_times();

    bool is_lidar_closed = false;
    while(recv_flag)
    {
        if(lidar_state_judgment(is_lidar_closed))
        {
            if(is_lidar_closed == true)
            {
                recv_flag = false;
                return;
            }

            size_t count = localBufSize;
            ans = node_lidar.lidar_data_processing.waitScanData(local_buf, count);

            if(IS_OK(ans))
            {
                node_lidar.lidar_status.lidar_restart_try = false;
                node_lidar.lidar_time.system_start_time = current_times();
            }
            else if(current_times()-node_lidar.lidar_time.system_start_time > 3000)
            {
                if(!node_lidar.lidar_status.lidar_restart_try)
                {
                    node_lidar.lidar_status.lidar_restart_try = true;
                    node_lidar.lidar_status.lidar_trap_restart = true;
                }
                else
                {
                    printf("[BLIDAR] lidar blocked\n");
                    node_lidar.lidar_status.lidar_abnormal_state |= 0x01;
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }

            for(size_t pos = 0; pos < count; ++pos)
            {
                if (local_buf[pos].sync_flag & LIDAR_RESP_MEASUREMENT_SYNCBIT)
                {
                    if (local_scan[0].sync_flag & LIDAR_RESP_MEASUREMENT_SYNCBIT)
                    {
                        local_scan[0].stamp = local_buf[pos].stamp;
                        local_scan[0].scan_frequence = local_buf[pos].scan_frequence;

                        if (node_lidar.lidar_general_info.version == M1CT_TOF)
                        {
                            if (local_scan[0].scan_frequence > 200 || local_scan[0].scan_frequence < 10)
                            {
                                if (current_times() - node_lidar.lidar_time.lidar_frequence_abnormal_time > 30000)
                                {
                                    node_lidar.lidar_status.lidar_abnormal_state |= 0x02;
                                }
                            }
                            else
                            {
                                node_lidar.lidar_time.lidar_frequence_abnormal_time = current_times();
                            }
                        }

                        node_lidar._lock.lock();
                        if ((node_lidar.lidar_time.scan_time_current - node_lidar.lidar_time.scan_time_record) > 2000)
                        {
                            node_lidar.lidar_time.scan_time_record = node_lidar.lidar_time.scan_time_current;
                        }

                        node_lidar.lidar_time.scan_start_time = node_lidar.lidar_time.tim_scan_start;
                        node_lidar.lidar_time.scan_end_time = node_lidar.lidar_time.tim_scan_end;

                        if (node_lidar.lidar_time.tim_scan_start != node_lidar.lidar_time.tim_scan_end)
                        {
                            node_lidar.lidar_time.tim_scan_start = node_lidar.lidar_time.tim_scan_end;
                        }

                        memcpy(node_lidar.scan_node_buf, local_scan, scan_count * sizeof(node_info));
                        node_lidar.scan_node_count = scan_count;
                        node_lidar.lidar_time.scan_time_current = current_times();
                        node_lidar._dataEvent.set();
                        node_lidar._lock.unlock();
                    }
                    scan_count = 0;
                }

                local_scan[scan_count++] = local_buf[pos];
                if (scan_count == localScanSize)
                {
                    scan_count = localScanSize - 1;
                }
            }
        }
        else
        {
            flushSerial();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    recv_flag = false;
    cleanup_lidar_resources();
}
