#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <thread>

#include "node_lidar.h"
#include "msg_recept.h"
#include "serial_port.h"
#include "lidar_data_processing.h"
#include "point_cloud_optimize.h"
#include "lidar_information.h"
#include "mtime.h"
#include "calibration.h"

using namespace std;

node_lidar_t::~node_lidar_t()
{
  	if(scan_node_buf)
	{
		delete[] scan_node_buf;
		scan_node_buf = NULL;
	}

	if (globalRecvBuffer)
	{
		delete[] globalRecvBuffer;
		globalRecvBuffer = NULL;
    }

	node_lidar.serial_port->close();
	node_lidar.lidar_status.lidar_ready = false;
	node_lidar.lidar_status.close_lidar = true;
	flushSerial();
}

void node_lidar_t::initialization_node_lidar()
{
    if(scan_node_buf)
    {
        delete[] scan_node_buf;
        scan_node_buf = NULL;
    }

    if (globalRecvBuffer)
    {
        delete[] globalRecvBuffer;
        globalRecvBuffer = NULL;
    }

    node_lidar.scan_node_count = 0;
    node_lidar.data_calibration = false;
    node_lidar.range_max = 0.0f;

    memset(node_lidar.recv_encryp, 0, sizeof(node_lidar.recv_encryp));
    memset(node_lidar.lidar_version, 0, sizeof(node_lidar.lidar_version));

    node_lidar.serial_port->close();

    node_lidar._dataEvent = Event();
    node_lidar._lock = Locker();

    node_lidar.lidar_block = lidar_block_t();
    node_lidar.lidar_time = lidar_time_t();
    node_lidar.lidar_general_info = lidar_general_info_t();
    node_lidar.lidar_robot_info = lidar_robot_info_t();
    node_lidar.lidar_status = lidar_status_t();
}

void flushSerial()
{
    if (!node_lidar.lidar_status.isConnected)
    {
		return;
	}
	
	size_t len = node_lidar.serial_port->available();
	if (len)
	{
		uint8_t *buffer = static_cast<uint8_t *>(alloca(len * sizeof(uint8_t)));
		size_t bytes_read = node_lidar.serial_port->read_data(buffer, len);
	}

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

/*Lidar operation status judgment function*/
bool lidar_state_judgment(bool& is_lidar_closed)
{
    is_lidar_closed = false;
    static bool wait_speed_right = false; // Whether to obtain speed adjustment information

    static uint64_t lidar_status_time = 0; // Time when the start command or restart command was received
    if(node_lidar.lidar_status.lidar_ready != node_lidar.lidar_status.lidar_last_status || node_lidar.lidar_status.close_lidar)
	{
        lidar_status_time = current_times();
        wait_speed_right = false;

		node_lidar.lidar_status.close_lidar = false;
		node_lidar.lidar_status.lidar_last_status = node_lidar.lidar_status.lidar_ready;

        flushSerial();
        if(node_lidar.lidar_status.lidar_ready)
        {
            node_lidar.serial_port->write_data(start_lidar, 4);
        }
	}

	if(node_lidar.lidar_status.lidar_trap_restart)
	{
        //printf("[BLIDAR] Restart due to abnormal status %lld\n", lidar_status_time);
        lidar_status_time = current_times();
        wait_speed_right = false;

        node_lidar.lidar_status.lidar_trap_restart = false;
        node_lidar.serial_port->write_data(end_lidar, 4);

        is_lidar_closed = true;
	}

	if(node_lidar.lidar_status.lidar_ready && !wait_speed_right)
	{
        uint64_t cur_time = current_times();
        if(cur_time - lidar_status_time > 1000)
		{
            node_lidar.serial_port->write_data(start_lidar, 4);
            wait_speed_right = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
		}

        node_lidar.lidar_time.lidar_frequence_abnormal_time = cur_time;
        node_lidar.lidar_time.system_start_time = cur_time;
    }

	return wait_speed_right;
}

result_t grabScanData(uint32_t timeout)
{
    auto result = node_lidar._dataEvent.wait(timeout);
    if(result == Event::EVENT_TIMEOUT)
    {
        return RESULT_TIMEOUT;
    }
    else if(result == Event::EVENT_OK)
    {
        node_lidar._lock.lock();
        size_t node_cnt = node_lidar.scan_node_count;
        node_lidar._lock.unlock();

        if(node_cnt == size_t(0))
        {
            return RESULT_FAIL;
        }
        else
        {
            return RESULT_OK;
        }
    }
    else
    {
        return RESULT_FAIL;
    }
}

bool data_handling(LaserScan &outscan)
{
	//node_lidar.lidar_time.tim_scan_start = getTime();	
	if(grabScanData(2000)==RESULT_OK)
	{
		send_lidar_data(outscan);
		return true;
    }
    else
    {
		return false;
	}
}

void send_lidar_data(LaserScan &outscan)
{
	node_lidar._lock.lock();

	size_t count = node_lidar.scan_node_count;
    if((count < MAX_SCAN_NODES) && (count > 0))
	{	
		uint64_t scan_time = (node_lidar.lidar_time.scan_end_time - node_lidar.lidar_time.scan_start_time);
		node_lidar.lidar_block.lidar_zero_count = 0;

		outscan.config.angle_increment = (2.0*M_PI/count);
		outscan.config.min_angle = 0;
		outscan.config.max_angle = 2*M_PI;
        outscan.config.min_range = 0.05;
        outscan.config.max_range = 1.0;
		outscan.config.scan_time =  static_cast<float>(scan_time * 1.0 / 1e9);
    	outscan.config.time_increment = outscan.config.scan_time / (double)(count - 1);
		outscan.stamp = node_lidar.lidar_time.scan_start_time;

		if(node_lidar.lidar_status.isConnected)
		{
            outscan.points.clear();
			float range = 0;
			float angle = 0.0;
			uint16_t intensity = 0;
			for (int i = count-1; i > 0; i--)
			{
				LaserPoint point;
				LaserPoint point_check;
				angle = static_cast<float>((node_lidar.scan_node_buf[count -i].angle_q6_checkbit >>
											LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) /64.0f);
				range = (node_lidar.scan_node_buf[i].distance_q2/1000.0f);
				intensity = node_lidar.scan_node_buf[i].sync_quality;

				if(node_lidar.scan_node_buf[i].exp_m == 1)
				{
					intensity = 255;
                }
                else
                {
					if(intensity >= 255)
					{
						intensity = 254;
					}
				}

				point_check.angle = angle;
				point_check.range = range;
				point_check.intensity = intensity;
				
                //if(0 <= angle && angle <= 360)
                if(0 <= angle && angle <= 360)
                {
					point.range = range;
					point.angle = angle;
					point.intensity = intensity;
                }
                else
                {
					point.range = 0.0;
					point.intensity = 0;
					point.angle = 0.0;
				}

				if(range < 0.1)
				{
					node_lidar.lidar_block.lidar_zero_count++;
				}

				if(range <= 0.15 && intensity <=65)
				{
					point.range = 0.0;
					point.intensity = 0;
				}
				outscan.points.push_back(point);
			}
			if(node_lidar.data_calibration)
			{
				//lidar_calibration(outscan);
			}

			node_lidar._lock.unlock();
			node_lidar.optimize_lidar.lidar_blocked_judge(count);

			if (node_lidar.lidar_status.FilterEnable)
			{
				node_lidar.optimize_lidar.PointCloudFilter(&outscan);
			}
		}
	}
	node_lidar._lock.unlock();
}

bool lidar_set_port()
{
    if(node_lidar.lidar_status.isConnected)
	{
        node_lidar.lidar_status.isConnected = false;
        node_lidar.serial_port->close();
        return true;
	}

    node_lidar.serial_port = make_shared<Serial_Port>(node_lidar.lidar_general_info.port,
                                                      node_lidar.lidar_general_info.m_SerialBaudrate,
                                                      Timeout::simpleTimeout(DEFAULT_TIMEOUT));
    if(!node_lidar.serial_port->open())
	{
		return false;
	}

    node_lidar.lidar_status.isConnected = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    node_lidar.serial_port->setDTR(0);

	return true;
}

bool initialize()
{
    if(node_lidar.lidar_status.optimize_enable)
    {
		node_lidar.optimize_lidar.lidar_blocked_init();
	}

    node_lidar.lidar_general_info.m_SerialBaudrate = 230400;
    node_lidar.lidar_data_processing.PackageSampleBytes = 3;
    node_lidar.lidar_general_info.m_intensities = true;

    if(!lidar_set_port())
    {
		return false;
	}

	node_lidar.lidar_general_info.trans_delay = node_lidar.serial_port->getByteTime();
	node_lidar.scan_node_buf = new node_info[1000];
	node_lidar.globalRecvBuffer = new uint8_t[sizeof(node_packages)];
	return true;
}

void cleanup_lidar_resources()
{
    if(node_lidar.scan_node_buf)
    {
        delete[] node_lidar.scan_node_buf;
        node_lidar.scan_node_buf = nullptr;
    }

    if(node_lidar.globalRecvBuffer)
    {
        delete[] node_lidar.globalRecvBuffer;
        node_lidar.globalRecvBuffer = nullptr;
    }

    node_lidar.serial_port->close();
    node_lidar.lidar_status.lidar_ready = false;
    node_lidar.lidar_status.close_lidar = true;
    flushSerial();
}
