#ifndef POINT_CLOUD_OPTIMIZE_H
#define POINT_CLOUD_OPTIMIZE_H

//#include "node_lidar.h"
#include "lidar_data_processing.h"
#include <vector>
#include "lidar_information.h"

using namespace std;
class Point_cloud_optimize
{
private:
    int cnt=0;
    int cnt_all=0;
    int cnt_judge=0;
    int cnt_record=0;
    bool cumulation = false;
    short Lidar_Blocked[800];
    
public:

    void PointCloudFilter(LaserScan *Scan);
    int UltrasonicSimRanging(LaserPoint &pScan);
    void getLidarCoverAngle(char *charbuf);
    void lidar_blocked_judge(int count);
    void lidar_blocked_count(LaserPoint &pScan,int count_lidar);
    void lidar_cover_cut(float &,float &);
    void datas_clear();
    void lidar_blocked_init();
    
};

#endif
