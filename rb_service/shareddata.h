#ifndef SHAREDDATA_H
#define SHAREDDATA_H

#include <iostream>

enum SHARED_DATA_ADDR{
    SHM_ARM_CODE = 0 ,
    SHM_SYSTEM_VERSION,
    SHM_HEART_BEAT,

    SHM_NUM
};

const int SHARED_DATA_TYPE[SHM_NUM]{
    0,
    0,
    0,
};

typedef union{
    struct{
        int     info_robot_code;
        int     info_system_version;
        int     info_heart_beat;
    }sdata;
    float fdata[SHM_NUM];
    int   idata[SHM_NUM];
}ST_SHARED_DATA;

namespace rb_shareddata {
    bool initialize(std::string domain);
    void finalize();
    ST_SHARED_DATA* get(); // getter 함수
}
#endif // SHAREDDATA_H
