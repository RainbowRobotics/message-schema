#ifndef SHAREDDATA_H
#define SHAREDDATA_H

#include <iostream>

enum SHARED_DATA_ADDR{
    SHM_ARM_CODE = 0 ,
    SHM_SYSTEM_VERSION,
    SHM_HEART_BEAT,

    SHM_FOLLOW_MODE,
    SHM_FOLLOW_DATA_0,
    SHM_FOLLOW_DATA_1,
    SHM_FOLLOW_DATA_2,
    SHM_FOLLOW_DATA_3,
    SHM_FOLLOW_DATA_4,
    SHM_FOLLOW_DATA_5,
    SHM_FOLLOW_DATA_6,
    SHM_FOLLOW_DATA_7,
    SHM_FOLLOW_DATA_8,
    SHM_FOLLOW_DATA_9,
    SHM_FOLLOW_DATA_10,
    SHM_FOLLOW_DATA_11,
    SHM_FOLLOW_DATA_12,
    SHM_FOLLOW_DATA_13,
    SHM_FOLLOW_DATA_14,
    SHM_FOLLOW_DATA_15,
    SHM_FOLLOW_DATA_16,
    SHM_FOLLOW_DATA_17,
    SHM_FOLLOW_DATA_18,
    SHM_FOLLOW_DATA_19,
    SHM_FOLLOW_DATA_20,

    SHM_NUM
};

const int SHARED_DATA_TYPE[SHM_NUM]{
    0,
    0,
    0,

    0,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
};

typedef union{
    struct{
        int     info_robot_code;
        int     info_system_version;
        int     info_heart_beat;

        int     follow_mode;
        float   follow_data[21];
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
