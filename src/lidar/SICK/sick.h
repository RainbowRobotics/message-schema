#ifndef SICK_H
#define SICK_H

// defines
#include "global_defines.h"
#include "my_utils.h"

// Sick Lidar
#include "sick_safetyscanners_base/SickSafetyscanners.h"
#include "sick_safetyscanners_base/Exceptions.h"
#include "sick_safetyscanners_base/Types.h"
#include "sick_safetyscanners_base/datastructure/CommSettings.h"

// module
#include "config.h"
#include "logger.h"
#include "mobile.h"

// qt
#include <QObject>

class SICK : public QObject
{
    Q_OBJECT
public:
    explicit SICK(QObject *parent = nullptr);
    ~SICK();

    // mutex
    std::mutex mtx;

    // other modules
    CONFIG *config = NULL;
    LOGGER *logger = NULL;
    MOBILE *mobile = NULL;

    // extrinsics
    Eigen::Matrix4d pts_tf[2];

    // interface funcs
    void init();
    void open();
    void close();
    void sync(int idx);

    RAW_FRAME get_cur_frm(int idx);
    QString get_info_text(int idx);

    // grab loop
    std::atomic<bool> grab_flag[2] = {false, false};
    std::thread* grab_thread[2] = {nullptr, nullptr};
    void grab_loop(int idx);

    // flags
    std::atomic<bool> is_connected[2] = {false, false};
    std::atomic<bool> is_sync[2] = {false, false};
    std::atomic<bool> is_synced[2] = {false, false};

    // params
    std::atomic<double> offset_t[2] = {0, 0};
    std::atomic<double> cur_frm_t[2] = {0, 0};
    std::atomic<int> cur_pts_num[2] = {0, 0};

    // storage
    tbb::concurrent_queue<RAW_FRAME> raw_que[2];

    // vars
    const double angle_offset = 10.0; // LAKI:8.0, SICK:10.0
    RAW_FRAME cur_frm[2];
};

#endif // SICK_H
