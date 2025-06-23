#ifndef LIDAR_2D_H
#define LIDAR_2D_H

// global defines
#include "global_defines.h"
#include "my_utils.h"

// modules
#include "config.h"
#include "logger.h"
#include "mobile.h"

#include "lidar/SICK/sick.h"

#include <QObject>

constexpr unsigned int deskewing_que_max_size = 10;
constexpr unsigned int deskewing_storage_max_size = 10;
constexpr unsigned int merge_que_max_size = 10;

class LIDAR_2D : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(LIDAR_2D)

public:
    // make singleton
    static LIDAR_2D* instance(QObject* parent = nullptr);

    // initialization lidar_2d module
    void init();

    // open lidar_2d module as written in config.json
    void open();

    // close lidar_2d module
    void close();

    /***********************
     * interface funcs (get)
     ***********************/
    RAW_FRAME get_cur_raw(int idx);
    QString get_info_text();
    QString get_cur_state();
    bool get_is_sync();
    bool get_is_connected();

    // interface funcs (set)
    void set_cur_state(QString str);
    void set_sync_flag(bool flag);
    void set_is_connected(bool val);

    void clear_merged_queue();
    bool try_pop_merged_queue(FRAME& frm);

    /***********************
     * set other modules
     ***********************/
    void set_config_module(CONFIG* _config);
    void set_logger_module(LOGGER* _logger);
    void set_mobile_module(MOBILE* _mobile);

private:
    explicit LIDAR_2D(QObject *parent = nullptr);
    ~LIDAR_2D();

    // mutex
    std::mutex mtx;

    // other modules
    CONFIG* config;
    LOGGER* logger;
    MOBILE* mobile;
    SICK* sick;

    // deskewing loop
    std::atomic<bool> deskewing_flag[2] = {false, false};
    std::array<std::unique_ptr<std::thread>, 2> deskewing_thread;
    void deskewing_loop(int idx);

    // merge loop
    std::atomic<bool> merge_flag = {false};
    std::unique_ptr<std::thread> merge_thread;
    void merge_loop();

    // for watchdog
    QString cur_state = "none";

    // params
    std::atomic<bool> is_connected = {false};
    std::atomic<bool> is_sync = {false};

    std::atomic<double> cur_merged_frm_t = 0;
    std::atomic<int> cur_merged_num = 0;

    // storage
    tbb::concurrent_queue<RAW_FRAME> deskewing_que[2];
    tbb::concurrent_queue<FRAME> merged_que;

    // https://www.researchgate.net/figure/Schematic-of-shadows-created-by-a-moving-LIDAR-Vehicle-graphic-credit-ShapeNet-23_fig1_364897453
    std::vector<std::pair<Eigen::Vector3d, bool>> scan_shadow_filter(const std::vector<Eigen::Vector3d>& dsk, int shadow_window);
    bool is_shadow(const double r1, const double r2, const double included_angle, const double min_angle_tan, const double max_angle_tan);

};

#endif // LIDAR_2D_H
