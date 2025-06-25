#ifndef MAPPING_H
#define MAPPING_H

// defines
#include "global_defines.h"
#include "my_utils.h"

// module
#include "config.h"
#include "logger.h"
#include "unimap.h"
#include "lidar_2d.h"
#include "localization.h"

// third party
#include "pgo.h"

#include <QObject>

class MAPPING : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(MAPPING)
public:
    // make singleton
    static MAPPING* instance(QObject* parent = nullptr);

    // start mapping module
    void start();

    // stop mapping module
    void stop();

    // clear all progress
    void clear();

    /***********************
     * interface funcs
     ***********************/
    bool get_is_mapping();                                      // check if mapping
    bool try_pop_kfrm_update_que(int& kfrm_id);                 // try pop key-frame update number
    size_t get_kfrm_storage_size();                             // get key-frame storage size
    KFRAME get_kfrm(int kfrm_id);                               // get key-frame input index
    QString get_info_text();                                    // get all mapping info
    std::shared_ptr<std::vector<PT_XYZR>> get_live_cloud_pts(); // get live cloud (mapping-cloud)
    std::shared_ptr<std::vector<KFRAME>> get_kfrm_storage();    // get key frame storage

    void last_loop_closing();

    /***********************
     * set other modules
     ***********************/
    void set_config_module(CONFIG* _config);
    void set_logger_module(LOGGER* _logger);
    void set_mobile_module(MOBILE* _mobile);
    void set_unimap_module(UNIMAP* _unimap);
    void set_lidar_2d_module(LIDAR_2D* _lidar_2d);
    void set_localization_module(LOCALIZATION* _loc);

private:
    explicit MAPPING(QObject *parent = nullptr);
    ~MAPPING();

    // mutex
    std::shared_mutex mtx;

    // other modules
    CONFIG* config;
    LOGGER* logger;
    UNIMAP* unimap;
    LIDAR_2D* lidar_2d;
    LOCALIZATION* loc;

    // loop
    std::atomic<bool> kfrm_flag = {false};
    std::unique_ptr<std::thread> kfrm_thread;
    void kfrm_loop();

    std::atomic<bool> loop_closing_flag = {false};
    std::unique_ptr<std::thread> loop_closing_thread;
    void loop_closing_loop();

    // algorithms
    double frm_icp(KD_TREE_XYZR& tree, XYZR_CLOUD& cloud, FRAME& frm, Eigen::Matrix4d& G);
    double kfrm_icp(KFRAME& frm0, KFRAME& frm1, Eigen::Matrix4d& dG);
    double calc_overlap_ratio(std::vector<Eigen::Vector3d>& pts0, std::vector<Eigen::Vector3d>& pts1);

    // flag
    std::atomic<bool> is_mapping = {false};

    // storage
    tbb::concurrent_queue<KFRAME> kfrm_que;
    tbb::concurrent_queue<int> kfrm_update_que;
    std::shared_ptr<std::vector<KFRAME>> kfrm_storage;

    // live kd_tree
    XYZR_CLOUD live_cloud;
    std::shared_ptr<std::vector<PT_XYZR>> live_cloud_pts;
    std::unique_ptr<KD_TREE_XYZR> live_tree;

    // for plot
    std::atomic<double> proc_time_map_a = {0};
    std::atomic<double> proc_time_map_b = {0};

    // for graph optimization
    void clear_pose_graph();
    PGO pgo;
};

#endif // MAPPING_H
