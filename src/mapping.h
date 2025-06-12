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
public:
    explicit MAPPING(QObject *parent = nullptr);
    ~MAPPING();

    // mutex
    std::recursive_mutex mtx;

    // other modules
    CONFIG *config = NULL;
    LOGGER *logger = NULL;
    UNIMAP *unimap = NULL;
    LIDAR_2D *lidar_2d = NULL;
    LOCALIZATION *loc = NULL;

    // interface functions
    void start();
    void stop();
    void clear();

    // loop
    std::atomic<bool> a_flag = {false};
    std::thread* a_thread = NULL;
    void a_loop();

    std::atomic<bool> b_flag = {false};
    std::thread* b_thread = NULL;
    void b_loop();

    // algorithms
    double frm_icp(KD_TREE_XYZR& tree, XYZR_CLOUD& cloud, FRAME& frm, Eigen::Matrix4d& G);
    double kfrm_icp(KFRAME& frm0, KFRAME& frm1, Eigen::Matrix4d& dG);
    double calc_overlap_ratio(std::vector<Eigen::Vector3d>& pts0, std::vector<Eigen::Vector3d>& pts1);

    // flag
    std::atomic<bool> is_mapping = {false};

    // storage
    tbb::concurrent_queue<KFRAME> kfrm_que;
    tbb::concurrent_queue<int> kfrm_update_que;
    std::vector<KFRAME> kfrm_storage;

    // live kd_tree
    XYZR_CLOUD live_cloud;
    KD_TREE_XYZR *live_tree = NULL;

    // for plot
    std::atomic<double> proc_time_map_a = {0};
    std::atomic<double> proc_time_map_b = {0};

    // for graph optimization
    void clear_pose_graph();
    void last_lc();
    PGO pgo;
};

#endif // MAPPING_H
