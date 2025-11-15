#ifndef POLICY_H
#define POLICY_H

// defines
#include "global_defines.h"
#include "my_utils.h"

// module
#include "config.h"
#include "logger.h"
#include "unimap.h"
#include "localization.h"

#include <QObject>

class POLICY : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(POLICY)
public:
    // make singleton
    static POLICY* instance(QObject* parent = nullptr);

    // initialization policy module
    void init();

    // open policy module
    void open();

    // close policy module
    void close();

    /***********************
     * interface funcs
     ***********************/
    QString get_cur_node();
    QString get_cur_link();
    QString get_cur_zone();

    NODE_INFO get_node_info();
    NODE_INFO get_link_info();
    NODE_INFO get_zone_info();

    /***********************
     * set other modules
     ***********************/
    void set_config_module(CONFIG* _config);
    void set_logger_module(LOGGER* _logger);
    void set_unimap_module(UNIMAP* _unimap);
    void set_localization_module(LOCALIZATION* _localization);

    std::vector<PATH> drive_policy(PATH path);
    void speed_policy(const PATH& path, std::vector<double>& ref_v);

private:
    explicit POLICY(QObject *parent = nullptr);
    ~POLICY();

    // mutex
    std::mutex mtx;

    // other modules
    CONFIG* config;
    LOGGER* logger;
    UNIMAP* unimap;
    LOCALIZATION *loc;

    // node loop
    std::atomic<bool> node_flag = {false};
    std::unique_ptr<std::thread> node_thread;
    void node_loop();

    // link loop
    std::atomic<bool> link_flag = {false};
    std::unique_ptr<std::thread> link_thread;
    void link_loop();

    // zone loop
    std::atomic<bool> zone_flag = {false};
    std::unique_ptr<std::thread> zone_thread;
    void zone_loop();

    // policy loop
    std::atomic<bool> policy_flag = {false};
    std::unique_ptr<std::thread> policy_thread;
    void policy_loop();

    void apply_slow(bool on);
    void apply_fast(bool on);
    void apply_warning_beep(bool on);
    void apply_ignore_2d(bool on);
    void apply_ignore_3d(bool on);
    void apply_ignore_cam(bool on);
    void apply_ignore_obs_2d(bool on);
    void apply_ignore_obs_3d(bool on);
    void apply_ignore_obs_cam(bool on);

    QString cur_node = "";
    QString cur_link = "";
    QString cur_zone = "";

    NODE_INFO node_info;
    NODE_INFO link_info;
    NODE_INFO zone_info;

    std::vector<LINK_INFO> link_list;

    PATH global_path;

};

#endif // POLICY_H
