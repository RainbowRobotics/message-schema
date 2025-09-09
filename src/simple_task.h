#ifndef TASK_H
#define TASK_H

#include "global_defines.h"
#include "my_utils.h"

#include "config.h"
#include "unimap.h"
#include "localization.h"
#include "autocontrol.h"

#include <QObject>

class TASK : public QObject
{
    Q_OBJECT

public:
    // make singleton
    static TASK* instance(QObject* parent = nullptr);

    // interface funcs
    void play(QString _driving_mode);
    void pause();
    void cancel();
    void clear();

    void add_task(NODE node);
    void del_task(NODE node);
    void save_task(QString path);
    void load_task(QString path);

    bool get_is_task();
    std::vector<QString> get_task_node_list();

    void set_is_start(bool flag);
    void set_use_looping(bool flag);

    int check_node_list_size();

    /***********************
     * set other modules
     ***********************/
    void set_config_module(CONFIG* _config);
    void set_logger_module(LOGGER* _logger);
    void set_unimap_module(UNIMAP* _unimap);
    void set_mobile_module(MOBILE* _mobile);
    void set_autocontrol_module(AUTOCONTROL* _ctrl);
    void set_localization_module(LOCALIZATION* _loc);

private:
    explicit TASK(QObject *parent = nullptr);
    ~TASK();

    // mutex
    std::recursive_mutex mtx;

    // other modules
    CONFIG* config;
    LOGGER* logger;
    UNIMAP* unimap;
    MOBILE* mobile;
    AUTOCONTROL* ctrl;
    LOCALIZATION* loc;

    // task list
    std::vector<QString> task_node_list;
    QString driving_mode;

    // task loop
    std::atomic<bool> task_flag = {false};
    std::unique_ptr<std::thread> task_thread;
    void task_loop();

    // path
    QString task_dir;
    std::vector<NODE> nodes;

    // flags
    std::atomic<bool> is_start = {false};
    std::atomic<bool> is_loaded = {false};
    std::atomic<bool> is_task = {false};
    std::atomic<bool> use_looping = {false};

    std::atomic<int> last_task_state = {0};
};

#endif // TASK_H
