#ifndef TASK_H
#define TASK_H

#include "global_defines.h"
#include "utils.h"

#include "config.h"
#include "unimap.h"
#include "slam_2d.h"
#include "obsmap.h"
#include "autocontrol.h"

#include <QObject>

class TASK : public QObject
{
    Q_OBJECT
public:
    explicit TASK(QObject *parent = nullptr);
    ~TASK();

    // mutex
    std::mutex mtx;

    // other modules
    CONFIG *config    = NULL;
    UNIMAP *unimap = NULL;
    SLAM_2D *slam     = NULL;
    OBSMAP *obsmap    = NULL;
    AUTOCONTROL *ctrl = NULL;
    MOBILE *mobile    = NULL;

    // interface funcs
    void play();
    void pause();
    void cancel();

    void add_task(NODE* node);
    void del_task(NODE* node);
    void save_task(QString path);
    void load_task(QString path);

    // task list
    std::vector<QString> task_node_list;

    // task loop
    std::atomic<bool> a_flag = {false};
    std::thread* a_thread = NULL;
    void a_loop();

    // path
    QString task_dir;
    std::vector<NODE> nodes;

    // flags
    std::atomic<bool> start_signal = {false};
    std::atomic<bool> continue_signal = {false};
    std::atomic<bool> is_loaded = {false};
    std::atomic<bool> is_tasking = {false};
    std::atomic<bool> use_looping = {false};

    std::atomic<int> last_task_state = {0};

};

#endif // TASK_H
