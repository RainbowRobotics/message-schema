#ifndef TASK_H
#define TASK_H

#include "slamnav_debug_types.h"
#include "my_utils.h"

#include "config.h"
#include "unimap.h"
//#include "slam_2d.h"
#include "localization.h"
#include "obsmap.h"
#include "autocontrol.h"

#include <QObject>

class TASK : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(TASK)

public:
    static TASK* instance(QObject* parent = nullptr);

    // mutex
    std::recursive_mutex mtx;


    // other modules
    //CONFIG *config    = NULL;
    //UNIMAP *unimap = NULL;
    //OBSMAP *obsmap    = NULL;
    //AUTOCONTROL *ctrl = NULL;
    //MOBILE *mobile    = NULL;
    CONFIG          *config;
    UNIMAP          *unimap;
    OBSMAP          *obsmap;
    LOCALIZATION    *loc;
    MOBILE          *mobile;
    AUTOCONTROL     *ctrl;


    // interface funcs
    void init();
    void play(QString _driving_mode);
    void pause();
    void cancel();

    void add_task(const NODE* node);
    void del_task(NODE* node);
    void save_task(QString path);
    void load_task(QString path);



    // task list
    std::vector<QString> task_node_list;
    QString driving_mode;

    // task loop
    std::atomic<bool> a_flag = {false};
    std::thread* a_thread = NULL;
    void a_loop();

    // path
    QString task_dir;
    std::vector<NODE> nodes;

    // flags
    std::atomic<bool> is_start = {false};
    std::atomic<bool> is_loaded = {false};
    std::atomic<bool> is_tasking = {false};
    std::atomic<bool> use_looping = {false};

    std::atomic<int> last_task_state = {0};

    //accuracy
    bool accuracy_save_enabled = false;
    std::vector<Eigen::Matrix4d> recorded_positions;
    QString accuracyLogFileName;

private:
    explicit TASK();
    ~TASK();

};

#endif // TASK_H
