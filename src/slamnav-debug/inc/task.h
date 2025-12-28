#ifndef TASK_H
#define TASK_H

#include "slamnav_debug_types.h"
#include "my_utils.h"

#include "config.h"
#include "unimap.h"
#include "localization.h"
#include "obsmap.h"
#include "autocontrol.h"

#include <QObject>
#include <memory>

class TASK : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(TASK)

public:
    static TASK* instance(QObject* parent = nullptr);

    void init();
    void play(QString _driving_mode);
    void pause();
    void cancel();

    void add_task(const NODE* node);
    void del_task(NODE* node);
    void save_task(QString path);
    void load_task(QString path);

    std::vector<QString> task_node_list;
    QString driving_mode;

    QString task_dir;
    std::vector<NODE> nodes;

    std::atomic<bool> is_start = {false};
    std::atomic<bool> is_loaded = {false};
    std::atomic<bool> is_tasking = {false};
    std::atomic<bool> use_looping = {false};
    std::atomic<int> last_task_state = {0};

    bool accuracy_save_enabled = false;
    std::vector<Eigen::Matrix4d> recorded_positions;
    QString accuracyLogFileName;

private:
    explicit TASK();
    ~TASK();

    std::recursive_mutex mtx;

    CONFIG          *config = nullptr;
    UNIMAP          *unimap = nullptr;
    OBSMAP          *obsmap = nullptr;
    LOCALIZATION    *loc = nullptr;
    MOBILE          *mobile = nullptr;
    AUTOCONTROL     *ctrl = nullptr;

    std::atomic<bool> a_flag = {false};
    std::unique_ptr<std::thread> a_thread;
    void a_loop();

    void stop_thread();
};

#endif // TASK_H
