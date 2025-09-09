#include "simple_task.h"

TASK* TASK::instance(QObject* parent)
{
    static TASK* inst = nullptr;
    if(!inst && parent)
    {
        inst = new TASK(parent);
    }
    else if(inst && parent && inst->parent() == nullptr)
    {
        inst->setParent(parent);
    }
    return inst;
}

TASK::TASK(QObject *parent) : QObject{parent}
  , config(nullptr)
  , logger(nullptr)
  , unimap(nullptr)
  , mobile(nullptr)
  , ctrl(nullptr)
  , loc(nullptr)
{

}

TASK::~TASK()
{
    task_flag = false;
    if(task_thread && task_thread->joinable())
    {
        task_thread->join();
    }
    task_thread.reset();
}

void TASK::play(QString _driving_mode)
{
    // stop first
    task_flag = false;
    if(task_thread && task_thread->joinable())
    {
        task_thread->join();
    }
    task_thread.reset();

    ctrl->stop();

    driving_mode = _driving_mode;

    // task loop running
    task_flag = true;
    task_thread = std::make_unique<std::thread>(&TASK::task_loop, this);
}

void TASK::pause()
{

}

void TASK::cancel()
{
    // cancel
    task_flag = false;
    if(task_thread && task_thread->joinable())
    {
        task_thread->join();
    }
    task_thread.reset();

    ctrl->stop();
}

void TASK::clear()
{
    std::lock_guard<std::recursive_mutex> lock(mtx);
    task_node_list.clear();
}

bool TASK::get_is_task()
{
    return static_cast<bool>(is_task.load());
}

void TASK::set_is_start(bool flag)
{
    is_start.store(flag);
}

void TASK::set_use_looping(bool flag)
{
    use_looping.store(flag);
}

std::vector<QString> TASK::get_task_node_list()
{
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return task_node_list;
}

int TASK::check_node_list_size()
{
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return task_node_list.size();
}

void TASK::add_task(NODE node)
{
    if(node.id.isEmpty() || node.id.isNull())
    {
        return;
    }

    task_node_list.push_back(node.id);
}

void TASK::del_task(NODE node)
{
    auto it = std::find(task_node_list.begin(), task_node_list.end(), node.id);
    if(it != task_node_list.end())
    {
        task_node_list.erase(it);
        node.name = "";
    }
    else
    {
        qDebug() << "[TASK] not found to be deleted";
    }
}

void TASK::save_task(QString path)
{
    // save task.json
    QString task_path = path + "/task_"+ get_time_str()+".json";
    QFile task_file(task_path);
    if(task_file.open(QIODevice::WriteOnly | QFile::Truncate))
    {
        QJsonArray arr;
        for(size_t p = 0; p < task_node_list.size(); p++)
        {
            QJsonObject obj;
            obj["id"] = task_node_list[p];

            arr.append(obj);
        }

        QJsonDocument doc(arr);
        task_file.write(doc.toJson());
        task_file.close();

        printf("[TASK] %s saved\n", task_path.toLocal8Bit().data());
    }
}

void TASK::load_task(QString path)
{
    printf("[TASK] load\n");

    // clear flag
    is_loaded = false;

    // load topology file
    QFileInfo task_info(path);
    if(task_info.exists() && task_info.isFile())
    {
        QFile task_file(path);
        if(task_file.open(QIODevice::ReadOnly))
        {
            task_node_list.clear();

            QByteArray data = task_file.readAll();
            QJsonDocument doc = QJsonDocument::fromJson(data);

            QJsonArray arr = doc.array();
            Q_FOREACH(const QJsonValue &val, arr)
            {
                QJsonObject obj = val.toObject();

                QString task;
                task = obj["id"].toString();

                task_node_list.push_back(task);
            }
            task_file.close();

            printf("[TASK] %s loaded\n", path.toStdString().c_str());
        }
    }

    // set flag
    is_loaded = true;
}

// loop
void TASK::task_loop()
{
    is_task = true;

    //get task list
    std::vector<QString> node_list = task_node_list;
    for(size_t p = 0; p < node_list.size(); p++)
    {
        printf("[TASK] seq: %d, node_id: %s\n", (int)p, node_list[p].toLocal8Bit().data());
    }

    // init state
    int state = TASK_IDLE;
    last_task_state = state;

    // seq num
    int idx = 0;

    bool is_first = true;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dist(1, 10);


    printf("[TASK] task_loop start\n");
    while(task_flag)
    {
        if(state == TASK_IDLE)
        {
            if(is_start)
            {
                idx = 0;
                state = TASK_MOVE;
                last_task_state = state;
                printf("[TASK] TASK_IDLE -> TASK_MOVE\n");
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                continue;
            }
        }
        else if(state == TASK_MOVE)
        {
            if(is_first)
            {
                is_first = false;
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::minutes(dist(gen)));
            }

            NODE node = unimap->get_node_by_id(task_node_list[idx]);
            if(node.id.isEmpty() || node.id.isNull())
            {
                state = TASK_WAIT;
                last_task_state = state;
                printf("[TASK] TASK_MOVE -> TASK_WAIT(seq:%d, node_id:%s)\n", idx, node_list[idx].toLocal8Bit().data());
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                continue;
            }

            Eigen::Vector3d xi = TF_to_se2(node.tf);

            DATA_MOVE msg;
            msg.command = "goal";
            msg.method = "pp";
            msg.preset = 0;
            msg.goal_node_id = node.id;
            msg.tgt_pose_vec[0] = xi[0];
            msg.tgt_pose_vec[1] = xi[1];
            msg.tgt_pose_vec[2] = node.tf(2,3);
            msg.tgt_pose_vec[3] = xi[2];

            ctrl->request_move(CommandType::MOVE, msg);

            state = TASK_CHECK_MOVE;
            last_task_state = state;
            printf("[TASK] TASK_MOVE -> TASK_CHECK_MOVE(seq:%d, node_id:%s)\n", idx, node_list[idx].toLocal8Bit().data());
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            continue;
        }
        else if(state == TASK_CHECK_MOVE)
        {
            QString multi_state = ctrl->get_multi_reqest_state();
            if(multi_state == "recv_path")
            {
                printf("[TASK] TASK_CHECK_MOVE -> TASK_PROGRESS\n");

                state = TASK_PROGRESS;
                last_task_state = state;
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                continue;
            }
        }
        else if(state == TASK_PROGRESS)
        {
            QString multi_state = ctrl->get_multi_reqest_state();
            if(multi_state == "none")
            {
                printf("[TASK] TASK_PROGRESS -> TASK_WAIT\n");

                state = TASK_WAIT;
                last_task_state = state;
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                continue;
            }
        }
        else if(state == TASK_WAIT)
        {
            // increase idx
            state = TASK_MOVE;
            last_task_state = state;
            idx++;

            // check last
            if(idx == (int)node_list.size())
            {
                if(use_looping)
                {
                    idx = 0;
                    state = TASK_MOVE;
                    last_task_state = state;

                    printf("[TASK] TASK RESTART -> TASK_MOVE\n");
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    continue;
                }
                else
                {
                    is_start = false;

                    state = TASK_IDLE;
                    last_task_state = state;
                    printf("[TASK] TASK_COMPLETE -> TASK_IDLE\n");
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    break;
                }
            }

            printf("[TASK] do next seq (%d->%d)\n", idx-1, idx);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    printf("[TASK] task_loop stop\n");

    //clear
    is_start = false;
    is_task = false;
}

void TASK::set_config_module(CONFIG* _config)
{
    config = _config;
}

void TASK::set_logger_module(LOGGER* _logger)
{
    logger = _logger;
}

void TASK::set_unimap_module(UNIMAP* _unimap)
{
    unimap = _unimap;
}

void TASK::set_mobile_module(MOBILE* _mobile)
{
    mobile = _mobile;
}

void TASK::set_autocontrol_module(AUTOCONTROL* _ctrl)
{
    ctrl = _ctrl;
}

void TASK::set_localization_module(LOCALIZATION *_loc)
{
    loc = _loc;
}

