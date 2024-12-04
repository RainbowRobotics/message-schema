#include "task.h"

TASK::TASK(QObject *parent) : QObject{parent}
{

}

TASK::~TASK()
{
    if(a_thread != NULL)
    {
        a_flag = false;
        a_thread->join();
        a_thread = NULL;
    }
}

void TASK::play(QString _driving_mode)
{
    // stop first
    if(a_thread != NULL)
    {
        a_flag = false;
        a_thread->join();
        a_thread = NULL;
    }
    ctrl->stop();

    driving_mode = _driving_mode;

    // task loop running
    if(a_thread == NULL)
    {
        a_flag = true;
        a_thread = new std::thread(&TASK::a_loop, this);
    }
}

void TASK::pause()
{

}

void TASK::cancel()
{
    // cancel
    if(a_thread != NULL)
    {
        a_flag = false;
        a_thread->join();
        a_thread = NULL;
    }

    ctrl->stop();
}

void TASK::add_task(NODE* node)
{
    task_node_list.push_back(node->id);
}

void TASK::del_task(NODE* node)
{
    auto it = std::find(task_node_list.begin(), task_node_list.end(), node->id);
    if(it != task_node_list.end())
    {
        task_node_list.erase(it);
        node->name = "";
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
    if(task_file.open(QIODevice::WriteOnly|QFile::Truncate))
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
void TASK::a_loop()
{
    is_tasking = true;

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

    printf("[TASK] task_loop start\n");
    while(a_flag)
    {
        if(state == TASK_IDLE)
        {
            if(is_start)
            {
                idx = 0;
                state = TASK_MOVE;
                last_task_state = state;
                printf("[TASK] TASK_IDLE -> TASK_MOVE\n");
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                continue;
            }
        }
        else if(state == TASK_MOVE)
        {
            NODE* node = unimap->get_node_by_id(task_node_list[idx]);

            if(ctrl->is_multi)
            {
                ctrl->set_goal(node->id);
            }
            else
            {
                ctrl->move_pp(node->tf, 0);
            }

            state = TASK_CHECK_MOVE;
            last_task_state = state;
            printf("[TASK] TASK_MOVE -> TASK_CHECK_MOVE(seq:%d, node_id:%s)\n", idx, node_list[idx].toLocal8Bit().data());
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }
        else if(state == TASK_CHECK_MOVE)
        {
            if(ctrl->fsm_state == AUTO_FSM_COMPLETE)
            {
                printf("[TASK] TASK_CHECK_MOVE -> TASK_PROGRESS\n");

                state = TASK_WAIT;
                last_task_state = state;
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                continue;
            }
        }
        else if(state == TASK_WAIT)
        {
            // obs clear
            obsmap->clear();

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
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    continue;
                }
                else
                {
                    is_start = false;

                    state = TASK_IDLE;
                    last_task_state = state;
                    printf("[TASK] TASK_COMPLETE -> TASK_IDLE\n");
                    break;
                }
            }

            printf("[TASK] do next seq (%d->%d)\n", idx-1, idx);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    printf("[TASK] task_loop stop\n");

    //clear
    is_start = false;
    is_tasking = false;
}
