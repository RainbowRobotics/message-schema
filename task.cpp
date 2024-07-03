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

void TASK::play()
{
    // stop first
    if(a_thread != NULL)
    {
        a_flag = false;
        a_thread->join();
        a_thread = NULL;
    }
    ctrl->stop();


    // task loop running
    if(a_thread == NULL)
    {
        a_flag = true;
        a_thread = new std::thread(&TASK::a_loop, this);
    }
}

void TASK::pause()
{
    ctrl->stop();
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

    // set load task dir
//    task_dir = path;

    // load topology file
    task_node_list.clear();
//    QString topo_path = map_dir + "/topo.json";
    QFileInfo task_info(path);
    if(task_info.exists() && task_info.isFile())
    {
        QFile task_file(path);
        if(task_file.open(QIODevice::ReadOnly))
        {
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

    bool use_confirm = false;

    //get task list
    std::vector<QString> node_list = task_node_list;
    for(size_t p = 0; p < node_list.size(); p++)
    {
        printf("[TASK] seq:%zu, node_id:%s\n", p, node_list[p].toLocal8Bit().data());
    }

    // init state
    int state = TASK_IDLE;
    last_task_state = state;

    // for check stair
    bool stair_st = false;

    // seq num
    int idx = 0;

    printf("[TASK] task_loop start\n");
    while(a_flag)
    {
        if(state == TASK_IDLE)
        {
            if(start_signal)
            {
                idx = 0;
                state = TASK_MOVE;
                last_task_state = state;
                printf("[TASK] TASK_IDLE -> TASK_MOVE\n");
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }
        }
        else if(state == TASK_MOVE)
        {
            NODE* node = unimap->get_node_by_id(task_node_list[idx]);
            if(node->type == "GOAL")
            {
                ctrl->move_pp(node->tf, 0);
            }
            else if(node->type == "STAIR")
            {
                if(!stair_st)
                {
                    ctrl->move_pp(node->tf, 0);
                    stair_st = true;
                    printf("[TASK] move_pp to first stair node\n");
                }
                else
                {
                    ctrl->move_hpp(node->tf, 0);
                    stair_st = false;
                    printf("[TASK] move_hpp to second stair node\n");
                }
            }

            state = TASK_CHECK_MOVE;
            last_task_state = state;
            printf("[TASK] TASK_MOVE -> TASK_CHECK_MOVE(seq:%d, node_id:%s)\n", idx, node_list[idx].toLocal8Bit().data());
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }
        else if(state == TASK_CHECK_MOVE)
        {
            NODE* node = unimap->get_node_by_id(task_node_list[idx]);
            if(node->type == "GOAL")
            {
                if(ctrl->fsm_state == AUTO_FSM_COMPLETE)
                {
                    printf("[TASK] TASK_CHECK_MOVE -> TASK_PROGRESS\n");

                    state = TASK_WAIT;
                    last_task_state = state;
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    continue;
                }
            }
        }
        else if(state == TASK_PROGRESS)
        {
            // not use yet AMR
            state = TASK_CHECK_PROGRESS;
            last_task_state = state;

            printf("[TASK] TASK_PROGRESS -> TASK_CHECK_PROGRESS\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }
        else if(state == TASK_CHECK_PROGRESS)
        {
            // not use yet AMR
//          std::this_thread::sleep_for(std::chrono::milliseconds(2000));
//          if(feedback)
            {
                state = TASK_WAIT;
                last_task_state = state;
                printf("[TASK] TASK_CHECK_POSE -> TASK_WAIT\n");
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }
        }
        else if(state == TASK_WAIT)
        {
            if(!use_confirm)
            {
                printf("[TASK] WAITING TEST(confirm X)\n");
                std::this_thread::sleep_for(std::chrono::milliseconds(3000));
                continue_signal = true;
            }

//            printf("[TASK] TASK_WAIT : %s\n", node_list[idx].toLocal8Bit().data());
            if(continue_signal)
            {
                mobile->move(0.0, 0.0, 0.0);

                //obs clear
                obsmap->clear();

                state = TASK_MOVE;
                last_task_state = state;
                idx++;
                printf("idx:%d\n", idx);

                if(idx == node_list.size())
                {
                    if(use_looping)
                    {
                        printf("[TASK] TASK RESTART -> TASK_MOVE\n");

                        idx = 0;
                        state = TASK_MOVE;
                        std::this_thread::sleep_for(std::chrono::milliseconds(500));
                        continue;
                    }
                    else
                    {
                        state = TASK_IDLE;
                        last_task_state = state;
                        printf("[TASK] TASK_COMPLETE -> TASK_IDLE\n");
                        start_signal = false;
                        continue_signal = false;
                        break;
                    }
                }

                continue_signal = false;
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    printf("[TASK] task_loop stop\n");

    //clear
    start_signal = false;
    is_tasking = false;
    continue_signal = false;
    last_task_state = state;
}
