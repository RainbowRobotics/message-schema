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

    if (accuracy_save_enabled)
    {
        printf("[TASK] accuracy_save_enabled\n");
        QString logFolder = QCoreApplication::applicationDirPath();// + "/home/rainbow/slamnav2/snlog";
        QDir dir(logFolder);
        if (!dir.exists())
        {
            dir.mkpath(logFolder);
        }
        // Make new CSV file - file name : Today + time
        accuracyLogFileName = logFolder + "/accuracy_log_" + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss") + ".csv";
        QFile file(accuracyLogFileName);
        if (file.open(QIODevice::WriteOnly | QIODevice::Text))
        {
            QTextStream out(&file);
            out << "timestamp,actual_x,actual_y,actual_theta,target_id,target_name,target_type,target_info,"
                               "planned_x,planned_y,planned_theta,error_distance,error_angle\n";
                        file.close();
            printf("[TASK] Make ccuracy log file: %s\n", accuracyLogFileName.toLocal8Bit().data());
        }
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
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                continue;
            }
        }
        else if(state == TASK_MOVE)
        {
            NODE* node = unimap->get_node_by_id(task_node_list[idx]);
            if(node == NULL)
            {
                state = TASK_WAIT;
                last_task_state = state;
                printf("[TASK] TASK_MOVE -> TASK_WAIT(seq:%d, node_id:%s)\n", idx, node_list[idx].toLocal8Bit().data());
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                continue;
            }

            Eigen::Vector3d xi = TF_to_se2(node->tf);

            DATA_MOVE msg;
            msg.command = "goal";
            msg.method = "pp";
            msg.preset = 0;
            msg.goal_node_id = node->id;
            msg.tgt_pose_vec[0] = xi[0];
            msg.tgt_pose_vec[1] = xi[1];
            msg.tgt_pose_vec[2] = node->tf(2,3);
            msg.tgt_pose_vec[3] = xi[2];
            Q_EMIT ctrl->signal_move(msg);

            state = TASK_CHECK_MOVE;
            last_task_state = state;
            printf("[TASK] TASK_MOVE -> TASK_CHECK_MOVE(seq:%d, node_id:%s)\n", idx, node_list[idx].toLocal8Bit().data());
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            continue;
        }
        else if(state == TASK_CHECK_MOVE)
        {
            QString multi_state = ctrl->get_multi_req();
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
            QString multi_state = ctrl->get_multi_req();
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
            // obs clear
            //obsmap->clear();

            if(accuracy_save_enabled)
            {
                Eigen::Matrix4d current_pose = slam->get_cur_tf();
                double actual_x = current_pose(0,3);
                double actual_y = current_pose(1,3);
                double actual_theta = TF_to_se2(current_pose)[2];
                double actual_theta_deg = actual_theta * R2D;

                //get arrived current node info
                NODE* target_node = unimap->get_node_by_id(node_list[idx]);
                QString target_id = (target_node != NULL) ? target_node->id : "unknown";
                QString target_name = (target_node != NULL) ? target_node->name : "unknown";
                QString target_type = (target_node != NULL) ? target_node->type : "unknown";
                QString target_info = (target_node != NULL) ? target_node->info : "unknown";

                double planned_x = 0.0, planned_y = 0.0, planned_theta = 0.0;
                if(target_node)
                {
                    planned_x = target_node->tf(0,3);
                    planned_y = target_node->tf(1,3);
                    planned_theta = TF_to_se2(target_node->tf)[2];
                }
                double error_distance = std::sqrt(std::pow(actual_x - planned_x, 2) + std::pow(actual_y - planned_y, 2));
                double error_angle_deg = std::abs((actual_theta - planned_theta) * R2D);

                QString timeStr = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss");
                QFile file(accuracyLogFileName);
                if (file.open(QIODevice::Append | QIODevice::Text))
                {
                    QTextStream out(&file);
                    // CSV format
                    out << timeStr << ","
                        << actual_x << "," << actual_y << "," << actual_theta_deg << ","
                        << target_id << "," << target_name << "," << target_type << "," << target_info << ","
                        << planned_x << "," << planned_y << "," << planned_theta * R2D << ","
                        << error_distance << "," << error_angle_deg << "\n";
                    file.close();
                    printf("[TASK] LOG: %s, actual: (%.2f, %.2f, %.2f), target: (%s, %s, %s), planned: (%.2f, %.2f, %.2f), error: (%.2f, %.2f)\n",
                           timeStr.toLocal8Bit().data(),
                           actual_x, actual_y, actual_theta_deg,
                           target_id.toLocal8Bit().data(), target_name.toLocal8Bit().data(), target_type.toLocal8Bit().data(),
                           planned_x, planned_y, planned_theta * R2D,
                           error_distance, error_angle_deg);
                }
                else
                {
                    printf("[TASK] failed wrtie CSV file \n");
                }
            }


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
    is_tasking = false;
}
