#include "task.h"

namespace
{
    const char* MODULE_NAME = "TASK";
}

TASK* TASK::instance(QObject* parent)
{
    static TASK inst;
    return &inst;
}

TASK::TASK() : QObject(nullptr)
{
}

TASK::~TASK()
{
    stop_thread();
}

void TASK::stop_thread()
{
    std::lock_guard<std::recursive_mutex> lock(mtx);

    if(a_thread && a_thread->joinable())
    {
        a_flag = false;
        a_thread->join();
        a_thread.reset();
    }
}

void TASK::init()
{
    log_info("task init");

    config = CONFIG::instance();
    unimap = UNIMAP::instance();
    obsmap = OBSMAP::instance();
    loc = LOCALIZATION::instance();
    mobile = MOBILE::instance();
    ctrl = AUTOCONTROL::instance();

    if (ctrl == nullptr || unimap == nullptr || loc == nullptr)
    {
        spdlog::critical("[TASK] Failed to initialize required modules!");
    }
    else
    {
        spdlog::info("[TASK] Initialized with required modules.");
    }
}

void TASK::play(QString _driving_mode)
{
    spdlog::info("[TASK] play start");

    {
        std::lock_guard<std::recursive_mutex> lock(mtx);

        stop_thread();

        if(ctrl)
        {
            ctrl->stop();
        }

        driving_mode = _driving_mode;

        a_flag = true;
        a_thread = std::make_unique<std::thread>(&TASK::a_loop, this);
    }
}

void TASK::pause()
{
    spdlog::info("[TASK] Pause command received. but not yet");
}

void TASK::cancel()
{
    spdlog::info("[TASK] Cancel command received.");

    {
        std::lock_guard<std::recursive_mutex> lock(mtx);
        stop_thread();
    }

    if(ctrl)
    {
        ctrl->stop();
    }
}

void TASK::add_task(const NODE* node)
{
    spdlog::info("[TASK] Add task");

    if (node)
    {
        std::lock_guard<std::recursive_mutex> lock(mtx);
        task_node_list.push_back(node->id);
    }
}

void TASK::del_task(NODE* node)
{
    spdlog::info("[TASK] Delete task");
    if (!node)
    {
        return;
    }

    std::lock_guard<std::recursive_mutex> lock(mtx);

    auto it = std::find(task_node_list.begin(), task_node_list.end(), node->id);
    if(it != task_node_list.end())
    {
        task_node_list.erase(it);
        node->name = "";
    }
    else
    {
        spdlog::warn("[TASK] Node with ID {} not found in task list to be deleted.", node->id.toStdString());
    }
}

void TASK::save_task(QString path)
{
    QString task_path = path + "/task_"+ get_time_str()+".json";
    QFile task_file(task_path);
    if(task_file.open(QIODevice::WriteOnly|QFile::Truncate))
    {
        QJsonArray arr;

        {
            std::lock_guard<std::recursive_mutex> lock(mtx);
            for(size_t p = 0; p < task_node_list.size(); p++)
            {
                QJsonObject obj;
                obj["id"] = task_node_list[p];
                arr.append(obj);
            }
        }

        QJsonDocument doc(arr);
        task_file.write(doc.toJson());
        task_file.close();

        spdlog::info("[TASK] {} saved", task_path.toStdString());
    }
}

void TASK::load_task(QString path)
{
    spdlog::info("[TASK] Loading task from: {}", path.toStdString());

    is_loaded = false;

    QFile task_file(path);

    if(task_file.open(QIODevice::ReadOnly))
    {
        {
            std::lock_guard<std::recursive_mutex> lock(mtx);
            task_node_list.clear();
        }

        QByteArray data = task_file.readAll();
        QJsonDocument doc = QJsonDocument::fromJson(data);
        QJsonArray arr = doc.array();

        {
            std::lock_guard<std::recursive_mutex> lock(mtx);
            for(const QJsonValue &val : arr)
            {
                QJsonObject obj = val.toObject();
                task_node_list.push_back(obj["id"].toString());
            }
        }

        task_file.close();
        spdlog::info("[TASK] {} loaded successfully. {} tasks found.", path.toStdString(), task_node_list.size());
        is_loaded = true;
    }
    else
    {
        spdlog::error("[TASK] Failed to open task file: {}", path.toStdString());
    }
}

void TASK::a_loop()
{
    is_tasking = true;
    spdlog::info("[TASK] Task loop started.");

    std::vector<QString> node_list;
    {
        std::lock_guard<std::recursive_mutex> lock(mtx);
        node_list = task_node_list;
    }

    for(size_t p = 0; p < node_list.size(); p++)
    {
        spdlog::info("[TASK] Sequence: {}, Node ID: {}", p, node_list[p].toStdString());
    }

    if (accuracy_save_enabled)
    {
        spdlog::info("[TASK] Accuracy save enabled.");
        QString logFolder = QCoreApplication::applicationDirPath() + "/snlog";
        QDir dir(logFolder);
        if (!dir.exists())
        {
            dir.mkpath(logFolder);
        }

        accuracyLogFileName = logFolder + "/accuracy_log_" + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss") + ".csv";
        QFile file(accuracyLogFileName);
        if (file.open(QIODevice::WriteOnly | QIODevice::Text))
        {
            QTextStream out(&file);
            out << "timestamp,actual_x,actual_y,actual_theta,target_id,target_name,target_type,target_info,"
                           "planned_x,planned_y,planned_theta,error_distance,error_angle,cur_ie,cur_ir\n";
            file.close();
            spdlog::info("[TASK] Created accuracy log file: {}", accuracyLogFileName.toStdString());
        }
    }

    int state = TASK_IDLE;
    last_task_state = state;

    int idx = 0;

    while(a_flag)
    {
        if(state == TASK_IDLE)
        {
            if(is_start)
            {
                idx = 0;
                state = TASK_MOVE;
                last_task_state = state;
                spdlog::info("[TASK] TASK_IDLE -> TASK_MOVE");
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                continue;
            }
        }
        else if(state == TASK_MOVE)
        {
            if(!unimap)
            {
                spdlog::error("[TASK] UNIMAP is null");
                state = TASK_WAIT;
                last_task_state = state;
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                continue;
            }

            NODE* node = unimap->get_node_by_id(task_node_list[idx]);
            if(node == nullptr)
            {
                state = TASK_WAIT;
                last_task_state = state;
                spdlog::warn("[TASK] State: MOVE -> WAIT (Node null for seq: {}, id: {})", idx, node_list[idx].toStdString());
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

            if(ctrl)
            {
                QMetaObject::invokeMethod(ctrl, "signal_move",
                                          Qt::QueuedConnection,
                                          Q_ARG(DATA_MOVE, msg));
            }

            state = TASK_CHECK_MOVE;
            last_task_state = state;
            spdlog::info("[TASK] State: MOVE -> CHECK_MOVE (seq: {}, id: {})", idx, node_list[idx].toStdString());
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            continue;
        }
        else if(state == TASK_CHECK_MOVE)
        {
            if(!ctrl)
            {
                spdlog::error("[TASK] AUTOCONTROL is null");
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                continue;
            }

            QString multi_state = ctrl->get_multi_reqest_state();
            if(multi_state == "recv_path")
            {
                spdlog::info("[TASK] TASK_CHECK_MOVE -> TASK_PROGRESS");

                state = TASK_PROGRESS;
                last_task_state = state;
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                continue;
            }
        }
        else if(state == TASK_PROGRESS)
        {
            if(!ctrl)
            {
                spdlog::error("[TASK] AUTOCONTROL is null");
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                continue;
            }

            QString multi_state = ctrl->get_multi_reqest_state();
            if(multi_state == "none")
            {
                spdlog::info("[TASK] TASK_PROGRESS -> TASK_WAIT");

                state = TASK_WAIT;
                last_task_state = state;
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                continue;
            }
        }
        else if(state == TASK_WAIT)
        {
            if(accuracy_save_enabled)
            {
                if(!loc || !unimap)
                {
                    spdlog::error("[TASK] Required modules are null for accuracy logging");
                }
                else
                {
                    Eigen::Matrix4d current_pose = loc->get_cur_tf();
                    double actual_x = current_pose(0,3);
                    double actual_y = current_pose(1,3);
                    double actual_theta = TF_to_se2(current_pose)[2];
                    double actual_theta_deg = actual_theta * R2D;

                    NODE* target_node = unimap->get_node_by_id(node_list[idx]);
                    QString target_id = (target_node != nullptr) ? target_node->id : "unknown";
                    QString target_name = (target_node != nullptr) ? target_node->name : "unknown";
                    QString target_type = (target_node != nullptr) ? target_node->type : "unknown";
                    QString target_info = (target_node != nullptr) ? target_node->context : "unknown";

                    double planned_x = 0.0, planned_y = 0.0, planned_theta = 0.0;
                    if(target_node)
                    {
                        planned_x = target_node->tf(0,3);
                        planned_y = target_node->tf(1,3);
                        planned_theta = TF_to_se2(target_node->tf)[2];
                    }

                    double error_distance = std::sqrt(std::pow(actual_x - planned_x, 2) + std::pow(actual_y - planned_y, 2));
                    double error_angle_deg = std::abs((actual_theta - planned_theta) * R2D);

                    Eigen::Vector2d cur_ieir = loc->get_cur_ieir();
                    double cur_ie = cur_ieir[0];
                    double cur_ir = cur_ieir[1];

                    QString timeStr = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss");
                    QFile file(accuracyLogFileName);
                    if (file.open(QIODevice::Append | QIODevice::Text))
                    {
                        QTextStream out(&file);
                        out << timeStr << ","
                            << actual_x << "," << actual_y << "," << actual_theta_deg << ","
                            << target_id << "," << target_name << "," << target_type << "," << target_info << ","
                            << planned_x << "," << planned_y << "," << planned_theta * R2D << ","
                            << error_distance << "," << error_angle_deg << ","<< cur_ie << "," << cur_ir << "\n";

                        file.close();
                        spdlog::info("[TASK] LOG: {}, actual: ({:.2f}, {:.2f}, {:.2f}), target: ({}, {}, {}), planned: ({:.2f}, {:.2f}, {:.2f}), error: ({:.2f}, {:.2f}), cur_ie: {:.3f}, cur_ir: {:.3f}",
                                           timeStr.toStdString(),
                                           actual_x, actual_y, actual_theta_deg,target_id.toStdString(), target_name.toStdString(), target_type.toStdString(),
                                           planned_x, planned_y, planned_theta * R2D,error_distance, error_angle_deg,cur_ie, cur_ir);
                    }
                    else
                    {
                        spdlog::warn("[TASK] Failed to write CSV file");
                    }
                }
            }

            state = TASK_MOVE;
            last_task_state = state;
            idx++;

            if(idx == (int)node_list.size())
            {
                if(use_looping)
                {
                    idx = 0;
                    state = TASK_MOVE;
                    last_task_state = state;

                    spdlog::info("[TASK] TASK RESTART -> TASK_MOVE");
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    continue;
                }
                else
                {
                    is_start = false;

                    state = TASK_IDLE;
                    last_task_state = state;
                    spdlog::info("[TASK] TASK_COMPLETE -> TASK_IDLE");
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    break;
                }
            }

            spdlog::info("[TASK] do next seq ({}->{})",idx-1, idx);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    spdlog::info("[TASK] Task loop stopped.");

    is_start = false;
    is_tasking = false;
}
