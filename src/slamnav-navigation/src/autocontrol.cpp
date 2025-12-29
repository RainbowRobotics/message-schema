#include "autocontrol.h"

namespace
{
    const char* MODULE_NAME = "AUTO";
}

AUTOCONTROL* AUTOCONTROL::instance(QObject* parent)
{
    static AUTOCONTROL* inst = nullptr;
    if(!inst && parent)
    {
        inst = new AUTOCONTROL(parent);
    }
    else if(inst && parent && inst->parent() == nullptr)
    {
        inst->setParent(parent);
    }
    return inst;
}

AUTOCONTROL::AUTOCONTROL(QObject *parent) : QObject{parent},
    config(nullptr),
    logger(nullptr),
    mobile(nullptr),
    unimap(nullptr),
    obsmap(nullptr),
    loc(nullptr)
{
    connect(this, SIGNAL(signal_move(DATA_MOVE)), this, SLOT(slot_move(DATA_MOVE)));
    connect(this, SIGNAL(signal_move_backward(DATA_MOVE)), this, SLOT(slot_move_backward(DATA_MOVE)));

    connect(this, SIGNAL(signal_move_multi()), this, SLOT(slot_move_multi()));
}

AUTOCONTROL::~AUTOCONTROL()
{
    // control loop stop
    control_flag = false;
    if(control_thread && control_thread->joinable())
    {
        control_thread->join();
    }
    control_thread.reset();

    // obs loop stop
    obs_flag = false;
    if(obs_thread && obs_thread->joinable())
    {
        obs_thread->join();
    }
    obs_thread.reset();

    // node loop stop
    node_flag = false;
    if(node_thread && node_thread->joinable())
    {
        node_thread->join();
    }
    node_thread.reset();
}

void AUTOCONTROL::init()
{
    params = load_preset(0);

    node_flag = true;
    node_thread = std::make_unique<std::thread>(&AUTOCONTROL::node_loop, this);
}

CTRL_PARAM AUTOCONTROL::load_preset(int preset)
{
    CTRL_PARAM res;

    // read
    QString preset_path = QCoreApplication::applicationDirPath() + "/config/" +
        config->get_robot_type_str() + "/preset_" + QString::number(preset) + ".json";

    QFileInfo info(preset_path);
    if(info.exists() && info.isFile())
    {
        QFile file(preset_path);
        if(file.open(QIODevice::ReadOnly))
        {
            QByteArray data = file.readAll();
            QJsonDocument doc = QJsonDocument::fromJson(data);
            QJsonObject obj = doc.object();

            // param load
            res.LIMIT_V        = obj["LIMIT_V"].toString().toDouble();
            res.LIMIT_W        = obj["LIMIT_W"].toString().toDouble();
            res.LIMIT_V_ACC    = obj["LIMIT_V_ACC"].toString().toDouble();
            res.LIMIT_V_DCC    = obj["LIMIT_V_DCC"].toString().toDouble();
            res.LIMIT_W_ACC    = obj["LIMIT_W_ACC"].toString().toDouble();
            res.LIMIT_PIVOT_W  = obj["LIMIT_PIVOT_W"].toString().toDouble();
            res.ST_V           = obj["ST_V"].toString().toDouble();
            res.ED_V           = obj["ED_V"].toString().toDouble();
            res.DRIVE_T        = obj["DRIVE_T"].toString().toDouble();
            res.DRIVE_H        = obj["DRIVE_H"].toString().toDouble();
            res.DRIVE_A        = obj["DRIVE_A"].toString().toDouble();
            res.DRIVE_B        = obj["DRIVE_B"].toString().toDouble();
            res.DRIVE_L        = obj["DRIVE_L"].toString().toDouble();
            res.DRIVE_K        = obj["DRIVE_K"].toString().toDouble();
            res.DRIVE_EPS      = obj["DRIVE_EPS"].toString().toDouble();

            log_info
            (
                "{} load\n"
                "  LIMIT_V       : {}\n"
                "  LIMIT_W       : {}\n"
                "  LIMIT_V_ACC   : {}\n"
                "  LIMIT_V_DCC   : {}\n"
                "  LIMIT_W_ACC   : {}\n"
                "  LIMIT_PIVOT_W : {}\n"
                "  ST_V          : {}\n"
                "  ED_V          : {}\n"
                "  DRIVE_T       : {}\n"
                "  DRIVE_H       : {}\n"
                "  DRIVE_A       : {}\n"
                "  DRIVE_B       : {}\n"
                "  DRIVE_L       : {}\n"
                "  DRIVE_K       : {}\n"
                "  DRIVE_EPS     : {}",

                preset_path.toStdString(),
                res.LIMIT_V,
                res.LIMIT_W,
                res.LIMIT_V_ACC,
                res.LIMIT_V_DCC,
                res.LIMIT_W_ACC,
                res.LIMIT_PIVOT_W,
                res.ST_V,
                res.ED_V,
                res.DRIVE_T,
                res.DRIVE_H,
                res.DRIVE_A,
                res.DRIVE_B,
                res.DRIVE_L,
                res.DRIVE_K,
                res.DRIVE_EPS
            );

            file.close();
        }
    }
    else
    {
        log_warn("invalid preset path");
    }

    return res;
}

PATH AUTOCONTROL::get_cur_global_path()
{
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return cur_global_path;
}

void AUTOCONTROL::set_cur_global_path(const PATH& val)
{
    std::lock_guard<std::recursive_mutex> lock(mtx);
    cur_global_path = val;
}

void AUTOCONTROL::set_global_step(const std::vector<int>& val)
{
    std::lock_guard<std::mutex> lock(path_mtx);
    global_step = val;
}

std::vector<int> AUTOCONTROL::get_global_step()
{
    std::lock_guard<std::mutex> lock(path_mtx);
    return global_step;
}

PATH AUTOCONTROL::get_cur_local_path()
{
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return cur_local_path;
}

void AUTOCONTROL::set_cur_local_path(const PATH& val)
{
    std::lock_guard<std::recursive_mutex> lock(mtx);
    cur_local_path = val;
}

void AUTOCONTROL::set_path(const std::vector<QString>& _global_node_path, int _global_preset, long long _global_path_time)
{
    if(_global_path_time == global_path_time.load() || _global_node_path.empty())
    {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(path_st_node_mtx);
        path_st_node_id = _global_node_path.front();
    }

    set_global_node_path(_global_node_path);

    std::vector<int> remove_duplicate_step = remove_duplicates_step(_global_node_path);
    set_global_step(remove_duplicate_step);

    if(remove_duplicate_step.size() == 1)
    {
        set_last_step(remove_duplicate_step.front());
    }
    if(remove_duplicate_step.size() > 1)
    {
        set_last_step(1);
    }

    set_global_preset(_global_preset);

    global_path_time.store(_global_path_time);
}

void AUTOCONTROL::set_cur_goal_state(QString str)
{
    std::lock_guard<std::recursive_mutex> lock(mtx);
    cur_move_state = str;
}

void AUTOCONTROL::set_multi_infomation(StateMultiReq val0, StateObsCondition val1, StateCurGoal val2)
{
    std::lock_guard<std::recursive_mutex> lock(mtx);
    if(val0 == StateMultiReq::NONE)
    {
        cur_multi_req = "none";
    }
    else if(val0 == StateMultiReq::RECV_PATH)
    {
        cur_multi_req = "recv_path";
    }
    else if(val0 == StateMultiReq::REQ_PATH)
    {
        cur_multi_req = "req_path";
    }

    if(val1 == StateObsCondition::NONE)
    {
        cur_obs_condition = "none";
    }
    else if(val1 == StateObsCondition::FAR)
    {
        cur_obs_condition = "far";
    }
    else if(val1 == StateObsCondition::NEAR)
    {
        cur_obs_condition = "near";
    }
    else if(val1 == StateObsCondition::VIR)
    {
        cur_obs_condition = "vir";
    }

    if(val2 == StateCurGoal::CANCEL)
    {
        cur_move_state = "cancel";
    }
    else if(val2 == StateCurGoal::COMPLETE)
    {
        cur_move_state = "complete";
    }
    else if(val2 == StateCurGoal::FAIL)
    {
        cur_move_state = "fail";
    }
    else if(val2 == StateCurGoal::MOVE)
    {
        cur_move_state = "move";
    }
    else if(val2 == StateCurGoal::NONE)
    {
        cur_move_state = "none";
    }
    else if(val2 == StateCurGoal::OBSTACLE)
    {
        cur_move_state = "obstacle";
    }
}

void AUTOCONTROL::set_is_rrs(bool val)
{
    is_rrs.store(val);
}

void AUTOCONTROL::set_is_moving(bool val)
{
    is_moving.store(val);
}

void AUTOCONTROL::set_is_pause(bool val)
{
    is_pause.store(val);
}

void AUTOCONTROL::set_is_debug(bool val)
{
    is_debug.store(val);
}

QString AUTOCONTROL::get_cur_move_state()
{
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return cur_move_state;
}

QString AUTOCONTROL::get_obs_condition()
{
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return cur_obs_condition;
}

double AUTOCONTROL::get_obs_dist()
{
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return cur_obs_dist;
}

std::vector<Eigen::Matrix4d> AUTOCONTROL::get_obs_traj()
{
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return obs_traj;
}

int AUTOCONTROL::get_fsm_state()
{
    return (int)fsm_state.load();
}

DATA_MOVE AUTOCONTROL::get_cur_move_info()
{
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return cur_move_info;
}

void AUTOCONTROL::set_cur_move_info(const DATA_MOVE& val)
{
    std::lock_guard<std::recursive_mutex> lock(mtx);
    cur_move_info = val;
}

void AUTOCONTROL::set_last_step(int val)
{
    last_step.store(val);
}

void AUTOCONTROL::set_global_node_path(const std::vector<QString>& val)
{
    std::lock_guard<std::mutex> lock(path_mtx);
    global_node_path = val;
}

void AUTOCONTROL::set_global_preset(int val)
{
    global_preset.store(val);
}

long long AUTOCONTROL::get_global_path_time()
{
    return static_cast<long long>(global_path_time.load());
}

CTRL_PARAM AUTOCONTROL::get_cur_ctrl_params()
{
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return params;
}

Eigen::Vector3d AUTOCONTROL::get_last_cur_pos()
{
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return last_cur_pos;
}

Eigen::Vector3d AUTOCONTROL::get_last_tgt_pos()
{
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return last_tgt_pos;
}

Eigen::Vector3d AUTOCONTROL::get_last_local_goal()
{
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return last_local_goal;
}

void AUTOCONTROL::set_obs_condition(QString str)
{
    std::lock_guard<std::recursive_mutex> lock(mtx);
    cur_obs_condition = str;
}

QString AUTOCONTROL::get_multi_reqest_state()
{
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return cur_multi_req;
}

QString AUTOCONTROL::get_auto_state()
{
    QString auto_state = "stop";
    if(is_pause)
    {
        auto_state = "pause";
    }

    else if(is_moving)
    {
        auto_state = "move";
    }

    if(mobile->get_cur_pdu_state() != "good")
    {
        auto_state = "not ready";
    }

    if(loc->get_cur_loc_state() != "good")
    {
        auto_state = "error";
    }

    if(get_obs_condition() == "vir")
    {
        auto_state = "vir";
    }

    return auto_state;
}

double AUTOCONTROL::get_process_time_control()
{
    return static_cast<double>(process_time_control.load());
}

double AUTOCONTROL::get_process_time_obs()
{
    return static_cast<double>(process_time_obs.load());
}

double AUTOCONTROL::get_process_time_node()
{
    return static_cast<double>(process_time_node.load());
}

double AUTOCONTROL::get_cur_deadzone()
{
    return static_cast<double>(cur_deadzone.load());
}

QString AUTOCONTROL::get_cur_node_id()
{
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return cur_node_id;
}

int AUTOCONTROL::get_last_step()
{
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return last_step;
}

void AUTOCONTROL::set_multi_request(QString str)
{
    std::lock_guard<std::recursive_mutex> lock(mtx);
    cur_multi_req = str;
}

void AUTOCONTROL::stop()
{
    // control loop stop
    stop_control_thread();

    // obs loop stop
    stop_obs_thread();

    clear_control_params();

    set_multi_infomation(StateMultiReq::NONE, StateObsCondition::NONE, StateCurGoal::CANCEL);

    // obsmap clear
    obsmap->clear();
}

void AUTOCONTROL::stop_control_thread()
{
    control_flag = false;
    if(control_thread && control_thread->joinable())
    {
        control_thread->join();
    }
    control_thread.reset();
}

void AUTOCONTROL::stop_obs_thread()
{
    obs_flag = false;
    if(obs_thread && obs_thread->joinable())
    {
        obs_thread->join();
    }
    obs_thread.reset();
}

void AUTOCONTROL::clear_control_params()
{
    mobile->move(0, 0, 0);
    mobile->set_is_auto_move(false);

    // control params clear
    is_moving = false;
    is_pause = false;

    // path clear
    clear_path();
}

void AUTOCONTROL::clear_path()
{
    std::lock_guard<std::recursive_mutex> lock(mtx);

    cur_global_path = PATH();
    Q_EMIT signal_global_path_updated();

    cur_local_path = PATH();
    Q_EMIT signal_local_path_updated();
}

void AUTOCONTROL::slot_move(DATA_MOVE msg)
{
    is_move_backward = false;

    // fill goal node name
    if(msg.goal_node_id != "" && msg.goal_node_name == "")
    {
        NODE* node = unimap->get_node_by_id(msg.goal_node_id);
        if(node)
        {
            msg.goal_node_name = node->name;
        }
    }

    set_cur_move_info(msg);

    if(msg.command == "goal")
    {
        if(is_rrs && config->get_use_multi())
        {
            set_multi_infomation(StateMultiReq::REQ_PATH, StateObsCondition::NO_CHANGE, StateCurGoal::MOVE);
        }
        else
        {
            if(msg.method == "hpp")
            {
                drive_method = DriveMethod::METHOD_HPP;
                log_info("cmd_method: HPP");
            }
            else if(msg.method == "side")
            {
                drive_method = DriveMethod::METHOD_SIDE;
                log_info("cmd_method: SIDE");
            }
            else
            {
                drive_method = DriveMethod::METHOD_PP;
                log_info("cmd_method: PP");
            }

            Eigen::Matrix4d tf = ZYX_to_TF(msg.tgt_pose_vec[0], msg.tgt_pose_vec[1], msg.tgt_pose_vec[2], 0, 0, msg.tgt_pose_vec[3]);
            move_single(tf, msg.preset);
        }
    }
    else if(msg.command == "change_goal")
    {
        log_info("just change goal");
    }
}

void AUTOCONTROL::slot_move_backward(DATA_MOVE msg)
{
    is_move_backward = true;

    if(msg.goal_node_id != "" && msg.goal_node_name == "")
    {
        NODE* node = unimap->get_node_by_id(msg.goal_node_id);
        if(node)
        {
            msg.goal_node_name = node->name;
        }
    }

    set_cur_move_info(msg);

    if(msg.command == "goal")
    {
        if(is_rrs && config->get_use_multi())
        {
            set_multi_infomation(StateMultiReq::REQ_PATH, StateObsCondition::NO_CHANGE, StateCurGoal::MOVE);
            return;
        }

        Eigen::Matrix4d tf = ZYX_to_TF(msg.tgt_pose_vec[0], msg.tgt_pose_vec[1], msg.tgt_pose_vec[2], 0, 0, msg.tgt_pose_vec[3]);
        move_single_backward(tf, msg.preset);

    }
    else if(msg.command == "change_goal")
    {
        log_info("just change goal");
    }
}

void AUTOCONTROL::slot_move_multi()
{
    move_multi();
}

void AUTOCONTROL::move_single(Eigen::Matrix4d goal_tf, int preset)
{
    // stop first
    stop();

    initial_drive_method = drive_method;

    // load preset
    params = load_preset(preset);

    // calc global path
    PATH path = calc_global_path(goal_tf);
    if(path.pos.size() > 0)
    {
        std::vector<PATH> policy_path = policy->drive_policy(path);

        std::vector<PATH> tmp_storage;
        tmp_storage.reserve(policy_path.size());
        for(size_t i = 0; i < policy_path.size(); i++)
        {
            const PATH& seg = policy_path[i];

            QString method = seg.drive_method.toUpper();
            if(method == "HPP")
            {
                drive_method = DriveMethod::METHOD_HPP;
            }
            else if(method == "SIDE")
            {
                drive_method = DriveMethod::METHOD_SIDE;
            }
            else
            {
                drive_method = initial_drive_method;
            }

            PATH _seg = calc_global_path(seg.node, i == 0);

            _seg.drive_dir = seg.drive_dir;
            _seg.drive_method = seg.drive_method;
            _seg.is_final   = seg.is_final;
            _seg.ed_tf = seg.is_final ? path.ed_tf : _seg.pose.back();

            log_info("[AUTO] Segment {} -> Method: {}, Dir: {}, Final: {}",
                         i,
                         _seg.drive_method.toStdString(),
                         (seg.drive_dir == DriveDir::FORWARD ? "FWD" : "REV"),
                         (seg.is_final ? "true" : "false"));

            tmp_storage.push_back(std::move(_seg));
        }

        // enque global path
        global_path_que.clear();
        if(!tmp_storage.empty())
        {
            for(size_t i = 0; i < tmp_storage.size(); i++)
            {
                global_path_que.push(tmp_storage[i]);
            }
        }
        else
        {
            global_path_que.push(path);
        }
    }

    // explicitly change flag (racing issue: control_loop <-> obs_loop)
    is_moving = true;
    mobile->set_is_auto_move(true);

    // start control loop
    if(control_flag == false)
    {
        control_flag = true;
        control_thread = std::make_unique<std::thread>(&AUTOCONTROL::control_loop, this);
    }

    if(obs_flag == false)
    {
        obs_flag = true;
        obs_thread = std::make_unique<std::thread>(&AUTOCONTROL::obs_loop, this);
    }
}

void AUTOCONTROL::move_multi()
{
    std::lock_guard<std::mutex> lock(path_mtx);
    if(global_node_path.size() == 0 || global_preset < 0)
    {
        log_info("node_path.size() == 0 || preset < 0");
        return;
    }

    is_move_backward = false;

    // control params clear
    is_pause = false;
    is_moving = false;
    mobile->set_is_auto_move(false);

    // symmetric cut
    std::vector<std::vector<QString>> path_list = symmetric_cut(global_node_path);

    // loop cut
    std::vector<std::vector<QString>> path_list2;
    for(size_t p = 0; p < path_list.size(); p++)
    {
        std::vector<std::vector<QString>> res = loop_cut(path_list[p]);
        for(size_t q = 0; q < res.size(); q++)
        {
            path_list2.push_back(res[q]);
        }
    }

    if(path_list2.size() == 0)
    {
        log_info("move_pp, path_list2 empty");
        stop();
        return;
    }

    // set flag
    set_multi_infomation(StateMultiReq::RECV_PATH, StateObsCondition::NO_CHANGE, StateCurGoal::NO_CHANGE);

    log_info("move_pp, recv path check");
    for(size_t p = 0; p < path_list2.size(); p++)
    {
        log_info("path_{}", p);
        for(size_t q = 0; q < path_list2[p].size(); q++)
        {
            log_info("{}", qUtf8Printable(path_list2[p][q]));
        }
    }

    // set path
    std::vector<PATH> tmp_storage;
    for(size_t p = 0; p < path_list2.size(); p++)
    {
        // enque path
        PATH path = calc_global_path(path_list2[p], p == 0);

        // check final path
        if(p == path_list2.size()-1)
        {
            NODE* node = unimap->get_node_by_id(path_list2[p].back());
            if(node == nullptr)
            {
                log_info("move_pp, path invalid");
                stop();
                return;
            }
        }

        tmp_storage.push_back(path);
    }

    // control loop shutdown but robot still moving
    if(control_flag)
    {
        // check path overlap or reset
        bool is_curve = false;
        std::vector<Eigen::Matrix4d> merged_tf_list;
        for(size_t p = 0; p < tmp_storage.size(); p++)
        {
            for(size_t q = 0; q < tmp_storage[p].pose.size(); q++)
            {
                merged_tf_list.push_back(tmp_storage[p].pose[q]);
            }
        }

        Eigen::Matrix4d cur_tf = loc->get_cur_tf();
        Eigen::Vector3d cur_xi = TF_to_se2(cur_tf);
        for(int p = 0; p < std::min<int>((int)merged_tf_list.size(), AUTOCONTROL_INFO::path_overlap_check_dist); p++)
        {
            Eigen::Vector3d xi = TF_to_se2(merged_tf_list[p]);
            double th = deltaRad(xi[2], cur_xi[2]);
            if(std::abs(th) > AUTOCONTROL_INFO::path_overlap_check_deg * D2R)
            {
                is_curve = true;
                break;
            }
        }

        is_path_overlap = true;

        stop_control_thread();

        if(is_curve)
        {
            mobile->move(0,0,0);
            log_info("move_pp, curve detected, stop");
        }
        else
        {
            log_info("move_pp, no curve, just change path");
        }
    }

    stop_obs_thread();

    // set global path
    global_path_que.clear();
    for(size_t p = 0; p < tmp_storage.size(); p++)
    {
        global_path_que.push(tmp_storage[p]);
    }

    // load preset
    params = load_preset(global_preset);

    // start obs loop
    if(obs_flag == false)
    {
        obs_flag = true;
        obs_thread = std::make_unique<std::thread>(&AUTOCONTROL::obs_loop, this);
    }

    // start control loop
    if(control_flag == false)
    {
        control_flag = true;
        control_thread = std::make_unique<std::thread>(&AUTOCONTROL::control_loop, this);
    }
}

std::vector<QString> AUTOCONTROL::remove_duplicates(std::vector<QString> node_path)
{
    constexpr int duplicates_check_min_node_size = 2;
    if(node_path.size() < duplicates_check_min_node_size)
    {
        return node_path;
    }

    std::vector<QString> res;
    res.push_back(node_path[0]);
    for(size_t p = 1; p < node_path.size(); p++)
    {
        if(res.back() == node_path[p])
        {
            continue;
        }

        res.push_back(node_path[p]);
    }

    return res;
}

std::vector<int> AUTOCONTROL::remove_duplicates_step(const std::vector<QString>& node_path)
{
    constexpr int duplicates_check_min_node_size = 2;
    if(node_path.size() < duplicates_check_min_node_size)
    {
        std::vector<int> step;
        for(size_t p = 0; p < node_path.size(); p++)
        {
            step.push_back(static_cast<int>(p+1));
        }
        return step;
    }

    std::vector<int> duplicated_step;
    for(size_t p = 1; p < node_path.size()-1; p++)
    {
        if(node_path[p] == node_path[p+1])
        {
            continue;
        }

        duplicated_step.push_back(static_cast<int>(p+1));
    }

    duplicated_step.push_back(static_cast<int>(node_path.size()));

    return duplicated_step;
}

std::vector<std::vector<QString>> AUTOCONTROL::symmetric_cut(std::vector<QString> node_path)
{
    constexpr int symmetric_cut_min_node_size = 3;
    std::vector<QString> path = remove_duplicates(node_path);
    if(path.size() >= symmetric_cut_min_node_size)
    {
        // find symmetric
        std::vector<int> split_idxs;
        for(size_t p = 1; p < path.size()-1; p++)
        {
            if(path[p-1] == path[p+1])
            {
                split_idxs.push_back(p);
            }
        }

        // split
        if(split_idxs.size() == 0)
        {
            std::vector<std::vector<QString>> res;
            res.push_back(path);
            return res;
        }
        else
        {
            std::vector<std::vector<QString>> res;

            int idx0 = 0;
            for(size_t i = 0; i < split_idxs.size(); i++)
            {
                int idx1 = split_idxs[i];
                std::vector<QString> sub_path(path.begin() + idx0, path.begin() + (idx1+1));
                res.push_back(sub_path);
                idx0 = idx1;
            }

            if(idx0 < (int)path.size()-1)
            {
                std::vector<QString> sub_path(path.begin() + idx0, path.end());
                res.push_back(sub_path);
            }

            return res;
        }
    }
    else
    {
        std::vector<std::vector<QString>> res;
        res.push_back(path);
        return res;
    }
}

void AUTOCONTROL::move_single_backward(Eigen::Matrix4d goal_tf, int preset)
{
    // stop first
    stop();

    // load preset
    params = load_preset(preset);

    // calc global path
    PATH path = calc_global_path(goal_tf);
    if(path.pos.size() > 0)
    {
        path.drive_dir = DriveDir::REVERSE;

        // enque global path
        global_path_que.clear();
        global_path_que.push(path);
    }

    // explicitly change flag (racing issue: control_loop <-> obs_loop)
    is_moving = true;
    mobile->set_is_auto_move(true);

    // start control loop
    if(control_flag == false)
    {
        control_flag = true;
        control_thread = std::make_unique<std::thread>(&AUTOCONTROL::control_loop, this);
    }

    if(obs_flag == false)
    {
        obs_flag = true;
        obs_thread = std::make_unique<std::thread>(&AUTOCONTROL::obs_loop, this);
    }
}

void AUTOCONTROL::move_multi_backward()
{
    std::lock_guard<std::mutex> lock(path_mtx);
    if(global_node_path.size() == 0 || global_preset < 0)
    {
        log_info("node_path.size() == 0 || preset < 0");
        return;
    }

    is_move_backward = true;

    // control params clear
    is_pause = false;
    is_moving = false;
    mobile->set_is_auto_move(false);

    // symmetric cut
    std::vector<std::vector<QString>> path_list = symmetric_cut(global_node_path);

    // loop cut
    std::vector<std::vector<QString>> path_list2;
    for(size_t p = 0; p < path_list.size(); p++)
    {
        std::vector<std::vector<QString>> res = loop_cut(path_list[p]);
        for(size_t q = 0; q < res.size(); q++)
        {
            path_list2.push_back(res[q]);
        }
    }

    if(path_list2.size() == 0)
    {
        log_info("move_pp, path_list2 empty");
        stop();
        return;
    }

    // set flag
    set_multi_infomation(StateMultiReq::RECV_PATH, StateObsCondition::NO_CHANGE, StateCurGoal::NO_CHANGE);

    log_info("move_pp, recv path check");
    for(size_t p = 0; p < path_list2.size(); p++)
    {
        log_info("path_{}", p);
        for(size_t q = 0; q < path_list2[p].size(); q++)
        {
            log_info("{}", qUtf8Printable(path_list2[p][q]));
        }
    }

    // set path
    std::vector<PATH> tmp_storage;
    for(size_t p = 0; p < path_list2.size(); p++)
    {
        // enque path
        PATH path = calc_global_path(path_list2[p], p == 0);

        // check final path
        if(p == path_list2.size()-1)
        {
            NODE* node = unimap->get_node_by_id(path_list2[p].back());
            if(node == nullptr)
            {
                log_info("move_pp, path invalid");
                stop();
                return;
            }
        }

        tmp_storage.push_back(path);
    }

    // control loop shutdown but robot still moving
    if(control_flag)
    {
        // check path overlap or reset
        bool is_curve = false;
        std::vector<Eigen::Matrix4d> merged_tf_list;
        for(size_t p = 0; p < tmp_storage.size(); p++)
        {
            for(size_t q = 0; q < tmp_storage[p].pose.size(); q++)
            {
                merged_tf_list.push_back(tmp_storage[p].pose[q]);
            }
        }

        Eigen::Matrix4d cur_tf = loc->get_cur_tf();
        Eigen::Vector3d cur_xi = TF_to_se2(cur_tf);
        for(int p = 0; p < std::min<int>((int)merged_tf_list.size(), AUTOCONTROL_INFO::path_overlap_check_dist); p++)
        {
            Eigen::Vector3d xi = TF_to_se2(merged_tf_list[p]);
            double th = deltaRad(xi[2], cur_xi[2]);
            if(std::abs(th) > AUTOCONTROL_INFO::path_overlap_check_deg * D2R)
            {
                is_curve = true;
                break;
            }
        }

        is_path_overlap = true;

        stop_control_thread();

        if(is_curve)
        {
            mobile->move(0,0,0);
            log_info("move_pp, curve detected, stop");
        }
        else
        {
            log_info("move_pp, no curve, just change path");
        }
    }

    // obs loop shutdown but robot still moving
    stop_obs_thread();

    // set global path
    global_path_que.clear();
    for(size_t p = 0; p < tmp_storage.size(); p++)
    {
        global_path_que.push(tmp_storage[p]);
    }

    // load preset
    params = load_preset(global_preset);

    // start obs loop
    if(obs_flag == false)
    {
        obs_flag = true;
        obs_thread = std::make_unique<std::thread>(&AUTOCONTROL::obs_loop, this);
    }

    // start control loop
    if(control_flag == false)
    {
        control_flag = true;
        control_thread = std::make_unique<std::thread>(&AUTOCONTROL::control_loop, this);
    }
}

std::vector<std::vector<QString>> AUTOCONTROL::loop_cut(std::vector<QString> node_path)
{
    constexpr int loop_cut_min_node_size = 4;
    std::vector<std::vector<QString>> res;
    if(node_path.size() < loop_cut_min_node_size)
    {
        res.push_back(node_path);
        return res;
    }

    // check loop
    std::vector<QString> tmp;
    for(size_t p = 0; p < node_path.size(); p++)
    {
        auto it = std::find(tmp.begin(), tmp.end(), node_path[p]);
        if(it == tmp.end())
        {
            tmp.push_back(node_path[p]);
        }
        else
        {
            QString _last_node_id = tmp.back();
            res.push_back(tmp);

            tmp.clear();
            tmp.push_back(_last_node_id);
            tmp.push_back(node_path[p]);
        }
    }

    // set last segment
    res.push_back(tmp);
    return res;
}

bool AUTOCONTROL::get_is_moving()
{
    return (bool)is_moving.load();
}

bool AUTOCONTROL::get_is_pause()
{
    return (bool)is_pause.load();
}

bool AUTOCONTROL::get_is_debug()
{
    return (bool)is_debug.load();
}

Eigen::Matrix4d AUTOCONTROL::get_approach_pose(Eigen::Matrix4d tf0, Eigen::Matrix4d tf1, Eigen::Matrix4d cur_tf)
{
    Eigen::Matrix4d res = Eigen::Matrix4d::Identity();

    std::vector<Eigen::Matrix4d> src;
    src.push_back(tf0);
    src.push_back(tf1);

    std::vector<Eigen::Matrix4d> dst = path_resampling(src, AUTOCONTROL_INFO::local_path_step);

    double min_d = std::numeric_limits<double>::max();
    for(size_t p = 0; p < dst.size(); p++)
    {
        double d = calc_dist_2d(dst[p].block(0,3,3,1) - cur_tf.block(0,3,3,1));
        if(d < min_d)
        {
            min_d = d;
            res = dst[p];
        }
    }

    return res;
}

PATH AUTOCONTROL::calc_global_path(Eigen::Matrix4d goal_tf)
{
    Eigen::Matrix4d cur_tf = loc->get_cur_tf();

    // get st node id
    Eigen::Vector3d cur_pos = cur_tf.block(0,3,3,1);
    QString st_node_id = unimap->get_node_id_edge(cur_pos);
    if(st_node_id == "")
    {
        log_info("st_node_id empty");
        return PATH();
    }

    // set ed node id
    Eigen::Vector3d goal_pos = goal_tf.block(0,3,3,1);
    QString ed_node_id = unimap->get_node_id_edge(goal_pos);
    if(ed_node_id == "")
    {
        log_info("ed_node_id empty");
        return PATH();
    }

    // topology path finding
    std::vector<QString> node_path = calc_node_path(st_node_id, ed_node_id);
    if(node_path.size() == 0)
    {
        log_info("node_path empty");
        return PATH();
    }

    // if the robot is on an edge, ensure both edge endpoints are included
    {
        std::vector<QString> edge_nodes = unimap->get_edge_nodes(cur_pos);
        if(edge_nodes.size() == 2 && node_path.size() > 0)
        {
            bool has0 = false;
            bool has1 = false;
            for(size_t i = 0; i < node_path.size(); i++)
            {
                if(node_path[i] == edge_nodes[0])
                {
                    has0 = true;
                }
                if(node_path[i] == edge_nodes[1])
                {
                    has1 = true;
                }
            }

            if(!(has0 && has1))
            {
                QString alt_st = "";
                if(node_path.front() == edge_nodes[0] && !has1)
                {
                    alt_st = edge_nodes[1];
                }
                else if(node_path.front() == edge_nodes[1] && !has0)
                {
                    alt_st = edge_nodes[0];
                }

                if(!alt_st.isEmpty() && alt_st != st_node_id)
                {
                    std::vector<QString> _node_path = calc_node_path(alt_st, ed_node_id);
                    if(_node_path.size() > 0)
                    {
                        node_path.swap(_node_path);
                        st_node_id = alt_st;
                    }
                }
            }
        }
    }

    // convert metric path
    std::vector<Eigen::Matrix4d> node_pose;
    for(size_t p = 0; p < node_path.size(); p++)
    {
        QString node_id = node_path[p];
        NODE* node = unimap->get_node_by_id(node_id);
        if(!node)
        {
            log_info("{}: node null", qUtf8Printable(node_id));
            return PATH();
        }
        node_pose.push_back(node->tf);
    }

    // add cur pos
    if(node_pose.size() == 1)
    {
        node_pose.insert(node_pose.begin(), cur_tf);
    }
    else
    {
        // approach point
        Eigen::Matrix4d app = get_approach_pose(node_pose[0], node_pose[1], cur_tf);
        node_pose.erase(node_pose.begin());
        node_pose.insert(node_pose.begin(), app);
        node_pose.insert(node_pose.begin(), cur_tf);
    }

    // add goal pos
    if(!node_pose.back().isApprox(goal_tf))
    {
        node_pose.push_back(goal_tf);
    }

    // divide and smooth metric path
    std::vector<Eigen::Matrix4d> path_pose;
    std::vector<Eigen::Vector3d> path_pos;

    if(drive_method == DriveMethod::METHOD_HPP || drive_method == DriveMethod::METHOD_SIDE)
    {
        for(size_t p = 0; p < node_pose.size(); p++)
        {
            Eigen::Matrix4d tf0 = node_pose[p];
            path_pose.push_back(tf0);
        }
    }
    else
    {
        path_pose = reorientation_path(node_pose);
    }
    path_pose = path_resampling(path_pose, AUTOCONTROL_INFO::global_path_step);

    for(size_t p = 0; p < path_pose.size(); p++)
    {
        path_pos.push_back(path_pose[p].block(0,3,3,1));
    }

    // set ref_v
    std::vector<double> ref_v;
    calc_ref_v(path_pose, ref_v, params.ST_V, AUTOCONTROL_INFO::global_path_step);
    ref_v.back() = params.ED_V;

    // smoothing ref_v
    ref_v = smoothing_v(ref_v, AUTOCONTROL_INFO::global_path_step);

    // set result
    PATH res;
    res.t = get_time();
    res.node = node_path;
    res.pose = path_pose;
    res.pos = path_pos;
    res.ref_v = ref_v;
    res.ed_tf = goal_tf;
    res.is_final = true;
    return res;
}

// for given path
PATH AUTOCONTROL::calc_global_path(std::vector<QString> node_path, bool add_cur_tf)
{
    NODE* ed_node = unimap->get_node_by_id(node_path.back());
    if(ed_node == nullptr)
    {
        return PATH();
    }

    Eigen::Matrix4d ed_tf = ed_node->tf;

    // convert metric path
    std::vector<Eigen::Matrix4d> node_pose;
    for(size_t p = 0; p < node_path.size(); p++)
    {
        QString node_id = node_path[p];
        NODE* node = unimap->get_node_by_id(node_id);
        if(node == nullptr)
        {
            log_info("{}: node null", qUtf8Printable(node_id));
            return PATH();
        }
        node_pose.push_back(node->tf);
    }

    // add cur pos
    if(add_cur_tf)
    {
        Eigen::Matrix4d cur_tf = loc->get_cur_tf();
        if(node_pose.size() == 1)
        {
            node_pose.insert(node_pose.begin(), cur_tf);
        }
        else
        {
            Eigen::Matrix4d app = get_approach_pose(node_pose[0], node_pose[1], cur_tf);
            node_pose.erase(node_pose.begin());
            node_pose.insert(node_pose.begin(), app);
            node_pose.insert(node_pose.begin(), cur_tf);
        }
    }

    // add ed pos
    if(!node_pose.back().isApprox(ed_tf))
    {
        node_pose.push_back(ed_tf);
    }

    // divide and smooth metric path
    std::vector<Eigen::Matrix4d> path_pose;
    std::vector<Eigen::Vector3d> path_pos;

    if(drive_method == DriveMethod::METHOD_HPP || drive_method == DriveMethod::METHOD_SIDE)
    {
        for(size_t p = 0; p < node_pose.size(); p++)
        {
            Eigen::Matrix4d tf0 = node_pose[p];
            path_pose.push_back(tf0);
        }
    }
    else
    {
        path_pose = reorientation_path(node_pose);
    }
    path_pose = path_resampling(path_pose, AUTOCONTROL_INFO::global_path_step);

    for(size_t p = 0; p < path_pose.size(); p++)
    {
        path_pos.push_back(path_pose[p].block(0,3,3,1));
    }

    // set ref_v
    std::vector<double> ref_v;
    if(add_cur_tf)
    {
        Eigen::Vector3d control_input = mobile->get_control_input();
        calc_ref_v(path_pose, ref_v, std::max<double>(params.ST_V, control_input[0]), AUTOCONTROL_INFO::global_path_step);
    }
    else
    {
        calc_ref_v(path_pose, ref_v, params.ST_V, AUTOCONTROL_INFO::global_path_step);
    }
    ref_v.back() = params.ED_V;

    // smoothing ref_v
    ref_v = smoothing_v(ref_v, AUTOCONTROL_INFO::global_path_step);

    // set result
    PATH res;
    res.t = get_time();
    res.node = node_path;
    res.pose = path_pose;
    res.pos = path_pos;
    res.ref_v = ref_v;
    res.ed_tf = ed_tf;
    return res;
}

std::vector<QString> AUTOCONTROL::calc_node_path(QString st_node_id, QString ed_node_id)
{
    // set st node, ed node
    NODE* st_node = unimap->get_node_by_id(st_node_id);
    NODE* ed_node = unimap->get_node_by_id(ed_node_id);
    if(st_node == nullptr || ed_node == nullptr)
    {
        log_info("st_node or ed_node null");
        return std::vector<QString>();
    }

    log_info("topo_path_finding, st: {} -> ed: {}", qUtf8Printable(st_node->id), qUtf8Printable(ed_node->id));

    // no need path finding
    if(st_node->id == ed_node->id)
    {
        log_info("st_node same as ed_node, just set ed_node");

        std::vector<QString> res;
        res.push_back(ed_node_id);
        return res;
    }

    std::vector<LINK> links = unimap->get_links();

    // astar search
    std::vector<ASTAR_NODE*> open_set;
    std::vector<ASTAR_NODE*> close_set;

    ASTAR_NODE *ed = new ASTAR_NODE();
    ed->node = ed_node;

    ASTAR_NODE *st = new ASTAR_NODE();
    st->node = st_node;
    st->g = 0;
    st->h = (ed->node->tf.block(0,3,3,1) - st->node->tf.block(0,3,3,1)).norm();
    st->f = st->g + st->h;
    open_set.push_back(st);

    while(open_set.size() > 0)
    {
        // get best node
        int cur_idx = 0;
        ASTAR_NODE *cur = open_set.front();
        for(size_t p = 0; p < open_set.size(); p++)
        {
            if(open_set[p]->f < cur->f)
            {
                cur = open_set[p];
                cur_idx = p;
            }
        }

        // pop open_set, push close_set
        open_set.erase(open_set.begin()+cur_idx);
        close_set.push_back(cur);

        // found goal
        if(cur->node->id == ed->node->id)
        {
            std::vector<QString> res;

            ASTAR_NODE* _cur = cur;
            while(_cur != NULL)
            {
                res.push_back(_cur->node->id);
                _cur = _cur->parent;
            }

            std::reverse(res.begin(), res.end());

            //printf("[AUTO] topo_path_finding complete, num:%d\n", (int)res.size());
            log_info("topo_path_finding complete, num: {}", (int)res.size());
            return res;
        }

        // append child node
        std::vector<QString> around;
        QString cur_id = cur->node->id;
        for(size_t p = 0; p < links.size(); p++)
        {
            const LINK &link = links[p];
            if(link.st_id != cur_id)
            {
                continue;
            }

            QString neighbor_id = link.ed_id;

            bool is_close = false;
            for(size_t q = 0; q < close_set.size(); q++)
            {
                if(close_set[q]->node->id == neighbor_id)
                {
                    is_close = true;
                    break;
                }
            }

            if(is_close == false)
            {
                around.push_back(neighbor_id);
            }
        }

        // calc child node
        for(size_t p = 0; p < around.size(); p++)
        {
            // calc heuristics
            NODE* node = unimap->get_node_by_id(around[p]);
            if(node == nullptr)
            {
                continue;
            }

            double g = cur->g + (node->tf.block(0,3,3,1) - cur->node->tf.block(0,3,3,1)).norm();
            double h = (node->tf.block(0,3,3,1) - ed->node->tf.block(0,3,3,1)).norm();
            double f = g + h;

            // check open set
            bool is_open_set = false;
            ASTAR_NODE* open_node = NULL;
            for(size_t q = 0; q < open_set.size(); q++)
            {
                if(open_set[q]->node->id == node->id)
                {
                    is_open_set = true;
                    open_node = open_set[q];
                    break;
                }
            }

            if(is_open_set)
            {
                if(g < open_node->g)
                {
                    open_node->parent = cur;
                    open_node->g = g;
                    open_node->h = h;
                    open_node->f = f;
                    continue;
                }
                else
                {
                    continue;
                }
            }

            // add new child to open set
            ASTAR_NODE *child = new ASTAR_NODE();
            child->parent = cur;
            child->node = node;
            child->g = g;
            child->h = h;
            child->f = f;
            open_set.push_back(child);
        }
    }

    log_warn("topo_path_finding, failed");
    return std::vector<QString>();
}

std::vector<Eigen::Vector3d> AUTOCONTROL::path_resampling(const std::vector<Eigen::Vector3d>& src, double step)
{
    if(src.size() < 2)
    {
        return src;
    }

    std::vector<Eigen::Vector3d> sampled_path;
    sampled_path.push_back(src.front());

    Eigen::Vector3d pre = src.front();
    double accumulated_dist = 0.0;

    for(size_t i = 1; i < src.size(); i++)
    {
        Eigen::Vector3d cur = src[i];
        double segment_length = (cur - pre).norm();

        while(accumulated_dist + segment_length >= step)
        {
            double t = (step - accumulated_dist) / segment_length;
            Eigen::Vector3d pt = pre + t*(cur-pre);
            sampled_path.push_back(pt);

            pre = pt;
            segment_length -= (step - accumulated_dist);
            accumulated_dist = 0.0;
        }

        accumulated_dist += segment_length;
        pre = cur;
    }

    if(sampled_path.back() != src.back())
    {
        sampled_path.push_back(src.back());
    }

    return sampled_path;
}

std::vector<Eigen::Matrix4d> AUTOCONTROL::path_resampling(const std::vector<Eigen::Matrix4d>& src, double step)
{
    if(src.size() < 2)
    {
        return src;
    }

    std::vector<Eigen::Matrix4d> sampled_path;
    sampled_path.push_back(src.front());

    Eigen::Matrix4d pre = src.front();
    Eigen::Vector3d pre_translation = pre.block<3,1>(0,3);
    Eigen::Quaterniond pre_rotation(pre.block<3,3>(0,0));
    double accumulated_dist = 0.0;

    for(size_t i = 1; i < src.size(); i++)
    {
        Eigen::Matrix4d cur = src[i];
        Eigen::Vector3d cur_translation = cur.block<3,1>(0,3);
        Eigen::Quaterniond cur_rotation(cur.block<3,3>(0,0));

        double segment_length = (cur_translation - pre_translation).norm();

        while(accumulated_dist + segment_length >= step)
        {
            double t = (step - accumulated_dist) / segment_length;

            Eigen::Vector3d interpolated_translation = pre_translation + t * (cur_translation - pre_translation);
            Eigen::Quaterniond interpolated_rotation = pre_rotation.slerp(t, cur_rotation);

            Eigen::Matrix4d interpolated_transform = Eigen::Matrix4d::Identity();
            interpolated_transform.block<3,3>(0,0) = interpolated_rotation.toRotationMatrix();
            interpolated_transform.block<3,1>(0,3) = interpolated_translation;

            sampled_path.push_back(interpolated_transform);

            pre_translation = interpolated_translation;
            pre_rotation = interpolated_rotation;

            segment_length -= (step - accumulated_dist);
            accumulated_dist = 0.0;
        }

        accumulated_dist += segment_length;
        pre_translation = cur_translation;
        pre_rotation = cur_rotation;
    }

    if(sampled_path.back() != src.back())
    {
        sampled_path.push_back(src.back());
    }

    return sampled_path;
}

std::vector<Eigen::Vector3d> AUTOCONTROL::path_ccma(const std::vector<Eigen::Vector3d>& src)
{
    // init params
    const int w_ma = 3;
    const int w_cc = 2;
    const int w_ccma = w_ma + w_cc + 1;

    // not enough points
    if(src.size() < w_ccma)
    {
        return src;
    }
    // padding func
    auto add_padding = [](std::vector<Eigen::Vector3d> points, int n_padding)->std::vector<Eigen::Vector3d>
    {
        std::vector<Eigen::Vector3d> padded;

        for (int i = 0; i < n_padding; ++i)
        {
            padded.push_back(points.front());
        }

        padded.insert(padded.end(), points.begin(), points.end());

        for (int i = 0; i < n_padding; ++i)
        {
            padded.push_back(points.back());
        }

        return padded;
    };

    // calc unit vector function
    auto get_unit_vector = [](const Eigen::Vector3d vec)->Eigen::Vector3d
    {
        double norm = vec.norm();
        if(norm > 0.0)
        {
            return vec/norm;
        }
        else
        {
            return vec;
        }
    };

    // calc pascal's triangle distribution
    std::function<std::vector<int>(int)> get_pascal_row = [&](int row_index)
    {
        std::vector<int> cur_row(1, 1);
        if(row_index == 0)
        {
            return cur_row;
        }

        std::vector<int> prev = get_pascal_row(row_index - 1);

        for(int idx = 1; idx < (int)prev.size(); idx++)
        {
            int cur = prev[idx-1] + prev[idx];
            cur_row.push_back(cur);
        }

        cur_row.push_back(1);
        return cur_row;
    };

    // calc weight kernels
    std::vector<std::vector<double>> weights_ma;
    for(int i = 0; i < w_ma + 1; i++)
    {
        int pascal_row_index = i * 2;
        std::vector<int> row = get_pascal_row(pascal_row_index);
        int row_sum = std::accumulate(row.begin(), row.end(), 0);

        std::vector<double> tmp;
        for(int j = 0; j < (int)row.size(); j++)
        {
            tmp.push_back((double)row[j]/row_sum);
        }

        weights_ma.push_back(tmp);
    }

    std::vector<std::vector<double>> weights_cc;
    for(int i = 0; i < w_cc + 1; i++)
    {
        int pascal_row_index = i * 2;
        std::vector<int> row = get_pascal_row(pascal_row_index);
        int row_sum = std::accumulate(row.begin(), row.end(), 0);

        std::vector<double> tmp;
        for(int j = 0; j < (int)row.size(); j++)
        {
            tmp.push_back((double)row[j]/row_sum);
        }

        weights_cc.push_back(tmp);
    }

    // convert 2d points to 3d points
    std::vector<Eigen::Vector3d> points;
    for(size_t p = 0; p < src.size(); p++)
    {
        points.push_back(Eigen::Vector3d(src[p][0], src[p][1], 0));
    }
    points = add_padding(points, w_ccma);

    // calc moving average points
    std::vector<Eigen::Vector3d> points_ma;
    for(int i = w_ma; i < (int)points.size() - w_ma; i++)
    {
        // convolution
        Eigen::Vector3d pt(0,0,0);
        for(int j = -w_ma; j <= w_ma; j++)
        {
            pt += weights_ma[w_ma][w_ma+j]*points[i+j];
        }

        points_ma.push_back(pt);
    }

    // calc curvature vector
    std::vector<Eigen::Vector3d> curvature_vector(points_ma.size(), Eigen::Vector3d(0,0,0));
    for (size_t p = 1; p < points_ma.size() - 1; p++)
    {
        Eigen::Vector3d p0 = points_ma[p-1];
        Eigen::Vector3d p1 = points_ma[p];
        Eigen::Vector3d p2 = points_ma[p+1];

        Eigen::Vector3d v1 = p1 - p0;
        Eigen::Vector3d v2 = p2 - p1;
        Eigen::Vector3d cross = v1.cross(v2);
        double cross_norm = cross.norm();

        double curvature = 0.0;
        if (cross_norm > 0.0)
        {
            double radius = v1.norm()*v2.norm()*(p2-p0).norm()/(2*cross_norm);
            curvature = 1.0/radius;
        }
        else
        {
            curvature = 0;
        }

        curvature_vector[p] = curvature * get_unit_vector(cross);
    }

    // calc curvatures
    std::vector<double> curvatures(curvature_vector.size(), 0);
    for(size_t p = 0; p < curvature_vector.size(); p++)
    {
        curvatures[p] = curvature_vector[p].norm();
    }

    // calc alphas
    std::vector<double> alphas(points_ma.size(), 0);
    for (size_t p = 1; p < points_ma.size()-1; p++)
    {
        if (curvatures[p] > 0.0)
        {
            double radius = 1.0/curvatures[p];
            double dist_neighbors = (points_ma[p+1] - points_ma[p-1]).norm();
            alphas[p] = std::sin((dist_neighbors/2)/radius);
        }
        else
        {
            alphas[p] = 0.0;
        }
    }

    // calc radii_ma
    std::vector<double> radii_ma(alphas.size(), 0);
    for (size_t i = 1; i < alphas.size()-1; i++)
    {
        radii_ma[i] = weights_ma[w_ma][w_ma];
        for (int k = 1; k <= w_ma; k++)
        {
            radii_ma[i] += 2*std::cos(alphas[i]*k) * weights_ma[w_ma][w_ma+k];
        }

        // Apply threshold
        radii_ma[i] = std::max(0.35, radii_ma[i]);
    }

    // calc ccma
    std::vector<Eigen::Vector3d> points_ccma(points.size() - 2*w_ccma, Eigen::Vector3d(0,0,0));
    for (size_t idx = 0; idx < points_ccma.size(); idx++)
    {
        // Get tangent vector for the shifting point
        Eigen::Vector3d unit_tangent = get_unit_vector(points_ma[w_cc + idx + 1 + 1] - points_ma[w_cc + idx - 1 + 1]);

        // Calculate the weighted shift
        Eigen::Vector3d shift(0,0,0);
        for (int idx_cc = 0; idx_cc < 2*w_cc + 1; idx_cc++)
        {
            if (curvatures[idx + idx_cc + 1] > 0.0)
            {
                Eigen::Vector3d u = get_unit_vector(curvature_vector[idx + idx_cc + 1]);
                double weight = weights_cc[w_cc][idx_cc];
                double shift_magnitude = (1.0/curvatures[idx + idx_cc + 1]) * (1.0/radii_ma[idx + idx_cc + 1] - 1);
                shift += u * weight * shift_magnitude;
            }
        }

        // Reconstruction
        points_ccma[idx] = points_ma[idx + w_cc + 1] + unit_tangent.cross(shift);
    }

    // result
    std::vector<Eigen::Vector3d> res(src.size(), Eigen::Vector3d(0, 0, 0));
    res[0] = src[0];
    for(size_t p = 1; p < points_ccma.size()-1; p++)
    {
        res[p] = Eigen::Vector3d(points_ccma[p][0], points_ccma[p][1], src[p][2]);
    }
    res.back() = src.back();
    return res;
}

void AUTOCONTROL::calc_ref_v(const std::vector<Eigen::Matrix4d>& src, std::vector<double>& ref_v, double st_v, double step)
{
    if(src.size() == 1)
    {
        std::vector<double> _ref_v(src.size(), st_v);
        ref_v = _ref_v;
        return;
    }

    const int ld = params.DRIVE_L/step;

    if(drive_method == DriveMethod::METHOD_HPP || drive_method == DriveMethod::METHOD_SIDE)
    {
        std::vector<double> _ref_v(src.size(), params.LIMIT_V);
        for(size_t p = 0; p < src.size()-1; p++)
        {
            int cur_idx = p;
            int tgt_idx = p+ld;
            if(tgt_idx > (int)src.size()-1)
            {
                tgt_idx = src.size()-1;
            }

            Eigen::Vector3d xi0 = TF_to_se2(src[cur_idx]);
            Eigen::Vector3d xi1 = TF_to_se2(src[tgt_idx]);

            double err_d = calc_dist_2d(xi1 - xi0);
            double err_th = deltaRad(xi1[2], xi0[2]) * params.DRIVE_H;

            double t_v = err_d/params.LIMIT_V;
            double t_w = std::abs(err_th)/(params.LIMIT_W*D2R);
            double t = std::max<double>(t_v, t_w);
            double v = err_d/t;
            _ref_v[p] = v;
        }

        _ref_v[_ref_v.size()-1] = _ref_v[_ref_v.size()-2];
        _ref_v[0] = st_v;

        // update result
        ref_v = _ref_v;
    }
    else
    {
        std::vector<double> _ref_th(src.size(), 0);
        for(size_t p = 0; p < src.size(); p++)
        {
            int cur_idx = p-ld;
            if(cur_idx < 0)
            {
                cur_idx = 0;
            }

            int tgt_idx = p+ld;
            if(tgt_idx > (int)src.size()-1)
            {
                tgt_idx = src.size()-1;
            }

            if(cur_idx == tgt_idx)
            {
                continue;
            }

            double dx = src[tgt_idx](0,3) - src[cur_idx](0,3);
            double dy = src[tgt_idx](1,3) - src[cur_idx](1,3);
            double th = std::atan2(dy, dx);
            _ref_th[p] = th;
        }

        std::vector<double> _ref_v(src.size(), params.LIMIT_V);
        for(size_t p = 0; p < src.size(); p++)
        {
            int cur_idx = p-ld;
            if(cur_idx < 0)
            {
                cur_idx = 0;
            }

            int tgt_idx = p+ld;
            if(tgt_idx > (int)src.size()-1)
            {
                tgt_idx = src.size()-1;
            }

            if(cur_idx == tgt_idx)
            {
                continue;
            }

            double dx = src[tgt_idx](0,3) - src[cur_idx](0,3);
            double dy = src[tgt_idx](1,3) - src[cur_idx](1,3);
            double err_d = std::sqrt(dx*dx + dy*dy);
            double err_th = deltaRad(std::atan2(dy, dx), _ref_th[cur_idx]) * params.DRIVE_H;

            double t_v = err_d/params.LIMIT_V;
            double t_w = std::abs(err_th)/(params.LIMIT_W*D2R);
            double t = std::max<double>(t_v, t_w);
            double v = err_d/t;

            _ref_v[p] = v;

        }
        _ref_v[0] = st_v;

        // update result
        ref_v = _ref_v;
    }
}

std::vector<double> AUTOCONTROL::smoothing_v(const std::vector<double>& src, double path_step)
{
    const double v_limit = params.LIMIT_V;
    const double v_acc = params.LIMIT_V_ACC;
    const double v_dcc = params.LIMIT_V_DCC;
    const double v_end = params.ED_V;

    std::vector<double> list0(src.size());
    double v0 = src.front();
    for(int p = 0; p < (int)src.size(); p++)
    {
        double v1 = std::sqrt(2*v_acc*path_step + v0*v0);
        if(v1 > v_limit)
        {
            v1 = v_limit;
        }

        if(src[p] > v0)
        {
            v0 = v1;
        }
        else
        {
            v0 = src[p];
        }

        list0[p] = v0;
    }

    std::vector<double> list1(src.size());
    v0 = src.back(); // v_end;
    for(int p = (int)src.size()-1; p >= 0; p--)
    {
        double v1 = std::sqrt(2*v_dcc*path_step + v0*v0);
        if(v1 > v_limit)
        {
            v1 = v_limit;
        }

        if(src[p] > v0)
        {
            v0 = v1;
        }
        else
        {
            v0 = src[p];
        }

        list1[p] = v0;
    }

    std::vector<double> res(src.size());
    for(size_t p = 0; p < src.size(); p++)
    {
        res[p] = std::min<double>(list0[p], list1[p]);
    }
    return res;
}

// for local path planning
std::vector<Eigen::Matrix4d> AUTOCONTROL::calc_trajectory(Eigen::Vector3d cur_vel, double dt, double predict_t, Eigen::Matrix4d _cur_tf)
{
    std::vector<Eigen::Matrix4d> res;

    double vx = cur_vel[0];
    double vy = cur_vel[1];
    double wz = cur_vel[2];

    double pre_x = 0;
    double pre_y = 0;
    double pre_th = 0;

    for(double t = 0; t <= predict_t; t += dt)
    {
        pre_th = pre_th + wz*dt;

        Eigen::Matrix3d pre_rot = Eigen::Matrix3d::Zero();

        pre_rot << std::cos(pre_th), -std::sin(pre_th), 0,
                std::sin(pre_th),  std::cos(pre_th), 0,
                0,                 0, 1;

        Eigen::Vector3d local_vel = Eigen::Vector3d(vx, vy, 0);
        Eigen::Vector3d vel = pre_rot*local_vel;

        pre_x += vel[0]*dt;
        pre_y += vel[1]*dt;

        Eigen::Matrix4d G = se2_to_TF(Eigen::Vector3d(pre_x, pre_y, pre_th));
        Eigen::Matrix4d predict_G = _cur_tf*G;
        res.push_back(predict_G);
    }

    return res;
}

int AUTOCONTROL::get_nn_idx(std::vector<Eigen::Vector3d>& path, Eigen::Vector3d cur_pos)
{
    if(path.size() == 1)
    {
        return 0;
    }
    else
    {
        int min_idx = 0;
        double min_d = 99999999;
        for(size_t p = 0; p < path.size()-1; p++)
        {
            // convert to 2D
            path[p][2] = 0.0;
            path[p+1][2] = 0.0;
            cur_pos[2] = 0.0;

            double d = calc_seg_dist(path[p], path[p+1], cur_pos);
            if(d < min_d)
            {
                min_d = d;

                double d0 = (path[p]-cur_pos).norm();
                double d1 = (path[p+1]-cur_pos).norm();
                if(d0 < d1)
                {
                    min_idx = p;
                }
                else
                {
                    min_idx = p+1;
                }
            }
        }
        return min_idx;
    }
}

PATH AUTOCONTROL::calc_local_path(PATH& global_path)
{
    // get cur params
    Eigen::Matrix4d cur_tf = loc->get_cur_tf();
    Eigen::Vector3d cur_pos = cur_tf.block(0,3,3,1);
    int cur_idx = get_nn_idx(global_path.pos, cur_pos);

    // get global path segment
    std::vector<Eigen::Matrix4d> _path_pose;
    std::vector<Eigen::Vector3d> _path_pos;
    int range = config->get_obs_local_goal_dist()/AUTOCONTROL_INFO::global_path_step;
    int st_idx = saturation(cur_idx - 10, 0, global_path.pos.size()-2);
    int ed_idx = saturation(cur_idx + range, 0, global_path.pos.size()-1);
    for(int p = st_idx; p <= ed_idx; p++)
    {
        _path_pose.push_back(global_path.pose[p]);
        _path_pos.push_back(global_path.pos[p]);
    }
    double st_v = global_path.ref_v[st_idx];

    if(_path_pose.size() == 1)
    {
        std::vector<double> ref_v;
        ref_v.push_back(st_v);

        // set result
        PATH res;
        res.t = get_time();
        res.pose = _path_pose;
        res.pos = _path_pos;
        res.ref_v = ref_v;
        res.ed_tf = _path_pose.back(); // local goal
        return res;
    }
    else
    {
        std::vector<Eigen::Matrix4d> path_pose;
        std::vector<Eigen::Vector3d> path_pos;

        if(drive_method == DriveMethod::METHOD_HPP || drive_method == DriveMethod::METHOD_SIDE)
        {
            // resampling
            path_pose = path_resampling(_path_pose, AUTOCONTROL_INFO::local_path_step);
            for(size_t p = 0; p < path_pose.size(); p++)
            {
                path_pos.push_back(path_pose[p].block(0,3,3,1));
            }
        }
        else
        {
            // resampling
            path_pose = reorientation_path(_path_pose);
            path_pose = path_resampling(path_pose, AUTOCONTROL_INFO::local_path_step);

            for(size_t p = 0; p < path_pose.size(); p++)
            {
                path_pos.push_back(path_pose[p].block(0,3,3,1));
            }

            // ccma
            path_pos = path_ccma(path_pos);
            for(size_t p = 0; p < path_pose.size(); p++)
            {
                path_pose[p].block(0,3,3,1) = path_pos[p];
            }
        }

        // calc ref_v
        std::vector<double> ref_v;
        calc_ref_v(path_pose, ref_v, st_v, AUTOCONTROL_INFO::local_path_step);

        // test
        PATH tmp;
        tmp.pose = path_pose;
        tmp.pos = path_pos;
        tmp.node = global_path.node;
        policy->speed_policy(tmp, ref_v);

        // check global path end
        double d = calc_dist_2d(global_path.ed_tf.block(0,3,3,1) - path_pos.back());
        if(d < config->get_drive_goal_dist())
        {
            ref_v.back() = params.ED_V;

            int edv_padding_num = std::min((int)(AUTOCONTROL_INFO::global_path_step/AUTOCONTROL_INFO::local_path_step*2), (int)ref_v.size());
            for(int i = 0; i < edv_padding_num; i++)
            {
                ref_v[ref_v.size() - i - 1] = params.ED_V;
            }

        }

        // smoothing ref_v
        ref_v = smoothing_v(ref_v, AUTOCONTROL_INFO::local_path_step);
        //////////////ms 추가////////////////////
        if(is_move_backward == true)
        {
            for(auto& v : ref_v)
                v = -v;
        }

        // set result
        PATH res;
        res.t = get_time();
        res.pose = path_pose;
        res.pos = path_pos;
        res.ref_v = ref_v;
        res.ed_tf = path_pose.back(); // local goal
        return res;
    }
}

PATH AUTOCONTROL::calc_local_path_with_cur_vel(PATH& global_path)
{
    // get current pose & velocity
    Eigen::Matrix4d cur_tf = loc->get_cur_tf();
    Eigen::Vector3d cur_vel = mobile->get_control_input();

    if (is_move_backward)
    {
        // flip robot frame for back mode
        Eigen::Matrix4d Rz = Eigen::Matrix4d::Identity();
        Rz(0,0) =  cos(M_PI);  Rz(0,1) = -sin(M_PI);
        Rz(1,0) =  sin(M_PI);  Rz(1,1) =  cos(M_PI);
        cur_tf = cur_tf * Rz;

        // make current velocity negative
        cur_vel[0] = -std::abs(cur_vel[0]);
    }

    Eigen::Vector3d cur_pos = cur_tf.block(0,3,3,1);
    int cur_idx = get_nn_idx(global_path.pos, cur_pos);

    // get local path segment
    std::vector<Eigen::Matrix4d> _path_pose;
    std::vector<Eigen::Vector3d> _path_pos;
    int range = config->get_obs_local_goal_dist() / AUTOCONTROL_INFO::global_path_step;
    int st_idx = saturation(cur_idx - 10, 0, global_path.pos.size()-2);
    int ed_idx = saturation(cur_idx + range, 0, global_path.pos.size()-1);

    for(int p = st_idx; p <= ed_idx; p++)
    {
        _path_pose.push_back(global_path.pose[p]);
        _path_pos.push_back(global_path.pos[p]);
    }

    double st_v = global_path.ref_v[st_idx];
    if (is_move_backward)
    {
        st_v = - std::abs(st_v);

    }
    if(_path_pose.size() == 1)
    {
        std::vector<double> ref_v{ st_v };
        PATH res;
        res.t = get_time();
        res.pose = _path_pose;
        res.pos = _path_pos;
        res.ref_v = ref_v;
        res.ed_tf = _path_pose.back();
        return res;
    }

    // resample & smooth
    std::vector<Eigen::Matrix4d> path_pose = reorientation_path(_path_pose);
    path_pose = path_resampling(path_pose, AUTOCONTROL_INFO::local_path_step);

    std::vector<Eigen::Vector3d> path_pos;
    for(size_t i=0; i<path_pose.size(); i++)
    {
        path_pos.push_back(path_pose[i].block(0,3,3,1));
    }

    // ccma smoothing
    path_pos = path_ccma(path_pos);
    for(size_t i=0; i<path_pose.size(); i++)
    {
        path_pose[i].block(0,3,3,1) = path_pos[i];
    }

    // calc ref_v
    std::vector<double> ref_v;
    calc_ref_v(path_pose, ref_v, st_v, AUTOCONTROL_INFO::local_path_step);

    // back mode: force all velocities negative
    if(is_move_backward)
    {
        for(size_t i=0; i<ref_v.size(); i++)
        {
            ref_v[i] = -std::abs(ref_v[i]);
        }
    }

    // adjust initial part to current velocity
    int cur_local_idx = get_nn_idx(path_pos, cur_pos);
    for(int i=0; i <= cur_local_idx && i < ref_v.size(); i++)
    {
        ref_v[i] = cur_vel[0];
    }

    // check global path end
    double d = calc_dist_2d(global_path.ed_tf.block(0,3,3,1) - path_pos.back());
    if(d < config->get_drive_goal_dist())
    {
        ref_v.back() = params.ED_V;
        int edv_padding_num = std::min((int)(AUTOCONTROL_INFO::global_path_step/AUTOCONTROL_INFO::local_path_step*2), (int)ref_v.size());
        for(int i = 0; i < edv_padding_num; i++)
        {
            ref_v[ref_v.size() - i - 1] = params.ED_V;
        }
    }

    // smoothing ref_v
    ref_v = smoothing_v(ref_v, AUTOCONTROL_INFO::local_path_step);

    // set result
    PATH res;
    res.t = get_time();
    res.pose = path_pose;
    res.pos = path_pos;
    res.ref_v = ref_v;
    res.ed_tf = path_pose.back();
    return res;
}

PATH AUTOCONTROL::calc_avoid_path(PATH& global_path)
{
    double st_time = get_time();

    // get cur params
    Eigen::Vector3d cur_vel = mobile->get_pose().vel;
    Eigen::Matrix4d cur_tf = loc->get_cur_tf();
    Eigen::Vector3d cur_pos = cur_tf.block(0,3,3,1);
    int cur_idx = get_nn_idx(global_path.pos, cur_pos);

    // get local goal
    int tgt_idx = cur_idx + config->get_obs_local_goal_dist()/AUTOCONTROL_INFO::global_path_step;
    if(tgt_idx > (int)global_path.pose.size()-1)
    {
        tgt_idx = global_path.pose.size()-1;
    }

    // local goal collision check
    Eigen::Matrix4d tgt_tf = global_path.pose[tgt_idx];
    {
        const int range = 0.5/AUTOCONTROL_INFO::global_path_step;
        int _tgt_idx = tgt_idx;
        for(int p = cur_idx; p <= tgt_idx - range; p++)
        {
            bool is_collision = false;
            for(int q = 0; q <= range; q++)
            {
                int i = p + q;
                if(obsmap->is_tf_collision(global_path.pose[i], false, config->get_obs_path_margin_x(), config->get_obs_path_margin_y()))
                {
                    is_collision = true;
                    break;
                }
            }

            if(!is_collision)
            {
                _tgt_idx = p + range;
            }
        }

        // update
        tgt_idx = _tgt_idx;
        tgt_tf = global_path.pose[_tgt_idx];
    }

    // set st, ed
    Eigen::Matrix4d st_tf = cur_tf;
    Eigen::Matrix4d ed_tf = tgt_tf;

    // for plot
    {
        std::lock_guard<std::recursive_mutex> lock(mtx);
        last_cur_pos = st_tf.block(0,3,3,1);
        last_local_goal = ed_tf.block(0,3,3,1);
    }

    // solve
    std::vector<Eigen::Matrix4d> path_pose = obsmap->calc_path(st_tf, ed_tf);
    if(path_pose.size() > 0)
    {
        // sample and interpolation
        path_pose = reorientation_path(path_pose);
        path_pose = path_resampling(path_pose, AUTOCONTROL_INFO::local_path_step);
        std::vector<Eigen::Vector3d> path_pos;
        for(size_t p = 0; p < path_pose.size(); p++)
        {
            path_pos.push_back(path_pose[p].block(0,3,3,1));
        }

        // set ref_v
        std::vector<double> ref_v;
        calc_ref_v(path_pose, ref_v, params.ST_V, AUTOCONTROL_INFO::local_path_step);
        ref_v.back() = params.ED_V;

        // smoothing ref_v
        ref_v = smoothing_v(ref_v, AUTOCONTROL_INFO::local_path_step);

        // set result
        PATH res;
        res.t = get_time();
        res.pose = path_pose;
        res.pos = path_pos;
        res.ref_v = ref_v;
        res.ed_tf = path_pose.back(); // avoid goal

        log_info("hybrid astar lpp, exact solution found");
        return res;
    }
    else
    {
        log_info("hybrid astar lpp, solution failed");
        return PATH();
    }
}

// check condition
int AUTOCONTROL::is_everything_fine()
{
    // check localization
    QString loc_state = loc->get_cur_loc_state();
    if(loc_state == "none" || loc_state == "fail")
    {
        log_error("localization fail, auto-drive stop");
        {
            std::lock_guard<std::recursive_mutex> lock(mtx);
            cur_move_info.time    = get_time();
            cur_move_info.result  = "fail";
            cur_move_info.message = "loc failed";
            Q_EMIT signal_move_response(cur_move_info);
        }
        return DRIVING_FAILED;
    }

    // skip motor check in simulation mode
    if(config->get_use_sim())
    {
        return DRIVING_FINE;
    }

    MOBILE_STATUS ms = mobile->get_status();
    if(ms.connection_m0 != 1 || ms.connection_m1 != 1)
    {
        log_error("failed (motor not connected)");
        {
            std::lock_guard<std::recursive_mutex> lock(mtx);
            cur_move_info.time    = get_time();
            cur_move_info.result  = "fail";
            cur_move_info.message = "not connected";
            Q_EMIT signal_move_response(cur_move_info);
        }
        return DRIVING_FAILED;
    }

    // for multiple error detect!
    if(ms.status_m0 > 1 || ms.status_m1 > 1)
    {
        int motor_err_code = (ms.status_m0 > 1) ? ms.status_m0 : ms.status_m1;
        QStringList err_list;
        QString err_str = "";

        for(int bit = 0; bit < 8; bit++)
        {
            if((motor_err_code >> bit) & 0x01)
            {
                const char* err_names[8] = {"BIT0", "MOD", "JAM", "CUR", "BIG", "IN", "PSI", "NON"};
                QString name = err_names[bit];
                err_list << name;

                log_error("failed (motor error name: {}, bit: {})", name.toStdString(), (1 << bit));
            }
        }

        if(!err_list.isEmpty())
        {
            err_str = err_list.join(" ");
            if(err_list.size() > 2)
            {
                log_error("multiple motor errors detected ({} errors): {}", err_list.size(), err_str.toStdString());
            }
            else
            {
                log_error("failed (multi motor error: {})", err_str.toStdString());
            }
        }

        std::lock_guard<std::recursive_mutex> lock(mtx);
        cur_move_info.time    = get_time();
        cur_move_info.result  = "fail";
        cur_move_info.message = err_str;
        Q_EMIT signal_move_response(cur_move_info);

        return DRIVING_FAILED;
    }

    if(ms.charge_state == 1)
    {
        log_error("failed (robot charging)");
        {
            std::lock_guard<std::recursive_mutex> lock(mtx);
            cur_move_info.time    = get_time();
            cur_move_info.result  = "fail";
            cur_move_info.message = "charging";
            Q_EMIT signal_move_response(cur_move_info);
        }
        return DRIVING_FAILED;
    }

    if(ms.status_m0 == 0 && ms.status_m1 == 0)
    {
        log_error("not ready (mootor lock off)");
        {
            std::lock_guard<std::recursive_mutex> lock(mtx);
            cur_move_info.time    = get_time();
            cur_move_info.result  = "fail";
            cur_move_info.message = "lock off";
            Q_EMIT signal_move_response(cur_move_info);
        }
        return DRIVING_NOT_READY;
    }

    return DRIVING_FINE;
}

void AUTOCONTROL::control_loop()
{
    // set flag
    is_moving = true;
    mobile->set_is_auto_move(true);

    // set state
    set_multi_infomation(StateMultiReq::RECV_PATH, StateObsCondition::NONE, StateCurGoal::MOVE);

    RobotModel robot_model = config->get_robot_model();

    // check global path
    log_info("global path que size: {}", global_path_que.unsafe_size());

    PATH global_path;
    if(global_path_que.try_pop(global_path))
    {
        // update global path and goal_tf
        {
            std::lock_guard<std::recursive_mutex> lock(mtx);
            cur_global_path = global_path;

            log_info("deque global path, size: {}", cur_global_path.pose.size());
        }

        Q_EMIT signal_global_path_updated();

        // method
        QString method = global_path.drive_method.toUpper();
        if(method == "HPP")
        {
            drive_method = DriveMethod::METHOD_HPP;
        }
        else if(method == "SIDE")
        {
            drive_method = DriveMethod::METHOD_SIDE;
        }
        else
        {
            drive_method = initial_drive_method;
        }

        is_move_backward = ((global_path.drive_dir == (DriveDir::REVERSE)) && drive_method == (DriveMethod::METHOD_PP));

        log_info("set back_mode from PATH: {}", (is_move_backward ? "true" : "false"));
    }

    if(global_path.pose.size() == 0)
    {
        clear_control_params();
        set_multi_infomation(StateMultiReq::NONE, StateObsCondition::NONE, StateCurGoal::FAIL);
        log_info("global path invalid");
        return;
    }

    // update goal
    Eigen::Matrix4d goal_tf = global_path.ed_tf;
    Eigen::Vector3d goal_xi = TF_to_se2(goal_tf);
    Eigen::Vector3d goal_pos = goal_tf.block(0,3,3,1);

    // set initial state
    fsm_state = ((drive_method == DriveMethod::METHOD_HPP || drive_method == DriveMethod::METHOD_SIDE) ?
        AUTO_FSM_DRIVING : AUTO_FSM_FIRST_ALIGN);
    if(drive_method == DriveMethod::METHOD_HPP)
    {
        log_info("CommandMethod:HPP ----- FSM_STATE : {}", fsm_state.load());
    }
    else if(drive_method == DriveMethod::METHOD_PP)
    {
        log_info("CommandMethod:PP ----- FSM_STATE : {}", fsm_state.load());
    }

    // check already goal
    if(global_path_que.unsafe_size() == 0)
    {
        Eigen::Matrix4d cur_tf = loc->get_cur_tf();
        if(fsm_state != AUTO_FSM_FINAL_ALIGN && is_move_backward == true)
        {
            Eigen::Matrix4d Rz = Eigen::Matrix4d::Identity();
            Rz(0,0) =  cos(M_PI);  Rz(0,1) = -sin(M_PI);
            Rz(1,0) =  sin(M_PI);  Rz(1,1) =  cos(M_PI);

            cur_tf = cur_tf*Rz;
        }

        Eigen::Vector3d dxi = TF_to_se2(cur_tf.inverse()*goal_tf);
        double err_d = calc_dist_2d(dxi);
        double err_th = std::abs(dxi[2]);
        if(err_d < config->get_drive_goal_dist())
        {
            if(!global_path.is_final)
            {
                clear_control_params();
                set_multi_infomation(StateMultiReq::NONE, StateObsCondition::NONE, StateCurGoal::MOVE);

                fsm_state = AUTO_FSM_COMPLETE;
                log_info("COMPLETE (already temp goal), err_d: {}", err_d);
                return;
            }

            // final goal reached
            if(err_th < config->get_drive_goal_th()*D2R)
            {
                // move response
                Eigen::Vector3d cur_pos = cur_tf.block(0,3,3,1);
                {
                    {
                        std::lock_guard<std::recursive_mutex> lock(mtx);
                        cur_move_info.cur_pos = cur_pos;
                        cur_move_info.result = "success";
                        cur_move_info.message = "already goal";
                        cur_move_info.time = get_time();
                    }
                    Q_EMIT signal_move_response(cur_move_info);
                }

                clear_control_params();
                set_multi_infomation(StateMultiReq::NONE, StateObsCondition::NONE, StateCurGoal::COMPLETE);

                fsm_state = AUTO_FSM_COMPLETE;
                log_info("COMPLETE (already goal), err_d: {}, err_th: {}", err_d, err_th*R2D);
                return;
            }
            else
            {
                // jump to final align
                fsm_state = AUTO_FSM_FINAL_ALIGN;
                log_info("jump to final align state, err_d: {}, err_th: {}", err_d, err_th*R2D);
            }
        }
    }
    log_info("initial fsm state: {}", AUTO_FSM_STATE_STR[fsm_state.load()].toStdString());

    // path storage
    PATH local_path;
    PATH avoid_path;

    // loop params
    const double dt = 0.05; // 20hz
    double pre_loop_time = get_time();

    // control params
    double extend_dt  = 0;
    double pre_err_th = 0;

    // for obs
    int obs_state = AUTO_OBS_CHECK;
    int obs_value = OBS_NONE;
    double obs_wait_st_time = 0;

    log_info("start control loop");
    while(control_flag)
    {
        // get current status
        Eigen::Matrix4d cur_tf = loc->get_cur_tf();
        if(fsm_state != AUTO_FSM_FINAL_ALIGN && is_move_backward == true)
        {
            Eigen::Matrix4d Rz = Eigen::Matrix4d::Identity();
            Rz(0,0) =  cos(M_PI);  Rz(0,1) = -sin(M_PI);
            Rz(1,0) =  sin(M_PI);  Rz(1,1) =  cos(M_PI);

            cur_tf = cur_tf*Rz;
        }

        Eigen::Vector3d cur_xi     = TF_to_se2(cur_tf);
        Eigen::Vector3d cur_pos    = cur_tf.block(0,3,3,1);
        Eigen::Vector3d cur_vel    = mobile->get_control_input();
        Eigen::Matrix4d cur_tf_inv = cur_tf.inverse();

        // for plot
        {
            std::lock_guard<std::recursive_mutex> lock(mtx);
            last_cur_pos = cur_pos;
        }

        // check everything
        int is_good_everything = is_everything_fine();
        if(is_good_everything == DRIVING_FAILED || is_good_everything == DRIVING_NOT_READY)
        {
            fsm_state = AUTO_FSM_COMPLETE;

            // clear
            clear_control_params();
            set_multi_infomation(StateMultiReq::NONE, StateObsCondition::NONE, StateCurGoal::FAIL);
            log_error("Driving failed");
            return;
        }

        // pause
        if(is_pause)
        {
            mobile->move(0, 0, 0);

            // set ref_v as ST_V to already passed global path
            int cur_global_idx = get_nn_idx(global_path.pos, cur_pos);
            for(int ref_v_index = 0; ref_v_index < cur_global_idx-1; ref_v_index++)
            {
                global_path.ref_v[ref_v_index] = params.ST_V;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // calc local path
        if(fsm_state != AUTO_FSM_FINAL_ALIGN)
        {
            if(avoid_path.pos.size() > 0)
            {
                // clear avoid path
                double goal_err_d = calc_dist_2d(avoid_path.pos.back() - goal_pos);
                if(goal_err_d >= config->get_drive_goal_dist())
                {
                    int avoid_idx = get_nn_idx(avoid_path.pos, cur_pos);
                    if(avoid_idx > avoid_path.pos.size() * AUTOCONTROL_INFO::avoid_path_check_idx)
                    {
                        // clear avoid path
                        avoid_path = PATH();
                        log_info("clear avoid_path");
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        continue;
                    }
                }

                // set avoid path to local path
                local_path = avoid_path;
            }
            else
            {
                // update local path
                if(get_time() - local_path.t > AUTOCONTROL_INFO::local_path_calc_dt)
                {
                    local_path = calc_local_path(global_path);
                }
            }

            set_cur_local_path(local_path);

            // for plot
            {
                std::lock_guard<std::recursive_mutex> lock(mtx);
                last_local_goal = local_path.ed_tf.block(0,3,3,1);
            }

            Q_EMIT signal_local_path_updated();
        }

        // finite state machine
        if(fsm_state == AUTO_FSM_FIRST_ALIGN)
        {
            // find tgt
            int cur_idx = get_nn_idx(local_path.pos, cur_pos);
            int tgt_idx = cur_idx + params.DRIVE_L/AUTOCONTROL_INFO::local_path_step;
            if(tgt_idx > (int)local_path.pos.size()-1)
            {
                tgt_idx = local_path.pos.size()-1;
            }

            Eigen::Vector3d tgt_pos = local_path.pos[tgt_idx];
            {
                std::lock_guard<std::recursive_mutex> lock(mtx);
                last_tgt_pos = tgt_pos;
            }

            // calc error
            double dx = local_path.pos[tgt_idx][0] - local_path.pos[cur_idx][0];
            double dy = local_path.pos[tgt_idx][1] - local_path.pos[cur_idx][1];
            double err_th = deltaRad(std::atan2(dy,dx), cur_xi[2]);

            // goal check
            if(std::abs(err_th) < config->get_drive_goal_th()*D2R)
            {
                mobile->move(0, 0, 0);

                fsm_state = AUTO_FSM_DRIVING;

                extend_dt  = 0;
                pre_err_th = 0;
                prev_local_ref_v_index    = 0;  // set 0 index
                cur_pos_at_start_driving  = cur_pos;
                ref_v_oscilation_end_flag = false;

                log_info("FIRST_ALIGN -> DRIVING, err_th: {}", err_th*R2D);

                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            // pivot control
            double kp = params.DRIVE_A;
            double kd = params.DRIVE_B;
            double w0 = cur_vel[2];
            double w = kp*err_th + kd*(err_th - pre_err_th)/dt;
            pre_err_th = err_th;

            w = saturation(w, w0 - params.LIMIT_W_ACC*D2R*dt, w0 + params.LIMIT_W_ACC*D2R*dt);
            w = saturation(w, -params.LIMIT_PIVOT_W*D2R, params.LIMIT_PIVOT_W*D2R);

            // for safe
            double v0 = mobile->get_pose().vel[0];
            if(std::abs(v0) > 0.1)
            {
                w = 0;
            }

            // obs check
            double temp_w = w;
            if(std::abs(temp_w) < 0.001)
            {
                temp_w = 0.001;
            }

            double temp_predict_time = std::min(abs(err_th/temp_w),config->get_obs_predict_time());
            std::vector<Eigen::Matrix4d> traj = calc_trajectory(Eigen::Vector3d(0, 0, w), 0.2, temp_predict_time, cur_tf);

            obs_value = obsmap->is_path_collision(traj, true);
            if(obs_value != OBS_NONE)
            {
                mobile->move(0, 0, 0);

                fsm_state = AUTO_FSM_OBS;
                obs_state = AUTO_OBS_CHECK;
                set_multi_infomation(StateMultiReq::NO_CHANGE, StateObsCondition::NO_CHANGE, StateCurGoal::OBSTACLE);

                log_info("FIRST_ALIGN -> OBS, err_th: {}", err_th*R2D);
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            // send control
            if(!is_debug)
            {
                mobile->move(0, 0, w);
            }
        }
        else if(fsm_state == AUTO_FSM_DRIVING)
        {
            // find tgt
            int cur_idx = get_nn_idx(local_path.pos, cur_pos);
            int tgt_idx = cur_idx + params.DRIVE_L/AUTOCONTROL_INFO::local_path_step;
            if(tgt_idx > (int)local_path.pos.size()-1)
            {
                tgt_idx = local_path.pos.size()-1;
            }

            Eigen::Matrix4d tgt_tf  = local_path.pose[tgt_idx];
            Eigen::Vector3d tgt_pos = local_path.pos[tgt_idx];
            {
                std::lock_guard<std::recursive_mutex> lock(mtx);
                last_tgt_pos = tgt_pos;
            }

            // goal check
            int max_idx = global_path.pos.size()-1;
            int goal_idx = get_nn_idx(global_path.pos, cur_pos);

            if(max_idx - goal_idx < 2)
            {
                Eigen::Vector3d _goal_pos = cur_tf_inv.block(0,0,3,3)*goal_pos + cur_tf_inv.block(0,3,3,1);
                double v0 = cur_vel[0];
                double v;
                double remain_dt = std::max(config->get_drive_extended_control_time() - extend_dt, 0.01);
                if(is_move_backward)
                {
                    double v_ref = -std::abs(config->get_drive_goal_approach_gain()*_goal_pos[0]/remain_dt);
                    double v_min = std::max(v0 - params.LIMIT_V_DCC*dt, -params.ED_V);
                    double v_max = 0.0;

                    v = saturation(v_ref, v_min, v_max);
                }
                else
                {
                    v = saturation(config->get_drive_goal_approach_gain()*_goal_pos[0]/remain_dt, v0 - params.LIMIT_V_DCC*dt, v0 + params.LIMIT_V_DCC*dt);
                }

                if(drive_method == DriveMethod::METHOD_HPP || drive_method == DriveMethod::METHOD_SIDE)
                {
                    double vx = saturation(config->get_drive_goal_approach_gain()*_goal_pos[0]/remain_dt, cur_vel[0] - params.LIMIT_V_DCC*dt, cur_vel[0] + params.LIMIT_V_ACC*dt);
                    double vy = saturation(config->get_drive_goal_approach_gain()*_goal_pos[1]/remain_dt, cur_vel[1] - params.LIMIT_V_DCC*dt, cur_vel[1] + params.LIMIT_V_ACC*dt);

                    double _err_th = deltaRad(goal_xi[2], cur_xi[2]);
                    double k_wz = 1.0 - std::exp(-std::abs(_err_th) * 3.0);
                    double wz = k_wz * saturation(config->get_drive_goal_approach_gain() * _err_th, -3.0 * D2R, 3.0 * D2R);

                    // send control
                    if(robot_model == RobotModel::QD)
                    {
                        if(drive_method == DriveMethod::METHOD_HPP)
                        {
                            mobile->moveQD(vx, vy, wz, 1);
                        }
                        else if(drive_method == DriveMethod::METHOD_SIDE)
                        {
                            mobile->moveQD(vx, vy, wz, 2);
                        }
                    }
                    else
                    {
                        mobile->move(vx, vy, wz);
                    }
                }
                else
                {
                    if(!is_move_backward)
                    {
                        double temp_v = v0;
                        if(v0 > 0)
                        {
                            temp_v -= params.LIMIT_V_DCC*dt;
                            if(temp_v < params.ED_V)
                            {
                                temp_v = params.ED_V;
                            }
                            v = saturation(v, -temp_v, temp_v);
                        }
                    }
                    mobile->move(v, 0, 0);
                }

                extend_dt += dt;
                if(extend_dt > config->get_drive_extended_control_time())
                {
                    log_debug("extended ctrl time!");

                    mobile->move(0, 0, 0);
                    extend_dt = 0;
                    pre_err_th = 0;

                    double goal_err_d = calc_dist_2d(_goal_pos);
                    if(global_path.is_final)
                    {
                        fsm_state = AUTO_FSM_FINAL_ALIGN;

                        log_info("DRIVING -> FINAL_ALIGN, err_d: {}", goal_err_d);
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        continue;
                    }
                    else
                    {
                        // deque
                        PATH _global_path;
                        if(global_path_que.try_pop(_global_path))
                        {
                            // method
                            QString method = _global_path.drive_method.toUpper();
                            if(method == "HPP")
                            {
                                drive_method = DriveMethod::METHOD_HPP;
                            }
                            else if(method == "SIDE")
                            {
                                drive_method = DriveMethod::METHOD_SIDE;
                            }
                            else
                            {
                                drive_method = initial_drive_method;
                            }

                            if(drive_method == DriveMethod::METHOD_HPP || drive_method == DriveMethod::METHOD_SIDE)
                            {
                                fsm_state = AUTO_FSM_DRIVING;
                            }
                            else
                            {
                                fsm_state = AUTO_FSM_FIRST_ALIGN;
                            }
                            log_info("CommandMethod:{} ----- FSM_STATE : {}", method.toStdString(), fsm_state.load());

                            // update global path
                            global_path = _global_path;
                            log_info("next global path, deque global path, size: {}", global_path.pos.size());

                            is_move_backward = ((global_path.drive_dir == (DriveDir::REVERSE)) && drive_method == (DriveMethod::METHOD_PP));
                            log_info("next segment back_mode: {}", is_move_backward.load() ? "true" : "false");

                            // update global goal
                            goal_tf = global_path.ed_tf;
                            goal_pos = goal_tf.block(0,3,3,1);
                            goal_xi = TF_to_se2(goal_tf);

                            // update local path
                            local_path = calc_local_path(global_path);

                            avoid_path = PATH();

                            set_cur_global_path(global_path);
                            Q_EMIT signal_global_path_updated();

                            set_cur_local_path(local_path);
                            Q_EMIT signal_local_path_updated();

                            // update global & local path
                            {
                                std::lock_guard<std::recursive_mutex> lock(mtx);
                                last_local_goal = local_path.ed_tf.block(0,3,3,1);
                            }

                            log_info("next global path, DRIVING -> {}, err_d: {}", goal_err_d, fsm_state.load());
                            std::this_thread::sleep_for(std::chrono::milliseconds(1));
                            continue;
                        }
                        else
                        {
                            fsm_state = AUTO_FSM_COMPLETE;

                            clear_control_params();
                            set_multi_infomation(StateMultiReq::NONE, StateObsCondition::NONE, StateCurGoal::MOVE);
                            log_info("DRIVING -> COMPLETE(temp goal), err_d: {}", goal_err_d);
                            return;
                        }
                    }
                }

                // for real time loop
                double cur_loop_time = get_time();
                double delta_loop_time = cur_loop_time - pre_loop_time;
                if(delta_loop_time < dt)
                {
                    int sleep_ms = (dt-delta_loop_time)*1000;
                    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
                }
                else
                {
                    log_warn("driving loop time drift, dt: {}", delta_loop_time);
                }
                pre_loop_time = get_time();
                continue;
            }

            // obstacle deceleation
            QString obs_condition = "none";
            double obs_decel_v = config->get_obs_map_min_v();
            {
                std::lock_guard<std::recursive_mutex> lock(mtx);
                obs_value     = cur_obs_value;
                obs_decel_v   = cur_obs_decel_v;
                obs_condition = cur_obs_condition;
            }

            // obstacle stop
            {
                double cur_velocity      = mobile->get_control_input()[0];
                double stopping_distance = 0; //(cur_velocity * cur_velocity) / (2 * params.LIMIT_V_DCC + 1e-06);
                double dynamic_deadzone  = stopping_distance + AUTOCONTROL_INFO::dynamic_deadzone_safety_margin;
                dynamic_deadzone = std::max(dynamic_deadzone, config->get_obs_deadzone());
                cur_deadzone = dynamic_deadzone;

                int chk_idx = cur_idx + dynamic_deadzone/AUTOCONTROL_INFO::local_path_step;
                log_debug("control_loop: obs check, obs_vel:{}, cur_vel:{},  stop_dist:{}, dyn_Dzon:{}", obs_decel_v, cur_velocity, stopping_distance, dynamic_deadzone);

                if(chk_idx > (int)local_path.pos.size()-1)
                {
                    chk_idx = local_path.pos.size()-1;
                }

                std::vector<Eigen::Matrix4d> traj;
                for(int p = cur_idx; p <= chk_idx; p++)
                {
                    traj.push_back(local_path.pose[p]);
                }

                obs_value = obsmap->is_path_collision(traj, true, 0, 0, 0, 10);

                if(obs_value != OBS_NONE)
                {
                    mobile->move(0, 0, 0);

                    fsm_state = AUTO_FSM_OBS;
                    obs_state = AUTO_OBS_CHECK;
                    set_multi_infomation(StateMultiReq::NO_CHANGE, StateObsCondition::NO_CHANGE, StateCurGoal::OBSTACLE);

                    log_info("DRIVING -> OBS, cur_obs_value:{}, fsm_state:{}", obs_value, (int)fsm_state);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }
            }

            // for mobile server
            set_obs_condition(obs_condition);

            // calc heading error
            Eigen::Matrix4d _tgt_tf = cur_tf_inv*tgt_tf;
            Eigen::Vector3d _tgt_xi = TF_to_se2(_tgt_tf);

            // only use mecanum model
            double dir_x = 0.0;
            double dir_y = 0.0;

            double err_d  = std::numeric_limits<double>::max();
            double err_th = std::numeric_limits<double>::max();
            if(drive_method == DriveMethod::METHOD_HPP || drive_method == DriveMethod::METHOD_SIDE)
            {
                Eigen::Vector3d local_tgt_pos = cur_tf_inv.block(0,0,3,3)*tgt_pos + cur_tf_inv.block(0,3,3,1);

                double local_dx = local_tgt_pos[0];
                double local_dy = local_tgt_pos[1];

                double local_d = std::sqrt(local_dx*local_dx+local_dy*local_dy);
                dir_x = local_dx / (local_d + 1.0e-6);
                dir_y = local_dy / (local_d + 1.0e-6);

                err_d  = calc_dist_2d(_tgt_xi);
            }
            else
            {
                err_d  = calc_dist_2d(_tgt_xi);
            }
            err_th = _tgt_xi[2];

            // calc cross track error
            double cte = calc_cte(local_path.pose, cur_pos);

            // get ref_v -- prevent oscilation caused by localization jitter
            double ref_v = 0.0;
            double diff_from_start_pos = calc_dist_2d(cur_pos - cur_pos_at_start_driving);
            if(diff_from_start_pos < 0.5 && ref_v_oscilation_end_flag == false)
            {
                // it is critical at staring point (vel from zero)
                if(cur_idx > prev_local_ref_v_index)
                {
                    prev_local_ref_v_index = cur_idx;
                }
                ref_v = local_path.ref_v[prev_local_ref_v_index];

            }
            else
            {
                ref_v_oscilation_end_flag = true;
                ref_v = local_path.ref_v[cur_idx];
            }

            // calc control input
            double v0 = cur_vel[0];
            double v = std::min<double>((params.LIMIT_V/params.DRIVE_L)*err_d, ref_v);

            if(drive_method == DriveMethod::METHOD_HPP || drive_method == DriveMethod::METHOD_SIDE)
            {
                v0 = std::sqrt(cur_vel[0]*cur_vel[0] + cur_vel[1]*cur_vel[1]);
            }

            if(is_move_backward)
            {
                v = saturation(v, v0 - params.LIMIT_V_ACC*dt, v0 + params.LIMIT_V_DCC*dt);

                if(v0 < 0)
                {
                    v = saturation(v, -obs_decel_v, 0.0);
                    v = saturation(v, -params.LIMIT_V, params.LIMIT_V);
                }
            }
            else
            {
                v = saturation(v, v0 - params.LIMIT_V_DCC*dt, v0 + params.LIMIT_V_ACC*dt);
                v = saturation(v, 0.0, obs_decel_v);
                v = saturation(v, -params.LIMIT_V, params.LIMIT_V);
            }

            if(pre_err_th == 0)
            {
                pre_err_th = err_th;
            }

            double w;
            if(drive_method == DriveMethod::METHOD_HPP || drive_method == DriveMethod::METHOD_SIDE)
            {
                // PD control
                w = params.DRIVE_A * err_th + params.DRIVE_B * (err_th - pre_err_th)/dt;
            }
            else
            {
                if(is_move_backward == true)
                {
                    v = -v;
                }

                // stanley control
                double th = (params.DRIVE_A * err_th)
                        + (params.DRIVE_B * (err_th-pre_err_th)/dt)
                        +  std::atan2(params.DRIVE_K * cte, v + params.DRIVE_EPS);
                th = saturation(th, -45.0 * D2R, 45.0 * D2R);
                w = std::tan(th) / params.DRIVE_L;

                if(is_move_backward == true)
                {
                    v = -v;
                }
            }
            pre_err_th = err_th;


            double w0 = cur_vel[2];
            w = saturation(w, w0 - params.LIMIT_W_ACC*D2R*dt, w0 + params.LIMIT_W_ACC*D2R*dt);
            w = saturation(w, -params.LIMIT_W*D2R, params.LIMIT_W*D2R);

            // scaling
            double scale_v = 1.0 - params.DRIVE_T*std::abs(w/(params.LIMIT_W*D2R));
            double scale_w = 1.0 - params.DRIVE_T*std::abs(v/params.LIMIT_V);
            v *= scale_v;
            w *= scale_w;

            // deadzone v
            double d_v = config->get_drive_v_deadzone();
            if(std::abs(v) < d_v)
            {
                v = sgn(v)*d_v;
            }

            // deadzone w
            double d_w = config->get_drive_w_deadzone()*D2R;
            if(std::abs(w) < d_w)
            {
                w = 0;
            }

            if(drive_method == DriveMethod::METHOD_HPP || drive_method == DriveMethod::METHOD_SIDE)
            {
                double vx = dir_x * v;
                double vy = dir_y * v;

                // send control
                if(robot_model == RobotModel::QD)
                {
                    if(drive_method == DriveMethod::METHOD_HPP)
                    {
                        mobile->moveQD(vx, vy, w, 1);
                    }
                    else if(drive_method == DriveMethod::METHOD_SIDE)
                    {
                        mobile->moveQD(vx, vy, w, 2);
                    }
                }
                else
                {
                    mobile->move(vx, vy, w);
                }
            }
            else
            {
                mobile->move(v, 0, w);
            }
        }
        else if(fsm_state == AUTO_FSM_FINAL_ALIGN)
        {
            Eigen::Matrix4d cur_tf_inv = cur_tf.inverse();
            Eigen::Vector3d local_goal_pos = cur_tf_inv.block(0,0,3,3) * goal_pos + cur_tf_inv.block(0,3,3,1);

            // calc heading error
            double err_x = local_goal_pos[0];
            double err_y = local_goal_pos[1];
            double err_d = calc_dist_2d(goal_pos - cur_pos);
            double err_th = deltaRad(goal_xi[2], cur_xi[2]);

            // goal check
            if(std::abs(err_th) < config->get_drive_goal_th()*D2R && err_d < config->get_drive_goal_dist())
            {
                extend_dt += dt;
                if(extend_dt > config->get_drive_extended_control_time())
                {
                    clear_control_params();

                    fsm_state = AUTO_FSM_COMPLETE;
                    set_multi_infomation(StateMultiReq::NONE, StateObsCondition::NONE, StateCurGoal::COMPLETE);

                    // update move info
                    {
                        std::lock_guard<std::recursive_mutex> lock(mtx);
                        cur_move_info.time    = get_time();
                        cur_move_info.result  = "success";
                        cur_move_info.message = "very good";
                        cur_move_info.cur_pos = cur_pos;
                        Q_EMIT signal_move_response(cur_move_info);
                    }

                    log_info("FINAL ALIGN COMPLETE(good), err_d: {}({},{}), err_th: {}", err_d, err_x, err_y, err_th*R2D);
                    return;
                }
            }

            std::vector<Eigen::Matrix4d> traj = intp_tf(cur_tf, goal_tf, 0.2, 10.0*D2R);
            obs_value = obsmap->is_path_collision(traj, true);
            if(obs_value == OBS_DYN)
            {
                mobile->move(0, 0, 0);

                fsm_state = AUTO_FSM_OBS;
                obs_state = AUTO_OBS_WAIT2;
                set_multi_infomation(StateMultiReq::NO_CHANGE, StateObsCondition::NO_CHANGE, StateCurGoal::OBSTACLE);
                obs_wait_st_time = get_time();

                log_info("FINAL ALIGN -> OBS");
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            if(pre_err_th == 0)
            {
                pre_err_th = err_th;
            }

            // pivot control
            double kp = params.DRIVE_A;
            double kd = params.DRIVE_B;
            double w0 = cur_vel[2];
            double w = kp*err_th + kd*(err_th - pre_err_th)/dt;
            pre_err_th = err_th;

            w = saturation(w, w0 - params.LIMIT_W_ACC*D2R*dt, w0 + params.LIMIT_W_ACC*D2R*dt);
            w = saturation(w, -params.LIMIT_PIVOT_W*D2R, params.LIMIT_PIVOT_W*D2R);

            double vx = 0.0;
            double vy = 0.0;
            if(drive_method == DriveMethod::METHOD_HPP)
            {
                vx = config->get_drive_goal_approach_gain()*err_x;
                vy = config->get_drive_goal_approach_gain()*err_y;

                vx = saturation(vx, -params.LIMIT_V, params.LIMIT_V);
                vy = saturation(vy, -params.LIMIT_V, params.LIMIT_V);
            }

            // send control
            if(!is_debug)
            {
                mobile->move(vx, vy, w);
            }
        }
        else if(fsm_state == AUTO_FSM_OBS)
        {
            if(obs_state == AUTO_OBS_CHECK)
            {
                log_debug("obs check");

                if(obs_value == OBS_DYN)
                {
                    set_multi_infomation(StateMultiReq::NO_CHANGE, StateObsCondition::NEAR, StateCurGoal::NO_CHANGE);

                    if(config->get_obs_avoid_mode() == 0)
                    {
                        // mode 0, just wait
                        obs_state = AUTO_OBS_WAIT;
                        obs_wait_st_time = get_time();

                        log_info("avoid mode 0, OBS_WAIT");
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        continue;
                    }
                    else
                    {
                        // mode 1, avoid
                        obs_state = AUTO_OBS_AVOID;

                        log_info("avoid mode 1, OBS_AVOID");
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        continue;
                    }
                }
                else if(obs_value == OBS_VIR)
                {
                    // for mobile server
                    set_multi_infomation(StateMultiReq::NO_CHANGE, StateObsCondition::VIR, StateCurGoal::NO_CHANGE);

                    // for vobs works
                    obs_state = AUTO_OBS_VIR;
                    obs_wait_st_time = get_time();

                    log_info("OBS_VIR");
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }
                else
                {
                    // just wait
                    obs_state = AUTO_OBS_WAIT;
                    obs_wait_st_time = get_time();

                    log_info("OBS_WAIT, just wait");
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }
            }
            else if(obs_state == AUTO_OBS_AVOID)
            {
                // calc avoid path
                avoid_path = calc_avoid_path(global_path);
                if(avoid_path.pos.size() > 0)
                {
                    extend_dt = 0;
                    pre_err_th = 0;

                    fsm_state = ((drive_method == DriveMethod::METHOD_HPP || drive_method == DriveMethod::METHOD_SIDE) ? AUTO_FSM_DRIVING : AUTO_FSM_FIRST_ALIGN);

                    set_multi_infomation(StateMultiReq::NO_CHANGE, StateObsCondition::NO_CHANGE, StateCurGoal::MOVE);

                    log_info("avoid path found, OBS_AVOID -> FIRST_ALIGN");
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }
                else
                {
                    obs_state = AUTO_OBS_RECOVERY;

                    log_info("avoid path failed, OBS_AVOID -> OBS_RECOVERY");
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }
            }
            else if(obs_state == AUTO_OBS_RECOVERY)
            {
                std::vector<Eigen::Vector3d> obs_pts = obsmap->get_obs_pts();

                Eigen::Vector3d vel(0,0,0);
                double max_d = 0;
                for(int deg = -30; deg <= 30; deg += 10)
                {
                    std::vector<Eigen::Matrix4d> traj = calc_trajectory(Eigen::Vector3d(-0.1, 0, deg*D2R), 0.1, 1.0, cur_tf);
                    if(obsmap->is_path_collision(traj))
                    {
                        continue;
                    }

                    double min_d = 9999; //std::numeric_limits<double>::max();
                    for(size_t p = 0; p < obs_pts.size(); p++)
                    {
                        double d = calc_dist_2d(obs_pts[p] - traj.back().block(0,3,3,1));
                        if(d < min_d)
                        {
                            min_d = d;
                        }
                    }

                    if(min_d > max_d)
                    {
                        max_d = min_d;
                        vel[0] = -0.1;
                        vel[1] = 0;
                        vel[2] = deg*D2R;
                    }
                }

                for(int deg = -30; deg <= 30; deg += 10)
                {
                    std::vector<Eigen::Matrix4d> traj = calc_trajectory(Eigen::Vector3d(0.1, 0, deg*D2R), 0.1, 1.0, cur_tf);
                    if(obsmap->is_path_collision(traj))
                    {
                        continue;
                    }

                    double min_d = std::numeric_limits<double>::max();
                    for(size_t p = 0; p < obs_pts.size(); p++)
                    {
                        double d = calc_dist_2d(obs_pts[p] - traj.back().block(0,3,3,1));
                        if(d < min_d)
                        {
                            min_d = d;
                        }
                    }

                    if(min_d > max_d)
                    {
                        max_d = min_d;
                        vel[0] = 0.1;
                        vel[1] = 0;
                        vel[2] = deg*D2R;
                    }
                }

                double margin_dx = config->get_obs_safe_margin_x();
                double margin_dy = config->get_obs_safe_margin_y();

                double margin_d = std::sqrt(margin_dx*margin_dx+ margin_dy*margin_dy);
                if(max_d > config->get_robot_radius() + margin_d)
                {
                    mobile->move(0, 0, 0);
                    obs_state = AUTO_OBS_AVOID;
                    log_info("max_d: {}, OBS_RECOVERY -> OBS_AVOID", max_d);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }

                double v0 = cur_vel[0];
                double w0 = cur_vel[2];
                double v = vel[0];
                double w = vel[2];
                w = saturation(w, w0 - params.LIMIT_W_ACC*D2R*dt, w0 + params.LIMIT_W_ACC*D2R*dt);
                w = saturation(w, -params.LIMIT_W*D2R, params.LIMIT_W*D2R);

                if(is_move_backward)
                {
                    // need test!
                    if(v0 < 0)
                    {
                        v = saturation(v, v0 - params.LIMIT_V_ACC*dt, v0 + params.LIMIT_V_DCC*dt);
                        v = saturation(v, -params.LIMIT_V, params.LIMIT_V);
                    }
                }
                else
                {
                    v = saturation(v, v0 - params.LIMIT_V_ACC*dt, v0 + params.LIMIT_V_ACC*dt);
                    v = saturation(v, -params.LIMIT_V, params.LIMIT_V);
                }



                if(!is_debug)
                {
                    mobile->move(v, 0, w);
                }
            }
            else if(obs_state == AUTO_OBS_WAIT)
            {
                if(get_time() - obs_wait_st_time > 2.5)
                {
                    bool path_clear = true;
                    if(local_path.pos.size() > 0)
                    {
                        log_debug("check path clear in OBS_WAIT");
                        int cur_idx = get_nn_idx(local_path.pos, cur_pos);
                        double hold_deadzone = std::max(cur_deadzone.load(), config->get_obs_deadzone());
                        int chk_idx = cur_idx + static_cast<int>(std::ceil(hold_deadzone / AUTOCONTROL_INFO::local_path_step));
                        if(chk_idx > static_cast<int>(local_path.pos.size()) - 1)
                        {
                            chk_idx = static_cast<int>(local_path.pos.size()) - 1;
                        }

                        std::vector<Eigen::Matrix4d> traj;
                        traj.reserve(std::max(0, chk_idx - cur_idx + 1));
                        for(int p = cur_idx; p <= chk_idx; ++p)
                        {
                            traj.push_back(local_path.pose[p]);
                        }

                        if(traj.size() > 0)
                        {
                            int val = obsmap->is_path_collision(traj, true, 0, 0, 0, 10);
                            path_clear = (val == OBS_NONE);
                        }
                    }

                    if(path_clear)
                    {
                        log_info("path cleared during OBS_WAIT");
                        extend_dt = 0;
                        pre_err_th = 0;

                        obs_state = AUTO_OBS_CHECK;

                        // set ref_v as ST_V to already passed global path
                        int cur_global_idx = get_nn_idx(global_path.pos, cur_pos);
                        for(int ref_v_index=0; ref_v_index<cur_global_idx-1; ref_v_index++)
                        {
                            global_path.ref_v[ref_v_index] = params.ST_V;
                        }

                        fsm_state = ((drive_method == DriveMethod::METHOD_HPP || drive_method == DriveMethod::METHOD_SIDE) ? AUTO_FSM_DRIVING : AUTO_FSM_FIRST_ALIGN);

                        set_multi_infomation(StateMultiReq::NO_CHANGE, StateObsCondition::NO_CHANGE, StateCurGoal::MOVE);
                        obs_value = OBS_NONE;

                        log_info("OBS_WAIT -> FIRST_ALIGN (path cleared)");
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        continue;
                    }
                    else
                    {
                        obs_wait_st_time = get_time();
                        log_info("OBS_WAIT hold (obstacle still detected)");
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        continue;
                    }
                }
            }
            else if(obs_state == AUTO_OBS_WAIT2)
            {
                if(get_time() - obs_wait_st_time > 2.5)
                {
                    std::vector<Eigen::Matrix4d> traj = intp_tf(cur_tf, goal_tf, 0.2, 10.0*D2R);
                    int val = obsmap->is_path_collision(traj, true);
                    if(val == OBS_NONE)
                    {
                        extend_dt = 0;
                        pre_err_th = 0;

                        fsm_state = AUTO_FSM_FINAL_ALIGN;
                        set_multi_infomation(StateMultiReq::NO_CHANGE, StateObsCondition::NO_CHANGE, StateCurGoal::MOVE);

                        log_info("OBS_WAIT2 -> FINAL_ALIGN (path cleared)");
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        continue;
                    }
                    else
                    {
                        obs_wait_st_time = get_time();
                        log_info("OBS_WAIT2 hold (obstacle still detected)");
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        continue;
                    }
                }


            }
            else if(obs_state == AUTO_OBS_VIR)
            {
                // check
                std::vector<Eigen::Vector3d> vir_pts = obsmap->get_vir_pts();
                if(vir_pts.size() == 0)
                {
                    mobile->move(0, 0, 0);

                    extend_dt = 0;
                    pre_err_th = 0;

                    obs_state = AUTO_OBS_WAIT;
                    obs_wait_st_time = get_time();
                    log_info("OBS_VIR(no pts) -> OBS_WAIT");
                    continue;
                }

                // global to local
                std::vector<Eigen::Vector3d> _vir_pts;
                for(size_t p = 0; p < vir_pts.size(); p++)
                {
                    Eigen::Vector3d P = vir_pts[p];
                    Eigen::Vector3d _P = cur_tf_inv.block(0,0,3,3)*P + cur_tf_inv.block(0,3,3,1);
                    _vir_pts.push_back(_P);
                }

                // find min pt
                double min_d = std::numeric_limits<double>::max();
                Eigen::Vector3d min_pt(0, 0, 0);
                for(size_t p = 0; p < _vir_pts.size(); p++)
                {
                    Eigen::Vector3d P = _vir_pts[p];
                    double d = calc_dist_2d(P);
                    if(d < min_d)
                    {
                        min_d = d;
                        min_pt = P;
                    }
                }

                // calc v
                double v = -0.1*sgn(min_pt[0]);

                // check
                if(min_d >= config->get_robot_radius() + 0.05 || get_time() - obs_wait_st_time > 0.5)
                {
                    mobile->move(0, 0, 0);

                    extend_dt = 0;
                    pre_err_th = 0;

                    obs_state = AUTO_OBS_WAIT;
                    obs_wait_st_time = get_time();

                    log_info("OBS_VIR -> OBS_WAIT, min_d: {}", min_d);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }

                if(!is_debug)
                {
                    mobile->move(v, 0, 0);
                }
            }
        }

        // for real time loop
        double cur_loop_time = get_time();
        double delta_loop_time = cur_loop_time - pre_loop_time;
        if(delta_loop_time < dt)
        {
            int sleep_ms = (dt-delta_loop_time)*1000;
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
        else
        {
            log_warn("drift in driving loop time: {}", delta_loop_time);
        }
        process_time_control = delta_loop_time;
        pre_loop_time = get_time();
    }

    if(is_path_overlap)
    {
        is_path_overlap = false;
        set_multi_infomation(StateMultiReq::NO_CHANGE, StateObsCondition::NONE, StateCurGoal::NO_CHANGE);
        log_info("path overlap, b_loop_pp stop");
        return;
    }

    // manual stopped
    clear_control_params();
    set_multi_infomation(StateMultiReq::NONE, StateObsCondition::NONE, StateCurGoal::CANCEL);

    {
        Eigen::Matrix4d cur_tf = loc->get_cur_tf();
        Eigen::Vector3d cur_pos = cur_tf.block(0,3,3,1);

        {
            std::lock_guard<std::recursive_mutex> lock(mtx);
            cur_move_info.cur_pos = cur_pos;
            cur_move_info.result = "fail";
            cur_move_info.message = "manual stopped";
            cur_move_info.time = get_time();
        }
        Q_EMIT signal_move_response(cur_move_info);
    }

    fsm_state = AUTO_FSM_COMPLETE;
    log_info("path stop, b_loop_pp stop");
}

void AUTOCONTROL::obs_loop()
{
    const double dt = 0.05; // 20hz
    double pre_loop_time = get_time();

    //logger->write_log("[AUTO] start obs loop");
    log_info("start obs loop");

    double obs_v_debug = 0.0;
    while(obs_flag)
    {
        if(!is_moving)
        {
            log_info("not moving: obs_loop");
            break;
        }
        //        qDebug()<<"obs";
        Eigen::Matrix4d cur_tf = loc->get_cur_tf();
        if (fsm_state != AUTO_FSM_FINAL_ALIGN && is_move_backward == true)
        {
            Eigen::Matrix4d Rz = Eigen::Matrix4d::Identity();
            Rz(0,0) =  cos(M_PI);  Rz(0,1) = -sin(M_PI);
            Rz(1,0) =  sin(M_PI);  Rz(1,1) =  cos(M_PI);

            cur_tf = cur_tf*Rz;
        }

        Eigen::Vector3d cur_pos = cur_tf.block(0,3,3,1);

        if(fsm_state == AUTO_FSM_DRIVING || fsm_state == AUTO_FSM_OBS)
        {
            int obs_value = OBS_NONE;
            QString obs_condition = "none";
            double obs_decel_v = config->get_obs_map_min_v();

            // predict trajectory
            std::vector<Eigen::Matrix4d> traj;
            double predict_time = config->get_obs_predict_time();

            for(double vv = config->get_obs_map_min_v(); vv <= params.LIMIT_V+0.01; vv += 0.025)
            {
                traj = calc_trajectory(Eigen::Vector3d(vv, 0, 0), 0.2, predict_time, cur_tf);

                bool is_collision = false;
                for(const auto& tf : traj)
                {
                    int val = obsmap->is_tf_collision(tf, true, config->get_obs_safe_margin_x(), config->get_obs_safe_margin_y());
                    if(val != OBS_NONE)
                    {
                        is_collision = true;
                        obs_value = val;
                        break;
                    }
                }

                if(is_collision)
                {
                    obs_condition = "far";
                    break;
                }

                obs_decel_v = vv;
            }

            obs_v_debug = obs_decel_v;

            if(obs_value != OBS_NONE)
            {
                log_debug("obs_loop obs detected, obs_value: {}, obs_decel_v:{}", obs_value, obs_decel_v);
            }

            std::vector<Eigen::Vector3d> dyn_pts = obsmap->get_dyn_pts();
            double min_dyn_dist = std::numeric_limits<double>::max();
            bool found_forward_obs = false;

            double yaw = std::atan2(cur_tf(1, 0), cur_tf(0, 0));
            Eigen::Vector2d heading_vec(std::cos(yaw), std::sin(yaw));

            for(const auto& pt : dyn_pts)
            {
                Eigen::Vector2d obs_vec(pt[0] - cur_pos[0], pt[1] - cur_pos[1]);

                bool near_traj = false;
                for(const auto& tf : traj)
                {
                    Eigen::Vector3d traj_pos = tf.block<3,1>(0, 3);
                    double dx = pt[0] - traj_pos[0];
                    double dy = pt[1] - traj_pos[1];
                    double dist = std::sqrt(dx * dx + dy * dy);

                    if(dist < AUTOCONTROL_INFO::obstacle_near_check_dist)
                    {
                        near_traj = true;
                        break;
                    }
                }

                if (!near_traj)
                {
                    continue;
                }

                double dx = pt[0] - cur_pos[0];
                double dy = pt[1] - cur_pos[1];
                double d = std::sqrt(dx * dx + dy * dy);

                if(d < min_dyn_dist)
                {
                    min_dyn_dist = d;
                    found_forward_obs = true;
                }
            }

            Eigen::Vector3d cur_pos = cur_tf.block(0,3,3,1);
            double buf_deadzone = config->get_obs_deadzone();

            double obs_dist;
            if(found_forward_obs)
            {
                obs_dist = std::max(0.0, min_dyn_dist - config->get_robot_size_x_max());
            }
            else
            {
                obs_dist = std::numeric_limits<double>::max();
                log_debug("obs_loop no forward dyn pts detected");
            }
            if(obs_condition != "none" && obs_decel_v > 0.0)
            {
                log_debug("obs_loop dyn pts, obs_condition: {}, obs_decel_v:{}", obs_condition.toUtf8().constData(),obs_decel_v);
            }

            // final update(conclusion)
            {
                std::lock_guard<std::recursive_mutex> lock(mtx);

                cur_obs_dist      = obs_dist;
                cur_obs_value     = obs_value;
                cur_obs_decel_v   = obs_decel_v;
                cur_obs_condition = obs_condition;
            }
        }

        // for real time loop
        double cur_loop_time = get_time();
        double delta_loop_time = cur_loop_time - pre_loop_time;
        if(delta_loop_time < dt)
        {
            int sleep_ms = (dt-delta_loop_time) * 1000;
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }

        process_time_obs = delta_loop_time;
        pre_loop_time = get_time();
    }

    log_info("stop obs_loop");
}

void AUTOCONTROL::node_loop()
{
    // loop params
    const double dt = 0.05; // 20hz
    double pre_loop_time = get_time();
    QString pre_node_id = "";

    log_info("start node loop");
    while(node_flag)
    {
        if(unimap->get_is_loaded() == MAP_LOADED)
        {
            Eigen::Matrix4d cur_tf = loc->get_cur_tf();
            QString _cur_node_id = unimap->get_node_id_edge(cur_tf.block(0,3,3,1));
            if(pre_node_id == "")
            {
                pre_node_id = _cur_node_id;

                // update
                std::lock_guard<std::recursive_mutex> lock(mtx);
                cur_node_id = pre_node_id;
            }
            else
            {
                // calc pre node id
                NODE* node = unimap->get_node_by_id(_cur_node_id);
                if(node != nullptr)
                {
                    double d = calc_dist_2d(node->tf.block(0,3,3,1) - cur_tf.block(0,3,3,1));
                    if(d < config->get_robot_radius())
                    {
                        int _last_step = get_last_step();
                        if(pre_node_id != _cur_node_id && !cur_node_id.isEmpty())
                        {
                            pre_node_id = _cur_node_id;

                            std::lock_guard<std::mutex> lock(path_mtx);

                            if(!global_step.empty())
                            {
                                last_step.store(global_step.front());
                                global_step.erase(global_step.begin());
                            }
                        }

                        // update
                        std::lock_guard<std::recursive_mutex> lock(mtx);
                        cur_node_id = pre_node_id;
                    }
                }
            }
        }

        // for real time loop
        double cur_loop_time = get_time();
        double delta_loop_time = cur_loop_time - pre_loop_time;
        if(delta_loop_time < dt)
        {
            int sleep_ms = (dt-delta_loop_time)*1000;
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }

        process_time_node = delta_loop_time;
        pre_loop_time = cur_loop_time;
    }

    log_info("node loop stop");
}

void AUTOCONTROL::set_config_module(CONFIG* _config)
{
    config = _config;
}

void AUTOCONTROL::set_logger_module(LOGGER* _logger)
{
    logger = _logger;
}

void AUTOCONTROL::set_mobile_module(MOBILE *_mobile)
{
    mobile = _mobile;
}

void AUTOCONTROL::set_unimap_module(UNIMAP* _unimap)
{
    unimap = _unimap;
}

void AUTOCONTROL::set_obsmap_module(OBSMAP* _obsmap)
{
    obsmap = _obsmap;
}

void AUTOCONTROL::set_localization_module(LOCALIZATION *_loc)
{
    loc = _loc;
}

void AUTOCONTROL::set_policy_module(POLICY* _policy)
{
    policy = _policy;
}