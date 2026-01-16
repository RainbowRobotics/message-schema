#include "comm_msa.h"

namespace
{
  const char* MODULE_NAME = "MSA";
}

template<typename T>
void COMM_MSA::add_to_obj(sio::object_message::ptr obj, const std::string& key, const T& value)
{
  if constexpr(std::is_same_v<T, double>)
  {
    obj->get_map()[key] = sio::double_message::create(value);
  }
  else if constexpr(std::is_same_v<T, int>)
  {
    obj->get_map()[key] = sio::int_message::create(value);
  }
  else if constexpr(std::is_same_v<T, long long>)
  {
    obj->get_map()[key] = sio::int_message::create(value);
  }
  else if constexpr(std::is_same_v<T, std::string>)
  {
    obj->get_map()[key] = sio::string_message::create(value);
  }
  else if constexpr(std::is_same_v<T, bool>)
  {
    obj->get_map()[key] = sio::bool_message::create(value);
  }
  else if constexpr(std::is_same_v<T, QString>)
  {
    obj->get_map()[key] = sio::string_message::create(value.toStdString());
  }
  else if constexpr(std::is_same_v<T, sio::object_message::ptr>)
  {
    obj->get_map()[key] = value;
  }
  else if constexpr(std::is_convertible_v<T, const char*>)
  {
    obj->get_map()[key] = sio::string_message::create(value);
  }
  else
  {
    static_assert(always_false<T>::value, "Unsupported type for add_to_obj.");
  }
}

template<typename T>
void COMM_MSA::add_to_obj(sio::object_message::ptr obj, const std::string& key, const std::vector<T>& values)
{
  auto  arr = sio::array_message::create();
  arr->get_vector().reserve(values.size());
  for(const auto& v : values)
  {
    if constexpr(std::is_same_v<T, double>)
    {
      arr->get_vector().push_back(sio::double_message::create(v));
    }
    else if constexpr(std::is_same_v<T, int>)
    {
      arr->get_vector().push_back(sio::int_message::create(v));
    }
    else
    {
      static_assert(always_false<T>::value, "Unsupported type for add_to_obj. ");
    }
  }
  obj->get_map()[key] = arr;
}

template void COMM_MSA::add_to_obj<double>(sio::object_message::ptr, const std::string&, const double&);
template void COMM_MSA::add_to_obj<int>(sio::object_message::ptr, const std::string&, const int&);
template void COMM_MSA::add_to_obj<long long>(sio::object_message::ptr, const std::string&, const long long&);
template void COMM_MSA::add_to_obj<bool>(sio::object_message::ptr, const std::string&, const bool&);
template void COMM_MSA::add_to_obj<std::string>(sio::object_message::ptr, const std::string&, const std::string&);
template void COMM_MSA::add_to_obj<QString>(sio::object_message::ptr, const std::string&, const QString&);
template void COMM_MSA::add_to_obj<const char*>(sio::object_message::ptr, const std::string&, const char* const&);
// sio::object_message / sio:array_message 는 같은 형.
// 둘다 명시적으로 선언할 경우, duplicate error 발생됨. 따라서 object_message 만 남김.
template void COMM_MSA::add_to_obj<sio::object_message::ptr>(sio::object_message::ptr, const std::string&, const sio::object_message::ptr&);

template void COMM_MSA::add_to_obj<double>(sio::object_message::ptr, const std::string&, const std::vector<double>&);
template void COMM_MSA::add_to_obj<int>(sio::object_message::ptr, const std::string&, const std::vector<int>&);


COMM_MSA* COMM_MSA::instance(QObject* parent)
{
  static COMM_MSA* inst = nullptr;
  if(!inst && parent)
  {
    inst = new COMM_MSA(parent);
  }
  else if(inst && parent && inst->parent() == nullptr)
  {
    inst->setParent(parent);
  }
  return inst;
}

COMM_MSA::COMM_MSA(QObject *parent)
  : QObject{parent}
{
  // set recv callbacks
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;

  // set socket
  rrs_socket = std::make_unique<sio::client>();
  rrs_socket->set_open_listener(std::bind(&COMM_MSA::connected, this));
  rrs_socket->set_close_listener(std::bind(&COMM_MSA::disconnected, this));

  // command
  sio::socket::ptr sock = rrs_socket->socket("slamnav");
  BIND_EVENT(sock, "moveRequest",         std::bind(&COMM_MSA::recv_message,       this, std::placeholders::_1));
  BIND_EVENT(sock, "loadRequest",         std::bind(&COMM_MSA::recv_message,       this, std::placeholders::_1));
  BIND_EVENT(sock, "localizationRequest", std::bind(&COMM_MSA::recv_message,       this, std::placeholders::_1));
  BIND_EVENT(sock, "mappingRequest",      std::bind(&COMM_MSA::recv_message,       this, std::placeholders::_1));
  BIND_EVENT(sock, "pathRequest",         std::bind(&COMM_MSA::recv_message,       this, std::placeholders::_1));
  BIND_EVENT(sock, "sensorRequest",       std::bind(&COMM_MSA::recv_message,       this, std::placeholders::_1));
  BIND_EVENT(sock, "controlRequest",      std::bind(&COMM_MSA::recv_message_array, this, std::placeholders::_1));
  BIND_EVENT(sock, "settingRequest",      std::bind(&COMM_MSA::recv_message_array, this, std::placeholders::_1));

  BIND_EVENT(sock, "updateRequest",       std::bind(&COMM_MSA::recv_message_single_shot,       this, std::placeholders::_1));
  // status
  BIND_EVENT(sock, "vobs", std::bind(&COMM_MSA::recv_message, this, std::placeholders::_1));

  // reconncetion
  reconnect_timer = new QTimer(this);
  connect(reconnect_timer, &QTimer::timeout, this, &COMM_MSA::reconnect_loop);
}

COMM_MSA::~COMM_MSA()
{
  rrs_socket->socket("slamnav")->off_all();
  rrs_socket->socket("slamnav")->off_error();
  rrs_socket->sync_close();

  reconnect_timer->stop();

  stop_all_thread();
}

void COMM_MSA::init()
{
  log_info("init() called");

  if(!config)
  {
    log_warn("config module not set");
    return;
  }

  if(!config->get_use_msa())
  {
    log_warn("not use msa");
    return;
  }

  // start reconnect loop
  reconnect_timer->start(COMM_MSA_INFO::reconnect_time * 1000);
  log_info("start reconnect timer");

  start_all_thread();
}

void COMM_MSA::reconnect_loop()
{
  if(is_connected)
  {
    log_debug("already connected");
    return;
  }

  rrs_socket->connect("ws://localhost:15001");
  rrs_socket->socket("slamnav");
}

void COMM_MSA::connected()
{
  if(is_connected)
  {
    log_info("already connected");
    return;
  }

  if(!ctrl)
  {
    log_error("not ready to modules");
    return;
  }
  ctrl->set_is_rrs(true);

  is_connected = true;
  log_info("connected to MSA server");
}

void COMM_MSA::disconnected()
{
  if(!is_connected)
  {
    log_info("already disconnected");
    return;
  }

  if(!ctrl)
  {
    log_error("not ready to modules");
    return;
  }
  ctrl->set_is_rrs(false);

  is_connected = false;
  log_info("disconnected to MSA server");
}

void COMM_MSA::recv_message(sio::event& ev)
{
  sio::message::ptr msg = ev.get_message();
  if(!msg)
  {
    log_warn("recv_message, invalid msg");
    return;
  }

  if(msg->get_flag() == sio::message::flag_string)
  {
    std::string data = msg->get_string();
    log_info("this data flag is string.. not object : {}" , data);
  }
  else if(msg->get_flag() == sio::message::flag_integer)
  {
    int val = msg->get_int();
    log_info("this data flag is int.. not object : {}" , val);
  }
  else if(msg->get_flag() == sio::message::flag_object)
  {
    auto obj = msg->get_map();
    QJsonObject json_obj = sio_object_to_qt_json_object(obj);

    QJsonObject root;
    root.insert("topic", QString::fromStdString(ev.get_name()));
    root.insert("data", json_obj);

    QString wrapped = QString(QJsonDocument(root).toJson(QJsonDocument::Compact));
    {
      std::lock_guard<std::mutex> lock(recv_mtx);
      recv_queue.push(wrapped);
      recv_cv.notify_one();
    }

    set_last_receive_msg(wrapped);
  }
}

void COMM_MSA::recv_message_array(sio::event& ev)
{
  sio::message::ptr msg = ev.get_message();
  if(!msg || msg->get_flag() != sio::message::flag_object)
  {
    log_warn("recv_message_array, invalid msg");
    return;
  }

  auto obj = msg->get_map();
  QJsonObject json_obj = sio_object_to_qt_json_object(obj);
  QString cmd = get_json(json_obj, "command"); // command 확인
  if(cmd == "setDigitalIO")
  {
    if(json_obj.contains("mcuDio"))
    {
      auto msg_array = obj["mcuDio"];
      QJsonValue parsed = convert_item(msg_array);
      json_obj.insert("mcuDio", parsed);
    }

    if(json_obj.contains("mcuDin"))
    {
      auto msg_array = obj["mcuDin"];
      QJsonValue parsed = convert_item(msg_array);
      json_obj.insert("mcuDin", parsed);
    }
  }

  QJsonObject root;
  root.insert("topic", QString::fromStdString(ev.get_name()));
  root.insert("data", json_obj);

  QString wrapped = QString(QJsonDocument(root).toJson(QJsonDocument::Compact));
  {
    std::lock_guard<std::mutex> lock(recv_mtx);
    recv_queue.push(wrapped);
    recv_cv.notify_one();
  }

  set_last_receive_msg(wrapped);
}

void COMM_MSA::recv_message_single_shot(sio::event& ev)
{
  sio::message::ptr msg = ev.get_message();
  if(!msg || msg->get_flag() != sio::message::flag_object)
  {
    log_warn("recv_message_single_shot, invalid msg");
    return;
  }


  auto obj = msg->get_map();
  QJsonObject json_obj = sio_object_to_qt_json_object(obj);

  QJsonObject root_obj;
  root_obj.insert("topic", QString::fromStdString(ev.get_name()));
  root_obj.insert("data", json_obj);

  QString wrapped = QString(QJsonDocument(root_obj).toJson(QJsonDocument::Compact));
  set_last_receive_msg(wrapped);

  QString topic = get_json(root_obj, "topic");
  if(topic == "updateRequest")
  {
    handle_update_cmd(json_obj);
  }
}

void COMM_MSA::recv_loop()
{
  log_info("recv_loop STARTED");
  while(is_recv_running)
  {
    std::unique_lock<std::mutex> lock(recv_mtx);
    recv_cv.wait(lock, [this]
    {
      return !recv_queue.empty() || !is_recv_running;
    });

    if(!is_recv_running)
    {
      break;
    }

    QString recv_msg = recv_queue.front();
    recv_queue.pop();
    lock.unlock();

    QJsonObject root_obj = QJsonDocument::fromJson(recv_msg.toUtf8()).object();
    QJsonObject data = root_obj.value("data").toObject();

    QString topic = get_json(root_obj, "topic");
    if(topic == "moveRequest")
    {
      handle_move_cmd(data);
    }
    else if(topic == "loadRequest")
    {
      handle_load_cmd(data);
    }
    else if(topic == "localizationRequest")
    {
      handle_localization_cmd(data);
    }
    else if(topic == "controlRequest")
    {
      handle_control_cmd(data);
    }
    else if(topic == "mappingRequest")
    {
      handle_mapping_cmd(data);
    }
    else if(topic == "pathRequest")
    {
      handle_path_cmd(data);
    }
    else if(topic == "sensorRequest")
    {
      handle_sensor_cmd(data);
    }
    else if(topic == "vobs")
    {
      handle_vobs_cmd(data);
    }
    else if(topic == "settingRequest")
    {
      handle_setting_cmd(data);
    }

    log_info("recv, command: {}, time: {}", topic.toStdString(), get_time0());
  }
}

QJsonObject COMM_MSA::sio_object_to_qt_json_object(const std::map<std::string, sio::message::ptr>& obj)
{
  QJsonObject json_obj;
  for(auto& kv : obj)
  {
    QString key = QString::fromStdString(kv.first);

    if(kv.second->get_flag() == sio::message::flag_string)
    {
      json_obj.insert(key, QString::fromStdString(kv.second->get_string()));
    }
    else if(kv.second->get_flag() == sio::message::flag_integer)
    {
      json_obj.insert(key, QJsonValue::fromVariant(QVariant::fromValue<qint64>(kv.second->get_int())));
    }
    else if(kv.second->get_flag() == sio::message::flag_double)
    {
      json_obj.insert(key, kv.second->get_double());
    }
    else if(kv.second->get_flag() == sio::message::flag_boolean)
    {
      json_obj.insert(key, kv.second->get_bool());
    }
    else if(kv.second->get_flag() == sio::message::flag_array || kv.second->get_flag() == sio::message::flag_object)
    {
      json_obj.insert(key, convert_item(kv.second)); // 재귀 호출
    }
  }

  return json_obj;
}

QJsonValue COMM_MSA::convert_item(sio::message::ptr item)
{
  if(!item)
  {
    return QJsonValue();
  }

  int flag = item->get_flag();
  if(flag == sio::message::flag_string)
  {
    return QString::fromStdString(item->get_string());
  }
  else if(flag == sio::message::flag_integer)
  {
    return QJsonValue::fromVariant(QVariant::fromValue<qint64>(item->get_int()));
  }
  else if(flag == sio::message::flag_double)
  {
    return item->get_double();
  }
  else if(flag == sio::message::flag_boolean)
  {
    return item->get_bool();
  }
  else if(flag == sio::message::flag_array)
  {
    QJsonArray arr;
    for(auto& sub_item : item->get_vector())
    {
      arr.append(convert_item(sub_item));
    }
    return arr;
  }
  else if(flag == sio::message::flag_object)
  {
    QJsonObject obj;
    for (auto& kv : item->get_map())
    {
      obj.insert(QString::fromStdString(kv.first), convert_item(kv.second));
    }
    return obj;
  }
  else
  {
    return QJsonValue();
  }
}

// send status
void COMM_MSA::send_move_status()
{
  if(!is_connected || !ctrl || !mobile || !unimap || !dctrl)
  {
    return;
  }

  QString cur_node_id  = ctrl->get_cur_node_id();
  QString goal_node_id = ctrl->get_cur_move_info().goal_node_id;
  const MOBILE_POSE mo = mobile->get_pose();
  const Eigen::Matrix4d cur_tf = loc->get_cur_tf();
  const Eigen::Vector3d cur_xi = TF_to_se2(cur_tf);

  QString cur_node_name = "";
  if(unimap->get_is_loaded() == MAP_LOADED && !cur_node_id.isEmpty())
  {
    cur_node_id = ctrl->get_cur_node_id();
    if(NODE* node = unimap->get_node_by_id(cur_node_id))
    {
      cur_node_name = node->name;
    }
  }

  QString goal_node_name = "";
  Eigen::Vector3d goal_xi(0,0,0);
  if(unimap->get_is_loaded() == MAP_LOADED && !goal_node_id.isEmpty())
  {
    if(NODE* node = unimap->get_node_by_id(goal_node_id))
    {
      goal_node_name = node->name;
      goal_xi = TF_to_se2(node->tf);
    }
  }

  // // create move object
  // sio::object_message::ptr obj_move_state = sio::object_message::create();
  // add_to_obj(obj_move_state, "auto_move", ctrl->get_auto_state()); // "stop", "move", "pause", "error", "not ready", "vir"
  // add_to_obj(obj_move_state, "dock_move", "stop");
  // add_to_obj(obj_move_state, "jog_move", "none");
  // add_to_obj(obj_move_state, "obs", ctrl->get_obs_condition());
  // add_to_obj(obj_move_state, "path", ctrl->get_multi_reqest_state()); // "none", "req_path", "recv_path"
  // add_to_obj(obj_move_state, "path_time", QString::number(ctrl->get_global_path_time()));
  // add_to_obj(obj_move_state, "step", ctrl->get_last_step());

  // // create pose object
  // sio::object_message::ptr obj_pose = sio::object_message::create();
  // add_to_obj(obj_pose, "x", cur_xi[0]);
  // add_to_obj(obj_pose, "y", cur_xi[1]);
  // add_to_obj(obj_pose, "rz", cur_xi[2] * R2D);

  // // create velocity object
  // sio::object_message::ptr obj_velocity = sio::object_message::create();
  // add_to_obj(obj_velocity, "vx", mo.vel[0]);
  // add_to_obj(obj_velocity, "vy", mo.vel[1]);
  // add_to_obj(obj_velocity, "wz", mo.vel[2] * R2D);

  // // create current node object
  // sio::object_message::ptr obj_cur_node = sio::object_message::create();
  // add_to_obj(obj_cur_node, "x", cur_xi[0]);
  // add_to_obj(obj_cur_node, "y", cur_xi[1]);
  // add_to_obj(obj_cur_node, "rz", cur_xi[2] * R2D);
  // add_to_obj(obj_cur_node, "id", cur_node_id);
  // add_to_obj(obj_cur_node, "name", cur_node_name);
  // add_to_obj(obj_cur_node, "state", "");

  // // create goal node object
  // sio::object_message::ptr obj_goal_node = sio::object_message::create();
  // add_to_obj(obj_goal_node, "x", goal_xi[0]);
  // add_to_obj(obj_goal_node, "y", goal_xi[1]);
  // add_to_obj(obj_goal_node, "rz", goal_xi[2] * R2D);
  // add_to_obj(obj_goal_node, "id", goal_node_id);
  // add_to_obj(obj_goal_node, "name", goal_node_name);
  // add_to_obj(obj_goal_node, "state", ctrl->get_cur_move_state());

  // // create root object
  // sio::object_message::ptr obj_root = sio::object_message::create();
  // add_to_obj(obj_root, "vel", obj_velocity);
  // add_to_obj(obj_root, "pose", obj_pose);
  // add_to_obj(obj_root, "cur_node", obj_cur_node);
  // add_to_obj(obj_root, "goal_node", obj_goal_node);
  // add_to_obj(obj_root, "move_state", obj_move_state);
  // add_to_obj(obj_root, "time", QString::number(static_cast<long long>(get_time0()*1000)));

// create move object
  sio::object_message::ptr obj_move_state = sio::object_message::create();
  obj_move_state->get_map()["auto_move"]  = sio::string_message::create(ctrl->get_auto_state().toStdString()); // "stop", "move", "pause", "error", "not ready", "vir"
  obj_move_state->get_map()["dock_move"]  = sio::string_message::create("stop");
  obj_move_state->get_map()["jog_move"]   = sio::string_message::create("none");
  obj_move_state->get_map()["obs"]        = sio::string_message::create(ctrl->get_obs_condition().toStdString());

  auto path_time_step = ctrl->get_path_time_and_step();

  obj_move_state->get_map()["path_time"]  = sio::string_message::create(path_time_step.first.toStdString());
  obj_move_state->get_map()["step"]       = sio::int_message::create(path_time_step.second);

  obj_move_state->get_map()["path"]       = sio::string_message::create(ctrl->get_multi_reqest_state().toStdString()); // "none", "req_path", "recv_path"
  //obj_move_state->get_map()["path_time"]  = sio::string_message::create(ctrl->get_global_path_time().toStdString());
  //obj_move_state->get_map()["step"]       = sio::int_message::create(ctrl->get_last_step());

  // create pose object
  sio::object_message::ptr obj_pose = sio::object_message::create();
  obj_pose->get_map()["x"]  = sio::double_message::create(cur_xi[0]);
  obj_pose->get_map()["y"]  = sio::double_message::create(cur_xi[1]);
  obj_pose->get_map()["rz"] = sio::double_message::create(cur_xi[2] * R2D);

  // create velocity object
  sio::object_message::ptr obj_velocity = sio::object_message::create();
  obj_velocity->get_map()["vx"] = sio::double_message::create(mo.vel[0]);
  obj_velocity->get_map()["vy"] = sio::double_message::create(mo.vel[1]);
  obj_velocity->get_map()["wz"] = sio::double_message::create(mo.vel[2] * R2D);

  // create current node object
  sio::object_message::ptr obj_cur_node = sio::object_message::create();
  obj_cur_node->get_map()["x"]     = sio::double_message::create(cur_xi[0]);
  obj_cur_node->get_map()["y"]     = sio::double_message::create(cur_xi[1]);
  obj_cur_node->get_map()["rz"]    = sio::double_message::create(cur_xi[2] * R2D);
  obj_cur_node->get_map()["id"]    = sio::string_message::create(cur_node_id.toStdString());
  obj_cur_node->get_map()["name"]  = sio::string_message::create(cur_node_name.toStdString());
  obj_cur_node->get_map()["state"] = sio::string_message::create("");

  // create goal node object
  sio::object_message::ptr obj_goal_node = sio::object_message::create();
  obj_goal_node->get_map()["x"]     = sio::double_message::create(goal_xi[0]);
  obj_goal_node->get_map()["y"]     = sio::double_message::create(goal_xi[1]);
  obj_goal_node->get_map()["rz"]    = sio::double_message::create(goal_xi[2] * R2D);
  obj_goal_node->get_map()["id"]    = sio::string_message::create(goal_node_id.toStdString());
  obj_goal_node->get_map()["name"]  = sio::string_message::create(goal_node_name.toStdString());
  obj_goal_node->get_map()["state"] = sio::string_message::create(ctrl->get_cur_move_state().toStdString());

  // create root object
  sio::object_message::ptr obj_root = sio::object_message::create();
  obj_root->get_map()["vel"]        = obj_velocity;
  obj_root->get_map()["pose"]       = obj_pose;
  obj_root->get_map()["cur_node"]   = obj_cur_node;
  obj_root->get_map()["goal_node"]  = obj_goal_node;
  obj_root->get_map()["move_state"] = obj_move_state;
  obj_root->get_map()["time"] = sio::string_message::create(QString::number(static_cast<long long>(get_time0()*1000)).toStdString());

  {
    std::lock_guard<std::mutex> sock_lock(send_mtx);
    rrs_socket->socket("slamnav")->emit("moveStatus", obj_root);
  }
}

void COMM_MSA::handle_path_cmd(const QJsonObject& data)
{
  // Todo time time_1 무조건 time 으로 변경해야함 !!!!!!!!!!!!!!!!!!!!!!!!!!!
  DATA_PATH msg;
  msg.time = get_json(data, "time_1"); // msg.time = get_json(data, "time");
  // msg.time = get_json_double(data, "time");
  // if(msg.time == 0)
  // {
  //     msg.time = data["time"].toString().toDouble();
  // }
  msg.preset = get_json_int(data, "preset");
  msg.command = get_json(data, "command");

  // path 배열 파싱
  QJsonArray path_arr = data["path"].toArray();
  QStringList path_list;
  for(const auto& item : path_arr)
  {
    path_list.append(item.toString());
  }
  msg.path_str = path_list.join(",");

  QJsonArray vobs_arr = data["vobs_c"].toArray();
  QStringList vobs_list;
  for(const auto& item : vobs_arr)
  {
    vobs_list.append(item.toString());
  }
  msg.vobs_closures_str = vobs_list.join(",");

  QJsonDocument doc(data);
  QString json_string = doc.toJson(QJsonDocument::Compact);
  log_info("path cmd: {}", json_string.toStdString());
  {

    std::lock_guard<std::mutex> lock(path_mtx);
    path_queue.push(std::move(msg));
    path_cv.notify_one();
  }
}

void COMM_MSA::handle_sensor_cmd(const QJsonObject& data)
{
  DATA_SENSOR_INFO msg;
  msg.time = get_json_double(data, "time")/1000;
  msg.id = get_json(data, "id");
  msg.command = get_json(data, "command");
  msg.target = get_json(data, "target");
  msg.index = parse_index_json(data, "index");
  {
    std::lock_guard<std::mutex> lock(sensor_mtx);
    sensor_queue.push(std::move(msg));
    sensor_cv.notify_one();
  }
}

void COMM_MSA::handle_vobs_cmd(const QJsonObject& data)
{
  DATA_VOBS msg;
  msg.time                    = get_json_double(data, "time")/1000;
  msg.command                 = get_json(data, "command");
  msg.vobs_robots             = get_json(data, "vobs_r");
  msg.vobs_clousers           = get_json(data, "vobs_c");
  msg.is_vobs_closures_change = get_json(data, "is_vobs_c");
  {
    std::lock_guard<std::mutex> lock(vobs_mtx);
    vobs_queue.push(std::move(msg));
    vobs_cv.notify_one();
  }
}

void COMM_MSA::handle_move_cmd(const QJsonObject& data)
{
  DATA_MOVE msg;
  msg.id              = get_json(data, "id");
  msg.time            = get_json_double(data, "time") / 1000.0;
  msg.preset          = get_json_int(data, "preset");
  msg.command         = get_json(data, "command");
  msg.method          = get_json(data, "method");
  msg.direction       = get_json(data, "direction");
  msg.goal_node_id    = get_json(data, "goalId");
  msg.goal_node_name  = get_json(data, "goalName");
  msg.tgt_pose_vec[0] = get_json_double(data, "x");
  msg.tgt_pose_vec[1] = get_json_double(data, "y");
  msg.tgt_pose_vec[2] = get_json_double(data, "z");
  msg.tgt_pose_vec[3] = get_json_double(data, "rz") * D2R;
  msg.jog_val[0]      = get_json_double(data, "vx");
  msg.jog_val[1]      = get_json_double(data, "vy");
  msg.jog_val[2]      = get_json_double(data, "wz");
  msg.target          = get_json_double(data, "target");
  msg.speed           = get_json_double(data, "speed");
  msg.meassured_dist  = get_json_double(data, "measuredDist");
  msg.remaining_dist  = get_json_double(data, "remainingDist");
  msg.remaining_time  = get_json_double(data, "remainingTime");
  msg.bat_percent     = get_json_int(data, "battery");
  msg.result          = get_json(data, "result");
  msg.message         = get_json(data, "message");

  QJsonDocument doc(data);
  QString json_string = doc.toJson(QJsonDocument::Compact);
  log_info("move cmd: {}", json_string.toStdString());
  {
    std::lock_guard<std::mutex> lock(move_mtx);
    move_queue.push(std::move(msg));
    move_cv.notify_one();
  }
}

void COMM_MSA::handle_load_cmd(const QJsonObject& data)
{
  DATA_LOAD msg;
  msg.id       = get_json(data, "id");
  msg.time     = get_json(data, "time").toDouble() / 1000;
  msg.command  = get_json(data, "command");
  msg.map_name = get_json(data, "mapName");
  {
    std::lock_guard<std::mutex> lock(load_mtx);
    load_queue.push(msg);
    load_cv.notify_one();
  }
}

void COMM_MSA::handle_mapping_cmd(const QJsonObject& data)
{
  DATA_MAPPING msg;
  msg.id       = get_json(data, "id");
  msg.time     = get_json_double(data, "time")/1000;
  msg.command  = get_json(data, "command");
  msg.map_name = get_json(data, "mapName");

  QJsonDocument doc(data);
  QString json_string = doc.toJson(QJsonDocument::Compact);
  log_info("mapping cmd: {}", json_string.toStdString());
  {
    std::lock_guard<std::mutex> lock(mapping_mtx);
    mapping_queue.push(msg);
    mapping_cv.notify_one();
  }
}

void COMM_MSA::handle_localization_cmd(const QJsonObject& data)
{
  DATA_LOCALIZATION msg;
  msg.id      = get_json(data, "id");
  msg.time    = get_json_double(data, "time")/1000;
  msg.command = get_json(data, "command");

  if (msg.command == "init")
  {
    msg.tgt_pose_vec[0] = get_json_double(data, "x");
    msg.tgt_pose_vec[1] = get_json_double(data, "y");
    msg.tgt_pose_vec[2] = get_json_double(data, "z");
    msg.tgt_pose_vec[3] = get_json_double(data, "rz");

    set_last_tgt_pose_vec(msg.tgt_pose_vec);
  }
  else if (msg.command == "start")
  {
    msg.tgt_pose_vec = get_last_tgt_pose_vec();
  }
  else if (msg.command == "stop")
  {

  }
  else
  {
    msg.result = "reject";
    msg.message = "undefined command";
    send_localization_response(msg);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(localization_mtx);
    localization_queue.push(msg);
    localization_cv.notify_one();
  }
}

void COMM_MSA::handle_update_cmd(const QJsonObject& data)
{
  DATA_SOFTWARE msg;
  msg.id = get_json(data, "id");
  msg.time = get_json_double(data, "time")/1000;
  msg.command = get_json(data, "command");

  if(msg.command == "getVersion")
  {

  }
  else if(msg.command == "update")
  {
    msg.branch = get_json(data, "branch");
    msg.version = get_json(data, "version");
    handle_update_software(msg);
  }
  else
  {
    msg.result = "reject";
    msg.message = "undefined command";
//        send_update_response(msg);
    return;
  }

}

void COMM_MSA::handle_control_cmd(const QJsonObject& data)
{
  DATA_CONTROL msg;
  msg.id              = get_json(data, "id");
  msg.time            = get_json_double(data, "time")/1000;
  msg.command         = get_json(data, "command");

  if(msg.command == "setSafetyField" || msg.command == "getSafetyField")
  {
    msg.safetyField = get_json(data, "safetyField");
  }
  else if(msg.command == "resetSafetyField" || msg.command == "setSafetyFlag")
  {
    // safetyFlags 배열 파싱: [{"name": "obstacle", "value": false}, {"name": "bumper", "value": false}]
    QJsonArray safetyFlagsArr = data["safetyFlags"].toArray();
    for(const QJsonValue& val : safetyFlagsArr)
    {
      QJsonObject flagObj = val.toObject();
      QString name = get_json(flagObj, "name");
      bool value = get_json_bool(flagObj, "value");
      msg.resetFlags.append(qMakePair(name, value));
    }

    // 하위 호환성을 위해 기존 resetField도 지원
    if(msg.resetFlags.isEmpty() && data.contains("resetField"))
    {
      msg.resetField = data["resetField"].toString();
    }
  }
  else if(msg.command == "ledControl")
  {
    msg.onoff = get_json_bool(data, "onoff");
    msg.color = get_json(data, "color");
  }
  else if(msg.command == "lidarOnOff" || msg.command == "pathOnOff")
  {
    msg.onoff = get_json_bool(data, "onoff");
    msg.frequency = get_json_int(data, "frequency");
  }
  else if(msg.command == "motorOnOff")
  {
    msg.onoff = get_json_bool(data, "onoff");
  }
  else if(msg.command == "setDigitalIO")
  {
    handle_safetyio_cmd(data);
    return;
  }
  else if(msg.command == "getDigitalIO")
  {
    handle_send_safetyIO(data);
    return;
  }
  else if(msg.command == "chargeTrigger")
  {
    handle_charge_trigger(msg);
  }

  {
    std::lock_guard<std::mutex> lock(control_mtx);
    control_queue.push(msg);
    control_cv.notify_one();
  }
}

void COMM_MSA::handle_setting_cmd(const QJsonObject& data)
{
  DATA_PDU_UPDATE msg;
  msg.id = get_json(data, "id");
  msg.time = get_json_double(data, "time")/1000;
  msg.command = get_json(data, "command");

  if(msg.command ==  "setPduParam" || msg.command ==  "getPduParam")
  {
    QJsonArray totalArr = data["params"].toArray();

    if(totalArr.isEmpty())
    {
      log_error("missing name or type field");
      msg.result = "reject";
      msg.message = "missing name or type field";
      send_config_request_response(msg);
      return;
    }

    QJsonObject obj = totalArr.first().toObject();

    QString key  = obj.value("name").toString();
    QString value = obj.value("value").toString();
    QString type  = obj.value("type").toString();

    msg.param_list.push_back(DATA_PDU_UPDATE::PARAM_ITEM(key, type, value));
  }
  else if(msg.command == "getDriveParam" )
  {
    // nothing to do
  }
  else if(msg.command == "param_set")
  {
    // nothing to do
  }
  else
  {
    msg.result = "reject";
    msg.message = "undefined command";
    send_config_request_response(msg);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(setting_mtx);
    setting_queue.push(msg);
    setting_cv.notify_one();
  }
}

void COMM_MSA::setting_loop()
{
  while(is_setting_running)
  {
    std::unique_lock<std::mutex> lock(setting_mtx);
    setting_cv.wait(lock, [this]
    {
      return !setting_queue.empty() || !is_setting_running;
    });

    DATA_PDU_UPDATE msg = setting_queue.front();
    setting_queue.pop();
    lock.unlock();

    QString command = msg.command;

    if(command == "setPduParam")
    {
      handle_set_pdu_param(msg);
    }
    else if(command == "getPduParam")
    {
      log_info("getPduParam not developed");
      msg.result = "reject";
      msg.message = "not developed";

      send_config_request_response(msg);
    }
    else if(command == "param_set")
    {
      log_info("param_set not developed");
      msg.result = "reject";
      msg.message = "not developed";

      send_config_request_response(msg);
    }
    else if(command == "getDriveParam")
    {
      handle_get_drive_param(msg);
    }
  }
}

void COMM_MSA::handle_set_pdu_param(DATA_PDU_UPDATE& msg)
{
  if(!mobile)
  {
    msg.result = "reject";
    msg.message = "mobile not connected";
    send_config_request_response(msg);
    return;
  }

  if(msg.param_list.size() != 1)
  {
    log_error("set PDU param : msg.param_list error");
    msg.result = "reject";
    msg.message = "param_list size is not 1";
    send_config_request_response(msg);
    return;
  }
  log_info("set PDU Param");

  auto key = msg.param_list.front().key;
  auto val = msg.param_list.front().value;
  auto type = msg.param_list.front().type;

  if(key == "use_sf_obstacle_detect")
  {
    if(type != "boolean")
    {
      msg.result = "reject";
      msg.message = "invalid type";
      send_config_request_response(msg);
      return;
    }

    bool is_true;
    if(val == "true")
    {
      is_true = true;
    }
    else if(val == "false")
    {
      is_true = false;
    }
    else
    {
      msg.result = "reject";
      msg.message = "invalid value";
      send_config_request_response(msg);
      return;
    }

    msg.result = "accept";
    msg.message = "";
    send_config_request_response(msg);

    mobile->set_detect_mode(is_true);
    delayed_tasks_.schedule(std::chrono::milliseconds(10), [this, msg]() {
      //no way to check

      DATA_PDU_UPDATE response_msg = msg;
      response_msg.result = "success";
      response_msg.message = "";
      send_config_request_response(response_msg);
    });
  }
  else
  {
    log_error("[MSA] slot_config_request, invalid parameter name ");
    msg.result = "reject";
    msg.message = "invalid parameter name";
    send_config_request_response(msg);
    return;
  }
}

void COMM_MSA::handle_get_drive_param(DATA_PDU_UPDATE& msg)
{
  if(!mobile)
  {
    msg.result = "reject";
    msg.message = "mobile not connected";
    send_config_request_response(msg);
    return;
  }
  log_info("get Drive Param");

  mobile->robot_request();
  delayed_tasks_.schedule(std::chrono::milliseconds(1000), [this, msg]() {
    auto ms = mobile->get_setting();

    DATA_PDU_UPDATE response_msg = msg;
    response_msg.result = "success";
    response_msg.message = "";
    response_msg.setting = ms;

    send_config_request_response(response_msg);
  });
}

void COMM_MSA::send_config_request_response(const DATA_PDU_UPDATE& msg)
{
  if(!is_connected)
  {
    return;
  }

  sio::object_message::ptr root_obj = sio::object_message::create();
  add_to_obj(root_obj, "id", msg.id);
  add_to_obj(root_obj, "time", static_cast<long long>(msg.time*1000));
  add_to_obj(root_obj, "command", msg.command);
  add_to_obj(root_obj, "result",  msg.result);
  add_to_obj(root_obj, "message", msg.message);

  sio::array_message::ptr param_array = sio::array_message::create();

  if(msg.command == "getDriveParam")
  {
    auto add_param = [&](const QString& name,
        const QString& value,
        const QString& type) {
      sio::object_message::ptr obj = sio::object_message::create();
      add_to_obj(obj, "name", name);
      add_to_obj(obj, "value", value);
      add_to_obj(obj, "type", type);
      param_array->get_vector().push_back(obj);
    };

    add_param("version", QString::number(msg.setting.version),"int");
    add_param("robot_type", QString::number(msg.setting.robot_type), "int");
    add_param("v_limit", QString::number(msg.setting.v_limit, 'f', 3), "float");
    add_param("w_limit", QString::number(msg.setting.w_limit * R2D, 'f', 3), "float");
    add_param("a_limit", QString::number(msg.setting.a_limit, 'f', 3), "float");
    add_param("b_limit", QString::number(msg.setting.b_limit * R2D, 'f', 3), "float");
    add_param("v_limit_jog", QString::number(msg.setting.v_limit_jog, 'f', 3), "float");
    add_param("w_limit_jog", QString::number(msg.setting.w_limit_jog * R2D, 'f', 3), "float");
    add_param("a_limit_jog", QString::number(msg.setting.a_limit_jog, 'f', 3), "float");
    add_param("b_limit_jog", QString::number(msg.setting.b_limit_jog * R2D, 'f', 3), "float");
    add_param("v_limit_monitor", QString::number(msg.setting.v_limit_monitor, 'f', 3), "float");
    add_param("w_limit_monitor", QString::number(msg.setting.w_limit_monitor * R2D, 'f', 3), "float");
    add_param("safety_v_limit", QString::number(msg.setting.safety_v_limit, 'f', 3), "float");
    add_param("safety_w_limit", QString::number(msg.setting.safety_w_limit * R2D, 'f', 3), "float");
    add_param("w_s", QString::number(msg.setting.w_s, 'f', 3), "float");
    add_param("w_r", QString::number(msg.setting.w_r, 'f', 3), "float");
    add_param("gear", QString::number(msg.setting.gear, 'f', 3), "float");
    add_param("dir", QString::number(msg.setting.dir, 'f', 3), "float");

    add_to_obj(root_obj, "params", param_array);
  }
  else if(msg.command == "getPduParam" || msg.command == "setPduParam")
  {
    for(const auto& param : msg.param_list)
    {
      sio::object_message::ptr obj = sio::object_message::create();
      add_to_obj(obj, "name", param.key);
      add_to_obj(obj, "value", param.value);
      add_to_obj(obj, "type", param.type);
      param_array->get_vector().push_back(obj);

      add_to_obj(root_obj, "params", param_array);
    }
  }

  SOCKET_MESSAGE socket_msg;
  socket_msg.event = "settingResponse";
  socket_msg.data  = root_obj;
  {
    std::lock_guard<std::mutex> lock(send_response_mtx);
    send_response_queue.push(socket_msg);
    send_response_cv.notify_one();
  }
}

void COMM_MSA::move_loop()
{
  while(is_move_running)
  {
    std::unique_lock<std::mutex> lock(move_mtx);
    move_cv.wait(lock, [this]
    {
      return !move_queue.empty() || !is_move_running;
    });

    DATA_MOVE msg = move_queue.front();
    move_queue.pop();
    lock.unlock();

    QString command = msg.command;
    if(command == "jog")
    {
      handle_move_jog(msg);
    }
    else if(command == "target")
    {
      handle_move_target(msg);
    }
    else if(command == "goal" || command == "change_goal")
    {
      handle_move_goal(msg);
    }
    else if(command == "pause")
    {
      handle_move_pause(msg);
    }
    else if(command == "resume")
    {
      handle_move_resume(msg);
    }
    else if(command == "stop")
    {
      handle_move_stop(msg);
    }
    else if(command == "xLinear" || command == "yLinear" || command == "circular" || command == "rotate")
    {
      handle_move_profile(msg);
    }
  }
}

void COMM_MSA::path_loop()
{
  while(is_path_running)
  {
    std::unique_lock<std::mutex> lock(path_mtx);
    path_cv.wait(lock, [this]
    {
      return !path_queue.empty() || !is_path_running;
    });

    DATA_PATH msg = path_queue.front();
    path_queue.pop();
    lock.unlock();

    double st_time = get_time0();

    handle_path(msg);

    double ed_time = get_time0();
    process_time_path = ed_time - st_time;
    if(max_process_time_path < process_time_path)
    {
      max_process_time_path = static_cast<double>(process_time_path.load());
    }
  }
}

void COMM_MSA::vobs_loop()
{
  while(is_vobs_running)
  {
    std::unique_lock<std::mutex> lock(vobs_mtx);
    vobs_cv.wait(lock, [this]
    {
      return !vobs_queue.empty() || !is_vobs_running;
    });

    DATA_VOBS msg = vobs_queue.front();
    vobs_queue.pop();
    lock.unlock();

    double st_time = get_time0();

    QString command = msg.command;
    if(command == "vobs")
    {
      handle_vobs(msg);
    }

    double ed_time = get_time0();
    process_time_vobs = ed_time - st_time;
    if(max_process_time_vobs < process_time_vobs)
    {
      max_process_time_vobs = (double)process_time_vobs;
    }
  }
}

void COMM_MSA::load_loop()
{
  while(is_load_running)
  {
    std::unique_lock<std::mutex> lock(load_mtx);
    load_cv.wait(lock, [this]
    {
      return !load_queue.empty() || !is_load_running;
    });

    DATA_LOAD msg = load_queue.front();
    load_queue.pop();
    lock.unlock();

    QString command = msg.command;
    if(command == "loadMap")
    {
      handle_load_map(msg);
    }
    else if(command == "loadTopo")
    {
      handle_load_topo(msg);
    }
    else
    {
      msg.result = "reject";

      send_load_response(msg);
      log_error("Unknown error load loop");
    }
  }
}

void COMM_MSA::mapping_loop()
{
  while(is_mapping_running)
  {
    std::unique_lock<std::mutex> lock(mapping_mtx);
    mapping_cv.wait(lock, [this]
    {
      return !mapping_queue.empty() || !is_mapping_running;
    });


    DATA_MAPPING msg = std::move(mapping_queue.front());
    mapping_queue.pop();
    lock.unlock();

    const QString command = msg.command;
    if(command == "mappingStart")
    {
      handle_mapping_start(msg);
    }
    else if(command == "mappingStop")
    {
      handle_mapping_stop(msg);
    }
    else if(command == "mappingSave")
    {
      handle_mapping_save(msg);
    }
    else if(command == "mappingReload")
    {
      handle_mapping_reload(msg);
    }
    else
    {
      msg.result = "reject";
      send_mapping_response(msg);
    }
  }
}

void COMM_MSA::localization_loop()
{
  while(is_localization_running)
  {
    std::unique_lock<std::mutex> lock(localization_mtx);
    localization_cv.wait(lock, [this]
    {
      return !localization_queue.empty() || !is_localization_running;
    });

    DATA_LOCALIZATION msg = localization_queue.front();
    localization_queue.pop();
    lock.unlock();

    QString command = msg.command;
    if(command == "semiautoinit")
    {
      handle_localization_semiautoinit(msg);
    }
    else if(command == "init")
    {
      handle_localization_init(msg);
    }
    else if(command == "start")
    {
      handle_localization_start(msg);
    }
    else if(command == "stop")
    {
      handle_localization_stop(msg);
    }
    else if(command == "randominit")
    {
      handle_localization_randominit(msg);
    }
    else
    {
      msg.result = "reject";
      send_localization_response(msg);
    }
  }
}

void COMM_MSA::sensor_loop()
{
  while(is_sensor_running)
  {
    std::unique_lock<std::mutex> lock(sensor_mtx);
    sensor_cv.wait(lock, [this]
    {
      return !sensor_queue.empty() || !is_sensor_running;
    });

    DATA_SENSOR_INFO msg = sensor_queue.front();
    sensor_queue.pop();
    lock.unlock();

    QString command = msg.command;
    QString target = msg.target;

    if(target == "cam")
    {
      if(command == "getInfo")
      {
        handle_camera_get_info(msg);
      }
      else if(command == "setOrder")
      {
        handle_camera_set_info(msg);
      }
    }
    else if(target == "lidar2d")
    {

    }
    else if(target == "lidar3d")
    {
      if(command == "setOn")
      {
        handle_lidar3d_set_on(msg);
      }
      else if(command == "setOff")
      {
        handle_lidar3d_set_off(msg);
      }
    }
    else
    {
      msg.result = "reject";
      send_sensor_response(msg);
    }
  }
}

void COMM_MSA::control_loop()
{
  while(is_control_running)
  {
    std::unique_lock<std::mutex> lock(control_mtx);
    control_cv.wait(lock, [this]
    {
      return !control_queue.empty() || !is_control_running;
    });

    DATA_CONTROL msg = control_queue.front();
    control_queue.pop();
    lock.unlock();

    QString command = msg.command;
    dctrl->set_cmd_id(msg.id);

    if(command == "dock")
    {
      handle_dock_start(msg);
    }
    else if(command == "undock")
    {
      handle_undock_start(msg);
    }
    else if(command == "dockStop")
    {
      handle_dock_stop(msg);
    }
    else if(command == "randomSeq")
    {
      log_info("RandomSeq not developed");
      msg.result = "reject";
      msg.message = "not developed";

      send_control_response(msg);
    }
    else if(command == "ledControl")
    {
      // main->is_user_led
      // main->user_led_color
      log_info("LedControl not developed");
      msg.result = "reject";
      msg.message = "not developed";

      send_control_response(msg);
    }
    else if(command == "lidarOnOff")
    {
      // main->lidar_view_frequency = msg.frequency;
      log_info("LidarOnOff not developed");
      msg.result = "reject";
      msg.message = "not developed";

      send_control_response(msg);
    }
    else if(command == "pathOnOff")
    {
      // main->path_view_frequency = msg.frequency;
      log_info("PathOnOff not developed");
      msg.result = "reject";
      msg.message = "not developed";

      send_control_response(msg);
    }
    else if(command == "motorOnOff")
    {
      handle_motor_control(msg);
    }
    else if(command == "setSafetyField")
    {
      handle_set_safety_field(msg);
    }
    else if(command == "getSafetyField")
    {
      handle_get_safety_field(msg);
    }
    else if(command == "resetSafetyField" || command == "setSafetyFlag")
    {
      handle_reset_safety_field(msg);
    }
    else if(command == "getSafetyFlag")
    {
      handle_get_safety_flag(msg);
    }
    else
    {
      msg.result = "reject";
      msg.message = "command 값이 잘못되었습니다.";
      log_error("Invalid command value");

      send_control_response(msg);
    }
  }
}

void COMM_MSA::handle_localization_semiautoinit(DATA_LOCALIZATION& msg)
{
  if(unimap->get_is_loaded() != MAP_LOADED)
  {
    log_error("Map is not loaded");

    msg.result = "reject";
    send_localization_response(msg);
    return;
  }

  if(config->get_loc_mode() == "2D" && !lidar_2d->get_is_connected())
  {
    log_error("LIDAR 2D is not connected");

    msg.result = "reject";
    send_localization_response(msg);
    return;
  }
  else if(config->get_loc_mode() == "3D" && !lidar_3d->get_is_connected())
  {
    log_error("LIDAR 3D is not connected");

    msg.result = "reject";
    send_localization_response(msg);
    return;
  }

  if(loc->get_is_busy())
  {
    log_info("Localization is already running");

    msg.result = "reject";
    send_localization_response(msg);
    return;
  }

  msg.result = "accept";
  msg.message = "";
  send_localization_response(msg);

  log_info("recv_loc, start semi-auto init");
  loc->stop();

  // semi auto init
  if(semi_auto_init_thread)
  {
    log_info("recv_loc, thread already running.");

    if(semi_auto_init_thread->joinable())
    {
      semi_auto_init_thread->join();
    }
    semi_auto_init_thread.reset();
  }
  semi_auto_init_thread = std::make_unique<std::thread>(&LOCALIZATION::start_semiauto_init, loc);

  // check loc state & response
  delayed_tasks_.schedule(std::chrono::milliseconds(1000), [this, msg]() {
    auto state = loc->get_cur_loc_state();

    DATA_LOCALIZATION response_msg = msg;
    if(state == "good")
    {
      response_msg.result = "success";
    }
    else
    {
      response_msg.result = "fail";
    }
    send_localization_response(response_msg);
  });
}

void COMM_MSA::handle_localization_init(DATA_LOCALIZATION& msg)
{
  if(unimap->get_is_loaded() != MAP_LOADED)
  {
    log_error("Map is not loaded");

    msg.result = "reject";
    send_localization_response(msg);
  }

  QString loc_mode = config->get_loc_mode();
  if(loc_mode == "2D" && !lidar_2d->get_is_connected())
  {
    log_error("LIDAR 2D is not connected");

    msg.result = "reject";
    send_localization_response(msg);

    return;
  }
  else if(loc_mode == "3D" && !lidar_3d->get_is_connected())
  {
    log_error("LIDAR 3D is not connected");

    msg.result = "reject";
    send_localization_response(msg);
    return;
  }
  log_info("recv, command: init, time: {}", msg.time);

  msg.result = "accept";
  msg.message = "";
  send_localization_response(msg);

  // manual init
  double x    = msg.tgt_pose_vec[0];
  double y    = msg.tgt_pose_vec[1];
  double test = msg.tgt_pose_vec[2];
  double rz   = msg.tgt_pose_vec[3];

  loc->stop();
  loc->set_cur_tf(se2_to_TF(Eigen::Vector3d(x, y, rz*D2R)));

  delayed_tasks_.schedule(std::chrono::milliseconds(10), [this, msg]() {
    DATA_LOCALIZATION response_msg = msg;
    response_msg.result = "success";
    response_msg.message = "";
    send_localization_response(response_msg);
  });
}

void COMM_MSA::handle_localization_start(DATA_LOCALIZATION& msg)
{
  double x = msg.tgt_pose_vec[0];
  double y = msg.tgt_pose_vec[1];
  double rz = msg.tgt_pose_vec[3];
  if(isnan(x) || isnan(y) || isnan(rz))
  {
    log_error("invalid params");

    msg.result = "reject";
    send_localization_response(msg);
    return;
  }
  log_info("recv_loc, start localization");
  
  msg.result = "accept";
  msg.message = "";
  send_localization_response(msg);

  loc->stop();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  loc->set_cur_tf(se2_to_TF(Eigen::Vector3d(x, y, rz*D2R)));
  loc->start();

  delayed_tasks_.schedule(std::chrono::milliseconds(50), [this, msg]() {
    
    DATA_LOCALIZATION response_msg = msg;
    if(loc->get_is_loc() == true)
    {
      response_msg.result = "success";
    }
    else
    {
      response_msg.result = "fail";
    }
    response_msg.message = "";
    send_localization_response(response_msg);
  });
}

void COMM_MSA::handle_localization_stop(DATA_LOCALIZATION& msg)
{
  log_info("recv_loc, stop localization");
  msg.result = "accept";
  msg.message = "";
  send_localization_response(msg);

  loc->stop();

  delayed_tasks_.schedule(std::chrono::milliseconds(50), [this, msg]() {
    
    DATA_LOCALIZATION response_msg = msg;
    if(loc->get_is_loc() == false)
    {
      response_msg.result = "success";
    }
    else
    {
      response_msg.result = "fail";
    }
    response_msg.message = "";
    send_localization_response(response_msg);
  });
}

void COMM_MSA::handle_localization_randominit(DATA_LOCALIZATION& msg)
{
  // todo
  return;
}

void COMM_MSA::handle_mapping_start(DATA_MAPPING& msg)
{
  if(!lidar_2d->get_is_connected())
  {
    msg.result = "reject";
    send_mapping_response(msg);
    return;
  }

  msg.result = "accept";
  msg.message = "";
  send_mapping_response(msg);

  last_send_kfrm_idx = 0;
  Q_EMIT (signal_map_build_start());

  delayed_tasks_.schedule(std::chrono::milliseconds(100), [this, msg]() {
    
    DATA_MAPPING response_msg = msg;
    if(mapping->get_is_mapping() == true)
    {
      response_msg.result = "success";
    }
    else
    {
      response_msg.result = "fail";
    }
    response_msg.message = "";
    send_mapping_response(response_msg);
  });
}

void COMM_MSA::handle_mapping_stop(DATA_MAPPING& msg)
{
  msg.result = "accept";
  msg.message = "";
  send_mapping_response(msg);

  Q_EMIT (signal_map_build_stop());

  delayed_tasks_.schedule(std::chrono::milliseconds(100), [this, msg]() {
    
    DATA_MAPPING response_msg = msg;
    if(mapping->get_is_mapping() == true)
    {
      response_msg.result = "success";
    }
    else
    {
      response_msg.result = "fail";
    }
    response_msg.message = "";
    send_mapping_response(response_msg);
  });
}

void COMM_MSA::handle_mapping_save(DATA_MAPPING& msg)
{
  if(!mapping)
  {
    msg.result = "fail";
    msg.message = "Mapping is not running";
    send_mapping_response(msg);
    return;
  }

  msg.result = "accept";
  msg.message = "";
  send_mapping_response(msg);

  std::pair<bool, QString> val = mapping->sync_map_save(msg.map_name);
  if(!val.first)
  {
    msg.result  = "fail";
    msg.message = val.second;
    send_mapping_response(msg);
    return;
  }

  msg.result = "success";
  send_mapping_response(msg);
}

void COMM_MSA::handle_mapping_reload(DATA_MAPPING& msg)
{
  msg.result = "accept";
  msg.message = "";
  send_mapping_response(msg);

  last_send_kfrm_idx = 0;

  // no way to check state
  msg.result = "success";
  send_mapping_response(msg);
}

void COMM_MSA::handle_safetyio_cmd(const QJsonObject& data)
{
  DATA_SAFTYIO msg;
  msg.id = get_json(data, "id");
  msg.time = get_json_double(data, "time")/1000;
  msg.command = get_json(data, "command");

  QJsonArray totalArr = data.value("mcuDio").toArray();
  if(totalArr.size() < 2)
  {
    return;
  }

  // MCU0
  QJsonArray mcu0_arr = totalArr[0].toArray();
  for(int i = 0; i < 8 && i < mcu0_arr.size(); i++)
  {
    //        msg.mcu0_dio[i] = static_cast<unsigned char>(mcu0_arr[i].toInt());
    msg.mcu0_dio[i] = (mcu0_arr[i].toInt() != 0) ? 1 : 0;
  }

  // MCU1
  QJsonArray mcu1_arr = totalArr[1].toArray();
  for(int i = 0; i < 8 && i < mcu1_arr.size(); i++)
  {
    //        msg.mcu1_dio[i] = static_cast<unsigned char>(mcu1_arr[i].toInt());
    msg.mcu1_dio[i] = (mcu1_arr[i].toInt() != 0) ? 1 : 0;
  }

  slot_safety_io(msg);
}

// for safetyio
void COMM_MSA::slot_safety_io(DATA_SAFTYIO msg)
{
  send_safetyio_response(msg);

  // packet
  for (int i = 0; i < 2; i++) // 0:mcu0, 1:mcu1
  {
    unsigned char* dio_arr = (i == 0) ? msg.mcu1_dio : msg.mcu0_dio;
    if (!dio_arr)
    {
      continue;
    }

    int offset = (i == 0) ? 0 : 8; //  target mcu1->0~7 ,mcu0 8~15
    for (int n = 0; n < 8; n++)
    {
      bool value = dio_arr[n]; // 0 or 1
      if(value != dio_arr_old[offset+n])
      {
        mobile->set_IO_individual_output(offset + n, value);
        dio_arr_old[offset + n] = value; //save change value
      }
    }
  }
}

void COMM_MSA::handle_send_safetyIO(const QJsonObject& data)
{
  auto toSioArray = [](unsigned char arr[8]) {
    sio::array_message::ptr jsonArr = sio::array_message::create();
    for(int i = 0; i < 8; ++i)
    {
      jsonArr->get_vector().push_back(sio::int_message::create(arr[i]));
    }
    return jsonArr;
  };

  sio::object_message::ptr root_obj = sio::object_message::create();
  MOBILE_STATUS ms = mobile->get_status();

  sio::array_message::ptr mcuDioArr = sio::array_message::create();
  mcuDioArr->get_vector().push_back(toSioArray(ms.mcu0_dio));
  mcuDioArr->get_vector().push_back(toSioArray(ms.mcu1_dio));

  sio::array_message::ptr mcuDinArr = sio::array_message::create();
  mcuDinArr->get_vector().push_back(toSioArray(ms.mcu0_din));
  mcuDinArr->get_vector().push_back(toSioArray(ms.mcu1_din));

  add_to_obj(root_obj, "id", get_json(data, "id"));
  add_to_obj(root_obj, "time", static_cast<long long>(get_time0() * 1000));
  add_to_obj(root_obj, "result", "success");

  add_to_obj(root_obj, "mcuDio", mcuDioArr);
  add_to_obj(root_obj, "mcuDin", mcuDinArr);

  {
    std::lock_guard<std::mutex> sock_lock(send_mtx);
    rrs_socket->socket("slamnav")->emit("controlResponse", root_obj);
  }
}

void COMM_MSA::handle_dock_start(DATA_CONTROL& msg)
{
  if(!dctrl || !ctrl)
  {
    msg.result = "reject";
    send_control_response(msg);
    return;
  }

  if(ctrl->get_is_moving())
  {
    msg.result = "reject";
    send_control_response(msg);
    log_info("Dock Start Failed : IS_MOVING is true");
    return;
  }
  msg.result = "accept";
  msg.message = "";
  send_control_response(msg);


  log_info("Docking start");
  int d_field = config->get_docking_field();
  if(d_field == -1)
  {
    mobile->set_detect_mode(0.0);
  }
  else
  {
    mobile->setlidarfield(d_field);
  }

  ctrl->set_is_moving(true);
  send_control_response(msg);

  Q_EMIT (signal_docking_start());


  delayed_tasks_.schedule(std::chrono::milliseconds(1000), [this, msg]() {

    auto running = dctrl->is_dock_running();

    DATA_CONTROL response_msg = msg;
    if(running == true)
    {
      response_msg.result = "success";
    }
    else
    {
      response_msg.result = "fail";
    }
    send_control_response(response_msg);
  });
}

void COMM_MSA::handle_undock_start(DATA_CONTROL& msg)
{
  if(!dctrl || !ctrl)
  {
    msg.result = "reject";
    send_control_response(msg);
    return;
  }

  if(dctrl->get_dock_fsm_state() != DOCKING_FSM_OFF)
  {
    msg.result = "reject";
    send_control_response(msg);
    log_info("UnDock Failed : DOCK_FSM_STATE is not OFF");
    return;
  }
  msg.result = "accept";
  msg.message = "";
  send_control_response(msg);


  log_info("UnDocking start");
  int d_field = config->get_docking_field();
  if(d_field == -1)
  {
    mobile->set_detect_mode(0.0);
  }
  else
  {
    mobile->setlidarfield(d_field);
  }
  send_control_response(msg);

  Q_EMIT (signal_undocking_start());

  delayed_tasks_.schedule(std::chrono::milliseconds(1000), [this, msg]() {

    auto running = dctrl->is_undock_running();

    DATA_CONTROL response_msg = msg;
    if(running == true)
    {
      response_msg.result = "success";
    }
    else
    {
      response_msg.result = "fail";
    }
    send_control_response(response_msg);
  });
}

void COMM_MSA::handle_dock_stop(DATA_CONTROL& msg)
{
  if(!dctrl || !ctrl)
  {
    msg.result = "reject";
    send_control_response(msg);
    return;
  }
  msg.result = "accept";
  msg.message = "";
  send_control_response(msg);


  
  log_info("Dock stop");
  int d_field = config->get_docking_field();
  if(d_field == -1)
  {
    mobile->set_detect_mode(0.0);
  }

  dctrl->stop();
  ctrl->set_is_moving(false);

  Q_EMIT (signal_docking_stop());

  double t = std::abs(config->get_robot_size_x_max() / 0.05) + 1.0;


  delayed_tasks_.schedule(std::chrono::milliseconds(static_cast<int>(t * 1000)), [this, msg]() {
    ctrl->set_is_moving(false);

    DATA_CONTROL response_msg = msg;
    auto dock_state = dctrl->get_dock_state();
    if(dock_state == false)
    {
      response_msg.result = "success";
    }
    else
    {
      response_msg.result = "fail";
    }
    send_control_response(response_msg);
  });
}

void COMM_MSA::handle_motor_control(DATA_CONTROL& msg)
{
  if(!mobile)
  {
    msg.result = "reject";
    send_control_response(msg);
    return;
  }

  msg.result = "accept";
  msg.message = "";
  send_control_response(msg);

  if(msg.onoff)
  {
    log_info("Motor On");
    mobile->motor_on();

    msg.result = "success";
    msg.message = "";
    send_control_response(msg);
  }
  else
  {
    log_info("Motor Off");
    //mobile->motor_off();

    msg.result = "reject";
    msg.message = "";
    send_control_response(msg);
  }
}

void COMM_MSA::handle_set_safety_field(DATA_CONTROL& msg)
{
  if(!mobile)
  {
    msg.result = "reject";
    send_control_response(msg);
    return;
  }
  msg.result = "accept";
  msg.message = "";
  send_control_response(msg);


  unsigned int filed = msg.safetyField.toInt();
  log_info("SetSafetyField : {}", (int)filed);
  msg.result = "success";
  msg.message = "";
  mobile->setlidarfield(filed);

  send_control_response(msg);
}

void COMM_MSA::handle_get_safety_field(DATA_CONTROL& msg)
{
  if(!mobile)
  {
    msg.result = "reject";
    send_control_response(msg);
    return;
  }
  msg.result = "accept";
  msg.message = "";
  send_control_response(msg);

  auto status = mobile->get_status();
  msg.result = "success";
  msg.message = "";
  msg.safetyField = QString::number(static_cast<int>(status.lidar_field));

  send_control_response(msg);
}

void COMM_MSA::handle_reset_safety_field(DATA_CONTROL& msg)
{
  if(!mobile)
  {
    msg.result = "reject";
    send_control_response(msg);
    return;
  }
  msg.result = "accept";
  msg.message = "";
  send_control_response(msg);


  bool allSuccess = true;
  QStringList failedFields;
  QStringList processedFields;

  log_info("Received ResetSafetyField/setSafetyFlag command");

  // 새로운 safetyFlags 배열 방식 처리
  if(!msg.resetFlags.isEmpty())
  {
    for(const auto& flagPair : msg.resetFlags)
    {
      QString name = flagPair.first;
      bool value = flagPair.second;

      // value가 false인 경우에만 리셋 수행 (flag를 클리어)
      if(!value)
      {
        if(name == "bumper")
        {
          if(mobile) mobile->clearbumperstop();
          processedFields.append(name);
        }
        else if(name == "interlock")
        {
          if(mobile) mobile->clearinterlockstop();
          processedFields.append(name);
        }
        else if(name == "obstacle")
        {
          if(mobile) mobile->clearobs();
          processedFields.append(name);
        }
        else if(name == "operationStop")
        {
          if(mobile) mobile->recover();
          processedFields.append(name);
        }
        else
        {
          allSuccess = false;
          failedFields.append(name);
        }
      }
    }
  }
  // 기존 resetField 문자열 방식 (하위 호환성)
  else if(!msg.resetField.isEmpty())
  {
    QStringList resetFieldList = msg.resetField.split(",", Qt::SkipEmptyParts);

    for(const QString& field : resetFieldList)
    {
      QString trimmedField = field.trimmed();

      if(trimmedField == "bumper")
      {
        if(mobile) mobile->clearbumperstop();
        processedFields.append(trimmedField);
      }
      else if(trimmedField == "interlock")
      {
        if(mobile) mobile->clearinterlockstop();
        processedFields.append(trimmedField);
      }
      else if(trimmedField == "obstacle")
      {
        if(mobile) mobile->clearobs();
        processedFields.append(trimmedField);
      }
      else if(trimmedField == "operationStop")
      {
        if(mobile) mobile->recover();
        processedFields.append(trimmedField);
      }
      else
      {
        allSuccess = false;
        failedFields.append(trimmedField);
      }
    }
  }

  // 결과 설정
  if(allSuccess && !processedFields.isEmpty())
  {
    msg.result = "success";
    msg.message = "";
    log_info("ResetSafetyField success: {}", processedFields.join(",").toStdString().c_str());
  }
  else if(processedFields.isEmpty() && failedFields.isEmpty())
  {
    msg.result = "reject";
    msg.message = "";
    log_error("Empty safetyFlags/resetField value");
  }
  else if(!failedFields.isEmpty())
  {
    // 일부 성공, 일부 실패한 경우
    msg.result = processedFields.isEmpty() ? "reject" : "partial";
    msg.message = QString("Invalid fields: %1").arg(failedFields.join(","));
    log_error("Invalid resetField values: %s", failedFields.join(",").toStdString().c_str());
  }

  send_control_response(msg);
}

void COMM_MSA::handle_get_safety_flag(DATA_CONTROL& msg)
{
  if(!mobile)
  {
    msg.result = "reject";
    msg.message = "mobile module not available";
    log_error("Mobile module not available for GetSafetyFlag");
    send_control_response(msg);
    return;
  }
  msg.result = "accept";
  msg.message = "";
  send_control_response(msg);

  auto ms = mobile->get_status();

  // Get safety flag states (true = triggered/active, false = normal)
  bool obstacleFlag = (ms.safety_state_obstacle_detected_1 != 0) || (ms.safety_state_obstacle_detected_2 != 0);
  bool bumperFlag = (ms.safety_state_bumper_stop_1 != 0) || (ms.safety_state_bumper_stop_2 != 0);
  bool interlockFlag = (ms.safety_state_interlock_stop_1 != 0) || (ms.safety_state_interlock_stop_2 != 0);
  bool operationStopFlag = (ms.operational_stop_state_flag_1 != 0) || (ms.operational_stop_state_flag_2 != 0);

  // Store in resetFlags for response (reusing the field for response)
  msg.resetFlags.clear();
  msg.resetFlags.append(qMakePair(QString("obstacle"), obstacleFlag));
  msg.resetFlags.append(qMakePair(QString("bumper"), bumperFlag));
  msg.resetFlags.append(qMakePair(QString("interlock"), interlockFlag));
  msg.resetFlags.append(qMakePair(QString("operationStop"), operationStopFlag));

  //spdlog::info("[MSA] GetSafetyFlag - obstacle: {}, bumper: {}, interlock: {}, operationStop: {}",
  //             obstacleFlag, bumperFlag, interlockFlag, operationStopFlag);

  msg.result = "success";
  msg.message = "";

  send_control_response(msg);
}

void COMM_MSA::handle_charge_trigger(DATA_CONTROL& msg)
{
  if(!mobile)
  {
    msg.result = "reject";
    msg.message = "mobile module not available";
    send_control_response(msg);
    return;
  }

  if(config->get_charge_type() == "XNERGY")
  {
    msg.result = "acceept";
    msg.message = "type : XNERGY";
    send_control_response(msg);

    int non_used_int = 0;
    mobile->xnergy_command(0, non_used_int);
    delayed_tasks_.schedule(std::chrono::milliseconds(3000), [this, msg]() {
      //no way to check state
      double xnergy_set_current = config->get_xnergy_set_current();
      mobile->xnergy_command(3, xnergy_set_current);

      DATA_CONTROL response_msg = msg;
      response_msg.result = "success";
      response_msg.message = "current : " + QString::number(xnergy_set_current);
      send_control_response(response_msg);
    });
  }
  else
  {
    msg.result = "reject";
    msg.message = "type : Not XNERGY";

    send_control_response(msg);
    return;
  }
}

void COMM_MSA::send_local_path()
{
  if(!is_connected || !ctrl)
  {
    return;
  }

  PATH path = ctrl->get_cur_local_path();
  if(path.pos.size() == last_sent_path.pos.size() &&
         std::equal(path.pos.begin(), path.pos.end(), last_sent_path.pos.begin(),
         [](const auto& a, const auto& b) { return a.isApprox(b, 1e-6); }))
  {
    return;
  }

  last_sent_path = path;

  auto root_obj = sio::object_message::create();
  auto arr = sio::array_message::create();
  auto to_sio_vec = [](const Eigen::Vector3d& p)
  {
    auto a = sio::array_message::create();
    for(int i=0; i<3; ++i)
    {
      a->get_vector().push_back(sio::double_message::create(p[i]));
    }
    return a;
  };

  for(size_t p = 0; p < path.pos.size(); p++)
  {
    if(p == 0 || p == path.pos.size() - 1 || p % 10 == 0)
    {
      arr->get_vector().push_back(to_sio_vec(path.pos[p]));
    }
  }
  add_to_obj(root_obj, "path", arr);
  add_to_obj(root_obj, "time", static_cast<long long>(get_time0() * 1000));
  {
    std::lock_guard<std::mutex> sock_lock(send_mtx);
    rrs_socket->socket("slamnav")->emit("localPath", root_obj);
  }
}

void COMM_MSA::send_global_path()
{
  if(!is_connected || !ctrl)
  {
    return;
  }

  PATH path = ctrl->get_cur_global_path();
  auto root_obj = sio::object_message::create();
  auto arr = sio::array_message::create();

  // Eigen::Vector3d를 sio::array_message로 변환하는 람다
  auto to_sio_vec = [](const Eigen::Vector3d& p)
  {
    auto a = sio::array_message::create();
    for(int i = 0; i < 3; ++i)
    {
      a->get_vector().push_back(sio::double_message::create(p[i]));
    }
    return a;
  };

  for(const auto& P : path.pos)
  {
    arr->get_vector().push_back(to_sio_vec(P));
  }
  add_to_obj(root_obj, "path", arr);
  add_to_obj(root_obj, "time", static_cast<long long>(get_time0() * 1000));

  {
    std::lock_guard<std::mutex> sock_lock(send_mtx);
    rrs_socket->socket("slamnav")->emit("globalPath", root_obj);
  }
}

void COMM_MSA::send_lidar_2d()
{
  if(!is_connected || !loc || !lidar_2d)
  {
    return;
  }

  std::vector<Eigen::Vector3d> pts = lidar_2d->get_cur_frm().pts;
  Eigen::Vector3d cur_xi = TF_to_se2(loc->get_cur_tf());

  if(!std::isfinite(cur_xi[0]) || !std::isfinite(cur_xi[1]) || !std::isfinite(cur_xi[2]) || pts.empty())
  {
    return;
  }

  auto root_obj = sio::object_message::create();
  add_to_obj(root_obj, "pose", create_pose_obj(cur_xi));

  // points 최소거리로 배열 생성.
  // x y r
  double max = config->get_lidar_2d_max_range();
  std::array<std::array<double, 3>, 360> bins;
  bins.fill({max, max, 0});
  for(const auto& p : pts)
  {
    double x = p.x();
    double y = p.y();

    if(!std::isfinite(x) || !std::isfinite(y))
    {
      continue;
    }

    double deg = std::atan2(y, x) * R2D;
    int idx = static_cast<int>(std::floor(deg));
    if(idx < 0)
    {
      idx += 360;
    }
    idx %= 360;

    double& bin_x = bins[idx][0];
    double& bin_y = bins[idx][1];
    double& bin_range = bins[idx][2];
    double range = x*x + y*y;

    if(bin_range == 0 ||  range < bin_range)
    {
      bin_x = x;
      bin_y = y;
      bin_range = range;
    }
  }

  // 빈 공간 채우기 및 JSON 배열 생성
  sio::array_message::ptr arr = sio::array_message::create();
  arr->get_vector().reserve(bins.size());
  for(const auto& p : bins)
  {
    sio::array_message::ptr p_arr = sio::array_message::create();
    auto& v = p_arr->get_vector();
    v.push_back(sio::double_message::create(p[0]));
    v.push_back(sio::double_message::create(p[1]));
    v.push_back(sio::double_message::create(0));
    arr->get_vector().push_back(p_arr);
  }
  add_to_obj(root_obj, "data", arr);
  add_to_obj(root_obj, "time", static_cast<long long>(get_time0() * 1000));

  {
    std::lock_guard<std::mutex> sock_lock(send_mtx);
    rrs_socket->socket("slamnav")->emit("lidarCloud", root_obj);
  }
}

void COMM_MSA::send_lidar_3d()
{
  if(!is_connected || !loc || !lidar_3d)
  {
    return;
  }

  std::vector<Eigen::Vector3d> pts = lidar_3d->get_cur_frm().pts;
  Eigen::Vector3d cur_xi = TF_to_se2(loc->get_cur_tf());

  if(!std::isfinite(cur_xi[0]) || pts.empty())
  {
    return;
  }

  sio::object_message::ptr root_obj = sio::object_message::create();
  add_to_obj(root_obj, "pose", create_pose_obj(cur_xi));

  sio::array_message::ptr arr = sio::array_message::create();
  arr->get_vector().reserve(pts.size());
  for(const auto& p : pts)
  {
    sio::array_message::ptr p_arr = sio::array_message::create();
    auto& v = p_arr->get_vector();
    v.push_back(sio::double_message::create(p.x()));
    v.push_back(sio::double_message::create(p.y()));
    v.push_back(sio::double_message::create(p.z()));
    v.push_back(sio::double_message::create(100.0)); // Intensity
    arr->get_vector().push_back(p_arr);
  }
  add_to_obj(root_obj, "data", arr);
  add_to_obj(root_obj, "time", static_cast<long long>(get_time0() * 1000));

  {
    std::lock_guard<std::mutex> sock_lock(send_mtx);
    rrs_socket->socket("slamnav")->emit("3DlidarCloud", root_obj);
  }
}

void COMM_MSA::send_mapping_cloud()
{
  if(!is_connected || !mapping || !config)
  {
    return;
  }

  if(mapping->get_is_mapping() && last_send_kfrm_idx < static_cast<int>(mapping->get_kfrm_storage_size()))
  {
    // send kfrm
    KFRAME kfrm = mapping->get_kfrm(last_send_kfrm_idx);
    const int accum_num = config->get_mapping_icp_do_accum_num();

    Eigen::Matrix3d R0 = kfrm.opt_G.block(0, 0, 3, 3);
    Eigen::Vector3d t0 = kfrm.opt_G.block(0, 3, 3, 1);

    sio::array_message::ptr json_array = sio::array_message::create();
    for(const auto& pt : kfrm.pts)
    {
      if(pt.do_cnt < accum_num)
      {
        continue;
      }

      Eigen::Vector3d P(pt.x, pt.y, pt.z);
      Eigen::Vector3d _P = R0 * P + t0;

      sio::array_message::ptr json_obj = sio::array_message::create();

      json_obj->get_vector().push_back(sio::double_message::create(_P[0]));
      json_obj->get_vector().push_back(sio::double_message::create(_P[1]));
      json_obj->get_vector().push_back(sio::double_message::create(_P[2]));
      json_obj->get_vector().push_back(sio::double_message::create(pt.r));

      json_array->get_vector().push_back(json_obj);
    }

    sio::object_message::ptr root_obj = sio::object_message::create();
    add_to_obj(root_obj, "data", json_array);
    add_to_obj(root_obj, "time", static_cast<long long>(get_time0() * 1000));

    {
      std::lock_guard<std::mutex> sock_lock(send_mtx);
      rrs_socket->socket("slamnav")->emit("mappingCloud", root_obj);
    }

    last_send_kfrm_idx++;
  }
}

void COMM_MSA::send_path_response(const DATA_PATH& msg)
{
  if(!is_connected)
  {
    return;
  }

  sio::object_message::ptr root_obj = sio::object_message::create();
  add_to_obj(root_obj, "result", msg.result);
  add_to_obj(root_obj, "message", msg.message);
  add_to_obj(root_obj, "time", msg.time);

  SOCKET_MESSAGE socket_msg;
  socket_msg.event = "pathResponse";
  socket_msg.data  = root_obj;
  send_response_queue.push(socket_msg);
}

void COMM_MSA::send_update_response(const DATA_SOFTWARE& msg)
{
  if(!is_connected)
  {
    return;
  }

  sio::object_message::ptr root_obj = sio::object_message::create();
  add_to_obj(root_obj, "id",      msg.id);
  add_to_obj(root_obj, "result",  msg.result);
  add_to_obj(root_obj, "message", msg.message);
  add_to_obj(root_obj, "branch",  msg.branch);
  add_to_obj(root_obj, "version", msg.version);
  add_to_obj(root_obj, "time",    static_cast<long long>(msg.time*1000));

  SOCKET_MESSAGE socket_msg;
  socket_msg.event = "updateResponse";
  socket_msg.data  = root_obj;
  send_response_queue.push(socket_msg);
}

void COMM_MSA::send_sensor_response(const DATA_SENSOR_INFO& msg)
{
  if(!is_connected)
  {
    return;
  }

  sio::object_message::ptr root_obj = sio::object_message::create();
  add_to_obj(root_obj, "id",      msg.id);
  add_to_obj(root_obj, "result",  msg.result);
  add_to_obj(root_obj, "message", msg.message);
  add_to_obj(root_obj, "target", msg.target);
  add_to_obj(root_obj, "time", static_cast<long long>(msg.time*1000));
  add_to_obj(root_obj, "index", create_index_obj(msg.index));

  SOCKET_MESSAGE socket_msg;
  socket_msg.event = "sensorResponse";
  socket_msg.data  = root_obj;
  {
    std::lock_guard<std::mutex> lock(send_response_mtx);
    send_response_queue.push(socket_msg);
    send_response_cv.notify_one();
  }
}

void COMM_MSA::send_dock_response(const DATA_DOCK& msg)
{
  if(!is_connected)
  {
    return;
  }

  sio::object_message::ptr root_obj = sio::object_message::create();
  add_to_obj(root_obj, "id", msg.id);
  add_to_obj(root_obj, "command", msg.command);
  add_to_obj(root_obj, "result", msg.result);
  add_to_obj(root_obj, "message", msg.message);
  add_to_obj(root_obj, "time", static_cast<long long>(msg.time*1000));

  SOCKET_MESSAGE socket_msg;
  socket_msg.event = "controlResponse";
  socket_msg.data  = root_obj;
  {
    std::lock_guard<std::mutex> lock(send_response_mtx);
    send_response_queue.push(socket_msg);
  }
}

void COMM_MSA::send_control_response(const DATA_CONTROL& msg)
{
  if(!is_connected)
  {
    return;
  }

  sio::object_message::ptr root_obj = sio::object_message::create();

  add_to_obj(root_obj, "id", msg.id);
  add_to_obj(root_obj, "time", QString::number((long long)(msg.time*1000), 10));
  add_to_obj(root_obj, "command", msg.command);
  add_to_obj(root_obj, "result", msg.result);
  add_to_obj(root_obj, "message", msg.message);

  if(msg.command == "dock" || msg.command == "undock" || msg.command == "randomSeq")
  {
  }
  else if(msg.command == "ledControl")
  {
    add_to_obj(root_obj, "onoff", msg.onoff);
    if(msg.onoff)
    {
      add_to_obj(root_obj, "color", msg.color);
    }
  }
  else if(msg.command == "lidarOnOff" || msg.command == "pathOnOff")
  {
    add_to_obj(root_obj, "onoff", msg.onoff);
    add_to_obj(root_obj, "frequency", msg.frequency);
  }
  else if(msg.command == "motorOnOff")
  {
    add_to_obj(root_obj, "onoff", msg.onoff);
  }
  else if(msg.command == "setSafetyField" || msg.command == "getSafetyField")
  {
    add_to_obj(root_obj, "safetyField", msg.safetyField);
  }
  else if(msg.command == "getSafetyFlag")
  {
    // Return safetyFlags array: [{"name": "obstacle", "value": true/false}, ...]
    sio::array_message::ptr safetyFlagsArr = sio::array_message::create();
    for(const auto& flag : msg.resetFlags)
    {
      sio::object_message::ptr flagObj = sio::object_message::create();
      add_to_obj(flagObj, "name", flag.first);
      add_to_obj(flagObj, "value", flag.second);
      safetyFlagsArr->get_vector().push_back(flagObj);
    }
    add_to_obj(root_obj, "safetyFlags", safetyFlagsArr);
  }
  else if(msg.command == "resetSafetyField")
  {
    add_to_obj(root_obj, "resetField", msg.resetField);
  }


  SOCKET_MESSAGE socket_msg;
  socket_msg.event = "controlResponse";
  socket_msg.data  = root_obj;
  {
    std::lock_guard<std::mutex> lock(send_response_mtx);
    send_response_queue.push(socket_msg);
    send_response_cv.notify_one();
  }
}

void COMM_MSA::send_safetyio_response(const DATA_SAFTYIO& msg)
{
  if(!is_connected)
  {
    return;
  }
  sio::object_message::ptr root_obj = sio::object_message::create();

  add_to_obj(root_obj, "id", msg.id);
  add_to_obj(root_obj, "command", msg.command);
  add_to_obj(root_obj, "time", QString::number(static_cast<qint64>(msg.time * 1000)));

  // MCU 2차원 배열
  sio::array_message::ptr total_arr = sio::array_message::create();
  {
    sio::array_message::ptr mcu0_arr = sio::array_message::create();
    for(int i = 0; i < 8; i++)
    {
      mcu0_arr->get_vector().push_back(sio::int_message::create(msg.mcu0_dio[i]));
    }
    total_arr->get_vector().push_back(mcu0_arr);

    sio::array_message::ptr mcu1_arr = sio::array_message::create();
    for(int i = 0; i < 8; i++)
    {
      mcu1_arr->get_vector().push_back(sio::int_message::create(msg.mcu1_dio[i]));
    }
    total_arr->get_vector().push_back(mcu1_arr);
  }
  add_to_obj(root_obj, "mcuDio", total_arr);

  //    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
  {
    std::lock_guard<std::mutex> sock_lock(send_mtx);
    rrs_socket->socket("slamnav")->emit("controlResponse", root_obj);
  }
}

void COMM_MSA::send_status_loop()
{
  double duration_time_move_status = get_time();
  double duration_time_status = get_time();
  double duration_time_mapping_cloud = get_time();
  double duration_time_rtsp_cam_rgb = get_time();

  while(is_send_status_running)
  {
    if(get_time() - duration_time_move_status >= COMM_MSA_INFO::move_status_send_time)
    {
      duration_time_move_status = get_time();
      send_move_status();
    }

    if(get_time() - duration_time_status >= COMM_MSA_INFO::status_send_time)
    {
      duration_time_status = get_time();
      send_status();
    }

    if(get_time() - duration_time_mapping_cloud >= COMM_MSA_INFO::mapping_cloud_send_time)
    {
      duration_time_mapping_cloud = get_time();
      send_mapping_cloud();
    }

    // for variable loop
    double lidar_freq = static_cast<double>(lidar_view_frequency.load());
    if(lidar_freq > 0.0)
    {
      if(get_time() - last_lidar_view_time >= (1.0 / lidar_freq))
      {
        last_lidar_view_time = get_time();
        if (config->get_use_lidar_2d())
        {
          send_lidar_2d();
        }
        else
        {
          send_lidar_3d();
        }
      }
    }

    double path_freq = static_cast<double>(path_view_frequency.load());
    if (path_freq > 0.0)
    {
      if(get_time() - last_path_view_time >= (1.0 / path_freq))
      {
        last_path_view_time = get_time();
        if(is_local_path_update2.exchange(false))
        {
          send_local_path();
        }
        if(is_global_path_update2.exchange(false))
        {
          send_global_path();
        }
      }
    }

    if(config->get_use_rtsp() && config->get_use_cam())
    {
      if(get_time() - duration_time_rtsp_cam_rgb > COMM_MSA_INFO::rtsp_cam_rgb_send_time)
      {
        std::vector<bool> rtsp_flag = cam->get_rtsp_flag();
        if(rtsp_flag.size() != 0)
        {
          for(int p = 0; p < rtsp_flag.size(); p++)
          {
            QString msg = QString("[COMM] cam%1 rtsp writer %2").arg(p)
                .arg(rtsp_flag[p] ? "open success" : "open failed");
            logger->write_log(msg);
          }
        }
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

// send functions
void COMM_MSA::send_status()
{
  if(!is_connected || !config || !mobile || !unimap || !dctrl)
  {
    return;
  }

  MOBILE_STATUS ms = mobile->get_status();

  // Adding the motor array
  int motor_cnt = (config->get_robot_wheel_type() == "DD" ? 2 : 4);
  sio::array_message::ptr motor_array = sio::array_message::create();
  for(int p = 0; p < motor_cnt; p++)
  {
    motor_array->get_vector().push_back(create_motor_obj(ms, p));
  }

  sio::object_message::ptr root_obj = sio::object_message::create();
  add_to_obj(root_obj, "motor", motor_array);
  add_to_obj(root_obj, "condition", create_localization_score_obj(loc->get_cur_ieir(), loc->get_cur_ieir()));
  add_to_obj(root_obj, "robot_state", create_robot_state_obj(ms));
  add_to_obj(root_obj, "robot_safety_io_state", create_robot_io_obj(ms));
  add_to_obj(root_obj, "power", create_power_obj(ms));
  add_to_obj(root_obj, "map", create_map_info_obj());
  add_to_obj(root_obj, "setting", create_robot_info_obj());

  add_to_obj(root_obj, "time", static_cast<long long>(get_time0() * 1000));

  {
    std::lock_guard<std::mutex> sock_lock(send_mtx);
    rrs_socket->socket("slamnav")->emit("status", root_obj);
  }
}

void COMM_MSA::send_move_response(const DATA_MOVE& msg)
{
  if (!is_connected)
  {
    return;
  }

  sio::object_message::ptr root_obj = sio::object_message::create();

  // Basic info
  add_to_obj(root_obj, "id", msg.id);
  add_to_obj(root_obj, "command", msg.command);
  add_to_obj(root_obj, "result", msg.result);
  add_to_obj(root_obj, "message", msg.message);
  add_to_obj(root_obj, "method", msg.method);
  add_to_obj(root_obj, "goalId", msg.goal_node_id);
  add_to_obj(root_obj, "goalName", msg.goal_node_name);
  add_to_obj(root_obj, "preset", msg.preset);

  // Position & Velocity
  if (std::isfinite(msg.cur_pos.x()))
  {
    add_to_obj(root_obj, "cur_x", msg.cur_pos.x());
    add_to_obj(root_obj, "cur_y", msg.cur_pos.y());
    add_to_obj(root_obj, "cur_z", msg.cur_pos.z());
  }

  add_to_obj(root_obj, "x", msg.tgt_pose_vec[0]);
  add_to_obj(root_obj, "y", msg.tgt_pose_vec[1]);
  add_to_obj(root_obj, "z", msg.tgt_pose_vec[2]);
  add_to_obj(root_obj, "rz", msg.tgt_pose_vec[3] * R2D);

  add_to_obj(root_obj, "vx", msg.jog_val[0]);
  add_to_obj(root_obj, "vy", msg.jog_val[1]);
  add_to_obj(root_obj, "wz", msg.jog_val[2]);

  // Status & Time
  add_to_obj(root_obj, "time", static_cast<long long>(msg.time * 1000));
  add_to_obj(root_obj, "bat_percent", msg.bat_percent);
  add_to_obj(root_obj, "remaining_dist", msg.remaining_dist);
  add_to_obj(root_obj, "meassured_dist", msg.meassured_dist);

  SOCKET_MESSAGE socket_msg;
  socket_msg.event = "moveResponse";
  socket_msg.data  = root_obj;

  {
    std::lock_guard<std::mutex> lock(send_response_mtx);
    send_response_queue.push(socket_msg);
    send_response_cv.notify_one();
  }

  QJsonObject root_obj_json = sio_object_to_qt_json_object(root_obj->get_map());
  QString wrapped = QString(QJsonDocument(root_obj_json).toJson(QJsonDocument::Compact));
  log_info("{}", wrapped.toStdString());
}

void COMM_MSA::send_localization_response(const DATA_LOCALIZATION& msg)
{
  if(!is_connected)
  {
    return;
  }

  sio::object_message::ptr root_obj = sio::object_message::create();

  add_to_obj(root_obj, "id", msg.id);
  add_to_obj(root_obj, "command", msg.command);
  add_to_obj(root_obj, "result", msg.result);
  add_to_obj(root_obj, "message", msg.message);
  add_to_obj(root_obj, "x", msg.tgt_pose_vec[0]);
  add_to_obj(root_obj, "y", msg.tgt_pose_vec[1]);
  add_to_obj(root_obj, "z", msg.tgt_pose_vec[2]);
  add_to_obj(root_obj, "th", msg.tgt_pose_vec[3]);
  add_to_obj(root_obj, "time", static_cast<long long>(msg.time * 1000));

  SOCKET_MESSAGE socket_msg;
  socket_msg.event = "localizationResponse";
  socket_msg.data  = root_obj;

  {
    std::lock_guard<std::mutex> lock(send_response_mtx);
    send_response_queue.push(socket_msg);
    send_response_cv.notify_one();
  }

  QJsonObject root_obj_json = sio_object_to_qt_json_object(root_obj->get_map());
  QString wrapped = QString(QJsonDocument(root_obj_json).toJson(QJsonDocument::Compact));
  log_info("{}", wrapped.toStdString());
}

void COMM_MSA::send_load_response(const DATA_LOAD& msg)
{
  if(!is_connected)
  {
    return;
  }

  sio::object_message::ptr root_obj = sio::object_message::create();

  add_to_obj(root_obj, "id", msg.id);
  add_to_obj(root_obj, "command", msg.command);
  add_to_obj(root_obj, "result", msg.result);
  add_to_obj(root_obj, "message", msg.message);
  add_to_obj(root_obj, "mapName", msg.map_name);
  add_to_obj(root_obj, "time", static_cast<long long>(msg.time * 1000));

  SOCKET_MESSAGE socket_msg;
  socket_msg.event = "loadResponse";
  socket_msg.data  = root_obj;
  {
    std::lock_guard<std::mutex> lock(send_response_mtx);
    send_response_queue.push(socket_msg);
    send_response_cv.notify_one();
  }

  QJsonObject root_obj_json = sio_object_to_qt_json_object(root_obj->get_map());
  QString wrapped = QString(QJsonDocument(root_obj_json).toJson(QJsonDocument::Compact));
  log_info("{}", wrapped.toStdString());
}

void COMM_MSA::send_mapping_response(const DATA_MAPPING& msg)
{
  if(!is_connected)
  {
    return;
  }

  sio::object_message::ptr root_obj = sio::object_message::create();

  add_to_obj(root_obj, "id", msg.id);
  add_to_obj(root_obj, "command", msg.command);
  add_to_obj(root_obj, "result", msg.result);
  add_to_obj(root_obj, "message", msg.message);
  add_to_obj(root_obj, "time", static_cast<long long>(msg.time * 1000));

  {
    std::lock_guard<std::mutex> lock(send_response_mtx);
    send_response_queue.push({"mappingResponse", root_obj});
    send_response_cv.notify_one();
  }

  QJsonObject root_obj_json = sio_object_to_qt_json_object(root_obj->get_map());
  QString wrapped = QString(QJsonDocument(root_obj_json).toJson(QJsonDocument::Compact));
  log_info("{}", wrapped.toStdString());
}

void COMM_MSA::send_response_loop()
{
  while(is_send_response_running)
  {
    std::unique_lock<std::mutex> lock(send_response_mtx);
    send_response_cv.wait(lock, [this]
    {
      return !send_response_queue.empty() || !is_send_response_running;
    });

    if(!is_send_response_running)
    {
      break;
    }

    SOCKET_MESSAGE msg = std::move(send_response_queue.front());
    send_response_queue.pop();
    lock.unlock();

    std::lock_guard<std::mutex> sock_lock(send_mtx);
    rrs_socket->socket("slamnav")->emit(msg.event.toStdString(), msg.data);
  }
}

void COMM_MSA::handle_move_jog(const DATA_MOVE& msg)
{
  double vx = msg.jog_val[0];
  double vy = msg.jog_val[1];
  double wz = msg.jog_val[2]*D2R;

  mobile->slot_jog_update(Eigen::Vector3d(vx, vy, wz));
}

void COMM_MSA::handle_move_target(DATA_MOVE &msg)
{
  if(!unimap || !loc || !obsmap || !config || !ctrl || !mobile)
  {
    msg.result = "reject";
    send_move_response(msg);
    return;
  }

  QString method = msg.method;
  if(method == "pp")
  {
    if(unimap->get_is_loaded() != MAP_LOADED)
    {
      msg.result = "reject";
      send_move_response(msg);
      return;
    }

    if(!loc->get_is_loc())
    {
      msg.result = "reject";
      send_move_response(msg);
      return;
    }

    if(config->get_use_multi())
    {
      msg.result = "reject";
      send_move_response(msg);
      return;
    }

    double x = msg.tgt_pose_vec[0]; double y = msg.tgt_pose_vec[1];
    if(x < unimap->get_map_min_x() || x > unimap->get_map_max_x() || y < unimap->get_map_min_y() || y > unimap->get_map_max_y())
    {
      msg.result = "reject";
      send_move_response(msg);
      return;
    }

    Eigen::Vector4d pose_vec = msg.tgt_pose_vec;
    Eigen::Matrix4d goal_tf = se2_to_TF(Eigen::Vector3d(pose_vec[0], pose_vec[1], pose_vec[3]*D2R));
    goal_tf(2,3) = pose_vec[2];
    if(obsmap->is_tf_collision(goal_tf))
    {
      msg.result = "reject";
      send_move_response(msg);
      return;
    }

    msg.result = "accept";
    msg.message = "";
    send_move_response(msg);

    Q_EMIT (ctrl->signal_move(msg));
  }
  else if(method == "hpp")
  {
    if(config->get_robot_type() == RobotType::MECANUM_Q150 || config->get_robot_type() == RobotType::MECANUM_VALEO || config->get_robot_type() == RobotType::SEC_CORE)
    {
      if(unimap->get_is_loaded() != MAP_LOADED)
      {
        msg.result = "reject";
        send_move_response(msg);
        return;
      }
      if(!loc->get_is_loc())
      {
        msg.result = "reject";
        send_move_response(msg);
        return;
      }

      // Resolve goal_node_id from goal_node_name if goal_node_id is empty
      if(!resolve_goal_id(msg))
      {
        msg.result = "reject";
        msg.message = "goal not found";
        send_move_response(msg);
        return;
      }

      // Get the node using resolved goal_node_id
      NODE* node = unimap->get_node_by_id(msg.goal_node_id);
      if(!node)
      {
        msg.result = "reject";
        msg.message = "node not found";
        send_move_response(msg);
        return;
      }

      // Fill goal_node_name if not set
      if(msg.goal_node_name.isEmpty())
      {
        msg.goal_node_name = node->name;
      }

      const Eigen::Matrix4d cur_tf = loc->get_cur_tf();
      const Eigen::Vector3d cur_pos = cur_tf.block(0, 3, 3, 1);
      const Eigen::Vector3d xi = TF_to_se2(node->tf);
      msg.cur_pos = cur_pos;
      msg.tgt_pose_vec[0] = xi[0];
      msg.tgt_pose_vec[1] = xi[1];
      msg.tgt_pose_vec[2] = node->tf(2, 3);
      msg.tgt_pose_vec[3] = xi[2];

      const Eigen::Matrix4d goal_tf = node->tf;
      PATH global_path = ctrl->calc_global_path(goal_tf);
      if(global_path.pos.size() < 2)
      {
        msg.remaining_time = 0.0;
      }

      msg.result = "accept";
      send_move_response(msg);

      Q_EMIT (ctrl->signal_move(msg));
    }
    else
    {
      msg.result = "reject";
      send_move_response(msg);
    }
  }
  else
  {
    msg.result = "reject";
    send_move_response(msg);
  }
}

void COMM_MSA::handle_move_goal(DATA_MOVE &msg)
{
  QString method = msg.method;
  if(method == "pp" || method == "hpp")
  {
    if(unimap->get_is_loaded() != MAP_LOADED)
    {
      msg.result = "reject";
      send_move_response(msg);
      return;
    }

    if(!loc->get_is_loc())
    {
      msg.result = "reject";
      send_move_response(msg);
      return;
    }

    // Resolve goal_node_id from goal_node_name if goal_node_id is empty
    if(!resolve_goal_id(msg))
    {
      msg.result = "reject";
      msg.message = "goal not found";
      send_move_response(msg);
      return;
    }

    // Get the node using resolved goal_node_id
    NODE* node = unimap->get_node_by_id(msg.goal_node_id);
    if(!node)
    {
      msg.result = "reject";
      msg.message = "node not found";
      send_move_response(msg);
      return;
    }

    // Fill goal_node_name if not set
    if(msg.goal_node_name.isEmpty())
    {
      msg.goal_node_name = node->name;
    }

    mobile->move(0,0,0);

    Eigen::Matrix4d cur_tf = loc->get_cur_tf();
    Eigen::Vector3d cur_pos = cur_tf.block(0,3,3,1);
    msg.cur_pos = cur_pos;

    Eigen::Vector3d xi = TF_to_se2(node->tf);
    msg.tgt_pose_vec[0] = xi[0];
    msg.tgt_pose_vec[1] = xi[1];
    msg.tgt_pose_vec[2] = node->tf(2,3);
    msg.tgt_pose_vec[3] = xi[2];

    // calc eta (estimation time arrival)
    Eigen::Matrix4d goal_tf = node->tf;
    PATH global_path = ctrl->calc_global_path(goal_tf);
    if(global_path.pos.size() < 2)
    {
      msg.result = "accept";
      msg.message = "just change goal";
      msg.remaining_time = 0.0;
      msg.remaining_dist = 0.0;
    }
    else
    {
      msg.result = "accept";
      msg.message = "";
    }

    send_move_response(msg);
    Q_EMIT (ctrl->signal_move(msg));
  }
  else if(method == "tng")
  {
    msg.result = "reject";
    send_move_response(msg);
  }
  else
  {
    msg.result = "reject";
    send_move_response(msg);
  }
}

void COMM_MSA::handle_move_pause(DATA_MOVE& msg)
{
  msg.result = "accept";
  msg.message = "";
  send_move_response(msg);

  ctrl->set_is_pause(true);

  msg.result = "success";
  msg.message = "";
  send_move_response(msg);
}

void COMM_MSA::handle_move_resume(DATA_MOVE& msg)
{
  msg.result = "accept";
  msg.message = "";
  send_move_response(msg);

  ctrl->set_is_pause(false);

  msg.result = "success";
  msg.message = "";
  send_move_response(msg);
}

void COMM_MSA::handle_move_profile(DATA_MOVE& msg)
{
  float target_linear_ = 0.f;
  float target_speed_  = 0.f;

  const QString command = msg.command;
  if(command == "xLinear")
  {
    // setting target and speed unit
    target_linear_ = static_cast<float>(msg.target);
    target_speed_  = static_cast<float>(msg.speed);

    if(fabs(target_linear_) > 10.0 || fabs(target_speed_) > 1.5)
    {
      msg.result = "reject";
      send_move_response(msg);
      return;
    }

    // first accept response----------
    msg.result  = "accept";
    msg.message = "";
    send_move_response(msg);

    Q_EMIT (signal_auto_profile_move(msg));
  }
  else if(command == "yLinear")
  {
    // setting target and speed unit
    target_linear_ = static_cast<float>(msg.target);
    target_speed_ = static_cast<float>(msg.speed);

    if(fabs(target_linear_) > 10.0 || fabs(target_speed_) > 1.5)
    {
      //exception
      msg.result = "reject";
      send_move_response(msg);
      return;
    }

    msg.result = "accept";
    msg.message = "";
    send_move_response(msg);

    Q_EMIT (signal_auto_profile_move(msg));
  }
  else if(command == "circular")
  {
    target_linear_ = static_cast<float>(msg.target * D2R);
    target_speed_ = static_cast<float>(msg.speed * D2R);

    if(fabs(target_linear_) > 360.0 || fabs(target_speed_) > 60.0)
    {
      //exception
      msg.result = "reject";
      send_move_response(msg);
      return;
    }

    // setting direction
    int direction_ = -1;
    if(msg.dir == "right")
    {
      direction_ = 0;
    }
    else if(msg.dir == "left")
    {
      direction_ = 1;
    }

    if(direction_ == -1)
    {
      //exception
      msg.result = "reject";
      send_move_response(msg);
      return;
    }

    msg.result = "accept";
    msg.message = "";
    send_move_response(msg);

    Q_EMIT (signal_auto_profile_move(msg));
  }
  else if(command == "rotate")
  {
    // setting target and speed unit
    target_linear_ = static_cast<float>(msg.target * D2R);
    target_speed_ = static_cast<float>(msg.speed * D2R);

    // speed excption
    if(fabs(target_linear_) > 360.0 || fabs(target_speed_) > 60.0)
    {
      send_move_response(msg);
      return;
    }
    else
    {
      msg.result = "accept";
      msg.message = "";
      send_move_response(msg);

      Q_EMIT (signal_auto_profile_move(msg));
    }
  }
  else if(command == "stop")
  {
    //Q_EMIT (signal_mobile_profile_move());

    msg.result = "success";
    msg.message = "";
    send_move_response(msg);
  }
}

void COMM_MSA::handle_move_stop(DATA_MOVE &msg)
{
  msg.result = "accept";
  msg.message = "";
  send_move_response(msg);

  Q_EMIT (signal_auto_move_stop());
}

bool COMM_MSA::resolve_goal_id(DATA_MOVE& msg)
{
  // If goal_node_id is already set, use it directly
  if(!msg.goal_node_id.isEmpty())
  {
    return true;
  }

  // If goal_node_id is empty but goal_node_name exists, find the ID by name
  if(!msg.goal_node_name.isEmpty())
  {
    NODE* node = unimap->get_node_by_name(msg.goal_node_name);
    if(node)
    {
      msg.goal_node_id = node->id;
      log_info("[resolve_goal_id] Found goal_node_id: {} from goal_node_name: {}",
               msg.goal_node_id.toStdString(), msg.goal_node_name.toStdString());
      return true;
    }
    else
    {
      log_warn("[resolve_goal_id] Could not find node with name: {}", msg.goal_node_name.toStdString());
    }
  }

  // Both goal_node_id and goal_node_name are empty or node not found
  return false;
}

void COMM_MSA::calc_remaining_time_distance(DATA_MOVE &msg)
{
  // time align (first align + final align)
  CTRL_PARAM params = ctrl->get_cur_ctrl_params();
  Eigen::Matrix4d cur_tf = loc->get_cur_tf();

  PATH global_path = ctrl->get_cur_global_path();

  Eigen::Vector2d dtdr = dTdR(cur_tf, global_path.pose.front());
  double time_align = (dtdr[1] / (params.LIMIT_W*D2R + 1e-06)) * 2;

  // time driving
  double time_driving = 0.0;
  double remaining_dist = 0.0;
  for(size_t p = 0; p < global_path.pos.size()-1; p++)
  {
    Eigen::Vector3d pos0 = global_path.pos[p];
    Eigen::Vector3d pos1 = global_path.pos[p+1];
    double dist = (pos1 - pos0).norm();

    double ref_v0 = global_path.ref_v[p];
    double ref_v1 = global_path.ref_v[p+1];
    double v = (ref_v0 + ref_v1)/2;

    time_driving += dist/(v+1e-06);
    remaining_dist += dist;
  }

  if(time_driving == 0.0)
  {
    time_driving = 9999.0;
  }

  msg.remaining_dist = remaining_dist;
  msg.remaining_time = time_driving + time_align;
}

void COMM_MSA::handle_path(DATA_PATH& msg)
{
  if(msg.command == "path")
  {
    QString path_str = msg.path_str;
    QStringList path_str_list = path_str.split(",");

    std::vector<QString> path;
    for(int p = 0; p < path_str_list.size(); p++)
    {
      path.push_back(path_str_list[p]);
    }

    ctrl->set_path(path, msg.preset, msg.time);
    ctrl->signal_move_multi();
  }

  send_path_response(msg);
}

void COMM_MSA::handle_vobs(DATA_VOBS& msg)
{
  std::vector<Eigen::Vector3d> vobs_r_list;
  {
    QString vobs_str = msg.vobs_robots;
    QStringList vobs_str_list = vobs_str.split("\n");
    if(vobs_str_list.size() > 0)
    {
      for(int p = 0; p < vobs_str_list.size(); p++)
      {
        QStringList vobs_str_list2 = vobs_str_list[p].split(",");
        if(vobs_str_list2.size() == 3)
        {
          Eigen::Vector3d P;
          P[0] = vobs_str_list2[0].toDouble();
          P[1] = vobs_str_list2[1].toDouble();
          P[2] = vobs_str_list2[2].toDouble();
          vobs_r_list.push_back(P);
        }
      }
    }
  }

  std::vector<Eigen::Vector3d> vobs_c_list;
  {
    QString vobs_str = msg.vobs_clousers;
    QStringList vobs_str_list = vobs_str.split(",");

    // set vobs
    for(int p = 0; p < vobs_str_list.size(); p++)
    {
      QString node_id = vobs_str_list[p];
      if(node_id != "")
      {
        if(NODE* node = unimap->get_node_by_id(node_id))
        {
          vobs_c_list.push_back(node->tf.block(0,3,3,1));
        }
      }
    }
  }

  // update vobs
  {
    obsmap->set_vobs_list_robots(vobs_r_list);
    if(msg.is_vobs_closures_change == "true")
    {
      obsmap->set_vobs_list_closures(vobs_c_list);
    }
    obsmap->update_vobs_map();
  }
}

void COMM_MSA::handle_update_software(DATA_SOFTWARE& msg)
{
  MOBILE_STATUS ms = mobile->get_status();
  if(ms.charge_state == 1 || ms.motor_stop_state == 0)
  {
    QString script_path = "/home/rainbow/rainbow-deploy-kit/slamnav2/slamnav2-update.sh";

    QStringList args;
    args << script_path;

    if(!msg.version.isEmpty())
    {
      QString version_str = "--version=" + msg.version;
      args << version_str;
    }

    if(!msg.branch.isEmpty())
    {
      QString branch_str = "--mode=" + msg.branch;
      args << branch_str;
    }

    bool ok = QProcess::startDetached("bash", args, QString());
    if(ok)
    {
      logger->write_log("[COMM_RRS] swUpdate started update script detached");

      msg.result = "true";
      msg.message = "";
      //send_update_response(msg);
    }
    else
    {
      logger->write_log("[COMM_RRS] swUpdate fail to start update script");
    }

    return;
  }
  else
  {
    if(ms.charge_state == 0)
    {
      msg.result = "false";
      msg.message = "not charging";
      //send_update_response(msg);
    }

    if(ms.motor_stop_state >= 1)
    {
      msg.result = "false";
      msg.message = "emo released";
      //send_update_response(msg);
    }
  }
}

void COMM_MSA::handle_load_map(DATA_LOAD& msg)
{
  QString map_name = msg.map_name;

  if(map_name != "")
  {
    QString load_dir = "/data/maps/" + map_name;

    if(!QDir(load_dir).exists())
    {
      msg.result = "reject";
      send_load_response(msg);
      return;
    }

    QString map_exist_msg = unimap->is_load_map_check(load_dir);
    if(map_exist_msg == "no 2d map!")
    {
      msg.result = "reject";
      send_load_response(msg);
      return;
    }
    if(config->get_use_lidar_3d() == true)
    {
      if(map_exist_msg == "no 3d map!")
      {
        msg.result = "reject";
        send_load_response(msg);
        return;
      }
    }
    msg.result = "accept";
    msg.message = "";
    send_load_response(msg);


    loc->stop();
    obsmap->clear();
    config->set_map_path(load_dir);
    unimap->load_map(load_dir);

    if(unimap->get_is_loaded() == MAP_LOADED)
    {
      msg.result = "success";
      msg.message = "";
      send_load_response(msg);
    }
    else
    {
      msg.result = "fail";
      msg.message = "";
      send_load_response(msg);
    }

    Q_EMIT (signal_ui_all_update());
  }
  else
  {
    msg.result = "reject";
    msg.message = ERROR_MANAGER::instance()->getErrorMessage(ERROR_MANAGER::MAP_LOAD_INVALID_DIR, ERROR_MANAGER::LOAD_MAP);
    ERROR_MANAGER::instance()->logError(ERROR_MANAGER::MAP_LOAD_INVALID_DIR, ERROR_MANAGER::LOAD_MAP);

    send_load_response(msg);
  }
}

void COMM_MSA::handle_load_topo(DATA_LOAD& msg)
{
  if(!unimap)
  {
    msg.result = "reject";
    msg.result = "unimap is nullptr";
    send_load_response(msg);
    return;
  }

  // // Check if map is loaded
  // if(unimap->get_is_loaded() != MAP_LOADED)
  // {
  //     msg.result = "reject";
  //     send_load_response(msg);
  //     return;
  // }

  // Get current map path
  QString map_dir = unimap->get_map_path();
  if(map_dir.isEmpty())
  {
    msg.result = "reject";
    msg.result = "cannot find map dir";
    send_load_response(msg);
    return;
  }


  // Check if topo.json exists
  QString node_path = map_dir + "/node.json";
  QFileInfo node_info(node_path);
  QString topo_path = map_dir + "/topo.json";
  QFileInfo topo_info(topo_path);
  
  if(node_info.exists() && node_info.isFile())
  {
    msg.result = "accept";
    msg.message = "";
    send_load_response(msg);

    // Load node
    log_info("Loading node from: {}", node_path.toStdString());
    bool success = unimap->load_node();

    if(success)
    {
      msg.result = "success";
      msg.message = "";
      log_info("Successfully loaded node.json");
    }
    else
    {
      msg.result = "fail";
      log_error("Failed to load node.json");
    }
    send_load_response(msg);
  }
  else if(topo_info.exists() && topo_info.isFile())
  {
    msg.result = "accept";
    msg.message = "";
    send_load_response(msg);

    // Load node
    log_info("Loading topo from: {}", topo_path.toStdString());
    bool success = unimap->load_topo();

    if(success)
    {
      msg.result = "success";
      msg.message = "";
      log_info("Successfully loaded topo.json");
    }
    else
    {
      msg.result = "fail";
      log_error("Failed to load topo.json");
    }
    send_load_response(msg);
  }
  else
  {
    msg.result = "reject";
    msg.result = QString("node/topo.json cannot find : ") + map_dir;
    send_load_response(msg);
    return;
  }
}

void COMM_MSA::handle_camera_get_info(DATA_SENSOR_INFO& msg)
{
  if(!config)
  {
    msg.result = "reject";
    msg.result = "no config thread";
    send_sensor_response(msg);
    return;
  }
  msg.result = "accept";
  msg.message = "";
  send_sensor_response(msg);

  std::vector<std::pair<int, QString>> index;

  for(int idx = 0; idx < config->get_cam_num(); ++idx)
  {
    auto serial = config->get_cam_serial_number(idx);
    index.emplace_back(idx, serial);
  }
  msg.index.swap(index);
  msg.result = "success";
  msg.message = "";
  log_info("Successfully notify cam info");

  send_sensor_response(msg);
}

void COMM_MSA::handle_camera_set_info(DATA_SENSOR_INFO& msg)
{
  if(!config)
  {
    msg.result = "reject";
    msg.result = "no config thread";
    send_sensor_response(msg);
    return;
  }

  int cam_num = config->get_cam_num();
  for(const auto& v : msg.index)
  {
    if(v.first >= cam_num)
    {
      msg.result = "reject";
      msg.message = "fault index";
      log_error("fault cam index : fail to change cam info");
      send_sensor_response(msg);
      return;
    }
  }
  msg.result = "accept";
  msg.message = "";
  send_sensor_response(msg);


  std::vector<QString> cam_serial_number;
  cam_serial_number.resize(cam_num);
  for(int i = 0; i < cam_num; ++i)
  {
    bool found = false;
    for(const auto& v : msg.index)
    {
      if(v.first == i)
      {
        cam_serial_number[i] = v.second;
        found = true;
        break;
      }
    }

    if(!found)
    {
      cam_serial_number[i] = config->get_cam_serial_number(i);
    }
  }

  if(config->set_cam_order(cam_serial_number))
  {
    msg.result = "success";
    msg.message = "";
    log_info("Successfully change cam info");
  }
  else
  {
    msg.result = "fail";
    msg.message = "fail to call camera order";
    log_error("fail to call set_cam_order");
  }

  send_sensor_response(msg);
}

void COMM_MSA::handle_lidar3d_set_on(DATA_SENSOR_INFO& msg)
{
  if(!lidar_3d->get_is_connected())
  {
    msg.result = "reject";
    msg.message = "lidar3d not connected";
    log_error("lidar3d not connected : fail to connect lidar");
    send_sensor_response(msg);
    return;
  }

  std::vector<int> indexs;
  int lidar_num = config->get_lidar_3d_num();
  for(const auto& v : msg.index)
  {
    if(v.first >= lidar_num)
    {
      msg.result = "reject";
      msg.message = "fault index";
      log_error("fault lidar3d index : fail to set on");
      send_sensor_response(msg);
      return;
    }
    else
    {
      indexs.push_back(v.first);
    }
  }
  msg.result = "accept";
  msg.message = "";
  send_sensor_response(msg);

  log_info("lidar3d set on");

  Q_EMIT (lidar_3d->signal_set_on(indexs));

  msg.result = "success";
  msg.message = "";
  send_sensor_response(msg);
}

void COMM_MSA::handle_lidar3d_set_off(DATA_SENSOR_INFO& msg)
{
  if(!lidar_3d->get_is_connected())
  {
    msg.result = "reject";
    send_sensor_response(msg);
    return;
  }

  std::vector<int> indexs;
  int lidar_num = config->get_lidar_3d_num();
  for(const auto& v : msg.index)
  {
    if(v.first >= lidar_num)
    {
      msg.result = "reject";
      msg.message = "fault index";
      log_error("fault lidar3d index : fail to set off");
      send_sensor_response(msg);
      return;
    }
    else
    {
      indexs.push_back(v.first);
    }
  }
  msg.result = "accept";
  msg.message = "";
  log_info("lidar3d set off");
  send_sensor_response(msg);

  Q_EMIT (lidar_3d->signal_set_off(indexs));

  msg.result = "success";
  msg.message = "";
  send_sensor_response(msg);
}

sio::object_message::ptr COMM_MSA::create_localization_score_obj(const Eigen::Vector2d& loc_ieir,
                                const Eigen::Vector2d& mapping_ieir)
{
  sio::object_message::ptr obj = sio::object_message::create();
  add_to_obj(obj, "inlier_error", loc_ieir[0]);
  add_to_obj(obj, "inlier_ratio", loc_ieir[1]);
  add_to_obj(obj, "mapping_error", mapping_ieir[0]);
  add_to_obj(obj, "mapping_ratio", mapping_ieir[1]);
  return obj;
}

sio::object_message::ptr COMM_MSA::create_pose_obj(const Eigen::Vector3d& cur_xi)
{
  sio::object_message::ptr obj = sio::object_message::create();
  add_to_obj(obj, "x", cur_xi[0]);
  add_to_obj(obj, "y", cur_xi[1]);
  add_to_obj(obj, "rz", cur_xi[2] * R2D);
  return obj;
}

sio::object_message::ptr COMM_MSA::create_velocity_obj(const MOBILE_POSE& mo)
{
  sio::object_message::ptr obj = sio::object_message::create();
  add_to_obj(obj, "vx", mo.vel[0]);
  add_to_obj(obj, "vy", mo.vel[1]);
  add_to_obj(obj, "wz", mo.vel[2] * R2D);
  return obj;
}

sio::object_message::ptr COMM_MSA::create_move_state_obj()
{
  sio::object_message::ptr obj = sio::object_message::create();
  add_to_obj(obj, "auto_move", ctrl->get_auto_state());
  add_to_obj(obj, "dock_move", dctrl->get_dock_state());
  add_to_obj(obj, "jog_move", "none");
  add_to_obj(obj, "obs", ctrl->get_obs_condition());
  add_to_obj(obj, "path", ctrl->get_multi_reqest_state());
  return obj;
}

sio::object_message::ptr COMM_MSA::create_motor_obj(const MOBILE_STATUS& ms, int motor_idx)
{
  sio::object_message::ptr motorObj = sio::object_message::create();

  int connection = 0, status = 0, temp = 0, current = 0;

  if(motor_idx == 0)
  {
    connection = ms.connection_m0;
    status = ms.status_m0;
    temp = ms.temp_m0;
    current = ms.cur_m0;
  }
  else if(motor_idx == 1)
  {
    connection = ms.connection_m1;
    status = ms.status_m1;
    temp = ms.temp_m1;
    current = ms.cur_m1;
  }
  else if(motor_idx == 2)
  {
    connection = ms.connection_m2;
    status = ms.status_m2;
    temp = ms.temp_m2;
    current = ms.cur_m2;
  }
  else if(motor_idx == 3)
  {
    connection = ms.connection_m3;
    status = ms.status_m3;
    temp = ms.temp_m3;
    current = ms.cur_m3;
  }
  else
  {
    return motorObj;
  }

  add_to_obj(motorObj, "connection", connection == 1);
  add_to_obj(motorObj, "status", static_cast<double>(status));
  add_to_obj(motorObj, "temp", static_cast<double>(temp));
  add_to_obj(motorObj, "current", static_cast<double>(current) / 10.0);

  return motorObj;
}

sio::object_message::ptr COMM_MSA::create_robot_state_obj(const MOBILE_STATUS& ms)
{
  sio::object_message::ptr obj = sio::object_message::create();

  // Charge state mapping
  std::string charge_str = "none";
  RobotModel model = config->get_robot_model();
  if(model == RobotModel::D400 || model == RobotModel::MECANUM)
  {
    static const std::map<int, std::string> states =
    {
      {CHARGE_STATE_IDLE, "none"}, {CHARGE_STATE_TRIG_TO_CHARGE, "ready"},
      {CHARGE_STATE_BATTERY_ON, "battery_on"}, {CHARGE_STATE_CHARGING, "charging"},
      {CHARGE_STATE_TRIG_TO_STOP_CHARGE, "finish"}, {CHARGE_STATE_FAIL, "fail"}
    };

    if(states.count(ms.charge_state))
    {
      charge_str = states.at(ms.charge_state);
    }
    add_to_obj(obj, "emo", ms.motor_stop_state == 0);
  }
  else if(model == RobotModel::S100)
  {
    charge_str = (ms.charge_state == 1) ? "charging" : "none";
    add_to_obj(obj, "emo", ms.motor_stop_state == 1);
  }

  add_to_obj(obj, "charge", charge_str);
  add_to_obj(obj, "dock", dctrl->get_dock_state());
  add_to_obj(obj, "localization", loc->get_cur_loc_state().toStdString());
  add_to_obj(obj, "power", static_cast<int>(ms.power_state == 1));
  add_to_obj(obj, "sss_recovery", ms.sss_recovery_state == 1);
  add_to_obj(obj, "sw_reset", ms.sw_reset == 1);
  add_to_obj(obj, "sw_stop", ms.sw_stop == 1);
  add_to_obj(obj, "sw_start", ms.sw_start == 1);
  add_to_obj(obj, "sf_obs_detect", ms.safety_state_obstacle_detected_1 || ms.safety_state_obstacle_detected_2);
  add_to_obj(obj, "sf_bumper_detect", ms.safety_state_bumper_stop_1 || ms.safety_state_bumper_stop_2);
  add_to_obj(obj, "sf_operational_stop", ms.operational_stop_state_flag_1 || ms.operational_stop_state_flag_2);
  return obj;
}

sio::object_message::ptr COMM_MSA::create_robot_io_obj(const MOBILE_STATUS& ms)
{
  auto toSioArray = [](const unsigned char arr[8])
  {
    sio::array_message::ptr jsonArr = sio::array_message::create();
    for(int i = 0; i < 8; ++i)
    {
      jsonArr->get_vector().push_back(sio::int_message::create(arr[i]));
    }
    return jsonArr;
  };

  sio::object_message::ptr obj = sio::object_message::create();
  add_to_obj(obj, "mcu0_dio", toSioArray(ms.mcu0_dio));
  add_to_obj(obj, "mcu1_dio", toSioArray(ms.mcu1_dio));
  add_to_obj(obj, "mcu0_din", toSioArray(ms.mcu0_din));
  add_to_obj(obj, "mcu1_din", toSioArray(ms.mcu1_din));
  return obj;
}

sio::object_message::ptr COMM_MSA::create_power_obj(const MOBILE_STATUS& ms)
{
  sio::object_message::ptr obj = sio::object_message::create();
  add_to_obj(obj, "bat_in", static_cast<double>(ms.bat_in));
  add_to_obj(obj, "bat_out", static_cast<double>(ms.bat_out));
  add_to_obj(obj, "bat_current", static_cast<double>(ms.bat_current));
  add_to_obj(obj, "total_power", static_cast<double>(ms.total_power));
  add_to_obj(obj, "power", static_cast<double>(ms.power));
  add_to_obj(obj, "bat_percent", static_cast<double>(ms.bat_percent));
  add_to_obj(obj, "tabos_voltage", static_cast<double>(ms.tabos_voltage));
  add_to_obj(obj, "tabos_current", static_cast<double>(ms.tabos_current));
  add_to_obj(obj, "tabos_soc", static_cast<double>(ms.tabos_soc));
  add_to_obj(obj, "tabos_soh", static_cast<double>(ms.tabos_soh));
  add_to_obj(obj, "tabos_temp", static_cast<double>(ms.tabos_temperature));

  double chg_cur = 0.0, con_vol = 0.0;
  if(config->get_robot_model() != RobotModel::S100)
  {
    chg_cur = ms.charge_current;
    con_vol = ms.contact_voltage;
  }
  add_to_obj(obj, "charge_current", chg_cur);
  add_to_obj(obj, "contact_voltage", con_vol);
  return obj;
}

sio::object_message::ptr COMM_MSA::create_map_info_obj()
{
  sio::object_message::ptr obj = sio::object_message::create();

  std::string map_status = "none";
  int status = unimap->get_is_loaded();
  if(status == MAP_LOADING)
  {
    map_status = "loading";
  }
  else if(status == MAP_LOADED)
  {
    map_status = "loaded";
  }

  add_to_obj(obj, "map_name", unimap->get_map_path().split("/").last().toStdString());
  add_to_obj(obj, "map_status", map_status);
  return obj;
}

sio::object_message::ptr COMM_MSA::create_robot_info_obj()
{
  sio::object_message::ptr obj = sio::object_message::create();
  add_to_obj(obj, "platform_type", config->get_robot_type_str());
  add_to_obj(obj, "platform_name", "");
  return obj;
}

sio::array_message::ptr COMM_MSA::create_index_obj(const std::vector<std::pair<int, QString>>& index)
{
  sio::array_message::ptr arr = sio::array_message::create();

  for(const auto& v : index)
  {
    sio::object_message::ptr obj = sio::object_message::create();
    add_to_obj(obj, "id", v.first);
    add_to_obj(obj, "serial", v.second);
    arr->get_vector().push_back(obj);
  }
  return arr;
}

void COMM_MSA::set_global_path_update()
{
  is_global_path_update2 = true;
}

void COMM_MSA::set_local_path_update()
{
  is_local_path_update2 = true;
}

QString COMM_MSA::get_json(const QJsonObject& json, QString key)
{
  return json[key].toString();
}

int COMM_MSA::get_json_int(const QJsonObject& json, QString key)
{
  return json[key].toInt();
}

double COMM_MSA::get_json_double(const QJsonObject& json, QString key)
{
  return json[key].toDouble();
}

bool COMM_MSA::get_json_bool(const QJsonObject& json, QString key)
{
  return json[key].toBool();
}

std::vector<std::pair<int, QString>> COMM_MSA::parse_index_json(const QJsonObject& json, QString key)
{
  std::vector<std::pair<int, QString>> index;

  QJsonArray q_array = json[key].toArray();
  for(const auto& val : q_array)
  {
    QJsonObject obj = val.toObject();

    int id = obj.value("id").toInt();
    QString serial = obj.value("serial").toString();
    index.emplace_back(id, serial);
  }
  return index;
}

double COMM_MSA::get_process_time_path()
{
  return (double)process_time_path.load();
}

double COMM_MSA::get_process_time_vobs()
{
  return (double)process_time_vobs.load();
}

double COMM_MSA::get_max_process_time_path()
{
  return (double)max_process_time_path.load();
}

double COMM_MSA::get_max_process_time_vobs()
{
  return (double)max_process_time_vobs.load();
}

Eigen::Vector4d COMM_MSA::get_last_tgt_pose_vec()
{
  std::shared_lock<std::shared_mutex> lock(mtx);
  return last_tgt_pose_vec;
}

void COMM_MSA::set_last_tgt_pose_vec(const Eigen::Vector4d& val)
{
  std::unique_lock<std::shared_mutex> lock(mtx);
  last_tgt_pose_vec = val;
}

bool COMM_MSA::get_is_connected()
{
  return is_connected.load();
}

QString COMM_MSA::get_last_receive_msg()
{
  std::shared_lock<std::shared_mutex> lock(mtx);
  return last_receive_msg;
}

void COMM_MSA::set_last_receive_msg(QString val)
{
  std::unique_lock<std::shared_mutex> lock(mtx);
  last_receive_msg = val;
}

void COMM_MSA::set_config_module(CONFIG* _config)
{
  if(!_config)
  {
    return;
  }

  config = _config;
}

void COMM_MSA::set_logger_module(LOGGER* _logger)
{
  if(!_logger)
  {
    return;
  }

  logger = _logger;
}

void COMM_MSA::set_mobile_module(MOBILE* _mobile)
{
  if(!_mobile)
  {
    return;
  }

  mobile = _mobile;

  connect(this, &COMM_MSA::signal_mobile_jog_update,        mobile, &MOBILE::slot_jog_update);
}

void COMM_MSA::set_lidar_2d_module(LIDAR_2D* _lidar)
{
  if(!_lidar)
  {
    return;
  }

  lidar_2d = _lidar;
}

void COMM_MSA::set_lidar_3d_module(LIDAR_3D* _lidar)
{
  if(!_lidar)
  {
    return;
  }

  lidar_3d = _lidar;
}

void COMM_MSA::set_cam_module(CAM* _cam)
{
  if(!_cam)
  {
    return;
  }

  cam = _cam;
}

void COMM_MSA::set_localization_module(LOCALIZATION* _loc)
{
  if(!_loc)
  {
    return;
  }

  loc = _loc;
}

void COMM_MSA::set_mapping_module(MAPPING* _mapping)
{
  if(!_mapping)
  {
    return;
  }

  mapping = _mapping;

  connect(this, &COMM_MSA::signal_map_build_start, mapping, &MAPPING::slot_map_build_start);
  connect(this, &COMM_MSA::signal_map_build_stop,  mapping, &MAPPING::slot_map_build_stop);
}

void COMM_MSA::set_unimap_module(UNIMAP* _unimap)
{
  if(!_unimap)
  {
    return;
  }

  unimap = _unimap;
}

void COMM_MSA::set_obsmap_module(OBSMAP* _obsmap)
{
  if(!_obsmap)
  {
    return;
  }

  obsmap = _obsmap;
}

void COMM_MSA::set_autocontrol_module(AUTOCONTROL* _ctrl)
{
  if(!_ctrl)
  {
    return;
  }

  ctrl = _ctrl;

  connect(this, &COMM_MSA::signal_auto_profile_move, ctrl, &AUTOCONTROL::slot_profile_move);
  connect(this, &COMM_MSA::signal_auto_move_stop,    ctrl, &AUTOCONTROL::stop);
  connect(ctrl, &AUTOCONTROL::signal_move_response,  this, &COMM_MSA::send_move_response);
}

void COMM_MSA::set_dockcontrol_module(DOCKCONTROL* _dctrl)
{
  if(!_dctrl)
  {
    return;
  }

  dctrl = _dctrl;

  connect(this, &COMM_MSA::signal_docking_start, dctrl, &DOCKCONTROL::slot_docking_start);
  connect(this, &COMM_MSA::signal_undocking_start, dctrl, &DOCKCONTROL::slot_undocking_start);
  connect(this, &COMM_MSA::signal_docking_stop, dctrl, &DOCKCONTROL::slot_docking_stop);
}

void COMM_MSA::start_all_thread()
{
  start_recv_thread();
  start_move_thread();
  start_load_thread();
  start_mapping_thread();
  start_localization_thread();
  start_control_thread();
  start_path_thread();
  start_vobs_thread();
  start_sensor_thread();
  start_setting_thread();
  start_send_status_thread();
  start_send_response_thread();
}

void COMM_MSA::start_recv_thread()
{
  log_info("start_recv_thread() called, recv_thread is null: {}", (recv_thread == nullptr));

  if(recv_thread == nullptr)
  {
    log_info("creating new recv_thread");
    is_recv_running = true;
    recv_thread = std::make_unique<std::thread>(&COMM_MSA::recv_loop, this);
    log_info("recv_thread created successfully");
  }
  else
  {
    log_warn("recv_thread already exists, skipping creation");
  }
}

void COMM_MSA::start_move_thread()
{
  if(move_thread == nullptr)
  {
    is_move_running = true;
    move_thread = std::make_unique<std::thread>(&COMM_MSA::move_loop, this);
  }
}

void COMM_MSA::start_load_thread()
{
  if(load_thread == nullptr)
  {
    is_load_running = true;
    load_thread = std::make_unique<std::thread>(&COMM_MSA::load_loop, this);
  }
}

void COMM_MSA::start_mapping_thread()
{
  if(mapping_thread == nullptr)
  {
    is_mapping_running = true;
    mapping_thread = std::make_unique<std::thread>(&COMM_MSA::mapping_loop, this);
  }
}

void COMM_MSA::start_localization_thread()
{
  if(localization_thread == nullptr)
  {
    is_localization_running = true;
    localization_thread = std::make_unique<std::thread>(&COMM_MSA::localization_loop, this);
  }
}

void COMM_MSA::start_control_thread()
{
  if(control_thread == nullptr)
  {
    is_control_running = true;
    control_thread = std::make_unique<std::thread>(&COMM_MSA::control_loop, this);
  }
}

void COMM_MSA::start_path_thread()
{
  if(path_thread == nullptr)
  {
    is_path_running = true;
    path_thread = std::make_unique<std::thread>(&COMM_MSA::path_loop, this);
  }
}

void COMM_MSA::start_vobs_thread()
{
  if(vobs_thread == nullptr)
  {
    is_vobs_running = true;
    vobs_thread = std::make_unique<std::thread>(&COMM_MSA::vobs_loop, this);
  }
}

void COMM_MSA::start_sensor_thread()
{
  if(sensor_thread == nullptr)
  {
    is_sensor_running = true;
    sensor_thread = std::make_unique<std::thread>(&COMM_MSA::sensor_loop, this);
  }
}

void COMM_MSA::start_setting_thread()
{
  if(setting_thread == nullptr)
  {
    is_setting_running = true;
    setting_thread = std::make_unique<std::thread>(&COMM_MSA::setting_loop, this);
  }
}

void COMM_MSA::start_send_status_thread()
{
  if(send_status_thread == nullptr)
  {
    is_send_status_running = true;
    send_status_thread = std::make_unique<std::thread>(&COMM_MSA::send_status_loop, this);
  }
}

void COMM_MSA::start_send_response_thread()
{
  if(send_response_thread == nullptr)
  {
    is_send_response_running = true;
    send_response_thread = std::make_unique<std::thread>(&COMM_MSA::send_response_loop, this);
  }
}

void COMM_MSA::stop_all_thread()
{
  stop_recv_thread();
  stop_move_thread();
  stop_load_thread();
  stop_mapping_thread();
  stop_localization_thread();
  stop_control_thread();
  stop_path_thread();
  stop_vobs_thread();
  stop_sensor_thread();
  stop_setting_thread();
  stop_send_status_thread();
  stop_send_response_thread();
}

void COMM_MSA::stop_recv_thread()
{
  is_recv_running = false;
  recv_cv.notify_all();  // wake up waiting thread
  if(recv_thread && recv_thread->joinable())
  {
    recv_thread->join();
  }
  recv_thread.reset();  // ✅ 추가: thread 포인터 초기화
}

void COMM_MSA::stop_move_thread()
{
  is_move_running = false;
  move_cv.notify_all();
  if(move_thread && move_thread->joinable())
  {
    move_thread->join();
  }
  move_thread.reset();
}

void COMM_MSA::stop_load_thread()
{
  is_load_running = false;
  load_cv.notify_all();
  if(load_thread && load_thread->joinable())
  {
    load_thread->join();
  }
  load_thread.reset();
}

void COMM_MSA::stop_mapping_thread()
{
  is_mapping_running = false;
  mapping_cv.notify_all();
  if(mapping_thread && mapping_thread->joinable())
  {
    mapping_thread->join();
  }
  mapping_thread.reset();
}

void COMM_MSA::stop_localization_thread()
{
  is_localization_running = false;
  localization_cv.notify_all();
  if(localization_thread && localization_thread->joinable())
  {
    localization_thread->join();
  }
  localization_thread.reset();
}

void COMM_MSA::stop_control_thread()
{
  is_control_running = false;
  control_cv.notify_all();
  if(control_thread && control_thread->joinable())
  {
    control_thread->join();
  }
  control_thread.reset();
}

void COMM_MSA::stop_path_thread()
{
  is_path_running = false;
  path_cv.notify_all();
  if(path_thread && path_thread->joinable())
  {
    path_thread->join();
  }
  path_thread.reset();
}

void COMM_MSA::stop_vobs_thread()
{
  is_vobs_running = false;
  vobs_cv.notify_all();
  if(vobs_thread && vobs_thread->joinable())
  {
    vobs_thread->join();
  }
  vobs_thread.reset();
}

void COMM_MSA::stop_sensor_thread()
{
  is_sensor_running = false;
  sensor_cv.notify_all();
  if(sensor_thread && sensor_thread->joinable())
  {
    sensor_thread->join();
  }
  sensor_thread.reset();
}

void COMM_MSA::stop_setting_thread()
{
  is_setting_running = false;
  setting_cv.notify_all();
  if(setting_thread && setting_thread->joinable())
  {
    setting_thread->join();
  }
  setting_thread.reset();
}

void COMM_MSA::stop_send_status_thread()
{
  is_send_status_running = false;
  if(send_status_thread && send_status_thread->joinable())
  {
    send_status_thread->join();
  }
  send_status_thread.reset();
}

void COMM_MSA::stop_send_response_thread()
{
  is_send_response_running = false;
  send_response_cv.notify_all();
  if(send_response_thread && send_response_thread->joinable())
  {
    send_response_thread->join();
  }
  send_response_thread.reset();
}
