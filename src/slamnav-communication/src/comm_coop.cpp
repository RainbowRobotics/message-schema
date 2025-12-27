#include "comm_coop.h"
#include "mainwindow.h"

COMM_COOP* COMM_COOP::instance(QObject* parent)
{
    static COMM_COOP* inst = nullptr;
    if(!inst && parent)
    {
        inst = new COMM_COOP(parent);
    }
    else if(inst && parent && inst->parent() == nullptr)
    {
        inst->setParent(parent);
    }
    return inst;
}

COMM_COOP::COMM_COOP(QObject *parent) 
    : QObject{parent}
    , main(parent)
    , reconnect_timer(std::make_unique<QTimer>(this))
{
    // Connect WebSocket signals
    connect(&client, &QWebSocket::connected, this, &COMM_COOP::connected);
    connect(&client, &QWebSocket::disconnected, this, &COMM_COOP::disconnected);
    
    // Connect timer signal
    connect(reconnect_timer.get(), &QTimer::timeout, this, &COMM_COOP::reconnect_loop);
    
    // Connect internal signals
    connect(this, &COMM_COOP::signal_send_info, this, &COMM_COOP::slot_send_info);
}

COMM_COOP::~COMM_COOP()
{
    if(reconnect_timer)
    {
        reconnect_timer->stop();
    }

    if(is_connected_flag)
    {
        client.close();
    }
}

QString COMM_COOP::get_json(const QJsonObject& json, const QString& key) const
{
    return json[key].toString();
}

QString COMM_COOP::get_robot_id() const
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return robot_id;
}

bool COMM_COOP::is_connected() const
{
    return is_connected_flag;
}

void COMM_COOP::init()
{
    if(!config)
    {
        printf("[COMM_COOP] Warning: config module not set\n");
        return;
    }

    if(!config->get_use_coop())
    {
        return;
    }

    // Generate robot ID with timestamp
    const double current_time = get_time();
    robot_id = QString("R_%1").arg(static_cast<long long>(current_time * 1000));

    // Log robot ID
    printf("[COMM_COOP] ID: %s\n", robot_id.toLocal8Bit().constData());

    // Start reconnect timer
    reconnect_timer->start(3000);
    printf("[COMM_COOP] start reconnect timer\n");
}

void COMM_COOP::reconnect_loop()
{
    if(!is_connected_flag)
    {
        const QString server_addr = "ws://127.0.0.1:13334";
        client.open(QUrl(server_addr));
    }
}

void COMM_COOP::connected()
{
    if(!is_connected_flag)
    {
        // Connect binary message handler
        connect(&client, &QWebSocket::binaryMessageReceived, this, &COMM_COOP::recv_message);
        is_connected_flag = true;
        printf("[COMM_COOP] connected\n");
    }
}

void COMM_COOP::disconnected()
{
    if(is_connected_flag)
    {
        is_connected_flag = false;
        disconnect(&client, &QWebSocket::binaryMessageReceived, this, &COMM_COOP::recv_message);
        printf("[COMM_COOP] disconnected\n");
    }
}

void COMM_COOP::recv_message(const QByteArray &buf)
{
    // Decompress and parse message
    const QString message = qUncompress(buf);
    const QJsonDocument doc = QJsonDocument::fromJson(message.toUtf8());
    
    if(doc.isNull())
    {
        printf("[COMM_COOP] Warning: Invalid JSON received\n");
        return;
    }

    const QJsonObject data = doc.object();
    
    // Check if message is for this robot
    if(get_json(data, "rid") != robot_id)
    {
        return;
    }

    // Parse message based on type
    const QString message_type = get_json(data, "type");
    if(message_type == "control")
    {
        parse_control_message(data);
    }
}

void COMM_COOP::parse_control_message(const QJsonObject& data)
{
    const QString vel_str = get_json(data, "vel");
    const double timestamp = get_json(data, "t").toDouble() / 1000.0;
    
    process_velocity_command(vel_str, timestamp);
}

void COMM_COOP::process_velocity_command(const QString& vel_str, double timestamp)
{
    if(!mobile)
    {
        printf("[COMM_COOP] Warning: mobile module not set\n");
        return;
    }

    const QStringList str_list = vel_str.split(",", Qt::SkipEmptyParts);
    if(str_list.size() == 3)
    {
        const double vx = str_list[0].toDouble();
        const double vy = str_list[1].toDouble();
        const double wz = str_list[2].toDouble() * D2R;

        mobile->move(vx, vy, wz);
        
        if(logger)
        {
            logger->write_log(QString("[COMM_COOP] Received velocity command: vx=%1, vy=%2, wz=%3")
                             .arg(vx, 0, 'f', 3)
                             .arg(vy, 0, 'f', 3)
                             .arg(wz * R2D, 0, 'f', 3), "Green");
        }
    }
    else
    {
        printf("[COMM_COOP] Warning: Invalid velocity string format: %s\n", 
               vel_str.toLocal8Bit().constData());
    }
}

void COMM_COOP::slot_send_info()
{
    if(!is_connected_flag)
    {
        return;
    }

    send_robot_info();
}

void COMM_COOP::send_robot_info()
{
    if(!loc)
    {
        printf("[COMM_COOP] Warning: localization module not set\n");
        return;
    }

    const double current_time = get_time();
    
    // Check if we should send message (rate limiting)
    if(!should_send_message(current_time))
    {
        return;
    }

    // Get current robot state
    const Eigen::Matrix4d cur_tf = loc->get_cur_tf();
    const QString state = "move";

    // Create JSON object
    QJsonObject rootObj;
    rootObj["type"] = "info";
    rootObj["rid"] = robot_id;
    rootObj["ctf"] = TF_to_string(cur_tf);
    rootObj["st"] = state;
    rootObj["t"] = QString::number(static_cast<long long>(current_time * 1000));

    // Send compressed message
    const QJsonDocument doc(rootObj);
    const QByteArray buf = qCompress(doc.toJson(QJsonDocument::Compact));
    client.sendBinaryMessage(buf);
    
    // Update last send time
    last_send_time = current_time;
}

bool COMM_COOP::should_send_message(double current_time) const
{
    const double time_diff = current_time - last_send_time;
    return time_diff >= 0.05; // 50ms rate limiting
}

// Setter functions with null pointer checks
void COMM_COOP::set_config_module(CONFIG* _config)
{
    if(_config)
    {
        config = _config;
    }
}

void COMM_COOP::set_logger_module(LOGGER* _logger)
{
    if(_logger)
    {
        logger = _logger;
    }
}

void COMM_COOP::set_mobile_module(MOBILE* _mobile)
{
    if(_mobile)
    {
        mobile = _mobile;
    }
}

void COMM_COOP::set_unimap_module(UNIMAP* _unimap)
{
    if(_unimap)
    {
        unimap = _unimap;
    }
}

void COMM_COOP::set_obsmap_module(OBSMAP* _obsmap)
{
    if(_obsmap)
    {
        obsmap = _obsmap;
    }
}

void COMM_COOP::set_localization_module(LOCALIZATION* _loc)
{
    if(_loc)
    {
        loc = _loc;
    }
}

void COMM_COOP::set_autocontrol_module(AUTOCONTROL* _ctrl)
{
    if(_ctrl)
    {
        ctrl = _ctrl;
    }
}
