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

COMM_COOP::COMM_COOP(QObject *parent) : QObject{parent}
    , main(parent)
    , config(nullptr)
    , logger(nullptr)
    , mobile(nullptr)
    , loc(nullptr)
    , unimap(nullptr)
    , obsmap(nullptr)
    , reconnect_timer(nullptr)
{
    reconnect_timer = new QTimer(this);

    connect(&client, &QWebSocket::connected, this, &COMM_COOP::connected);
    connect(&client, &QWebSocket::disconnected, this, &COMM_COOP::disconnected);
    connect(reconnect_timer, SIGNAL(timeout()), this, SLOT(reconnect_loop()));

    connect(this, SIGNAL(signal_send_info()), this, SLOT(slot_send_info()));
}

COMM_COOP::~COMM_COOP()
{
    if(reconnect_timer)
    {
        reconnect_timer->stop();
    }

    if(is_connected)
    {
        client.close();
    }
}

QString COMM_COOP::get_json(QJsonObject& json, QString key)
{
    return json[key].toString();
}

void COMM_COOP::init()
{
    if(!config->get_use_coop())
    {
        return;
    }

    // make robot id
    QString _robot_id;
    _robot_id.sprintf("R_%lld", (long long)(get_time()*1000));

    // update robot id
    robot_id = _robot_id;
    printf("[COMM_CP] ID: %s\n", robot_id.toLocal8Bit().data());

    // start reconnect loop
    reconnect_timer->start(3000);
    printf("[COMM_CP] start reconnect timer\n");
}

void COMM_COOP::reconnect_loop()
{
    if(is_connected == false)
    {
        //QString server_addr;
        //server_addr.sprintf("ws://%s:13334", config->SERVER_IP.toLocal8Bit().data());

        QString server_addr = "ws://127.0.0.1:13334";
        client.open(QUrl(server_addr));
    }
}

void COMM_COOP::connected()
{
    if(!is_connected)
    {
        connect(&client, &QWebSocket::binaryMessageReceived, this, &COMM_COOP::recv_message);
        is_connected = true;
        printf("[COMM_CP] connected\n");
    }
}

void COMM_COOP::disconnected()
{
    if(is_connected)
    {
        is_connected = false;
        disconnect(&client, &QWebSocket::binaryMessageReceived, this, &COMM_COOP::recv_message);
        printf("[COMM_CP] disconnected\n");
    }
}

// recv callback
void COMM_COOP::recv_message(const QByteArray &buf)
{
    QString message = qUncompress(buf);
    QJsonObject data = QJsonDocument::fromJson(message.toUtf8()).object();
    if(get_json(data, "rid") != robot_id)
    {
        return;
    }

    // parsing
    if(get_json(data, "type") == "control")
    {
        QString vel_str = get_json(data, "vel");
        double time = get_json(data, "t").toDouble()/1000;

        QStringList str_list = vel_str.split(",");
        if(str_list.size() == 3)
        {
            double vx = str_list[0].toDouble();
            double vy = str_list[1].toDouble();
            double wz = str_list[2].toDouble()*D2R;

            mobile->move(vx, vy, wz);
        }
    }
}

// send slots
void COMM_COOP::slot_send_info()
{
    if(!is_connected)
    {
        return;
    }

    double time = get_time();
    Eigen::Matrix4d cur_tf = loc->get_cur_tf();
    QString state = "move";

    // Creating the JSON object
    QJsonObject rootObj;

    // Adding the command and time
    rootObj["type"] = "info";
    rootObj["rid"] = robot_id;
    rootObj["ctf"] = TF_to_string(cur_tf);
    rootObj["st"] = state;
    rootObj["t"] = QString::number((long long)(time*1000), 10);

    // send
    if(time - last_send_time >= 0.05)
    {
        QJsonDocument doc(rootObj);
        QByteArray buf = qCompress(doc.toJson(QJsonDocument::Compact));
        client.sendBinaryMessage(buf);
        last_send_time = time;
    }
}

void COMM_COOP::set_config_module(CONFIG* _config)
{
    config = _config;
}

void COMM_COOP::set_logger_module(LOGGER* _logger)
{
    logger = _logger;
}

void COMM_COOP::set_mobile_module(MOBILE* _mobile)
{
    mobile = _mobile;
}

void COMM_COOP::set_unimap_module(UNIMAP* _unimap)
{
    unimap = _unimap;
}

void COMM_COOP::set_obsmap_module(OBSMAP* _obsmap)
{
    obsmap = _obsmap;
}

void COMM_COOP::set_localization_module(LOCALIZATION *_loc)
{
    loc = _loc;
}

void COMM_COOP::set_autocontrol_module(AUTOCONTROL *_ctrl)
{
    ctrl= _ctrl;
}
