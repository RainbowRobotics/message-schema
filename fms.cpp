#include "fms.h"

FMS::FMS(QObject *parent)
    : QObject{parent}
    , reconnect_timer(this)
    , send_timer(this)
{
    connect(&client, &QWebSocket::connected, this, &FMS::connected);
    connect(&client, &QWebSocket::disconnected, this, &FMS::disconnected);
    connect(&reconnect_timer, SIGNAL(timeout()), this, SLOT(reconnect_loop()));
    connect(&send_timer, SIGNAL(timeout()), this, SLOT(send_loop()));
}

FMS::~FMS()
{
    reconnect_timer.stop();

    if(is_connected)
    {
        client.close();
    }
}

void FMS::init()
{
    // set id
    QString _id;
    _id.sprintf("R_%lld", (long long)(get_time0()*1000));

    mtx.lock();
    fms_info.id = _id;
    mtx.unlock();
    printf("[FMS] ID: %s\n", _id.toLocal8Bit().data());

    reconnect_timer.start(3000);
    printf("[FMS] start reconnect timer\n", _id.toLocal8Bit().data());

    send_timer.start(100);
    printf("[FMS] start send timer\n");
}

void FMS::reconnect_loop()
{
    if(is_connected == false)
    {
        QString server_addr;
        server_addr.sprintf("ws://%s:12334", config->SERVER_IP.toLocal8Bit().data());
        client.open(QUrl(server_addr));
    }
}

void FMS::connected()
{
    if(!is_connected)
    {
        is_connected = true;
        connect(&client, &QWebSocket::textMessageReceived, this, &FMS::recv_message);

        ctrl->is_multi = true;
        printf("[FMS] connected\n");
    }
}

void FMS::disconnected()
{
    if(is_connected)
    {
        is_connected = false;
        disconnect(&client, &QWebSocket::textMessageReceived, this, &FMS::recv_message);

        ctrl->is_multi = false;
        printf("[FMS] disconnected\n");
    }
}

// send loop
void FMS::send_loop()
{
    if(!is_connected)
    {
        return;
    }

    Eigen::Matrix4d cur_tf = slam->get_cur_tf();
    Eigen::Matrix4d goal_tf = ctrl->get_cur_goal_tf();
    QString loc_state = slam->get_cur_loc_state();
    QString pdu_state = mobile->get_cur_pdu_state();

    mtx.lock();
    fms_info.cur_tf = TF_to_string(cur_tf);
    fms_info.goal_tf = TF_to_string(goal_tf);

    fms_info.loc_state = loc_state;
    fms_info.pdu_state = pdu_state;

    if(ctrl->is_moving)
    {
        fms_info.req_state = "none";
    }
    else
    {
        Eigen::Vector3d cur_pos = cur_tf.block(0,3,3,1);
        Eigen::Vector3d goal_pos = goal_tf.block(0,3,3,1);

        double err_d = calc_dist_2d(goal_pos-cur_pos);
        if(err_d > config->DRIVE_GOAL_D)
        {
            fms_info.req_state = "req";
        }
    }

    printf("[FMS] id:%s, loc:%s, pdu:%s, req:%s\n",
           fms_info.id.toLocal8Bit().data(),
           fms_info.loc_state.toLocal8Bit().data(),
           fms_info.pdu_state.toLocal8Bit().data(),
           fms_info.req_state.toLocal8Bit().data());

    mtx.unlock();


}

// recv callback
void FMS::recv_message(QString message)
{

}
