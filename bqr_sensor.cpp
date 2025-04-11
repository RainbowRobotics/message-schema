#include "bqr_sensor.h"

BQR_SENSOR::BQR_SENSOR(QObject *parent)
    : QObject{parent}
    , reconnect_timer(this)
{
    connect(&socket_bqr, &QTcpSocket::connected, this, &BQR_SENSOR::connected);
    connect(&socket_bqr, &QTcpSocket::disconnected, this, &BQR_SENSOR::disconnected);

    connect(&reconnect_timer, SIGNAL(timeout()), this, SLOT(reconnect_loop()));
    connect(&check_recv_timer, SIGNAL(timeout()), this, SLOT(check_recv_loop()));

    check_recv_timer.setInterval(300);
    check_recv_timer.start();
}

BQR_SENSOR::~BQR_SENSOR()
{
    socket_bqr.disconnectFromHost();
    socket_bqr.close();
}

QString BQR_SENSOR::get_bqr_info_str()
{
    BQR_INFO _cur_code = get_cur_bqr();
    QString str;
    str.sprintf("[BQR]\nconnection:%d,recv:%d,id:%s,num:%d\nerr(x,y,th):%.3f,%.3f,%.3f,(xymcl):%.3f,%.3f",
                (int)is_connected, (int)is_recv_data, _cur_code.id.toStdString().c_str(), _cur_code.code_num,
                (double)err_x, (double)err_y, (double)err_th*R2D,
                _cur_code.xmcl, _cur_code.ymcl);

    return str;
}

void BQR_SENSOR::init()
{
    load_codes();
    socket_bqr.connectToHost(QHostAddress("192.168.2.12"), 2112);
    reconnect_timer.start(1000);
}

void BQR_SENSOR::readyread()
{
    QByteArray _data = socket_bqr.readAll();
    QString data_str = QString(_data.data());
    if(data_str.isEmpty() || data_str.isNull())
    {
        printf("[BQR] empty data\n");
        return;
    }

    check_recv_timer.start();
    is_recv_data = true;

    for (int p = 0; p < data_str.length(); p++)
    {
        if(data_str[0] == (char)0x02 && data_str[p] == (char)0x03)
        {
            process_data(recv_buf);
            recv_buf = "";
        }
        else
        {
            recv_buf.append(data_str[p]);
        }
    }
}

void BQR_SENSOR::process_data(QString data)
{
    data.remove(0,1);
    data.remove(data.length()-1, 1);
    QStringList data_list = data.split(';');

    BQR_INFO code;
    for(int p = 0; p < data_list.size(); p++)
    {
        QString str = data_list[p];
        if(p == 0)
        {
            // coord pos (ex:D2X0000Y0000)
            if(str.size() == 12)
            {
                code.id = str.left(2);

                QString product_code_str(str.at(5));
                QString tens_str(str.at(6));
                QString units_str(str.at(11));

                int product_code = product_code_str.toInt();
                if(product_code == 0)
                {
                    // this code is used at kai
                }
                else if(product_code == 1)
                {
                    // this code is used at pcmm
                }

                int tens = tens_str.toInt();
                if((tens < 0) || (tens > 9))
                {
                    tens = 0;
                }

                int units = units_str.toInt();
                if((units < 0) || (units > 9))
                {
                    units = -1;
                }

                code.code_num = tens * 10 + units;
            }
            else
            {
                code.id = "";
            }
        }
        else if(p == 1)
        {
            code.t = str.toInt();
        }
        else if(p == 2)
        {
            code.x = str.toInt();
        }
        else if(p == 3)
        {
            code.y = str.toInt();
        }
        else if(p == 4)
        {
            code.z = str.toInt();
        }
        else if(p == 5)
        {
            code.th = toWrap(str.toDouble()*D2R);
        }
        else if(p == 6)
        {
            code.xmcl = str.toDouble()/1000.0;
        }
        else if(p == 7)
        {
            code.ymcl = str.toDouble()/1000.0;
        }
    }

    if(ref_bqr_codes.find(code.id) == ref_bqr_codes.end())
    {
        QString str;
        str.sprintf("[BQR] no exist ref code.id: %s", code.id.toStdString().c_str());
        logger->write_log(str, "Red", true, false);
        return;
    }

    code_num = code.code_num;

    double _err_x = -code.xmcl;
    double _err_y = -code.ymcl;
    double _err_th = -code.th;

    err_x = _err_x + (double)offset_x;
    err_y = _err_y + (double)offset_y;
    err_th = _err_th + (double)offset_th;

    storage_err_x.push_back((double)err_x);
    if(storage_err_x.size() > storage_size)
    {
        storage_err_x.erase(storage_err_x.begin(), storage_err_x.begin()+(storage_err_x.size()-storage_size));
    }

    storage_err_y.push_back((double)err_y);
    if(storage_err_y.size() > storage_size)
    {
        storage_err_y.erase(storage_err_y.begin(), storage_err_y.begin()+(storage_err_y.size()-storage_size));
    }

    storage_err_th.push_back((double)err_th);
    if(storage_err_th.size() > storage_size)
    {
        storage_err_th.erase(storage_err_th.begin(), storage_err_th.begin()+(storage_err_th.size()-storage_size));
    }

    if(storage_err_x.size() > 0)
    {
        double _cum_err_x = 0;
        for(size_t p=0; p<storage_err_x.size(); p++)
        {
            _cum_err_x += storage_err_x[p];
        }
        _cum_err_x /= storage_err_x.size();
        cum_err_x = _cum_err_x;
    }

    if(storage_err_y.size() > 0)
    {
        double _cum_err_y = 0;
        for(size_t p=0; p<storage_err_y.size(); p++)
        {
            _cum_err_y += storage_err_y[p];
        }
        _cum_err_y /= storage_err_y.size();
        cum_err_y = _cum_err_y;
    }

    if(storage_err_th.size() > 0)
    {
        double _cum_err_th = 0;
        for(size_t p=0; p<storage_err_th.size(); p++)
        {
            _cum_err_th += storage_err_th[p];
        }
        _cum_err_th /= storage_err_th.size();
        cum_err_th = _cum_err_th;
    }

    // update result
    mtx.lock();
    cur_bqr = code;
    mtx.unlock();
}

void BQR_SENSOR::connected()
{
    is_connected = true;
    connect(&socket_bqr, &QTcpSocket::readyRead, this, &BQR_SENSOR::readyread);
    logger->write_log("[BQR] connected", "Green", true, false);
}

void BQR_SENSOR::disconnected()
{
    is_connected = false;
    QTcpSocket *socket = static_cast<QTcpSocket*>(sender());
    socket->deleteLater();

    logger->write_log("[BQR] disconnected", "Red", true, false);
}

void BQR_SENSOR::check_recv_loop()
{
    if(is_recv_data == true)
    {
        logger->write_log("[BQR] data not received", "Orange", true, false);

        BQR_INFO code;
        mtx.lock();
        cur_bqr = code;
        mtx.unlock();

        offset_x = 0.0;
        offset_y = 0.0;
        offset_th = 0.0;

        is_recv_data = false;
    }
}

void BQR_SENSOR::reconnect_loop()
{
    if(socket_bqr.state() == QAbstractSocket::ConnectedState)
    {

    }
    else if(socket_bqr.state() == QAbstractSocket::ConnectingState)
    {

    }
    else
    {
        if(is_connected == false)
        {
            socket_bqr.connectToHost(QHostAddress("192.168.2.12"), 2112);
        }
    }
}

void BQR_SENSOR::load_codes()
{
    ref_bqr_codes["A1"] = cv::Vec2d(config->CODE_A1_X, config->CODE_A1_Y);
    ref_bqr_codes["A2"] = cv::Vec2d(config->CODE_A2_X, config->CODE_A2_Y);
    ref_bqr_codes["A3"] = cv::Vec2d(config->CODE_A3_X, config->CODE_A3_Y);
    ref_bqr_codes["A4"] = cv::Vec2d(config->CODE_A4_X, config->CODE_A4_Y);

    ref_bqr_codes["B1"] = cv::Vec2d(config->CODE_B1_X, config->CODE_B1_Y);
    ref_bqr_codes["B2"] = cv::Vec2d(config->CODE_B2_X, config->CODE_B2_Y);
    ref_bqr_codes["B3"] = cv::Vec2d(config->CODE_B3_X, config->CODE_B3_Y);
    ref_bqr_codes["B4"] = cv::Vec2d(config->CODE_B4_X, config->CODE_B4_Y);

    ref_bqr_codes["C1"] = cv::Vec2d(config->CODE_C1_X, config->CODE_C1_Y);
    ref_bqr_codes["C2"] = cv::Vec2d(config->CODE_C2_X, config->CODE_C2_Y);
    ref_bqr_codes["C3"] = cv::Vec2d(config->CODE_C3_X, config->CODE_C3_Y);
    ref_bqr_codes["C4"] = cv::Vec2d(config->CODE_C4_X, config->CODE_C4_Y);

    ref_bqr_codes["D1"] = cv::Vec2d(config->CODE_D1_X, config->CODE_D1_Y);
    ref_bqr_codes["D2"] = cv::Vec2d(config->CODE_D2_X, config->CODE_D2_Y);
    ref_bqr_codes["D3"] = cv::Vec2d(config->CODE_D3_X, config->CODE_D3_Y);
    ref_bqr_codes["D4"] = cv::Vec2d(config->CODE_D4_X, config->CODE_D4_Y);
}

cv::Mat BQR_SENSOR::get_plot_img()
{
    mtx.lock();
    cv::Mat _plot_img = plot_img.clone();
    mtx.unlock();

    return _plot_img;
}

BQR_INFO BQR_SENSOR::get_cur_bqr()
{
    mtx.lock();
    BQR_INFO _code = cur_bqr;
    mtx.unlock();

    return _code;
}

TIME_POSE_ID BQR_SENSOR::get_cur_tpi()
{
    mtx.lock();
    TIME_POSE_ID _tpi = cur_tpi;
    mtx.unlock();

    return _tpi;
}
