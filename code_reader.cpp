#include "code_reader.h"

CODE_READER::CODE_READER(QObject *parent)
    : QObject{parent}
    , reconnect_timer(this)
{
    connect(&client, &QTcpSocket::connected, this, &CODE_READER::connected);
    connect(&client, &QTcpSocket::disconnected, this, &CODE_READER::disconnected);

    connect(&reconnect_timer, SIGNAL(timeout()), this, SLOT(reconnect_loop()));
    connect(&check_recv_timer, SIGNAL(timeout()), this, SLOT(check_recv_loop()));

    check_recv_timer.setInterval(100);
    check_recv_timer.start();

    load_codes();

    double th = 90*D2R;
    R(0,0) = std::cos(th);
    R(0,1) = -std::sin(th);
    R(1,0) = std::sin(th);
    R(1,1) = std::cos(th);
}

CODE_READER::~CODE_READER()
{
    client.close();
}

QString CODE_READER::get_code_info()
{
    CODE_INFO _cur_code = get_cur_code();
    QString str;
    str.sprintf("[CODE_INFO] rc:%d, err(x,y,th):%.2f,%.2f,%.2f\nid:%s, x:%d,y:%d,z:%d,\ndeg:%f", (int)is_recv_data,
                (double)err_x, (double)err_y, (double)err_th*R2D,
                _cur_code.id.toStdString().c_str(), _cur_code.x, _cur_code.y, _cur_code.z, _cur_code.th*R2D);

    return str;
}

void CODE_READER::init()
{
    client.connectToHost(QHostAddress("192.168.2.12"), 2112);
    reconnect_timer.start(1000);

}

void CODE_READER::readyread()
{
    check_recv_timer.start();
    is_recv_data = true;

    QByteArray _data = client.readAll();

    if(_data.front() == 0x02 && _data.back() == 0x03)
    {
        QByteArray data = _data.mid(1, _data.size() - 2);
        QList<QByteArray> list = data.split(';');

        CODE_INFO code;
        for(int p=0; p<list.size(); p++)
        {
            QString str = QString::fromUtf8(list[p]);

            // coord pos (ex:D2X0000Y0000)
            if(p == 0)
            {
                code.id = str;
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
        }

        //std::cout << "id:" << code.id.toStdString() << " err_x:" << code.x << " err_y:" << code.y << std::endl;

        cur_code = code;
        if(ref_codes.find(code.id) == ref_codes.end())
        {
            logger->PrintLog("[CODE] no exist ref code.", "Red", true, false);
            return;
        }
        else
        {
            double _err_x = (double(code.x) - ref_codes[code.id][0])/1000.0;
            double _err_y = (double(code.y) - ref_codes[code.id][1])/1000.0;

            cv::Vec2d _err_xy = R*(cv::Vec2d(_err_x, _err_y));
            _err_x = -1*_err_xy[0];
            _err_y = -1*_err_xy[1];

            double _err_th = -1*code.th;

            if(is_init == true)
            {
                is_init = false;
                err_x = _err_x;
                err_y = _err_y;
                err_th = _err_th;
            }
            else
            {
                err_x = 0.5*_err_x + 0.5*pre_err_x;
                err_y = 0.5*_err_y + 0.5*pre_err_y;
                err_th = 0.5*_err_th + 0.5*pre_err_th;
            }

            pre_err_x = err_x;
            pre_err_y = err_y;
            pre_err_th = err_th;
        }
    }
}

void CODE_READER::connected()
{
    is_connected = true;
    connect(&client, &QTcpSocket::readyRead, this, &CODE_READER::readyread);
    logger->PrintLog("[CODE] connected", "Green", true, false);
}

void CODE_READER::disconnected()
{
    is_connected = false;
    disconnect(&client, &QTcpSocket::readyRead, this, &CODE_READER::readyread);
    logger->PrintLog("[CODE] disconnected", "Red", true, false);
}

void CODE_READER::check_recv_loop()
{
    if(is_recv_data == true)
    {
        logger->PrintLog("[CODE] data not received", "Orange", true, false);
    }
    is_recv_data = false;
}

void CODE_READER::reconnect_loop()
{
    if(client.state() == QAbstractSocket::ConnectedState)
    {

    }
    else if(client.state() == QAbstractSocket::ConnectingState)
    {

    }
    else
    {
        if(is_connected == false)
        {
            client.connectToHost(QHostAddress("192.168.2.12"), 2112);
        }
    }
}

void CODE_READER::load_codes()
{
    {
        QString id = "A1X0000Y0000";
        ref_codes[id] = cv::Vec2d(-27.0, -27.0);
    }
    {
        QString id = "A2X0000Y0000";
        ref_codes[id] = cv::Vec2d(-27.0, -9.0);
    }
    {
        QString id = "A3X0000Y0000";
        ref_codes[id] = cv::Vec2d(-27.0, 9.0);
    }
    {
        QString id = "A4X0000Y0000";
        ref_codes[id] = cv::Vec2d(-27.0, 27.0);
    }
    {
        QString id = "B1X0000Y0000";
        ref_codes[id] = cv::Vec2d(-9.0, -27.0);
    }
    {
        QString id = "B2X0000Y0000";
        ref_codes[id] = cv::Vec2d(-9.0, -9.0);
    }
    {
        QString id = "B3X0000Y0000";
        ref_codes[id] = cv::Vec2d(-9.0, 9.0);
    }
    {
        QString id = "B4X0000Y0000";
        ref_codes[id] = cv::Vec2d(-9.0, 27.0);
    }
    {
        QString id = "C1X0000Y0000";
        ref_codes[id] = cv::Vec2d(9.0, -27.0);
    }
    {
        QString id = "C2X0000Y0000";
        ref_codes[id] = cv::Vec2d(9.0, -9.0);
    }
    {
        QString id = "C3X0000Y0000";
        ref_codes[id] = cv::Vec2d(9.0, 9.0);
    }
    {
        QString id = "C4X0000Y0000";
        ref_codes[id] = cv::Vec2d(9.0, 27.0);
    }
    {
        QString id = "D1X0000Y0000";
        ref_codes[id] = cv::Vec2d(27.0, -27.0);
    }
    {
        QString id = "D2X0000Y0000";
        ref_codes[id] = cv::Vec2d(27.0, -9.0);
    }
    {
        QString id = "D3X0000Y0000";
        ref_codes[id] = cv::Vec2d(27.0, 9.0);
    }
    {
        QString id = "D4X0000Y0000";
        ref_codes[id] = cv::Vec2d(27.0, 27.0);
    }

    for (const auto& k : ref_codes)
    {
        const cv::Vec2d& value = k.second;
        std::cout << "[CODE] " << k.first.toStdString() << ", (x,y):"<< value << std::endl;
    }
}

cv::Mat CODE_READER::get_plot_img()
{
    mtx.lock();
    cv::Mat _plot_img = plot_img.clone();
    mtx.unlock();

    return _plot_img;
}

CODE_INFO CODE_READER::get_cur_code()
{
    mtx.lock();
    CODE_INFO _code = cur_code;
    mtx.unlock();

    return _code;
}
