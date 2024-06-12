#include "code_reader.h"

CODE_READER::CODE_READER(QObject *parent)
    : QObject{parent}
    , reconnect_timer(this)
{
    connect(&client, &QTcpSocket::connected, this, &CODE_READER::connected);
    connect(&client, &QTcpSocket::disconnected, this, &CODE_READER::disconnected);

    connect(&reconnect_timer, SIGNAL(timeout()), this, SLOT(reconnect_loop()));
}

CODE_READER::~CODE_READER()
{
    client.close();
}

void CODE_READER::init()
{
    client.connectToHost(QHostAddress("192.168.2.12"), 2112);
    reconnect_timer.start(1000);
}

void CODE_READER::readyread()
{
    QByteArray _data = client.readAll();

    if(_data.front() == 0x02 && _data.back() == 0x03)
    {
        QByteArray data = _data.mid(1, _data.size() - 2);
        QList<QByteArray> list = data.split(';');

        CODE_INFO _code_info;
        for(int p=0; p<list.size(); p++)
        {
            QString str = QString::fromUtf8(list[p]);

            // coord pos (ex:D2X0000Y0000)
            if(p == 0)
            {
                _code_info.id = str;
            }
            else if(p == 1)
            {
                _code_info.xmcl = str.toDouble();
            }
            else if(p == 2)
            {
                _code_info.ymcl = str.toDouble();
            }
            else if(p == 3)
            {
                _code_info.tilt = str.toDouble();
            }
            else if(p == 4)
            {
                _code_info.timestamp = str.toInt();
            }
            else if(p == 5)
            {
                _code_info.x = str.toInt();
            }
            else if(p == 6)
            {
                _code_info.y = str.toInt();
            }
            else if(p == 7)
            {
                _code_info.z = str.toInt();
            }
        }

        code_info = _code_info;

        if(unimap->is_loaded == false)
        {
            return;
        }

        if(unimap->ref_codes.find(code_info.id) == unimap->ref_codes.end())
        {
            return;
        }
        else
        {
            err_x = double(code_info.x - unimap->ref_codes[code_info.id][0])/1000;
            err_y = double(code_info.y - unimap->ref_codes[code_info.id][1])/1000;
            tilt = double(code_info.tilt);
        }

        if(is_record == true)
        {
            QString code_id = code_info.id;
            if (ref_info.find(code_id) == ref_info.end())
            {
                ref_info[code_id] = cv::Vec3d(code_info.x, code_info.y, 1);
            }
            else
            {
                cv::Vec3d val = ref_info[code_id];
                double num = val[2] + 1;

                double x = val[0] + code_info.x;
                double y = val[1] + code_info.y;
                cv::Vec3d new_val(x,y,num);

                ref_info[code_id] = new_val;
            }

            bool _is_update_enough = true;
            for (const auto& k : ref_info)
            {
                const cv::Vec3d& value = k.second;
                if(value[2] < 20)
                {
                    _is_update_enough = false;
                }
            }
            is_update_enough = _is_update_enough;
        }
    }
}

void CODE_READER::save_codes()
{
    if(is_update_enough != true && is_record != false)
    {
        logger->PrintLog("[CODE] not enough ref codes", "Orange", true, false);
        return;
    }

    QString path = QDir::homePath()+ "/ref_code.ini";;
    if(unimap->map_dir != "")
    {
        path = unimap->map_dir + "/ref_code.ini";
    }
    QSettings settings(path, QSettings::IniFormat);

    is_saved = false;
    for (const auto& k : ref_info)
    {
        const QString& id = k.first;
        double x = k.second[0] / k.second[2];
        double y = k.second[1] / k.second[2];

        QString str;
        str.sprintf("%f,%f", x, y);
        settings.setValue(id, str);
    }

    logger->PrintLog("[CODE] ref codes saved", "Green", true, false);
}

void CODE_READER::connected()
{
    is_connected = true;
    connect(&client, &QTcpSocket::readyRead, this, &CODE_READER::readyread);
    logger->PrintLog("[CODE] code reader connected", "Green", true, false);
}

void CODE_READER::disconnected()
{
    is_connected = false;
    disconnect(&client, &QTcpSocket::readyRead, this, &CODE_READER::readyread);
    logger->PrintLog("[CODE] code reader disconnected", "Red", true, false);
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

CODE_INFO CODE_READER::get_code_info()
{
    CODE_INFO _code_info;
    _code_info = code_info;

    return _code_info;
}

cv::Mat CODE_READER::get_plot_img()
{
    cv::Mat _plot_img;
    _plot_img = plot_img.clone();

    return _plot_img;
}
