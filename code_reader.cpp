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
}

CODE_READER::~CODE_READER()
{
    client.close();
}

void CODE_READER::init()
{
    client.connectToHost(QHostAddress("192.168.2.12"), 2112);
    reconnect_timer.start(1000);

    QString ref_code_path = unimap->map_dir + "/ref_code.ini";
    QFileInfo ref_code_info(ref_code_path);
    if(ref_code_info.exists() && ref_code_info.isFile())
    {
        QSettings settings(ref_code_path, QSettings::IniFormat);
        {
            QString id = "A1X0000Y0000";
            QString str = settings.value(id).toString();
            QStringList str_list = str.split(',');
            if(str_list.size() == 2)
            {
                double x = str_list[0].toDouble();
                double y = str_list[1].toDouble();
                ref_codes[id] = cv::Vec2d(x,y);
            }
        }
        {
            QString id = "A2X0000Y0000";
            QString str = settings.value(id).toString();
            QStringList str_list = str.split(',');
            if(str_list.size() == 2)
            {
                double x = str_list[0].toDouble();
                double y = str_list[1].toDouble();
                ref_codes[id] = cv::Vec2d(x,y);
            }
        }
        {
            QString id = "A3X0000Y0000";
            QString str = settings.value(id).toString();
            QStringList str_list = str.split(',');
            if(str_list.size() == 2)
            {
                double x = str_list[0].toDouble();
                double y = str_list[1].toDouble();
                ref_codes[id] = cv::Vec2d(x,y);
            }
        }
        {
            QString id = "A4X0000Y0000";
            QString str = settings.value(id).toString();
            QStringList str_list = str.split(',');
            if(str_list.size() == 2)
            {
                double x = str_list[0].toDouble();
                double y = str_list[1].toDouble();
                ref_codes[id] = cv::Vec2d(x,y);
            }
        }
        {
            QString id = "B1X0000Y0000";
            QString str = settings.value(id).toString();
            QStringList str_list = str.split(',');
            if(str_list.size() == 2)
            {
                double x = str_list[0].toDouble();
                double y = str_list[1].toDouble();
                ref_codes[id] = cv::Vec2d(x,y);
            }
        }
        {
            QString id = "B2X0000Y0000";
            QString str = settings.value(id).toString();
            QStringList str_list = str.split(',');
            if(str_list.size() == 2)
            {
                double x = str_list[0].toDouble();
                double y = str_list[1].toDouble();
                ref_codes[id] = cv::Vec2d(x,y);
            }
        }
        {
            QString id = "B3X0000Y0000";
            QString str = settings.value(id).toString();
            QStringList str_list = str.split(',');
            if(str_list.size() == 2)
            {
                double x = str_list[0].toDouble();
                double y = str_list[1].toDouble();
                ref_codes[id] = cv::Vec2d(x,y);
            }
        }
        {
            QString id = "B4X0000Y0000";
            QString str = settings.value(id).toString();
            QStringList str_list = str.split(',');
            if(str_list.size() == 2)
            {
                double x = str_list[0].toDouble();
                double y = str_list[1].toDouble();
                ref_codes[id] = cv::Vec2d(x,y);
            }
        }
        {
            QString id = "C1X0000Y0000";
            QString str = settings.value(id).toString();
            QStringList str_list = str.split(',');
            if(str_list.size() == 2)
            {
                double x = str_list[0].toDouble();
                double y = str_list[1].toDouble();
                ref_codes[id] = cv::Vec2d(x,y);
            }
        }
        {
            QString id = "C2X0000Y0000";
            QString str = settings.value(id).toString();
            QStringList str_list = str.split(',');
            if(str_list.size() == 2)
            {
                double x = str_list[0].toDouble();
                double y = str_list[1].toDouble();
                ref_codes[id] = cv::Vec2d(x,y);
            }
        }
        {
            QString id = "C3X0000Y0000";
            QString str = settings.value(id).toString();
            QStringList str_list = str.split(',');
            if(str_list.size() == 2)
            {
                double x = str_list[0].toDouble();
                double y = str_list[1].toDouble();
                ref_codes[id] = cv::Vec2d(x,y);
            }
        }
        {
            QString id = "C4X0000Y0000";
            QString str = settings.value(id).toString();
            QStringList str_list = str.split(',');
            if(str_list.size() == 2)
            {
                double x = str_list[0].toDouble();
                double y = str_list[1].toDouble();
                ref_codes[id] = cv::Vec2d(x,y);
            }
        }
        {
            QString id = "D1X0000Y0000";
            QString str = settings.value(id).toString();
            QStringList str_list = str.split(',');
            if(str_list.size() == 2)
            {
                double x = str_list[0].toDouble();
                double y = str_list[1].toDouble();
                ref_codes[id] = cv::Vec2d(x,y);
            }
        }
        {
            QString id = "D2X0000Y0000";
            QString str = settings.value(id).toString();
            QStringList str_list = str.split(',');
            if(str_list.size() == 2)
            {
                double x = str_list[0].toDouble();
                double y = str_list[1].toDouble();
                ref_codes[id] = cv::Vec2d(x,y);
            }
        }
        {
            QString id = "D3X0000Y0000";
            QString str = settings.value(id).toString();
            QStringList str_list = str.split(',');
            if(str_list.size() == 2)
            {
                double x = str_list[0].toDouble();
                double y = str_list[1].toDouble();
                ref_codes[id] = cv::Vec2d(x,y);
            }
        }
        {
            QString id = "D4X0000Y0000";
            QString str = settings.value(id).toString();
            QStringList str_list = str.split(',');
            if(str_list.size() == 2)
            {
                double x = str_list[0].toDouble();
                double y = str_list[1].toDouble();
                ref_codes[id] = cv::Vec2d(x,y);
            }
        }

        for (const auto& k : ref_codes)
        {
            const cv::Vec2d& value = k.second;
            std::cout << "c: " << value << std::endl;
        }
    }
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
                code.x = str.toInt();
            }
            else if(p == 2)
            {
                code.y = str.toInt();
            }
            else if(p == 3)
            {
                code.z = str.toInt();
            }
            else if(p == 4)
            {
                code.deg = str.toInt();
            }
        }

        if(is_record == true)
        {
            QString code_id = code.id;
            if (record_codes.find(code_id) == record_codes.end())
            {
                record_codes[code_id] = cv::Vec3d(code.x, code.y, 1);
            }
            else
            {
                cv::Vec3d val = record_codes[code_id];
                double num = val[2] + 1;

                double x = val[0] + code.x;
                double y = val[1] + code.y;
                cv::Vec3d new_val(x,y,num);

                record_codes[code_id] = new_val;
            }

            bool _is_update_enough = true;
            for (const auto& k : record_codes)
            {
                const cv::Vec3d& value = k.second;
                if(value[2] < 20)
                {
                    _is_update_enough = false;
                }
            }
            is_update_enough = _is_update_enough;
        }

        if(unimap->is_loaded == false)
        {
            logger->PrintLog("[CODE] unimap not loaded.", "Red", true, false);
            return;
        }

        if(unimap->ref_codes.find(code.id) == unimap->ref_codes.end())
        {
            logger->PrintLog("[CODE] no exist ref code.", "Red", true, false);
            return;
        }

        else
        {
            err_x = double(code.x - ref_codes[code.id][0])/1000;
            err_y = double(code.y - ref_codes[code.id][1])/1000;
            err_th = deltaRad(90*D2R, code.deg*D2R);
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

    is_saved = false;

    QString path = QDir::homePath()+ "/ref_code.ini";;
    if(unimap->map_dir != "")
    {
        path = unimap->map_dir + "/ref_code.ini";
    }
    QSettings settings(path, QSettings::IniFormat);

    for (const auto& k : record_codes)
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

cv::Mat CODE_READER::get_plot_img()
{
    cv::Mat _plot_img;
    _plot_img = plot_img.clone();

    return _plot_img;
}
