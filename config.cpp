#include "config.h"

CONFIG::CONFIG(QObject *parent)
    : QObject{parent}
{
}

void CONFIG::config_to_ui()
{
    // clear first
    ui_table->clearContents();
    ui_table->setRowCount(0);

    // fill params
    int row_cnt = 0;
    for(size_t p = 0; p < params.size(); p++)
    {
        QTableWidgetItem* item0 = new QTableWidgetItem();
        item0->setFlags(item0->flags() & ~Qt::ItemIsEditable);
        item0->setText(params[p].first);

        QTableWidgetItem* item1 = new QTableWidgetItem();
        item1->setText(params[p].second);

        ui_table->insertRow(row_cnt);
        ui_table->setItem(row_cnt, 0, item0);
        ui_table->setItem(row_cnt, 1, item1);

        row_cnt++;
    }

    ui_table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
}

void CONFIG::load()
{
    // load params
    QFileInfo config_info(config_path);
    if(config_info.exists() && config_info.isFile())
    {
        // clear first
        params.clear();

        // read
        QFile config_file(config_path);
        if(config_file.open(QIODevice::ReadOnly))
        {
            QByteArray data = config_file.readAll();
            QJsonDocument doc = QJsonDocument::fromJson(data);
            QJsonObject obj = doc.object();

            // param load
            ROBOT_SIZE_X[0] = obj["ROBOT_SIZE_MIN_X"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("ROBOT_SIZE_MIN_X", obj["ROBOT_SIZE_MIN_X"].toString()));

            ROBOT_SIZE_X[1] = obj["ROBOT_SIZE_MAX_X"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("ROBOT_SIZE_MAX_X", obj["ROBOT_SIZE_MAX_X"].toString()));

            ROBOT_SIZE_Y[0] = obj["ROBOT_SIZE_MIN_Y"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("ROBOT_SIZE_MIN_Y", obj["ROBOT_SIZE_MIN_Y"].toString()));

            ROBOT_SIZE_Y[1] = obj["ROBOT_SIZE_MAX_Y"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("ROBOT_SIZE_MAX_Y", obj["ROBOT_SIZE_MAX_Y"].toString()));

            ROBOT_SIZE_Z[0] = obj["ROBOT_SIZE_MIN_Z"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("ROBOT_SIZE_MIN_Z", obj["ROBOT_SIZE_MIN_Z"].toString()));

            ROBOT_SIZE_Z[1] = obj["ROBOT_SIZE_MAX_Z"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("ROBOT_SIZE_MAX_Z", obj["ROBOT_SIZE_MAX_Z"].toString()));

            // complete
            is_load = true;
            config_file.close();
            printf("[CONFIG] %s, load successed\n", config_path.toLocal8Bit().data());
        }
    }
    else
    {
        printf("[CONFIG] %s, load failed\n", config_path.toLocal8Bit().data());
    }
}

void CONFIG::save()
{
    // check
    if(config_path == "")
    {
        printf("[CONFIG] no save path\n");
        return;
    }

    // save params
    QFile config_file(config_path);
    if(config_file.open(QIODevice::WriteOnly))
    {
        QJsonObject obj;

        for(int i = 0; i < ui_table->rowCount(); i++)
        {
            QTableWidgetItem *item0 = ui_table->item(i, 0);
            QTableWidgetItem *item1 = ui_table->item(i, 1);

            QString name = item0->text();
            QString val = item1->text();
            obj[name] = val;
        }

        QJsonDocument doc(obj);
        config_file.write(doc.toJson());

        // complete
        config_file.close();
        printf("[CONFIG] %s, save successed\n", config_path.toLocal8Bit().data());
    }
    else
    {
        printf("[CONFIG] %s, save failed\n", config_path.toLocal8Bit().data());
    }
}

