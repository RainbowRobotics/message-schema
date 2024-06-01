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
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            ROBOT_SIZE_X[1] = obj["ROBOT_SIZE_MAX_X"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("ROBOT_SIZE_MAX_X", obj["ROBOT_SIZE_MAX_X"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            ROBOT_SIZE_Y[0] = obj["ROBOT_SIZE_MIN_Y"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("ROBOT_SIZE_MIN_Y", obj["ROBOT_SIZE_MIN_Y"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            ROBOT_SIZE_Y[1] = obj["ROBOT_SIZE_MAX_Y"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("ROBOT_SIZE_MAX_Y", obj["ROBOT_SIZE_MAX_Y"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            ROBOT_SIZE_Z[0] = obj["ROBOT_SIZE_MIN_Z"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("ROBOT_SIZE_MIN_Z", obj["ROBOT_SIZE_MIN_Z"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            ROBOT_SIZE_Z[1] = obj["ROBOT_SIZE_MAX_Z"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("ROBOT_SIZE_MAX_Z", obj["ROBOT_SIZE_MAX_Z"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            ROBOT_WHEEL_BASE = obj["ROBOT_WHEEL_BASE"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("ROBOT_WHEEL_BASE", obj["ROBOT_WHEEL_BASE"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            ROBOT_WHEEL_RADIUS = obj["ROBOT_WHEEL_RADIUS"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("ROBOT_WHEEL_RADIUS", obj["ROBOT_WHEEL_RADIUS"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            ROBOT_RADIUS = obj["ROBOT_RADIUS"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("ROBOT_RADIUS", obj["ROBOT_RADIUS"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());



            MOTOR_ID_L = obj["MOTOR_ID_L"].toString().toInt();
            params.push_back(std::make_pair<QString, QString>("MOTOR_ID_L", obj["MOTOR_ID_L"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            MOTOR_ID_R = obj["MOTOR_ID_R"].toString().toInt();
            params.push_back(std::make_pair<QString, QString>("MOTOR_ID_R", obj["MOTOR_ID_R"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            ROBOT_WHEEL_RADIUS = obj["ROBOT_WHEEL_RADIUS"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("ROBOT_WHEEL_RADIUS", obj["ROBOT_WHEEL_RADIUS"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            MOTOR_DIR = obj["MOTOR_DIR"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("MOTOR_DIR", obj["MOTOR_DIR"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            MOTOR_GEAR_RATIO = obj["MOTOR_GEAR_RATIO"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("MOTOR_GEAR_RATIO", obj["MOTOR_GEAR_RATIO"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            ROBOT_WHEEL_RADIUS = obj["ROBOT_WHEEL_RADIUS"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("ROBOT_WHEEL_RADIUS", obj["ROBOT_WHEEL_RADIUS"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            MOTOR_LIMIT_V = obj["MOTOR_LIMIT_V"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("MOTOR_LIMIT_V", obj["MOTOR_LIMIT_V"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            MOTOR_LIMIT_V_ACC = obj["MOTOR_LIMIT_V_ACC"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("MOTOR_LIMIT_V_ACC", obj["MOTOR_LIMIT_V_ACC"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            MOTOR_LIMIT_W = obj["MOTOR_LIMIT_W"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("MOTOR_LIMIT_W", obj["MOTOR_LIMIT_W"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            MOTOR_LIMIT_W_ACC = obj["MOTOR_LIMIT_W_ACC"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("MOTOR_LIMIT_W_ACC", obj["MOTOR_LIMIT_W_ACC"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            MOTOR_GAIN_KP = obj["MOTOR_GAIN_KP"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("MOTOR_GAIN_KP", obj["MOTOR_GAIN_KP"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            MOTOR_GAIN_KI = obj["MOTOR_GAIN_KI"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("MOTOR_GAIN_KI", obj["MOTOR_GAIN_KI"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            MOTOR_GAIN_KD = obj["MOTOR_GAIN_KD"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("MOTOR_GAIN_KD", obj["MOTOR_GAIN_KD"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());



            LIDAR_MAX_RANGE = obj["LIDAR_MAX_RANGE"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("LIDAR_MAX_RANGE", obj["LIDAR_MAX_RANGE"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            LIDAR_TF_F = obj["LIDAR_TF_F"].toString();
            params.push_back(std::make_pair<QString, QString>("LIDAR_TF_F", obj["LIDAR_TF_F"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            LIDAR_TF_B = obj["LIDAR_TF_B"].toString();
            params.push_back(std::make_pair<QString, QString>("LIDAR_TF_B", obj["LIDAR_TF_B"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());



            SLAM_WINDOW_SIZE = obj["SLAM_WINDOW_SIZE"].toString().toInt();
            params.push_back(std::make_pair<QString, QString>("SLAM_WINDOW_SIZE", obj["SLAM_WINDOW_SIZE"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            SLAM_VOXEL_SIZE = obj["SLAM_VOXEL_SIZE"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("SLAM_VOXEL_SIZE", obj["SLAM_VOXEL_SIZE"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            SLAM_ICP_COST_THRESHOLD = obj["SLAM_ICP_COST_THRESHOLD"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("SLAM_ICP_COST_THRESHOLD", obj["SLAM_ICP_COST_THRESHOLD"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            SLAM_ICP_ERROR_THRESHOLD = obj["SLAM_ICP_ERROR_THRESHOLD"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("SLAM_ICP_ERROR_THRESHOLD", obj["SLAM_ICP_ERROR_THRESHOLD"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            SLAM_ICP_MAX_FEATURE_NUM = obj["SLAM_ICP_MAX_FEATURE_NUM"].toString().toInt();
            params.push_back(std::make_pair<QString, QString>("SLAM_ICP_MAX_FEATURE_NUM", obj["SLAM_ICP_MAX_FEATURE_NUM"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            SLAM_ICP_DO_ERASE_GAP = obj["SLAM_ICP_DO_ERASE_GAP"].toString().toInt();
            params.push_back(std::make_pair<QString, QString>("SLAM_ICP_DO_ERASE_GAP", obj["SLAM_ICP_DO_ERASE_GAP"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            SLAM_ICP_DO_ACCUM_NUM = obj["SLAM_ICP_DO_ACCUM_NUM"].toString().toInt();
            params.push_back(std::make_pair<QString, QString>("SLAM_ICP_DO_ACCUM_NUM", obj["SLAM_ICP_DO_ACCUM_NUM"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            SLAM_ICP_VIEW_THRESHOLD = obj["SLAM_ICP_VIEW_THRESHOLD"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("SLAM_ICP_VIEW_THRESHOLD", obj["SLAM_ICP_VIEW_THRESHOLD"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());



            SLAM_KFRM_UPDATE_NUM = obj["SLAM_KFRM_UPDATE_NUM"].toString().toInt();
            params.push_back(std::make_pair<QString, QString>("SLAM_KFRM_UPDATE_NUM", obj["SLAM_KFRM_UPDATE_NUM"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            SLAM_KFRM_LC_TRY_DIST = obj["SLAM_KFRM_LC_TRY_DIST"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("SLAM_KFRM_LC_TRY_DIST", obj["SLAM_KFRM_LC_TRY_DIST"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            SLAM_KFRM_LC_TRY_OVERLAP = obj["SLAM_KFRM_LC_TRY_OVERLAP"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("SLAM_KFRM_LC_TRY_OVERLAP", obj["SLAM_KFRM_LC_TRY_OVERLAP"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());



            LOC_ICP_COST_THRESHOLD = obj["LOC_ICP_COST_THRESHOLD"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("LOC_ICP_COST_THRESHOLD", obj["LOC_ICP_COST_THRESHOLD"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            LOC_ICP_ERROR_THRESHOLD = obj["LOC_ICP_ERROR_THRESHOLD"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("LOC_ICP_ERROR_THRESHOLD", obj["LOC_ICP_ERROR_THRESHOLD"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            LOC_ICP_MAX_FEATURE_NUM = obj["LOC_ICP_MAX_FEATURE_NUM"].toString().toInt();
            params.push_back(std::make_pair<QString, QString>("LOC_ICP_MAX_FEATURE_NUM", obj["LOC_ICP_MAX_FEATURE_NUM"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            LOC_CHECK_DIST = obj["LOC_CHECK_DIST"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("LOC_CHECK_DIST", obj["LOC_CHECK_DIST"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            LOC_CHECK_IE = obj["LOC_CHECK_IE"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("LOC_CHECK_IE", obj["LOC_CHECK_IE"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            LOC_CHECK_IR = obj["LOC_CHECK_IR"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("LOC_CHECK_IR", obj["LOC_CHECK_IR"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            LOC_FUSION_RATIO = obj["LOC_FUSION_RATIO"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("LOC_FUSION_RATIO", obj["LOC_FUSION_RATIO"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            ANNOT_QA_STEP = obj["ANNOT_QA_STEP"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("ANNOT_QA_STEP", obj["ANNOT_QA_STEP"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());


            SIM_MODE = obj["SIM_MODE"].toString().toInt();
            params.push_back(std::make_pair<QString, QString>("SIM_MODE", obj["SIM_MODE"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());



            DRIVE_GOAL_D = obj["DRIVE_GOAL_D"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("DRIVE_GOAL_D", obj["DRIVE_GOAL_D"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            DRIVE_GOAL_TH = obj["DRIVE_GOAL_TH"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("DRIVE_GOAL_TH", obj["DRIVE_GOAL_TH"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

            DRIVE_EXTENDED_CONTROL_TIME = obj["DRIVE_EXTENDED_CONTROL_TIME"].toString().toDouble();
            params.push_back(std::make_pair<QString, QString>("DRIVE_EXTENDED_CONTROL_TIME", obj["DRIVE_EXTENDED_CONTROL_TIME"].toString()));
            printf("[CONFIG] %s, %s\n", params.back().first.toLocal8Bit().data(), params.back().second.toLocal8Bit().data());

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

