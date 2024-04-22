#ifndef CONFIG_H
#define CONFIG_H

#include "global_defines.h"

#include <QObject>

class CONFIG : public QObject
{
    Q_OBJECT
public:
    explicit CONFIG(QObject *parent = nullptr);

public:
    // params (initial value ref from AMR200)
    double ROBOT_SIZE_X[2] = {-0.5, 0.5}; // min, max
    double ROBOT_SIZE_Y[2] = {-0.4, 0.4};
    double ROBOT_SIZE_Z[2] = {0.0, 0.3};
    double ROBOT_WHEEL_BASE = 0.467;
    double ROBOT_WHEEL_RADIUS = 0.075;

    int MOTOR_ID_L = 1;
    int MOTOR_ID_R = 0;
    double MOTOR_DIR = 1.0;
    double MOTOR_GEAR_RATIO = 11.0;
    double MOTOR_LIMIT_V = 2.0;
    double MOTOR_LIMIT_V_ACC = 1.5;
    double MOTOR_LIMIT_W = 180.0*D2R;
    double MOTOR_LIMIT_W_ACC = 360.0*D2R;
    double MOTOR_GAIN_KP=4400.0;
    double MOTOR_GAIN_KI=0.0;
    double MOTOR_GAIN_KD=100.0;


    std::vector<std::pair<QString, QString>> params;

public:
    // interface
    void load();
    void save();

    void config_to_ui();

    QString config_path = "";
    std::atomic<bool> is_load = {false};
    QTableWidget *ui_table = NULL;

Q_SIGNALS:

};

#endif // CONFIG_H
