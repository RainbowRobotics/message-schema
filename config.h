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
    // params
    double ROBOT_SIZE_X[2] = {-0.5, 0.5}; // min, max
    double ROBOT_SIZE_Y[2] = {-0.4, 0.4};
    double ROBOT_SIZE_Z[2] = {0.0, 0.3};

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
