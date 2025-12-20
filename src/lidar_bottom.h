//
// Created by jy on 25. 12. 18..
//

#ifndef SLAMNAV2_LIDAR_BOTTOM_H
#define SLAMNAV2_LIDAR_BOTTOM_H

#include "global_defines.h"

#include "config.h"
#include "mobile.h"

#include "lidar/COIN_D4/coin_d4.h"

#include <QObject>


class LIDAR_BOTTOM : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(LIDAR_BOTTOM)

public:

    // make singleton
    static LIDAR_BOTTOM* instance(QObject* parent = nullptr);

    // initialization lidar_2d module
    void init();

    // open lidar bottom module as written in config.json
    void open();

    // close lidar bottom module
    void close();
};

#endif //SLAMNAV2_LIDAR_BOTTOM_H