#ifndef FMS_H
#define FMS_H

// global defines
#include "global_defines.h"
#include "utils.h"

// other modules
#include "config.h"
#include "logger.h"
#include "slam_2d.h"
#include "unimap.h"
#include "obsmap.h"
#include "autocontrol.h"

// qt
#include <QObject>

class FMS : public QObject
{
    Q_OBJECT
public:
    explicit FMS(QObject *parent = nullptr);
    ~FMS();
    std::mutex mtx;

    // other modules
    CONFIG *config = NULL;
    LOGGER *logger = NULL;
    SLAM_2D *slam = NULL;
    UNIMAP *unimap = NULL;
    OBSMAP *obsmap = NULL;
    AUTOCONTROL *ctrl = NULL;

    // vars
    QString id;

    // funcs
    void init();

Q_SIGNALS:

private Q_SLOTS:

};

#endif // FMS_H
