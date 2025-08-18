#ifndef POLICY_H
#define POLICY_H

// defines
#include "global_defines.h"
#include "my_utils.h"

// module
#include "config.h"
#include "logger.h"
#include "unimap.h"
#include "localization.h"

#include <QObject>

class POLICY : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(POLICY)
public:
    // make singleton
    static POLICY* instance(QObject* parent = nullptr);

    // initialization policy module
    void init();

    // open policy module
    void open();

    // close policy module
    void close();


    /***********************
     * interface funcs
     ***********************/
    QString get_cur_zone();
    void set_cur_zone(QString str);

    /***********************
     * set other modules
     ***********************/
    void set_config_module(CONFIG* _config);
    void set_logger_module(LOGGER* _logger);
    void set_unimap_module(UNIMAP* _unimap);
    void set_localization_module(LOCALIZATION* _localization);

private:
    explicit POLICY(QObject *parent = nullptr);
    ~POLICY();

    // mutex
    std::mutex mtx;

    // other modules
    CONFIG* config;
    LOGGER* logger;
    UNIMAP* unimap;
    LOCALIZATION *loc;

    // zone loop
    std::atomic<bool> zone_flag = {false};
    std::unique_ptr<std::thread> zone_thread;
    void zone_loop();

    QString cur_zone = "";

};

#endif // POLICY_H
