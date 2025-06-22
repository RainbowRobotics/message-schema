#ifndef CAM_H
#define CAM_H

// global defines
#include "global_defines.h"
#include "my_utils.h"

// modules
#include "config.h"
#include "logger.h"
#include "mobile.h"

#include "cam/GEMINI2E/ORBBEC.h"

#include <QObject>

class CAM : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(CAM)

public:
    // make singleton
    static CAM* instance(QObject* parent = nullptr);

    // initialization cam module
    void init();

    // open cam module as written in config.json
    void open();

    // close cam module
    void close();

    /***********************
     * interface funcs
     ***********************/
    QString get_info_text();
    QString get_cur_state();
    void set_cur_state(QString str);
    void set_sync_flag(bool flag);
    void set_is_connected(bool val);

    /***********************
     * set other modules
     ***********************/
    void set_config_module(CONFIG* _config);
    void set_logger_module(LOGGER* _logger);
    void set_mobile_module(MOBILE* _mobile);


private:
    explicit CAM(QObject *parent = nullptr);
    ~CAM();

    // mutex
    std::mutex mtx;

    // other modules
    CONFIG* config;
    LOGGER* logger;
    MOBILE* mobile;
    ORBBEC* gemini_2e;

    // merge loop
    std::atomic<bool> merge_flag = {false};
    std::unique_ptr<std::thread> merge_thread;
    void merge_loop();

    // params
    std::atomic<bool> is_connected = {false};
    std::atomic<bool> is_sync = {false};

    tbb::concurrent_queue<FRAME> merged_que;

Q_SIGNALS:

};

#endif // CAM_H
