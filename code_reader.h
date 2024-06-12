#ifndef CODE_READER_H
#define CODE_READER_H

#include "global_defines.h"

#include "unimap.h"

#include <QObject>
#include <QTcpSocket>
#include <QByteArray>
#include <QHostAddress>
#include <QDir>
#include <QSettings>
#include <QTimer>

class CODE_READER : public QObject
{
    Q_OBJECT
public:
    explicit CODE_READER(QObject *parent = nullptr);
    ~CODE_READER();

    void init();
    void save_codes();

    CODE_INFO get_code_info();
    cv::Mat get_plot_img();

    // other modules
    CONFIG *config = NULL;
    LOGGER *logger = NULL;
    UNIMAP* unimap = NULL;

    QTcpSocket client;
    QTimer reconnect_timer;

    std::atomic<bool> is_connected = {false};
    std::atomic<bool> is_update_enough = {false};
    std::atomic<bool> is_record = {false};

    std::atomic<double> err_x = {0};
    std::atomic<double> err_y = {0};
    std::atomic<double> tilt = {0};

    cv::Mat plot_img;
    CODE_INFO code_info;

    std::map<QString, cv::Vec3d> ref_info;
    std::atomic<bool> is_saved = {true};

private Q_SLOTS:
    void connected();
    void disconnected();
    void reconnect_loop();

    void readyread();

private:
};

#endif // CODE_READER_H
