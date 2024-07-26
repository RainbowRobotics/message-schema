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

    cv::Mat get_plot_img();

    // other modules
    CONFIG *config = NULL;
    LOGGER *logger = NULL;
    UNIMAP* unimap = NULL;

    QTcpSocket client;
    QTimer reconnect_timer;
    QTimer check_recv_timer;

    std::atomic<bool> is_connected = {false};
    std::atomic<bool> is_update_enough = {false};
    std::atomic<bool> is_record = {false};
    std::atomic<bool> is_recv_data = {false};

    std::atomic<double> err_x = {0};
    std::atomic<double> err_y = {0};
    std::atomic<double> err_th = {0};

    cv::Mat plot_img;
    std::map<QString, cv::Vec3d> record_codes;
    std::map<QString, cv::Vec2d> ref_codes;
    std::atomic<bool> is_saved = {false};

private Q_SLOTS:
    void connected();
    void disconnected();
    void readyread();

    void reconnect_loop();
    void check_recv_loop();
private:
};

#endif // CODE_READER_H
