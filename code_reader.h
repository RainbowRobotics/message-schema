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
    std::recursive_mutex mtx;

    void open();
    void load_codes();

    cv::Mat get_plot_img();
    QString get_code_info();

    CODE_INFO get_cur_code();

    // other modules
    CONFIG *config = NULL;
    LOGGER *logger = NULL;
    UNIMAP* unimap = NULL;

    QTcpSocket client;
    QTimer reconnect_timer;
    QTimer check_recv_timer;

    std::atomic<bool> is_connected = {true};
    std::atomic<bool> is_recv_data = {false};
    std::atomic<bool> is_init = {true};

    double pre_err_x = 0;
    double pre_err_y = 0;
    double pre_err_th = 0;

    double cum_err_x = 0;
    double cum_err_y = 0;
    double cum_err_th = 0;

    std::atomic<double> err_x = {0};
    std::atomic<double> err_y = {0};
    std::atomic<double> err_th = {0};

    std::vector<double> storage_err_x;
    std::vector<double> storage_err_y;
    std::vector<double> storage_err_th;

    cv::Matx22d R;

    CODE_INFO cur_code;

    cv::Mat plot_img;
    std::map<QString, cv::Vec2d> ref_codes;

private Q_SLOTS:
    void connected();
    void disconnected();
    void readyread();

    void reconnect_loop();
    void check_recv_loop();
private:
};

#endif // CODE_READER_H
