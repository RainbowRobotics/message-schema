/*! \file LakiBeamHTTP.h */

/**
 * @file LakiBeamHTTP.h
 * @brief 基本功能类的定义
 * @details 包含用户可以使用的所有API集合
 * @mainpage SDK功能类集合
 * @author wzqm
 * @email 283080916@qq.com
 * @version 1.0
 * @date 2022年3月11日
 * @license GPL
 */

#ifndef LAKIBEAMHTTP_H
#define LAKIBEAMHTTP_H

#include <QString>

#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>
#include <boost/asio.hpp>
#include <boost/thread.hpp>


#include <sstream>
#include <cstdlib>
#include <iostream>
#include <stdio.h>
using namespace std;


/// 已使用声明：用来消除未使用变量报警，没有功能上面的意义
#ifndef USED
#define USED(x)                                     ((void)x)
#endif


/// 调试接口：快速打印调试信息
#ifndef DEBUG
#define DEBUG(x)                                    (cout<<x<<endl)
#endif


// 也可以使用这种方式调用
// using tcp_t = boost::asio::ip::tcp;
// namespace http = boost::beast::http;


/// 重定义命名空间
#ifndef http_n
namespace http_n                                    = boost::beast::http;
#endif


/**
 *  为了统一代码风格，对部分数据类型进行了重新定义
*/
#ifndef i32_t
typedef signed int                                  i32_t;
#endif
#ifndef u32_t
typedef unsigned int                                u32_t;
#endif
#ifndef u16_t
typedef unsigned short                              u16_t;
#endif
#ifndef u16_t
typedef unsigned short                              i16_t;
#endif
#ifndef u8_t
typedef unsigned char                               u8_t;
#endif
#ifndef c8_t
typedef char                                        i8_t;
#endif

#ifndef tcp_t
typedef boost::asio::ip::tcp                        tcp_t;
#endif
#ifndef endpoint_t
typedef boost::asio::ip::tcp::endpoint              tcp_endpoint_t;
#endif
#ifndef resolver_t
typedef boost::asio::ip::tcp::resolver              resolver_t;
#endif
#ifndef tcp_socket_t
typedef boost::asio::ip::tcp::socket                tcp_socket_t;
#endif
#ifndef address_t
typedef boost::asio::ip::address                    address_t;
#endif
#ifndef attribute_tcp_t
typedef boost::asio::socket_base::reuse_address     socket_attribute_t;
#endif
#ifndef io_context_t
typedef boost::asio::io_context                     io_context_t;
#endif
#ifndef document_t
typedef rapidjson::Document                         document_t;
#endif
#ifndef value_t
typedef rapidjson::Value                            value_t;
#endif
#ifndef sizetype_t
typedef rapidjson::SizeType                         sizetype_t;
#endif
#ifndef dynamic_body_t
typedef http_n::response<http_n::dynamic_body>      dynamic_body_t;
#endif
#ifndef service_t
typedef boost::asio::io_service                     service_t;
#endif
#ifndef resolver_t
typedef boost::asio::ip::tcp::resolver              resolver_t;
#endif
#ifndef query_t
typedef resolver_t::query                           query_t;
#endif
#ifndef attribute_tcp_t
typedef boost::asio::socket_base::reuse_address     attribute_tcp_t;
#endif
#ifndef iterator_t
typedef resolver_t::iterator                        iterator_t;
#endif

/// REST接口架构——获取雷达固件信息
#define FIREMWARE_CMD                       ("/api/v1/system/firmware")
#define FIREMWARE_MODEL                     ("model")
#define FIREMWARE_SN                        ("sn")
#define FIREMWARE_FPGA                      ("fpga")
#define FIREMWARE_CORE                      ("core")
#define FIREMWARE_AUX                       ("aux")

/// 获取雷达固件信息封装类
class firemware_c
{

public:
    firemware_c(){}

    string model;
    string sn;
    string fpga;
    string core;
    string aux;
};

/// REST接口架构——获取系统监控数据
#define MONITOR_CMD                         ("/api/v1/system/monitor")
#define MONITOR_LOAD_AVERAGE                ("load_average")
#define MONITOR_USEAGE                      ("mem_useage")
#define MONITOR_UPTIME                      ("uptime")

///
/**
* @brief 获取系统监控数据类封装 \n
* 系统数据信息
*/
class monitor_c
{

public:
    monitor_c(){}

    double load_average;                    ///< 系统平均负载
    double mem_useage;                      ///< 内存使用率
    double uptime;                          ///< 系统运行时间,指系统上电时间
};


/// REST接口架构——获取系网络信息
#define NETWORK_CMD                         ("/api/v1/system/network")
#define NETWORK_CARRIER                     ("carrier")
#define NETWORK_DUPLEX                      ("duplex")
#define NETWORK_ETHADDR                     ("ethaddr")
#define NETWORK_HOSTNAME                    ("hostname")
#define NETWORK_IPV4                        ("ipv4")
#define NETWORK_IPV4_DHCP                   ("dhcp")
#define NETWORK_IPV4_ADDR                   ("addr")
#define NETWORK_IPV4_OVERRIDE               ("override")
#define NETWORK_SPEED                       ("speed")

/**
* @brief 网络信息类封装 \n
* 网络信息各项参数
*/
class network_c
{

public:
    network_c(){}
    bool      carrier;                      ///< 以太网链路状态
    string    duplex;                       ///< 通信模式
    string    ethaddr;                      ///< mac地址
    string    hostname;                     ///< 主机名
    bool      ipv4_dhcp;                    ///< dhch是否使能
    string    ipv4_addr;                    ///< ipv4地址
    string    ipv4_override;                ///< 静态ipv4地址
    int         speed;                      ///< 网络速度
};


/// REST接口架构——配置静态地址
#define OVERRIDE_CONFIG_CMD                 ("/api/v1/system/network/override")
#define OVERRIDE_CONFIG_CMD_ENSURE          ("HTTP/1.1 200 OK")


/// REST接口架构——删除静态模式
#define OVERRIDE_DELETE_CMD                 ("/api/v1/system/network/override")
#define OVERRIDE_DELETE_CMD_ENSURE          ("HTTP/1.1 200 OK")


/// REST接口架构——复位服务器——这部分返回值是错误的无法设置成功
#define RESET_CMD                           ("/api/v1/system/reset")
#define RESET_MESSAGE                       ("reset")
#define RESET_ENSURE                        ("HTTP/1.1 200 OK")


/// REST接口架构——雷达信息
#define OVERVIEW_CMD                        ("/api/v1/sensor/overview")
#define OVERVIEW_SCANFREQ                   ("scanfreq")
#define OVERVIEW_MOTOR_RPM                  ("motor_rpm")
#define OVERVIEW_LASER_ENABLE               ("laser_enable")
#define OVERVIEW_RESOLUTION                 ("resolution")


/// 雷达信息扫描启动参数信息
#define OVERVIEW_SCAN_RANGE                 ("scan_range")
#define OVERVIEW_SCAN_RANGE_START           ("start")
#define OVERVIEW_SCAN_RANGE_STOP            ("stop")


/// 雷达信息滤波器参数信息
#define OVERVIEW_FILLTER                    ("filter")
#define OVERVIEW_FILLTER_LEVEL              ("level")
#define OVERVIEW_FILLTER_MIN_ANGLE          ("min_angle")
#define OVERVIEW_FILLTER_MAX_ANGLE          ("max_angle")
#define OVERVIEW_FILLTER_NEIGHBORS          ("neighbors")
#define OVERVIEW_FILLTER_WINDOW             ("window")


/// 雷达信息主机配置参数信息
#define OVERVIEW_HOST                       ("host")
#define OVERVIEW_HOST_IP                    ("ip")
#define OVERVIEW_HOST_PORT                  ("port")

/**
* @brief 复位服务器数据类 \n
* 服务器各项参数
*/
class overview_c
{

public:
    overview_c(){}
    int         scanfreq;                   ///< 扫描频率
    int         motor_rpm;                  ///< 电机转速
    bool        laser_enable;               ///< 雷达使能
    double      resolution;                 ///< 雷达角分辨率
    int         scan_range_start;           ///< 雷达扫描起始位置
    int         scan_range_stop;            ///< 雷达扫描结束位置
    int         filter_level;               ///< 滤波器扫描等级
    int         window;                     ///< 滤波器窗口值
    int         filter_min_angle;           ///< 最小滤波角度
    int         filter_max_angle;           ///< 最大滤波角度
    int         filter_neighbors;           ///< 关联滤波值个数
    string      host_ip;                    ///< 主机ip
    int         host_port;                  ///< 主机端口号

};


/// REST接口架构——设置雷达扫描频率
#define SCANFREQ_CONFIG_CMD                 ("/api/v1/sensor/scanfreq")
#define SCANFREQ_CONFIG_ENSURE              ("HTTP/1.1 200 OK")


/// REST接口架构——获取雷达扫描频率
#define SCANFREQ_CMD                        ("/api/v1/sensor/scanfreq")
#define SCANFREQ_CMD_ENSURE                 ("HTTP/1.1 200 OK")
#define SCANFREQ_FREQ                       ("scanfreq")


/// REST接口架构——获取雷达电机转速
#define MOTOR_RPM_CMD                       ("/api/v1/sensor/motor_rpm")
#define MOTOR_RPM_RPM                       ("motor_rpm")


/// REST接口架构——获取雷达电机转速
#define LASER_ENABLE_CMD                    ("/api/v1/sensor/laser_enable")
#define LASER_ENABLE_ENABLE                 ("laser_enable")


/// REST接口架构——切换雷达激光开关状态
#define LASER_ENABLE_CMD                    ("/api/v1/sensor/laser_enable")
#define LASER_ENABLE_ENSURE                 ("HTTP/1.1 200 OK")


/// REST接口架构——获取雷达水平角分辨率
#define LASER_SESOLUTION_CMD                ("/api/v1/sensor/resolution")
#define LASER_SESOLUTION_SESOLUTION         ("resolution")


/// REST接口架构——获取雷达角度扫描范围
#define SCAN_RANGE_CMD                      ("/api/v1/sensor/scan_range")
#define SCAN_RANGE_RANGE                    ("scan_range")
#define SCAN_RANGE_START                    ("start")
#define SCAN_RANGE_STOP                     ("stop")

/**
* @brief 获取雷达角度扫描范围封装类 \n
* 雷达扫描起始位置和结束位置
*/
class scan_range_c
{

public:
    scan_range_c(){}

    int start;                              ///< 扫描起始位置
    int stop;                               ///< 扫描结束位置

};


/// REST接口架构——获取雷达扫描起始角度
#define START_CMD                           ("/api/v1/sensor/scan_range/start")
#define START_START                         ("start")
#define START_CMD_ENSURE                    ("HTTP/1.1 200 OK")


/// REST接口架构——获取雷达扫描结束角度
#define STOP_CMD                            ("/api/v1/sensor/scan_range/stop")
#define STOP_STOP                           ("stop")
#define STOP_CMD_ENSURE                     ("HTTP/1.1 200 OK")


/// REST接口架构——设置雷达扫描起始角度
#define START_PUT_CMD                       ("/api/v1/sensor/scan_range/start")
#define START_PUT_CMD_ENSURE                ("HTTP/1.1 200 OK")


/// REST接口架构——设置雷达扫描结束角度
#define STOP_PUT_CMD                        ("/api/v1/sensor/scan_range/stop")
#define STOP_PUT_CMD_ENSURE                 ("HTTP/1.1 200 OK")


/// REST接口架构——获取雷达滤波器参数
#define FILTER_CMD                          ("/api/v1/sensor/filter")
#define FILTER                              ("start")
#define START_CMD_ENSURE                    ("HTTP/1.1 200 OK")


/// REST接口架构——获取雷达扫描结束角度
#define STOP_CMD                            ("/api/v1/sensor/scan_range/stop")
#define STOP_STOP                           ("stop")
#define STOP_CMD_ENSURE                     ("HTTP/1.1 200 OK")


/// REST接口架构——获取雷达滤波等级
#define FILTER_CMD                          ("/api/v1/sensor/filter")
#define FILTER_FILTER                       ("filter")
#define FILTER_LEVEL                        ("level")
#define FILTER_MIN_ANGLE                    ("min_angle")
#define FILTER_MAX_ANGLE                    ("max_angle")
#define FILTER_NEIGHBORS                    ("neighbors")
#define FILTER_WINDOW                       ("window")

/**
* @brief 雷达滤波等级类 \n
* 滤波器参数
*/
class filter_c
{

public:
    filter_c(){}
    int         level;                      ///< 滤波等级
    int         window;                     ///< 滤波窗口
    int         min_angle;                  ///< 最大滤波角度
    int         max_angle;                  ///< 最大滤波角度
    int         neighbors;                  ///< 关联滤波值个数

};


/// REST接口架构——设置滤波器等级
#define FILTER_LEVEL_CMD                    ("/api/v1/sensor/filter/level")
#define FILTER_LEVEL_ENSURE                 ("HTTP/1.1 200 OK")


/// REST接口架构——设置和获取滤波窗口大小
#define FILTER_WINDOW_CMD                   ("/api/v1/sensor/filter/window")
#define FILTER_WINDOW_WINDOW                ("window")
#define FILTER_WINDOW_ENSURE                ("HTTP/1.1 200 OK")


/// REST接口架构——设置和获取最大角度值
#define FILTER_MAX_ANGLE_CMD                ("/api/v1/sensor/filter/max_angle")
#define FILTER_MAX_ANGLE_MAX_ANGLE          ("max_angle")
#define FILTER_MAX_ANGLE_ENSURE             ("HTTP/1.1 200 OK")


/// REST接口架构——设置和获取最小角度值
#define FILTER_MIN_ANGLE_CMD                ("/api/v1/sensor/filter/min_angle")
#define FILTER_MIN_ANGLE_MIN_ANGLE          ("min_angle")
#define FILTER_MIN_ANGLE_ENSURE             ("HTTP/1.1 200 OK")


/// REST接口架构——设置和获取相邻滤波值
#define FILTER_NEIGHBORS_CMD                ("/api/v1/sensor/filter/neighbors")
#define FILTER_NEIGHBORS_NEIGHBORS          ("neighbors")
#define FILTER_NEIGHBORS_ENSURE             ("HTTP/1.1 200 OK")


/// REST接口架构——获取主机配置
#define HOST_CMD                            ("/api/v1/sensor/host")
#define HOST_HOST                           ("host")
#define HOST_ENSURE                         ("HTTP/1.1 200 OK")
#define HOST_IP_CMD                         ("/api/v1/sensor/host/ip")
#define HOST_IP_ENSURE                      ("HTTP/1.1 200 OK")
#define HOST_PORT_CMD                       ("/api/v1/sensor/host/port")
#define HOST_PORT_ENSURE                    ("HTTP/1.1 200 OK")
#define HOST_IP                             ("ip")
#define HOST_PORT                           ("port")

/**
* @brief 主机配置类 \n
* 主机的配置参数
*/
class host_c
{

public:
    host_c(){}
    string      ip;                         ///< ip地址
    int         port;                       ///< 端口号

};


/// HTTP响应超时时间
#define HTTP_OVER_TIME                      (100)


/// 部分参数设置推荐值
#define FILTER_MIN_ANGLE_RECOMMEND          ("20")
#define FILTER_MIAX_ANGLE_RECOMMEND         ("170")
#define FILTER_MIAX_WINDOW_RECOMMEND        ("1")
#define FILTER_MIAX_NEIGHBORS_RECOMMEND     ("1")


/// IP地址设置错误自动设置成默认参数
#define HOST_IP_DEFAULT                     ("192.168.2.123")
#define HTTP_IP_DEFAULT                     ("192.168.2.10")
#define HTTP_PORT_DEFAULT                   ("80")


/// 静态IP地址设置错误自动设置成默认参数
#define STATIC_IP_DEFAULT                   ("192.168.1.211/24")


/// 扫描频率设置错误设置成默认参数
#define SCANFREQ_DEFAULT                    ("25")


/**
* @brief 雷达配置文件类 \n
* 可以配置雷达所有的运行参数
*/
class LakiBeamHTTP
{

public:
    LakiBeamHTTP(string local_ip, string local_port, string web_ip, string web_port);


    /*!
        获取雷达固件信息
        \li 具体的参数信息查询 LakiBeam-User-Guide 手册

        \param firemware 雷达固件信息的引用类参数引用
        \return true：操作成功 false：操作失败
    */
    bool get_firemware(firemware_c& firemware);
    /*!
        获取雷达监测信息
        \li 具体的参数信息询 LakiBeam-User-Guide 手册

        \param monitor 获取系统监控数据,包含：系统平均负载、内存使用率和系统运行时间
        \return true:操作成功 false:操作失败
    */
    bool get_monitor(monitor_c& monitor);
    /*!
        获取网络信息
        \li 具体的参数信息查询 LakiBeam-User-Guide 手册

        \param network 网络信息返回值，包含：以太网链路状态、双工模式、MAC地址、主机名称、ipv4信息以及以太网传输速率
        \return true:操作成功 false:操作失败
    */
    bool get_network(network_c& network);
    /*!
        获取当前静态IP重写值

        \param override 静态IP重写地址设定值，当DHCP模式打开时操作有效，否则为无效操作
        \param result 本次操作的结果或者错误信息
        \return true：操作成功 false：操作失败
    */
    bool put_override(string override, string& result);
    /*!
        删除静态模式配置并设置为DHCP模式
        \li 具体的信息查询 LakiBeam-User-Guide 手册

        \param result 本次操作的结果
        \return true：操作成功 false：操作失败
    */
    bool delete_override(string& result);
    /*!
        系统复位

        \param result 本次操作的结果 true：操作成功 false：操作失败
        \return true：操作成功 false：操作失败
    */
    bool put_reset(string& result);
    /*!
        获取雷达整体测量信息
        \li 具体的参数信息查询 LakiBeam-User-Guide 手册

        \param overview 包含:雷达扫描频率、雷达实时转速、激光开关状态、分辨率、扫描角度范围、滤波等级和Host端IP地址及端口号
        \return true:操作成功 false:操作失败
    */
    bool get_overview(overview_c& overview);
    /*!
        设置雷达扫描频率

        \param freq 设置扫描频率设定值 设定选项：10 20 25 30 推荐值：25
        \return true:操作成功 false:操作失败
    */
    bool put_scanfreq(string& freq);
    /*!
        获取雷达扫描频率

        \param result 当前雷达扫描频率返回值 单位：hz
        \return true:操作成功 false:操作失败
    */
    bool get_scanfreq(string& result);
    /*!
        获取雷达实时转速

        \param motor_rpm 当前雷达转速返回值 单位：rpm
        \return true:操作成功 false:操作失败
    */
    bool get_motor_rpm(string& motor_rpm);
    /*!
        获取激光状态

        \param motor_rpm 当前激光状态返回值 true：使能 false：失能
        \return true：操作成功 false：操作失败
    */
    bool get_laser_enable(string& laser_state);
    /*!
        切换激光状态

        \param config_state 激光状态设定值 true：使能 false：失能
        \return true：操作成功 false：操作失败
    */
    bool put_laser_enable(string& config_state);
    /*!
        获取雷达当前水平角分辨率

        \param resolution 雷达水平角分辨率返回值，resolution(float)：为浮点数，
        当前雷达水平角分辨率,各扫描频率对应的水平角分辨率分别为:10Hz/0.1°,20Hz/0.25°,25Hz/0.25°,30Hz/0.25°

        \return true：操作成功 false：操作失败
    */
    bool get_resolution(string& resolution);
    /*!
        获取雷达扫描范围

        \param scan_range 雷达扫描范围返回值 单位：度
        \return true：操作成功 false：操作失败
    */
    bool get_scan_range(scan_range_c& scan_range);
    /*!
        获取激光扫描起始角度

        \param start 雷达扫描起始角度返回值 单位：度
        \return true：操作成功 false：操作失败
    */
    bool get_laser_start(string& start);
    /*!
        获取激光扫描结束角度

        \param start 雷达扫描结束角度返回值 单位：度
        \return true：参数信息操作成功 false：参数信息操作失败
    */
    bool get_laser_stop(string& stop);
    /*!
        设置激光扫描起始角度

        \param start 设置雷达扫描起始角度设定值 范围：0度~315度 注意：设定的起始角度应小于结束角度，否则设定失败
        \return true：参数信息操作成功 false：参数信息操作失败
    */
    bool put_laser_start(string& start);
    /*!
        设置激光扫描结束角度

        \param stop 设置雷达扫描结束角度设定值 范围：0度~315度 注意：设定的起始角度应小于结束角度，否则设定失败
        \return true：参数信息操作成功 false：参数信息操作失败
    */
    bool put_laser_stop(string& stop);
    /*!
        设置激光扫描结束角度
        \li 具体的参数信息查询 LakiBeam-User-Guide 手册

        \param level 滤波器信息结构体返回值
        \return true：参数信息操作成功 false：参数信息操作失败
    */
    bool get_filter_level(filter_c& level);
    /*!
        设置当前雷达数据滤波器件等级

        \param level 设置滤波器级别：0~3 0为关闭
        \return true：参数信息操作成功 false：参数信息操作失败
    */
    bool put_filter_level(string& level);
    /*!
        获取当前雷达数据滤波器滤波窗口值

        \param level 当前雷达滤波窗口返回值
        \return true：参数信息操作成功 false：参数信息操作失败
    */
    bool get_filter_window(string& window);
    /*!
        设置当前雷达数据滤波器滤波窗口值

        \param level 滤波窗口设定值：0~10 推荐值：1
        \return true：参数信息操作成功 false：参数信息操作失败
    */
    bool put_filter_window(string& window);
    /*!
        获取当前雷达数据滤波器最大角度值

        \param level 当前雷达数据滤波器最大角度返回值
        \return true：参数信息操作成功 false：参数信息操作失败
    */
    bool get_filter_max_angle(string& max_angle);
    /*!
        设定当前雷达数据滤波器最大角度值

        \param level 当前雷达数据滤波器最大角度设定值
        \return true：参数信息操作成功 false：参数信息操作失败
    */
    bool put_filter_max_angle(string& max_angle);
    /*!
        设定当前雷达数据滤波器最小角度值

        \param level 当前雷达数据滤波器最小角度返回值
        \return true：参数信息操作成功 false：参数信息操作失败
    */
    bool get_filter_min_angle(string& min_angle);
    /*!
        设定当前雷达数据滤波器最小角度值

        \param level 当前雷达数据滤波器最小角度设定值
        \return true：参数信息操作成功 false：参数信息操作失败
    */
    bool put_filter_min_angle(string& min_angle);
    /*!
        获取当前雷达数据滤波器关联数据个数

        \param level 当前雷达数据滤波器关联数据个数返回值
        \return true：参数信息操作成功 false：参数信息操作失败
    */
    bool get_filter_neighbors(string& neighbors);
    /*!
        设定当前雷达数据滤波器关联数据个数

        \param level 雷达数据滤波器关联数据个数设定值
        \return true：参数信息操作成功 false：参数信息操作失败
    */
    bool put_filter_neighbors(string& neighbors);
    /*!
        获取当前雷达Host端IP配置
        \li 具体的参数信息查询 LakiBeam-User-Guide 手册

        \param host Host端IP配置信息返回值
        \return true：参数信息操作成功 false：参数信息操作失败
    */
    bool get_host(host_c& host);
    /*!
        获取当前雷达Host端IP地址

        \param ip 雷达Host端IP地址返回值
        \return true：参数信息操作成功 false：参数信息操作失败
    */
    bool get_host_ip(string& ip);
    /*!
        设定当前雷达Host端IP地址

        \param ip 雷达Host端IP地址设定值
        \return true：参数信息操作成功 false：参数信息操作失败
    */
    bool put_host_ip(string& ip);
    /*!
        获取当前雷达Host端IP地址端口号

        \param port Host端IP地址端口号返回值
        \return true：参数信息操作成功 false：参数信息操作失败
    */
    bool get_host_port(string& port);
    /*!
        设定当前雷达Host端IP地址端口号

        \param ip Host端IP地址端口号设定值
        \return true：参数信息操作成功 false：参数信息操作失败
    */
    bool put_host_port(string& ip);


private:
    /// 本地ip
    string                                        local_ip;
    /// 本地端口号
    string                                        local_port;
    /// 雷达ip
    string                                        web_ip;
    /// 雷达端口号
    string                                        web_port;

private:
    /*!
        同步get命令

        \param target 操作目标
        \return 返回操作信息
    */
    string get_http(string target);
    /*!
        同步put命令

        \param target 操作目标
        \param message 操作内容
        \return 返回操作信息
    */
    string put_http(string target, string message);
    /*!
        同步delete命令

        \param target 操作目标
        \return 返回操作信息
    */
    string delete_http(string target);
    /*!
        异步get命令

        \param target 操作目标
        \return 返回操作信息
    */
    string get_async_http(string target);
    /*!
        异步put命令

        \param target 操作目标
        \param message 操作内容
        \return 返回操作信息
    */
    string put_async_http(string target, string message);
    /*!
        异步delete命令

        \param target 操作目标
        \return 返回操作信息
    */
    string delete_async_http(string target);
    /*!
        检测ip地址是否合法

        \return true：合法 false：不合法
    */
    bool check_ipconfig(void);

};

#endif // LAKIBEAMHTTP_H
