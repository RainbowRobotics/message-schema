#ifndef COMM_DATA_H
#define COMM_DATA_H

#include "global_defines.h"

struct DATA_MOVE
{
    double time;
    QString command; // goal, jog, target, pause, resume, stop
    QString method; // pp, hpp, tng
    QString goal_node_id;
    QString goal_node_name;

    int preset;
    Eigen::Vector3d cur_pos; // x, y, z
    Eigen::Vector4d tgt_pose_vec; // x, y, z, th
    Eigen::Vector3d jog_val; // vx, vy, wz

    double remaining_dist;
    double remaining_time; // estimated time of arrival
    int bat_percent; // battery percentage

    QString result;
    QString message;
    //QString remark;

    DATA_MOVE()
    {
        time = 0.0;
        command = "";
        method = "";

        goal_node_id = "";
        goal_node_name = "";

        preset = 0;
        cur_pos.setZero();
        tgt_pose_vec.setZero();
        jog_val.setZero();

        remaining_dist = 0.0;
        remaining_time = 9999.0;
        bat_percent = -1;

        result = "";
        message = "";
        //remark = "";
    }

    DATA_MOVE(const DATA_MOVE& p)
    {
        time = p.time;
        command = p.command;
        method = p.method;
        goal_node_id = p.goal_node_id;
        goal_node_name = p.goal_node_name;
        remaining_dist = p.remaining_dist;

        preset = p.preset;
        cur_pos = p.cur_pos;
        tgt_pose_vec = p.tgt_pose_vec;
        jog_val = p.jog_val;

        remaining_time = p.remaining_time;
        bat_percent = p.bat_percent;

        result = p.result;
        message = p.message;
        //remark = p.remark;
    }

    DATA_MOVE& operator=(const DATA_MOVE& p)
    {
        time = p.time;
        command = p.command;
        method = p.method;
        goal_node_id = p.goal_node_id;
        goal_node_name = p.goal_node_name;
        remaining_dist = p.remaining_dist;

        preset = p.preset;
        cur_pos = p.cur_pos;
        tgt_pose_vec = p.tgt_pose_vec;
        jog_val = p.jog_val;

        remaining_time = p.remaining_time;
        bat_percent = p.bat_percent;

        result = p.result;
        message = p.message;
        //remark = p.remark;
        return *this;
    }
};

struct DATA_LOCALIZATION
{
    double time;
    QString command; // autoinit, semiautoinit, init, start, stop, randominit
    QString seed;
    Eigen::Vector4d tgt_pose_vec; // x, y, z, th

    QString result;
    QString message;

    DATA_LOCALIZATION()
    {
        time = 0.0;
        command = "";
        seed = "";
        tgt_pose_vec.setIdentity();

        result = "";
        message = "";
    }

    DATA_LOCALIZATION(const DATA_LOCALIZATION& p)
    {
        time = p.time;
        command = p.command;
        seed = p.seed;
        tgt_pose_vec = p.tgt_pose_vec;

        result = p.result;
        message = p.message;
    }

    DATA_LOCALIZATION& operator=(const DATA_LOCALIZATION& p)
    {
        time = p.time;
        command = p.command;
        seed = p.seed;
        tgt_pose_vec = p.tgt_pose_vec;

        result = p.result;
        message = p.message;
        return *this;
    }
};

struct DATA_LOAD
{
    double time;
    QString command; // mapload
    QString map_name; // map name

    QString result;
    QString message;

    DATA_LOAD()
    {
        time = 0.0;
        command = "";
        map_name = "";

        result = "";
        message = "";
    }

    DATA_LOAD(const DATA_LOAD& p)
    {
        time = p.time;
        command = p.command;
        map_name = p.map_name;

        result = p.result;
        message = p.message;
    }

    DATA_LOAD& operator=(const DATA_LOAD& p)
    {
        time = p.time;
        command = p.command;
        map_name = p.map_name;

        result = p.result;
        message = p.message;
        return *this;
    }
};

struct DATA_RANDOMSEQ
{
    double time;
    QString command; // randomseq

    QString result;
    QString message;

    DATA_RANDOMSEQ()
    {
        time = 0.0;
        command = "";

        result = "";
        message = "";
    }

    DATA_RANDOMSEQ(const DATA_RANDOMSEQ& p)
    {
        time = p.time;
        command = p.command;

        result = p.result;
        message = p.message;
    }

    DATA_RANDOMSEQ& operator=(const DATA_RANDOMSEQ& p)
    {
        time = p.time;
        command = p.command;

        result = p.result;
        message = p.message;
        return *this;
    }
};

struct DATA_MAPPING
{
    double time;
    QString command; // start, stop, save, reload
    QString map_name;

    QString result;
    QString message;

    DATA_MAPPING()
    {
        time = 0.0;
        command = "";
        map_name = "";

        result = "";
        message = "";
    }

    DATA_MAPPING(const DATA_MAPPING& p)
    {
        time = p.time;
        command = p.command;
        map_name = p.map_name;

        result = p.result;
        message = p.message;
    }

    DATA_MAPPING& operator=(const DATA_MAPPING& p)
    {
        time = p.time;
        command = p.command;
        map_name = p.map_name;

        result = p.result;
        message = p.message;
        return *this;
    }
};

struct DATA_DOCK
{
    double time;
    QString command; // dock, undock

    QString result;
    QString message;

    DATA_DOCK()
    {
        time = 0.0;
        command = "";

        result = "";
        message = "";
    }

    DATA_DOCK(const DATA_DOCK& p)
    {
        time = p.time;
        command = p.command;

        result = p.result;
        message = p.message;
    }

    DATA_DOCK& operator=(const DATA_DOCK& p)
    {
        time = p.time;
        command = p.command;

        result = p.result;
        message = p.message;
        return *this;
    }
};

struct DATA_FIELD
{
    double time;
    QString command; // set, get

    int set_field;
    int get_field;

    QString result;
    QString message;

    DATA_FIELD()
    {
        time = 0.0;
        command = "";

        set_field = 0;
        get_field = 0;

        result = "";
        message = "";
    }

    DATA_FIELD(const DATA_FIELD& p)
    {
        time = p.time;
        command = p.command;

        set_field = p.set_field;
        get_field = p.get_field;

        result = p.result;
        message = p.message;
    }

    DATA_FIELD& operator=(const DATA_FIELD& p)
    {
        time = p.time;
        command = p.command;

        get_field = p.get_field;
        set_field = p.set_field;

        result = p.result;
        message = p.message;
        return *this;
    }
};

struct DATA_VIEW_LIDAR
{
    double time;
    QString command; // on, off
    int frequency;

    QString result;
    QString message;

    DATA_VIEW_LIDAR()
    {
        time = 0.0;
        command = "";
        frequency = 0;

        result = "";
        message = "";
    }

    DATA_VIEW_LIDAR(const DATA_VIEW_LIDAR& p)
    {
        time = p.time;
        command = p.command;
        frequency = p.frequency;

        result = p.result;
        message = p.message;
    }

    DATA_VIEW_LIDAR& operator=(const DATA_VIEW_LIDAR& p)
    {
        time = p.time;
        command = p.command;
        frequency = p.frequency;

        result = p.result;
        message = p.message;
        return *this;
    }
};

struct DATA_LED
{
    double time;
    QString command; // on, off
    QString led;

    QString result;
    QString message;

    DATA_LED()
    {
        time = 0.0;
        command = "";
        led = "";

        result = "";
        message = "";
    }

    DATA_LED(const DATA_LED& p)
    {
        time = p.time;
        command = p.command;
        led = p.led;

        result = p.result;
        message = p.message;
    }

    DATA_LED& operator=(const DATA_LED& p)
    {
        time = p.time;
        command = p.command;
        led = p.led;

        result = p.result;
        message = p.message;
        return *this;
    }
};

struct DATA_MOTOR
{
    double time;
    QString command; // on, off

    QString result;
    QString message;

    DATA_MOTOR()
    {
        time = 0.0;
        command = "";

        result = "";
        message = "";
    }

    DATA_MOTOR(const DATA_MOTOR& p)
    {
        time = p.time;
        command = p.command;

        result = p.result;
        message = p.message;
    }

    DATA_MOTOR& operator=(const DATA_MOTOR& p)
    {
        time = p.time;
        command = p.command;

        result = p.result;
        message = p.message;
        return *this;
    }
};

struct DATA_VIEW_PATH
{
    double time;
    QString command; // on, off
    int frequency;

    QString result;
    QString message;

    DATA_VIEW_PATH()
    {
        time = 0.0;
        command = "";
        frequency = -1;

        result = "";
        message = "";
    }

    DATA_VIEW_PATH(const DATA_VIEW_PATH& p)
    {
        time = p.time;
        command = p.command;
        frequency = p.frequency;

        result = p.result;
        message = p.message;
    }

    DATA_VIEW_PATH& operator=(const DATA_VIEW_PATH& p)
    {
        time = p.time;
        command = p.command;
        frequency = p.frequency;

        result = p.result;
        message = p.message;
        return *this;
    }
};

struct DATA_PATH
{
    long long time;
    QString command; // path
    QString path;
    QString vobs_closures;
    int preset;

    QString response;
    QString result;
    QString message;

    DATA_PATH()
    {
        time = 0;
        command = "";
        path = "";
        vobs_closures = "";
        preset = 0;

        response = "";
        result = "";
        message = "";
    }

    DATA_PATH(const DATA_PATH& p)
    {
        time = p.time;
        command = p.command;
        path = p.path;
        vobs_closures = p.vobs_closures;
        preset = p.preset;

        response = p.response;
        result = p.result;
        message = p.message;
    }

    DATA_PATH& operator=(const DATA_PATH& p)
    {
        time = p.time;
        command = p.command;
        path = p.path;
        vobs_closures = p.vobs_closures;
        preset = p.preset;

        response = p.response;
        result = p.result;
        message = p.message;
        return *this;
    }
};

struct DATA_VOBS
{
    double time;
    QString command; // vobs
    QString vobs_robots;
    QString vobs_clousers;
    QString is_vobs_closures_change;

    QString result;
    QString message;

    DATA_VOBS()
    {
        time = 0.0;
        command = "";
        vobs_robots = "";
        vobs_clousers = "";
        is_vobs_closures_change = "";

        result = "";
        message = "";
    }

    DATA_VOBS(const DATA_VOBS& p)
    {
        time = p.time;
        command = p.command;
        vobs_robots = p.vobs_robots;
        vobs_clousers = p.vobs_clousers;
        is_vobs_closures_change = p.is_vobs_closures_change;

        result = p.result;
        message = p.message;
    }

    DATA_VOBS& operator=(const DATA_VOBS& p)
    {
        time = p.time;
        command = p.command;
        vobs_robots = p.vobs_robots;
        vobs_clousers = p.vobs_clousers;
        is_vobs_closures_change = p.is_vobs_closures_change;

        result = p.result;
        message = p.message;
        return *this;
    }
};

struct DATA_SOFTWARE
{
    double time;
    QString branch;
    QString version;

    QString result;
    QString message;

    DATA_SOFTWARE()
    {
        time = 0.0;
        branch = "";
        version = "";

        result = "";
        message = "";
    }

    DATA_SOFTWARE(const DATA_SOFTWARE& p)
    {
        time = p.time;
        branch = p.branch;
        version = p.version;

        result = p.result;
        message = p.message;
    }

    DATA_SOFTWARE& operator=(const DATA_SOFTWARE& p)
    {
        time = p.time;
        branch = p.branch;
        version = p.version;

        result = p.result;
        message = p.message;
        return *this;
    }
};

struct DATA_FOOT
{
    double time;

    bool connection = false;
    int position;
    bool is_down = false;
    int state;

    QString result;
    QString message;

    DATA_FOOT()
    {
        time = 0.0;

        connection = false;
        position = 0;
        is_down = false;
        state = 0;

        result = "";
        message = "";
    }

    DATA_FOOT(const DATA_FOOT& p)
    {
        time = p.time;

        connection = p.connection;
        position = p.position;
        is_down = p.is_down;
        state = p.state;

        result = p.result;
        message = p.message;
    }

    DATA_FOOT& operator=(const DATA_FOOT& p)
    {
        time = p.time;

        connection = p.connection;
        position = p.position;
        is_down = p.is_down;
        state = p.state;

        result = p.result;
        message = p.message;
        return *this;
    }
};

struct DATA_SAFTYIO
{
    double time;

    unsigned char mcu0_dio[8] ={0,};
    unsigned char mcu1_dio[8] ={0,};

    unsigned char mcu0_din[8] ={0,};
    unsigned char mcu1_din[8] ={0,};

    DATA_SAFTYIO()
    {
        time = 0.0;

        memset(mcu0_dio, 0, sizeof(mcu0_dio));
        memset(mcu1_dio, 0, sizeof(mcu1_dio));
        memset(mcu0_din, 0, sizeof(mcu0_din));
        memset(mcu1_din, 0, sizeof(mcu1_din));
    }

    DATA_SAFTYIO(const DATA_SAFTYIO& p)
    {
        time = p.time;

        memcpy(mcu0_dio, p.mcu0_dio, sizeof(char)*8);
        memcpy(mcu1_dio, p.mcu1_dio, sizeof(char)*8);
        memcpy(mcu0_din, p.mcu0_din, sizeof(char)*8);
        memcpy(mcu1_din, p.mcu1_din, sizeof(char)*8);
    }

    DATA_SAFTYIO& operator=(const DATA_SAFTYIO& p)
    {
        time = p.time;

        memcpy(mcu0_dio, p.mcu0_dio, sizeof(char)*8);
        memcpy(mcu1_dio, p.mcu1_dio, sizeof(char)*8);
        memcpy(mcu0_din, p.mcu0_din, sizeof(char)*8);
        memcpy(mcu1_din, p.mcu1_din, sizeof(char)*8);

        return *this;
    }
};


struct DATA_TEMPERATURE
{
    double time;

    bool connection = false;
    float temperature_value;

    DATA_TEMPERATURE()
    {
        time = 0.0;

        connection = false;
        temperature_value = 0;
    }

    DATA_TEMPERATURE(const DATA_TEMPERATURE& p)
    {
        time = p.time;

        connection = p.connection;
        temperature_value = p.temperature_value;
    }

    DATA_TEMPERATURE& operator=(const DATA_TEMPERATURE& p)
    {
        time = p.time;

        connection = p.connection;
        temperature_value = p.temperature_value;

        return *this;
    }
};

struct DATA_CAM_INFO
{
    double time;
    QString serial_cam0;
    QString serial_cam1;

    QString result;
    QString message;

    DATA_CAM_INFO()
    {
        time = 0.0;
        serial_cam0 = "";
        serial_cam1 = "";

        result = "";
        message = "";
    }

    DATA_CAM_INFO(const DATA_CAM_INFO& p)
    {
        time = p.time;
        serial_cam0 = p.serial_cam0;
        serial_cam1 = p.serial_cam1;

        result = p.result;
        message = p.message;
    }

    DATA_CAM_INFO& operator=(const DATA_CAM_INFO& p)
    {
        time = p.time;
        serial_cam0 = p.serial_cam0;
        serial_cam1 = p.serial_cam1;

        result = p.result;
        message = p.message;
        return *this;
    }
};

struct DATA_UPDATE_VARIABLE
{
    QString key;
    QString value;

    QString result;
    QString message;

    DATA_UPDATE_VARIABLE()
    {
        key = "";
        value = "";

        result = "";
        message = "";
    }

    DATA_UPDATE_VARIABLE(const DATA_UPDATE_VARIABLE& p)
    {
        key = p.key;
        value = p.value;

        result = p.result;
        message = p.message;
    }

    DATA_UPDATE_VARIABLE& operator=(const DATA_UPDATE_VARIABLE& p)
    {
        key = p.key;
        value = p.value;

        result = p.result;
        message = p.message;
        return *this;
    }
};

struct DATA_COMMON
{
    enum class TYPE
    {
        LOAD,
        LOCALIZATION,
        RANDOMSEQ,
        MAPPING,
        DOCKING,
        VIEW_LIDAR,
        VIEW_PATH,
        LED,
        MOTOR,
        SW_UPDATE,
        UPDATE_VARIABLE,
        CAM_INFO
    };
    TYPE type;

    DATA_LOAD dload;
    DATA_LOCALIZATION dlocalization;
    DATA_RANDOMSEQ drandomseq;
    DATA_MAPPING dmapping;
    DATA_DOCK ddock;
    DATA_VIEW_LIDAR dlidar;
    DATA_VIEW_PATH dpath;
    DATA_LED dled;
    DATA_MOTOR dmotor;
    DATA_SOFTWARE dsw;
    DATA_UPDATE_VARIABLE duv;
    DATA_CAM_INFO dci;
};


Q_DECLARE_METATYPE(DATA_MOVE)
Q_DECLARE_METATYPE(DATA_LOCALIZATION)
Q_DECLARE_METATYPE(DATA_LOAD)
Q_DECLARE_METATYPE(DATA_RANDOMSEQ)
Q_DECLARE_METATYPE(DATA_MAPPING)
Q_DECLARE_METATYPE(DATA_DOCK)
Q_DECLARE_METATYPE(DATA_VIEW_LIDAR)
Q_DECLARE_METATYPE(DATA_VIEW_PATH)
Q_DECLARE_METATYPE(DATA_LED)
Q_DECLARE_METATYPE(DATA_MOTOR)
Q_DECLARE_METATYPE(DATA_PATH)
Q_DECLARE_METATYPE(DATA_VOBS)
Q_DECLARE_METATYPE(DATA_SOFTWARE)
Q_DECLARE_METATYPE(DATA_FOOT)
Q_DECLARE_METATYPE(DATA_FIELD)
Q_DECLARE_METATYPE(DATA_CAM_INFO)
Q_DECLARE_METATYPE(DATA_UPDATE_VARIABLE)
Q_DECLARE_METATYPE(DATA_COMMON)
Q_DECLARE_METATYPE(DATA_SAFTYIO)

#endif // COMM_DATA_H
