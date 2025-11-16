#ifndef COMM_DATA_H
#define COMM_DATA_H

#include <sio_client.h>
#include "global_defines.h"

struct SOCKET_MESSAGE
{
    QString event;
    sio::message::ptr data;
};
struct DATA_MOVE
{
    double time;               // timestamp
    QString id;                // rrs message id
    int preset;                // preset number

    QString command;           // goal, jog, target, pause, resume, stop
    QString method;            // pp, hpp, tng
    QString direction;         // move direction (e.g., fwd, back)
    QString dir;               // optional direction string (legacy)

    QString goal_node_id;
    QString goal_node_name;

    Eigen::Vector3d cur_pos;        // current position (x, y, z)
    Eigen::Vector4d tgt_pose_vec;   // target pose (x, y, z, th)
    Eigen::Vector3d jog_val;        // jog velocity (vx, vy, wz)

    double target;             // target distance or angle (m, deg)
    double speed;              // target speed (m/s, deg/s)
    double meassured_dist;     // measured distance moved (m, deg)

    double remaining_dist;     // remaining distance (m, deg)
    double remaining_time;     // estimated time of arrival (sec)
    int bat_percent;           // battery percentage

    QString result;            // result status
    QString message;           // message text

    DATA_MOVE()
    {
        time = 0.0;
        id = "";
        preset = 0;

        command = "";
        method = "";
        direction = "";
        dir = "";

        goal_node_id = "";
        goal_node_name = "";

        cur_pos.setZero();
        tgt_pose_vec.setZero();
        jog_val.setZero();

        target = 0.0;
        speed = 0.0;
        meassured_dist = 0.0;

        remaining_dist = 0.0;
        remaining_time = 9999.0;
        bat_percent = -1;

        result = "";
        message = "";
    }

    DATA_MOVE(const DATA_MOVE& p)
    {
        time = p.time;
        id = p.id;
        preset = p.preset;

        command = p.command;
        method = p.method;
        direction = p.direction;
        dir = p.dir;

        goal_node_id = p.goal_node_id;
        goal_node_name = p.goal_node_name;

        cur_pos = p.cur_pos;
        tgt_pose_vec = p.tgt_pose_vec;
        jog_val = p.jog_val;

        target = p.target;
        speed = p.speed;
        meassured_dist = p.meassured_dist;

        remaining_dist = p.remaining_dist;
        remaining_time = p.remaining_time;
        bat_percent = p.bat_percent;

        result = p.result;
        message = p.message;
    }

    DATA_MOVE& operator=(const DATA_MOVE& p)
    {
        if (this == &p) return *this;

        time = p.time;
        id = p.id;
        preset = p.preset;

        command = p.command;
        method = p.method;
        direction = p.direction;
        dir = p.dir;

        goal_node_id = p.goal_node_id;
        goal_node_name = p.goal_node_name;

        cur_pos = p.cur_pos;
        tgt_pose_vec = p.tgt_pose_vec;
        jog_val = p.jog_val;

        target = p.target;
        speed = p.speed;
        meassured_dist = p.meassured_dist;

        remaining_dist = p.remaining_dist;
        remaining_time = p.remaining_time;
        bat_percent = p.bat_percent;

        result = p.result;
        message = p.message;

        return *this;
    }
};


struct DATA_LOCALIZATION
{
    double time;
    QString id;
    QString command; // autoinit, semiautoinit, init, start, stop, randominit
    QString seed;
    Eigen::Vector4d tgt_pose_vec; // x, y, z, th

    QString result;
    QString message;

    DATA_LOCALIZATION()
    {
        time = 0.0;
        id = "";
        command = "";
        seed = "";
        tgt_pose_vec.setIdentity();

        result = "";
        message = "";
    }

    DATA_LOCALIZATION(const DATA_LOCALIZATION& p)
    {
        time = p.time;
        id = p.id;
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
        id = p.id;
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
    QString id;
    QString command; // mapload
    QString map_name; // map name

    QString result;
    QString message;

    DATA_LOAD()
    {
        id = "";
        time = 0.0;
        command = "";
        map_name = "";

        result = "";
        message = "";
    }

    DATA_LOAD(const DATA_LOAD& p)
    {
        id = p.id;
        time = p.time;
        command = p.command;
        map_name = p.map_name;

        result = p.result;
        message = p.message;
    }

    DATA_LOAD& operator=(const DATA_LOAD& p)
    {
        time = p.time;
        id = p.id;
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

struct DATA_CONTROL
{
    static constexpr const char* SetSafetyField = "setSafetyField";
    static constexpr const char* GetSafetyField = "getSafetyField";
    static constexpr const char* ResetSafetyField = "resetSafetyField";
    static constexpr const char* Dock = "dock";
    static constexpr const char* Undock = "undock";
    static constexpr const char* RandomSeq = "randomSeq";
    static constexpr const char* LedControl = "ledControl";
    static constexpr const char* LidarOnOff = "lidarOnOff";
    static constexpr const char* PathOnOff = "pathOnOff";
    static constexpr const char* MotorOnOff = "motorOnOff";
    static constexpr const char* safetyIO = "safetyIO";
    static constexpr const char* setDigitalIO = "setDigitalIO";
    static constexpr const char* getDigitalIO = "getDigitalIO";

    double time;
    QString id;
    QString command;

    bool onoff;
    QString color;
    int frequency;
    unsigned char mcu0_dio[8] ={0,};
    unsigned char mcu1_dio[8] ={0,};
    QString safetyField;
    QString resetField;

    QString result;
    QString message;

    DATA_CONTROL(){
        time = 0.0;
        command = "";
        id = "";
        onoff = false;
        color = "";
        frequency = 0;
        memset(mcu0_dio, 0, sizeof(mcu0_dio));
        memset(mcu1_dio, 0, sizeof(mcu1_dio));
        safetyField = "";
        resetField = "";

        result = "";
        message = "";
    }

    DATA_CONTROL(const DATA_CONTROL& p)
    {
        time = p.time;
        id = p.id;
        command = p.command;
        onoff = p.onoff;
        color = p.color;
        frequency = p.frequency;
        memcpy(mcu0_dio, p.mcu0_dio, sizeof(char)*8);
        memcpy(mcu1_dio, p.mcu1_dio, sizeof(char)*8);
        safetyField = p.safetyField;
        resetField = p.resetField;
        result = p.result;
        message = p.message;
    }

    DATA_CONTROL& operator=(const DATA_CONTROL& p)
    {
        time = p.time;
        id = p.id;
        command = p.command;
        onoff = p.onoff;
        color = p.color;
        frequency = p.frequency;
        memcpy(mcu0_dio, p.mcu0_dio, sizeof(char)*8);
        memcpy(mcu1_dio, p.mcu1_dio, sizeof(char)*8);
        safetyField = p.safetyField;
        resetField = p.resetField;
        result = p.result;
        message = p.message;
        return *this;
    }
};

struct DATA_MAPPING
{
    double time;
    QString id;
    QString command; // start, stop, save, reload
    QString map_name;

    QString result;
    QString message;

    DATA_MAPPING()
    {
        time = 0.0;
        id = "";
        command = "";
        map_name = "";

        result = "";
        message = "";
    }

    DATA_MAPPING(const DATA_MAPPING& p)
    {
        time = p.time;
        id = p.id;
        command = p.command;
        map_name = p.map_name;

        result = p.result;
        message = p.message;
    }

    DATA_MAPPING& operator=(const DATA_MAPPING& p)
    {
        time = p.time;
        id = p.id;
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

    QString id;

    DATA_DOCK()
    {
        time = 0.0;
        command = "";

        result = "";
        message = "";
        id ="";
    }

    DATA_DOCK(const DATA_DOCK& p)
    {
        time = p.time;
        command = p.command;

        result = p.result;
        message = p.message;
        id = p.id;
    }

    DATA_DOCK& operator=(const DATA_DOCK& p)
    {
        time = p.time;
        command = p.command;

        result = p.result;
        message = p.message;
        id = p.id;
        return *this;
    }
};

struct DATA_PROFILE_MOVE
{
    double time;
    QString command; // "lienarMove", "rotateMove", "circularMove", "stop"

    double target; // m , deg
    double speed; // m/s, deg/s

    double remain_dist; // m, deg
    double meassured_dist; // m, deg

    QString dir;

    QString result;
    QString message;

    DATA_PROFILE_MOVE()
    {
        time = 0.0;
        command = "";

        target = 0.0;
        speed = 0.0;

        remain_dist = 0.0;
        meassured_dist = 0.0;

        dir = "";

        result = "";
        message = "";
    }

    DATA_PROFILE_MOVE(const DATA_PROFILE_MOVE& p)
    {
        time = p.time;
        command = p.command;

        target = p.target;
        speed = p.speed;

        remain_dist = p.remain_dist;
        meassured_dist = p.meassured_dist;

        dir = p.dir;

        result = p.result;
        message = p.message;
    }

    DATA_PROFILE_MOVE& operator=(const DATA_PROFILE_MOVE& p)
    {
        time = p.time;
        command = p.command;

        target = p.target;
        speed = p.speed;

        remain_dist = p.remain_dist;
        meassured_dist = p.meassured_dist;

        dir = p.dir;

        result = p.result;
        message = p.message;
        return *this;
    }
};

struct DATA_PDU_UPDATE
{
    double time;
    QString command; // "updatePduParam"

    struct PARAM_ITEM
    {
        QString key;
        QString type;
        QString value;

        PARAM_ITEM() : key(""), type(""), value("") {}
        PARAM_ITEM(const QString& k, const QString& t, const QString& v)
            : key(k), type(t), value(v) {}
    };

    std::vector<PARAM_ITEM> param_list; // key, type, value 배열
    QString result;
    QString message;
    MOBILE_SETTING setting;

    DATA_PDU_UPDATE()
    {
        time = 0.0;
        command = "";

        param_list.clear();

        result = "";
        message = "";
        setting = MOBILE_SETTING();
    }

    DATA_PDU_UPDATE(const DATA_PDU_UPDATE& p)
    {
        time = p.time;
        command = p.command;

        param_list = p.param_list;

        result = p.result;
        message = p.message;
        setting = p.setting;
    }

    DATA_PDU_UPDATE& operator=(const DATA_PDU_UPDATE& p)
    {
        time = p.time;
        command = p.command;

        param_list = p.param_list;

        result = p.result;
        message = p.message;
        setting = p.setting;
        return *this;
    }
};

struct DATA_OBS_BOX
{
    double time;
    QString command; // set, get

    double obs_box_min_x;
    double obs_box_max_x;

    double obs_box_min_y;
    double obs_box_max_y;

    double obs_box_min_z;
    double obs_box_max_z;

    double obs_box_map_range;

    QString result;
    QString message;

    DATA_OBS_BOX()
    {
        time = 0.0;
        command = "";

        obs_box_min_x = 0.0;
        obs_box_max_x = 0.0;

        obs_box_min_y = 0.0;
        obs_box_max_y = 0.0;

        obs_box_min_z = 0.0;
        obs_box_max_z = 0.0;

        obs_box_map_range = 0.0;

        result = "";
        message = "";
    }

    DATA_OBS_BOX(const DATA_OBS_BOX& p)
    {
        time = p.time;
        command = p.command;

        obs_box_min_x = p.obs_box_min_x;
        obs_box_max_x = p.obs_box_max_x;

        obs_box_min_y = p.obs_box_min_y;
        obs_box_max_y = p.obs_box_max_y;

        obs_box_min_z = p.obs_box_min_z;
        obs_box_max_z = p.obs_box_max_z;

        obs_box_map_range = p.obs_box_map_range;

        result = p.result;
        message = p.message;
    }

    DATA_OBS_BOX& operator=(const DATA_OBS_BOX& p)
    {
        time = p.time;
        command = p.command;

        obs_box_min_x = p.obs_box_min_x;
        obs_box_max_x = p.obs_box_max_x;

        obs_box_min_y = p.obs_box_min_y;
        obs_box_max_y = p.obs_box_max_y;

        obs_box_min_z = p.obs_box_min_z;
        obs_box_max_z = p.obs_box_max_z;

        obs_box_map_range = p.obs_box_map_range;

        result = p.result;
        message = p.message;
        return *this;
    }
};

Q_DECLARE_METATYPE(DATA_OBS_BOX)

struct DATA_SAFETY
{
    double time;
    QString command; // set, get

    QString reset_flag; // safety reset bumper, obs ...
    int set_field;
    int get_field;

    QString result;
    QString message;

    DATA_SAFETY()
    {
        time = 0.0;
        command = "";

        reset_flag = "";
        set_field = 0;
        get_field = 0;

        result = "";
        message = "";
    }

    DATA_SAFETY(const DATA_SAFETY& p)
    {
        time = p.time;
        command = p.command;

        reset_flag = p.reset_flag;
        set_field = p.set_field;
        get_field = p.get_field;

        result = p.result;
        message = p.message;
    }

    DATA_SAFETY& operator=(const DATA_SAFETY& p)
    {
        time = p.time;
        command = p.command;

        reset_flag = p.reset_flag;
        get_field = p.get_field;
        set_field = p.set_field;

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
    QString action;
    QString direction;
    QString method;

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
        action = "";
        direction = "";
        method = "";
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
        action = p.action;
        direction = p.direction;
        method = p.method;
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
        action = p.action;
        direction = p.direction;
        method = p.method;
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
    QString id = "";
    double time;
    QString command = "";
    QString result = "";

    unsigned char mcu0_dio[8] ={0,};
    unsigned char mcu1_dio[8] ={0,};

    unsigned char mcu0_din[8] ={0,};
    unsigned char mcu1_din[8] ={0,};

    DATA_SAFTYIO()
    {
        id = "";
        time = 0.0;
        command = "";
        result = "";

        memset(mcu0_dio, 0, sizeof(mcu0_dio));
        memset(mcu1_dio, 0, sizeof(mcu1_dio));
        memset(mcu0_din, 0, sizeof(mcu0_din));
        memset(mcu1_din, 0, sizeof(mcu1_din));
    }

    DATA_SAFTYIO(const DATA_SAFTYIO& p)
    {
        id = p.id;
        time = p.time;
        command = p.command;
        result = p.result;

        memcpy(mcu0_dio, p.mcu0_dio, sizeof(char)*8);
        memcpy(mcu1_dio, p.mcu1_dio, sizeof(char)*8);
        memcpy(mcu0_din, p.mcu0_din, sizeof(char)*8);
        memcpy(mcu1_din, p.mcu1_din, sizeof(char)*8);
    }

    DATA_SAFTYIO& operator=(const DATA_SAFTYIO& p)
    {
        id = p.id;
        time = p.time;
        command = p.command;
        result = p.result;

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
        CAM_INFO,
        DATA_CONTROL
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
    DATA_CONTROL dctl;
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
Q_DECLARE_METATYPE(DATA_SAFETY)
Q_DECLARE_METATYPE(DATA_PDU_UPDATE)
Q_DECLARE_METATYPE(DATA_CONTROL)
Q_DECLARE_METATYPE(DATA_PROFILE_MOVE)


#endif // COMM_DATA_H
