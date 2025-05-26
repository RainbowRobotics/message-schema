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

    QString escape_dir; // BL, BR, FL, FR, OR, OF, OB

    double remaining_dist;
    double eta; // estimated time of arrival
    int bat_percent; // battery percentage

    QString result;
    QString message;

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

        escape_dir = "";

        remaining_dist = 0.0;
        eta = 9999.0;
        bat_percent = -1;

        result = "";
        message = "";
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
        escape_dir = p.escape_dir;

        eta = p.eta;
        bat_percent = p.bat_percent;

        result = p.result;
        message = p.message;
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
        escape_dir = p.escape_dir;

        eta = p.eta;
        bat_percent = p.bat_percent;

        result = p.result;
        message = p.message;
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
    double time;
    QString command; // path
    QString path;
    QString vobs_robots;
    QString vobs_closures;
    int preset;

    QString response;
    QString result;
    QString message;

    DATA_PATH()
    {
        time = 0.0;
        command = "";
        path = "";
        vobs_robots = "";
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
        vobs_robots = p.vobs_robots;
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
        vobs_robots = p.vobs_robots;
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

#endif // COMM_DATA_H
