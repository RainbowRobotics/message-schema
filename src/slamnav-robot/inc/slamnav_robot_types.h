//
// Created by jy on 25. 12. 27..
//

#ifndef SLAMNAV2_SLAMNAV_ROBOT_TYPES_H
#define SLAMNAV2_SLAMNAV_ROBOT_TYPES_H

#include <cstdint>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

struct MOBILE_SETTING
{
    // for safety setting
    unsigned int version = 0;
    unsigned char robot_type = 0;

    float v_limit =.0;
    float w_limit =.0;
    float a_limit =.0;
    float b_limit =.0;

    float v_limit_jog = .0;
    float w_limit_jog = .0;
    float a_limit_jog = .0;
    float b_limit_jog = .0;

    float v_limit_monitor = .0;
    float w_limit_monitor = .0;

    float safety_v_limit =.0;
    float safety_w_limit =.0;

    float w_s = .0;
    float w_r = .0;
    float gear = .0;
    float dir = .0;

    float lx = .0;
    float ly = .0;

    unsigned char robot_wheel_type = 0;
    unsigned char use_interlock_commnad_bypass = 0;
    unsigned char use_safety_obstacle_detection = 0;
    unsigned char use_safety_bumper = 0;
    unsigned char use_safety_interlock = 0;
    unsigned char use_safety_cross_monitor = 0;
    unsigned char use_safety_speed_control = 0;
    unsigned char use_sw_io = 0;

    unsigned char d_out[16] = {0,};
};


struct MOBILE_STATUS
{
    // for timesync
    double t = 0;
    float  return_time = 0;
    uint32_t recv_tick = 0;

    /* motor status */

    // motor connetion
    uint8_t connection_m0 = 0;
    uint8_t connection_m1 = 0;
    uint8_t connection_m2 = 0;
    uint8_t connection_m3 = 0;

    // motor status
    uint8_t status_m0 = 0;
    uint8_t status_m1 = 0;
    uint8_t status_m2 = 0;
    uint8_t status_m3 = 0;

    // motor temperature (using inlier sensor)
    uint8_t temp_m0 = 0;
    uint8_t temp_m1 = 0;
    uint8_t temp_m2 = 0;
    uint8_t temp_m3 = 0;

    // motor temperature (estimation)
    uint8_t esti_temp_m0 = 0;
    uint8_t esti_temp_m1 = 0;
    uint8_t esti_temp_m2 = 0;
    uint8_t esti_temp_m3 = 0;

    // motor core temperature (estimation)
    float core_temp0 = 0;
    float core_temp1 = 0;
    uint8_t state = 0;      // S100 state

    // motor current Amphere
    uint8_t cur_m0 = 0;
    uint8_t cur_m1 = 0;
    uint8_t cur_m2 = 0;
    uint8_t cur_m3 = 0;

    // PDU default state
    uint8_t charge_state     = 0;
    uint8_t power_state      = 0;
    uint8_t motor_stop_state = 0;
    uint8_t remote_state     = 0;

    // battery
    float power = 0;
    float bat_in = 0;
    float bat_out = 0;
    float bat_current = 0;
    float bat_voltage = 0;
    float total_power = 0;
    uint8_t bat_percent = 0;

    // lift (extra module)
    float lift_voltage_in = 0.;
    float lift_voltage_out = 0.;
    float lift_current = 0.;

    // docking charging
    float charge_current  = 0;  // current charging Amphere
    float contact_voltage = 0; // current charging voltage (docking)

    // imu status
    float imu_gyr_x = 0;
    float imu_gyr_y = 0;
    float imu_gyr_z = 0;
    float imu_acc_x = 0;
    float imu_acc_y = 0;
    float imu_acc_z = 0;

    // inter lock
    uint8_t inter_lock_state = 0;

    // for safety
    uint8_t sw_stop  = 0;
    uint8_t sw_reset = 0;
    uint8_t sw_start = 0;
    uint8_t om_state = 0;
    uint8_t ri_state = 0;
    uint8_t lidar_field;
    uint8_t bumper_state     = 0;
    uint8_t auto_manual_sw   = 0;
    uint8_t brake_release_sw = 0;


    uint8_t safety_state_emo_pressed_1          = 0;
    uint8_t safety_state_ref_meas_mismatch_1    = 0;
    uint8_t safety_state_over_speed_1           = 0;
    uint8_t safety_state_obstacle_detected_1    = 0;
    uint8_t safety_state_speed_field_mismatch_1 = 0;
    uint8_t safety_state_interlock_stop_1       = 0;
    uint8_t safety_state_bumper_stop_1          = 0;
    uint8_t operational_stop_state_flag_1       = 0;

    uint8_t safety_state_emo_pressed_2          = 0;
    uint8_t safety_state_ref_meas_mismatch_2    = 0;
    uint8_t safety_state_over_speed_2           = 0;
    uint8_t safety_state_obstacle_detected_2    = 0;
    uint8_t safety_state_speed_field_mismatch_2 = 0;
    uint8_t safety_state_interlock_stop_2       = 0;
    uint8_t safety_state_bumper_stop_2          = 0;
    uint8_t operational_stop_state_flag_2       = 0;

    short ref_dps[2]  = {0,0};
    short meas_dps[2] = {0,0};

    unsigned char mcu0_dio[8] ={0,};
    unsigned char mcu1_dio[8] ={0,};

    unsigned char mcu0_din[8] ={0,};
    unsigned char mcu1_din[8] ={0,};

    unsigned char adc_value[4] ={0,};
    unsigned char dac_value[4] ={0,};

    float tabos_rc           = 0.f; // remain capacity -ah
    float tabos_ae           = 0.f; // availiable energy -wh
    float tabos_voltage      = 0.f; // v
    float tabos_current      = 0.f; // a
    float tabos_temperature  = 0.;  // battery temperature-c
    uint16_t tabos_status    = 0;
    unsigned short tabos_ttf = 0;   // time to full-min
    unsigned short tabos_tte = 0;   // time to empty-min
    unsigned char tabos_soc  = 0;   // state of charge-%
    unsigned char tabos_soh  = 0;   // state of health-%

    uint8_t xnergy_main_state = 0;
    uint32_t xnergy_error_code_low = 0;
    uint32_t xnergy_error_code_high = 0;

    uint8_t bms_type         = 0;

    float res_linear_dist = 0.0;
    float res_linear_remain_dist = 0.0;

    uint8_t sss_recovery_state = 0;
};

struct MOBILE_RESPONSE
{
    float linear_x_meassured_dist = 0.0;
    float linear_x_remain_dist = 0.0;

    float linear_w_meassured_dist = 0.0;
    float linear_w_remain_dist = 0.0;

    float circular_w_meassured_dist = 0.0;
    float circular_w_remain_dist = 0.0;

    MOBILE_RESPONSE()
    {
        linear_x_remain_dist = 0.0;
        linear_x_meassured_dist = 0.0;

        linear_w_remain_dist = 0.0;
        linear_w_meassured_dist = 0.0;

        circular_w_remain_dist = 0.0;
        circular_w_meassured_dist = 0.0;
    }

    MOBILE_RESPONSE(const MOBILE_RESPONSE& p)
    {
        linear_x_remain_dist = p.linear_x_remain_dist;
        linear_x_meassured_dist = p.linear_x_meassured_dist;

        linear_w_remain_dist = p.linear_w_remain_dist;
        linear_w_meassured_dist = p.linear_w_meassured_dist;

        circular_w_remain_dist = p.circular_w_remain_dist;
        circular_w_meassured_dist = p.circular_w_meassured_dist;
    }

    MOBILE_RESPONSE& operator=(const MOBILE_RESPONSE& p)
    {
        linear_x_remain_dist = p.linear_x_remain_dist;
        linear_x_meassured_dist = p.linear_x_meassured_dist;

        linear_w_remain_dist = p.linear_w_remain_dist;
        linear_w_meassured_dist = p.linear_w_meassured_dist;

        circular_w_remain_dist = p.circular_w_remain_dist;
        circular_w_meassured_dist = p.circular_w_meassured_dist;

        return *this;
    }
};

struct MOBILE_POSE
{
    double t;

    Eigen::Vector3d pose; // global pose(x, y, rz)
    Eigen::Vector3d vel; // local vel(vx, vy, wz)

    MOBILE_POSE()
    {
        t = 0;
        pose.setZero();
        vel.setZero();
    }

    MOBILE_POSE(const MOBILE_POSE& p)
    {
        t = p.t;
        pose = p.pose;
        vel = p.vel;
    }

    MOBILE_POSE& operator=(const MOBILE_POSE& p)
    {
        t = p.t;
        pose = p.pose;
        vel = p.vel;
        return *this;
    }
};

struct MOBILE_IMU
{
    double t = 0;
    double acc_x = 0;
    double acc_y = 0;
    double acc_z = 0;
    double gyr_x = 0;
    double gyr_y = 0;
    double gyr_z = 0;
    double rx = 0;
    double ry = 0;
    double rz = 0;
};

#endif //SLAMNAV2_SLAMNAV_ROBOT_TYPES_H