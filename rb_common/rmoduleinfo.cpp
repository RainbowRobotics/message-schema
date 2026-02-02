#include "rmoduleinfo.h"

namespace rb_module {
    Module_Info Get_Module_Info(MCODE t_mcdoe){
        Module_Info ret;
        switch(t_mcdoe){
            case MCODE::M400:
            {
                ret.code_num = (int)t_mcdoe;
                ret.encoder_resolution = 16384.;
                ret.gear_ratio = 101.;
                ret.gear_stiffness = 1./(20.00 * 10000.) * 57.;
                ret.torque_Constant = 0.15;
                ret.torque_Max_Rept = 738.;
                ret.torque_Max_Moment = 1400.;
                ret.max_Vel = 121.;
                ret.max_ACC = 450.;
                ret.max_Cur_mA = 100000.;
                ret.rotor_i = 1500.;
                ret.rotor_a = 4500.;
                ret.rotor_b = 3000.;
                ret.shake_pulse = 1000;
                ret.shake_count = 200;
                ret.temperature_esti[0] = 34.560;
                ret.temperature_esti[1] = 7720.879;
                ret.resi_R_q = 0.0165;
                ret.current_scaler = 10.;
                break;
            }
            case MCODE::M250:
            {
                ret.code_num = (int)t_mcdoe;
                ret.encoder_resolution = 16384.;
                ret.gear_ratio = 101.;
                ret.gear_stiffness = 1./(5.00 * 10000.) * 57.;
                ret.torque_Constant = 0.1;
                ret.torque_Max_Rept = 204.;
                ret.torque_Max_Moment = 369.;
                ret.max_Vel = 181.;
                ret.max_ACC = 450.;
                ret.max_Cur_mA = 23000.;
                ret.rotor_i = 250.;
                ret.rotor_a = 950.;
                ret.rotor_b = 800.;
                ret.shake_pulse = 800;
                ret.shake_count = 160;
                ret.temperature_esti[0] = 2.0791;
                ret.temperature_esti[1] = 165.4974;
                ret.resi_R_q = 0.165;
                ret.current_scaler = 1.;
                break;
            }
            case MCODE::M200:
            {
                ret.code_num = (int)t_mcdoe;
                ret.encoder_resolution = 16384.;
                ret.gear_ratio = 101.;
                ret.gear_stiffness = 1./(2.50 * 10000.) * 57.;
                ret.torque_Constant = 0.1;
                ret.torque_Max_Rept = 107.;
                ret.torque_Max_Moment = 191.;
                ret.max_Vel = 181.;
                ret.max_ACC = 450.;
                ret.max_Cur_mA = 18000.;
                ret.rotor_i = 130.;
                ret.rotor_a = 850.;
                ret.rotor_b = 500.;
                ret.shake_pulse = 1000;
                ret.shake_count = 140;
                ret.temperature_esti[0] = 1.5655;
                ret.temperature_esti[1] = 81.3997;
                ret.resi_R_q = 0.240;
                ret.current_scaler = 1.;
                break;
            }
            case MCODE::M140:
            {
                ret.code_num = (int)t_mcdoe;
                ret.encoder_resolution = 16384.;
                ret.gear_ratio = 101.;
                ret.gear_stiffness = 1./(0.61 * 10000.) * 57.;
                ret.torque_Constant = 0.09;
                ret.torque_Max_Rept = 36.;
                ret.torque_Max_Moment = 70.;
                ret.max_Vel = 181.;
                ret.max_ACC = 450.;
                ret.max_Cur_mA = 9000.;
                ret.rotor_i = 36.;
                ret.rotor_a = 280.;
                ret.rotor_b = 290.;
                ret.shake_pulse = 600;
                ret.shake_count = 120;
                ret.temperature_esti[0] = 0.2193;
                ret.temperature_esti[1] = 13.2591;
                ret.resi_R_q = 0.825;
                ret.current_scaler = 1.;
                break;
            }

            case MCODE::M250S:
            {
                ret.code_num = (int)t_mcdoe;
                ret.encoder_resolution = 16384.;
                ret.gear_ratio = 101.;
                ret.gear_stiffness = 1./(5.00 * 10000.) * 57.;
                ret.torque_Constant = 0.06;//<----------------------------
                ret.torque_Max_Rept = 204.;
                ret.torque_Max_Moment = 369.;
                ret.max_Vel = 275.;//<----------------------------
                ret.max_ACC = 450.;
                ret.max_Cur_mA = 46000.;//<----------------------------
                ret.rotor_i = 250.;
                ret.rotor_a = 950.;
                ret.rotor_b = 800.;
                ret.shake_pulse = 800;
                ret.shake_count = 160;
                ret.temperature_esti[0] = 5;//<----------------------------
                ret.temperature_esti[1] = 165.4974;
                ret.resi_R_q = 0.08;//<----------------------------
                ret.current_scaler = 1.;
                break;
            }
            case MCODE::M200S:
            {
                ret.code_num = (int)t_mcdoe;
                ret.encoder_resolution = 16384.;
                ret.gear_ratio = 101.;
                ret.gear_stiffness = 1./(2.50 * 10000.) * 57.;
                ret.torque_Constant = 0.06;//<----------------------------
                ret.torque_Max_Rept = 107.;
                ret.torque_Max_Moment = 191.;
                ret.max_Vel = 275.;//<----------------------------
                ret.max_ACC = 450.;
                ret.max_Cur_mA = 30000.;//<----------------------------
                ret.rotor_i = 130.;
                ret.rotor_a = 850.;
                ret.rotor_b = 500.;
                ret.shake_pulse = 1000;
                ret.shake_count = 140;
                ret.temperature_esti[0] = 3;//<----------------------------
                ret.temperature_esti[1] = 81.3997;
                ret.resi_R_q = 0.120;//<----------------------------
                ret.current_scaler = 1.;
                break;
            }
            case MCODE::M140S:
            {
                ret.code_num = (int)t_mcdoe;
                ret.encoder_resolution = 16384.;
                ret.gear_ratio = 101.;
                ret.gear_stiffness = 1./(0.61 * 10000.) * 57.;
                ret.torque_Constant = 0.05;;//<----------------------------
                ret.torque_Max_Rept = 36.;
                ret.torque_Max_Moment = 70.;
                ret.max_Vel = 365.;;//<----------------------------
                ret.max_ACC = 450.;
                ret.max_Cur_mA = 12000.;;//<----------------------------
                ret.rotor_i = 36.;
                ret.rotor_a = 280.;
                ret.rotor_b = 290.;
                ret.shake_pulse = 600;
                ret.shake_count = 120;
                ret.temperature_esti[0] = 0.5;;//<----------------------------
                ret.temperature_esti[1] = 13.2591;
                ret.resi_R_q = 0.5;;//<----------------------------
                ret.current_scaler = 1.;
                break;
            }
            case MCODE::MFHB:
            {
                ret.code_num = (int)t_mcdoe;
                ret.encoder_resolution = 4096.;
                ret.gear_ratio = 202.;
                ret.gear_stiffness = 1./(0.61 * 10000.) * 57.;
                ret.torque_Constant = 0.05;;//<----------------------------
                ret.torque_Max_Rept = 36.;
                ret.torque_Max_Moment = 70.;
                ret.max_Vel = 365.;;//<----------------------------
                ret.max_ACC = 450.;
                ret.max_Cur_mA = 12000.;;//<----------------------------
                ret.rotor_i = 36.;
                ret.rotor_a = 280.;
                ret.rotor_b = 290.;
                ret.shake_pulse = 600;
                ret.shake_count = 120;
                ret.temperature_esti[0] = 0.5;;//<----------------------------
                ret.temperature_esti[1] = 13.2591;
                ret.resi_R_q = 0.5;;//<----------------------------
                ret.current_scaler = 1.;
                break;
            }
        }
        return ret;
    }
}