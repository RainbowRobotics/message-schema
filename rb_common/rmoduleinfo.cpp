#include "rmoduleinfo.h"

namespace rb_module {
    Module_Info Get_Module_Info(MCODE t_mcdoe){
        Module_Info ret;
        switch(t_mcdoe){
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
        }
        return ret;
    }
}