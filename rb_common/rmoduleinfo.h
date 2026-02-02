#ifndef RMODULEINFO_H
#define RMODULEINFO_H


namespace rb_module {
    enum class MCODE
    {
        M400 = 400,
        M250 = 250,
        M200 = 200,
        M140 = 140,

        M250S = 255,
        M200S = 205,
        M140S = 145,
        MFHB = 487,
    };

    struct Module_Info{
        int code_num;
        float encoder_resolution;
        
        float gear_ratio;
        float gear_stiffness;

        float torque_Constant;
        float torque_Max_Rept;
        float torque_Max_Moment;
        
        float max_Vel;
        float max_ACC;
        float max_Cur_mA;
        
        float rotor_i;
        float rotor_a;
        float rotor_b;

        int   shake_pulse;
        int   shake_count;

        float resi_R_q;
        
        float temperature_esti[2];

        float current_scaler;//1 digit = ? mA
    };

    Module_Info Get_Module_Info(MCODE t_mcdoe);
}
#endif // RMODULEINFO_H