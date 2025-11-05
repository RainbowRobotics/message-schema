#ifndef SYSTEM_H
#define SYSTEM_H

#include <unistd.h>
#include <dof.h>
#include <rmoduleinfo.h>

#include "common.h"
#include "rmath.h"


struct SELF_COLL_CHECK_COMBI{
    int                 ind_A;
    int                 ind_B;
};

struct ROBOT_CONFIG{
    int                 can_Ch[NO_OF_JOINT + 1];
    int                 redundancy_type;

    float               arm_coll_min;
    float               arm_coll_max;

    float               arm_max_pos_vel;
    float               arm_max_pos_acc;
    float               arm_max_rot_vel;
    float               arm_max_rot_acc;
    float               arm_max_payload;
    std::string         arm_name;

    rb_module::MCODE    modules_type[NO_OF_JOINT];
    float               modules_range_low[NO_OF_JOINT];
    float               modules_range_up[NO_OF_JOINT];
    int                 modules_axis[NO_OF_JOINT];

    double              link_mass_m[NO_OF_JOINT];
    Eigen::Vector3d     link_mass_C[NO_OF_JOINT];
    Eigen::Matrix3d     link_inerti[NO_OF_JOINT];
    
    Eigen::Vector3d     link_offset[NO_OF_JOINT];

    Eigen::Vector3d     link_capsule_dw[NO_OF_JOINT];
    Eigen::Vector3d     link_capsule_up[NO_OF_JOINT];
    float               link_capsule_radi[NO_OF_JOINT];

    Eigen::Vector3d     link_ee_offset;
    Eigen::Vector3d     link_ee_capsule_dw;
    Eigen::Vector3d     link_ee_capsule_up;
    float               link_ee_capsule_radi;

    std::vector<SELF_COLL_CHECK_COMBI>  self_coll_check_list;
    
};

struct TCP_CONFIG{
    std::string         tool_name;

    double              com_mass;
    Eigen::Vector3d     com_offset;

    Eigen::Vector3d     tcp_offset;
    Eigen::Matrix3d     tcp_rotation;

    int                 box_type;
    float               box_parameter[9];
    // 0 : Nothing
    // 1 : Sphere
    // 2 : Capsule
    // 3 : Box

    // [0] = center X
    // [1] = center Y
    // [2] = center Z
    // [3] = rotation Rx
    // [4] = rotation Ry
    // [5] = rotation Rz
        // [6] = radius
        // [6][7] = Capsule radius, height
        // [6][7][8] = Box size
    
};

struct USERF_CONFIG{
    std::string         userf_name;
    Eigen::Vector3d     userf_offset;
    Eigen::Matrix3d     userf_rotation;
};

struct AREA_CONFIG{
    std::string         area_name;
    int                 area_type;
    Eigen::Vector3d     area_offset;
    Eigen::Matrix3d     area_rotation;
    float               area_parameter[3];
    // 0 : Nothing
    // 1 : Sphere
    // 2 : Capsule
    // 3 : Box
};

namespace rb_system {
    bool initialize(std::string domain, int th_cpu);

    enum class SystemTimeScaler         {SPEEDBAR, USER, PAUSE, BREAK, COLLISION_OUT, COLLISION_SELF, NUM};
    enum class PowerOption              {NONE, Off, On};
    enum class ServoState               {NONE, PWCHECK, DEVCHECK, PARACHECK, MOTORCHECK, TORQUECHECK, DONE};

    struct TimeScaler{
        float  input;
        double output;        
        double lpf_alpha;
    };

    std::tuple<std::string, std::string, std::string, std::string> Get_System_Basic_Info();

    
    // Robot Parameter
    ROBOT_CONFIG    Get_CurrentRobotParameter();
    
    // ---------------------------------------------------------------
    // JOINT
    // ---------------------------------------------------------------
    VectorJd        Get_Motor_Limit_Ang_Low();
    VectorJd        Get_Motor_Limit_Ang_Up();
    VectorJd        Get_Motor_Limit_Vel();
    VectorJd        Get_Motor_Reco_Vel();
    VectorJd        Get_Motor_Reco_Acc();
    VectorJd        Get_Motor_HwMax_Vel();
    VectorJd        Get_Motor_HwMax_Acc();

    std::array<float, NO_OF_JOINT>      Get_Motor_Encoder();
    std::array<float, NO_OF_JOINT>      Get_Torque(int option);
    std::array<float, NO_OF_JOINT>      Get_Temperature(int option);

    // ---------------------------------------------------------------
    // USER FRAME
    // ---------------------------------------------------------------
    int             Change_UserFrame_Number(int u_num);
    int             Get_CurrentUserFrameNumber();
    USERF_CONFIG    Get_CurrentUserFrameParameter();
    USERF_CONFIG    Get_DesiredUserFrameParameter(unsigned int u_num);
    int             Save_UserFrameParameter(unsigned int u_num, USERF_CONFIG u_conf);
    // ---------------------------------------------------------------
    // AREA
    // ---------------------------------------------------------------
    AREA_CONFIG     Get_DesiredAreaParameter(unsigned int a_num);
    int             Save_AreaParameter(unsigned int a_num, AREA_CONFIG a_conf);

    // ---------------------------------------------------------------
    // TCP 
    // ---------------------------------------------------------------
    int             Change_Tool_Number(int t_num);
    int             Get_CurrentTcpNumber();
    TCP_CONFIG      Get_CurrentTcpParameter();
    TCP_CONFIG      Get_DesiredTcpParameter(int t_num);
    int             Save_TcpParameter(unsigned int t_num, TCP_CONFIG t_conf);
    // ---------------------------------------------------------------
    // Collision 
    // ---------------------------------------------------------------
    int                             Call_Reset_Out_Coll();
    int                             Set_Out_Coll_Para(int onoff, int react, float th);
    std::tuple<int, int, float>     Get_Out_Coll_Para();
    int                             Save_Out_Coll_Para(int onoff, int react, float th);

    int                             Set_Self_Coll_Para(int mode, float dist_int, float dist_ext);
    std::tuple<int, float, float>   Get_Self_Coll_Para();
    int                             Save_Self_Coll_Para(int mode, float dist_int, float dist_ext);

    std::tuple<int, float, float, float>    Get_Gravity_Para();
    int                                     Save_Gravity_Para(int mode, float gx, float gy, float gz);
    Eigen::Vector3d                                Get_CurrentGravityParameter();

    std::array<float, NO_OF_JOINT>  Get_Direct_Teaching_Sensitivity();
    int                             Save_Direct_Teaching_Sensitivity(std::array<float, NO_OF_JOINT> f_targets);
    
    // ---------------------------------------------------------------
    // MoveFlow
    // ---------------------------------------------------------------
    void            Set_MoveSpeedBar(double alpha);
    double          Get_MoveSpeedBar();
    
    void            Set_MoveUserSpeedBar(double alpha);
    double          Get_MoveUserSpeedBar();

    int             Call_MovePause();
    int             Call_MoveResume();
    int             Get_MovePauseState();

    void            Call_MoveBreak(double t_time);
    void            Reset_MoveBreak();

    // ---------------------------------------------------------------
    // IO
    // ---------------------------------------------------------------
    std::array<uint8_t, NO_OF_DOUT>     Get_Box_Dout();
    std::array<uint8_t, NO_OF_DIN>      Get_Box_Din();
    std::array<float, NO_OF_AOUT>       Get_Box_Aout();
    std::array<float, NO_OF_AIN>        Get_Box_Ain();

    std::array<uint8_t, NO_OF_DOUT>     Get_EX_Dout();
    std::array<uint8_t, NO_OF_DIN>      Get_EX_Din();
    std::array<float, NO_OF_AOUT>       Get_EX_Aout();
    std::array<float, NO_OF_AIN>        Get_EX_Ain();
    
    std::array<uint8_t, NO_OF_DOUT>     Get_Box_Special_Dout();
    int                                 Save_Box_Special_Dout(unsigned int p_num, unsigned int func_no);

    std::array<uint8_t, NO_OF_DIN>      Get_Box_Special_Din();
    int                                 Save_Box_Special_Din(unsigned int p_num, unsigned int func_no);

    std::array<uint8_t, NO_OF_DIN>      Get_Box_FilterCount_Din();
    int                                 Save_Box_FilterCount_Din(unsigned int p_num, unsigned int f_cnt);

    int                                 Set_Digital_Output(unsigned int p_num, unsigned int value);
    int                                 Set_Analog_Output(unsigned int p_num, float value);
    

    // ---------------------------------------------------------------
    // Power & ServoOn
    // ---------------------------------------------------------------
    int             Set_Servo(int option, int who_issue);
    ServoState      Get_Servo();
    int             Set_Power(PowerOption opt, bool is_call_from_RT);
    bool            Get_Power();
    int             Get_Power_Switch();
    int             Get_Lan2Can_State();
    void            Notify_PowerChanged();

    // ---------------------------------------------------------------
    // Reference
    // ---------------------------------------------------------------
    int             Set_ReferenceOnOff(bool opt);
    bool            Get_ReferenceOnOff();
    double          Set_Output_LPF(double t_alpha);

    // ---------------------------------------------------------------
    // State / Flags
    // ---------------------------------------------------------------

    bool            Get_Is_Idle();
    int             Get_Flag_Direct_Teaching();
    int             Get_Flag_Out_Collision_Occur();
    int             Get_Flag_Self_Collision_Occur();
    uint8_t         Get_Flag_Heart_Beat();

    // ---------------------------------------------------------------
    // RT
    // ---------------------------------------------------------------
    void            Task_RealTime();
}

#endif // SYSTEM_H
