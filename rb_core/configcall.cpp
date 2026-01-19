#define P_NAME  "CONFIG"

#include <Eigen/Dense>

#include "configcall.h"
#include "sqlite_function.h"
#include "common.h"
#include "system.h"


namespace rb_config {
    namespace {
        RainbowDataBase core_DB;

        int Call_Value_I(DB_ADDR_LIST addr, int default_){
            int read_value = default_;
            std::string coll_name = std::string(DB_MAP[(int)addr].name);
            RBDB_RET t_rd_ret = core_DB.RBDB_Read_I(coll_name);
            if(t_rd_ret.is_success == 1){
                read_value = t_rd_ret.ret_val_i;
            }else{
                if(core_DB.RBDB_Write_I(coll_name, default_) != 0){
                    LOG_WARNING("DB RESET : " + coll_name);
                }
            }
            return read_value;
        }
        float Call_Value_F(DB_ADDR_LIST addr, float default_){
            float read_value = default_;
            std::string coll_name = std::string(DB_MAP[(int)addr].name);
            RBDB_RET t_rd_ret = core_DB.RBDB_Read_F(coll_name);
            if(t_rd_ret.is_success == 1){
                read_value = t_rd_ret.ret_val_f;
            }else{
                if(core_DB.RBDB_Write_F(coll_name, default_) != 0){
                    LOG_WARNING("DB RESET : " + coll_name);
                }
            }
            return read_value;
        }
        std::string Call_Value_S(DB_ADDR_LIST addr, std::string default_){
            std::string read_value = default_;
            std::string coll_name = std::string(DB_MAP[(int)addr].name);
            RBDB_RET t_rd_ret = core_DB.RBDB_Read_S(coll_name);
            if(t_rd_ret.is_success == 1){
                read_value = t_rd_ret.ret_str;
            }else{
                if(core_DB.RBDB_Write_S(coll_name, default_) != 0){
                    LOG_WARNING("DB RESET : " + coll_name);
                }
            }
            return read_value;
        }

        bool Write_Value_I(DB_ADDR_LIST addr, int target){
            std::string coll_name = std::string(DB_MAP[(int)addr].name);
            int ret = core_DB.RBDB_Write_I(coll_name, target);
            if(ret == 1){
                LOG_ERROR("DB WRITE FAIL INIT" + coll_name);
                return false;
            }else if(ret == 2){
                LOG_ERROR("DB WRITE FAIL QUERY" + coll_name);
                return false;
            }else if(ret == 0){
                return true;
            }else{
                LOG_ERROR("DB WRITE FAIL OTHERS" + coll_name);
                return false;
            }
        }
        bool Write_Value_F(DB_ADDR_LIST addr, float target){
            std::string coll_name = std::string(DB_MAP[(int)addr].name);
            int ret = core_DB.RBDB_Write_F(coll_name, target);
            if(ret == 1){
                LOG_ERROR("DB WRITE FAIL INIT" + coll_name);
                return false;
            }else if(ret == 2){
                LOG_ERROR("DB WRITE FAIL QUERY" + coll_name);
                return false;
            }else if(ret == 0){
                return true;
            }else{
                LOG_ERROR("DB WRITE FAIL OTHERS" + coll_name);
                return false;
            }
        }
        bool Write_Value_S(DB_ADDR_LIST addr, std::string target){
            std::string coll_name = std::string(DB_MAP[(int)addr].name);
            int ret = core_DB.RBDB_Write_S(coll_name, target);
            if(ret == 1){
                LOG_ERROR("DB WRITE FAIL INIT" + coll_name);
                return false;
            }else if(ret == 2){
                LOG_ERROR("DB WRITE FAIL QUERY" + coll_name);
                return false;
            }else if(ret == 0){
                return true;
            }else{
                LOG_ERROR("DB WRITE FAIL OTHERS" + coll_name);
                return false;
            }
        }
    }

    bool initialize(std::string db_name){
        core_DB.RBDB_Init(db_name);
        std::cout<<"DB Item Num: "<<core_DB.RBDB_Read_LineNum().ret_val_i<<std::endl;

        if(!core_DB.RBDB_IsWorking()){
            return false;
        }

        return true;
    }

    int READ_System_Gate_IP(int octet){
        ;// octet : 0 ~ 3
        if(octet == 0){
            return Call_Value_I(DB_ADDR_LIST::SYS_GATE_IP_0, 10);
        }else if(octet == 1){
            return Call_Value_I(DB_ADDR_LIST::SYS_GATE_IP_1, 0);
        }else if(octet == 2){
            return Call_Value_I(DB_ADDR_LIST::SYS_GATE_IP_2, 1);
        }else if(octet == 3){
            return Call_Value_I(DB_ADDR_LIST::SYS_GATE_IP_3, 1);
        }else{
            LOG_ERROR("Invalid IP Octet Number");
            return 0;
        }
    }
    int READ_System_Gate_Port(){
        return Call_Value_I(DB_ADDR_LIST::SYS_GATE_PORT, 1977);
    } 

    int READ_System_Port_Number(int port_type){
        // port_type
        // 0 : MBUS
        // 1 : SOCKCMD
        // 2 : SOCKDATA
        if(port_type == 0){
            return Call_Value_I(DB_ADDR_LIST::SYS_PORT_NO_MBUS, 502);
        }else if(port_type == 1){
            return Call_Value_I(DB_ADDR_LIST::SYS_PORT_NO_SOCKCMD, 5000);
        }else if(port_type == 2){
            return Call_Value_I(DB_ADDR_LIST::SYS_PORT_NO_SOCKDATA, 5001);
        }else{
            LOG_ERROR("Invalid Port Type Number");
            return 60000;
        }
    }

    ROBOT_CONFIG READ_Robot_Parameter(int r_code){
        bool valid_number = false;
        ROBOT_CONFIG ret;
        ret.self_coll_check_list.clear();
        
        if(r_code == 3001200){
            valid_number = true;
            ret.arm_name = "CB3-1200E";
            ret.arm_max_payload = 3;
            ret.redundancy_type = -1;

            ret.arm_max_pos_vel = 1000;//mm/s
            ret.arm_max_pos_acc = 20000;
            ret.arm_max_rot_vel = 360;//deg/s
            ret.arm_max_rot_acc = 720;
            

            ret.arm_coll_max = 10;
            ret.arm_coll_min = 0;

            ret.can_Ch[0] = ret.can_Ch[1] = ret.can_Ch[2] = 1;
            ret.can_Ch[3] = ret.can_Ch[4] = ret.can_Ch[5] = ret.can_Ch[6] = 0;

            ret.modules_axis[0] = 2;
            ret.modules_axis[1] = 1;
            ret.modules_axis[2] = 1;
            ret.modules_axis[3] = 1;
            ret.modules_axis[4] = 2;
            ret.modules_axis[5] = 1;

            ret.modules_type[0] = ret.modules_type[1] = ret.modules_type[2] =rb_module::MCODE::M250;
            ret.modules_type[3] = ret.modules_type[4] = ret.modules_type[5] =rb_module::MCODE::M140;

            ret.modules_range_low[0] = ret.modules_range_low[1] = ret.modules_range_low[2]
            = ret.modules_range_low[3] = ret.modules_range_low[4] = ret.modules_range_low[5] = -360;

            ret.modules_range_up[0] = ret.modules_range_up[1] = ret.modules_range_up[2]
            = ret.modules_range_up[3] = ret.modules_range_up[4] = ret.modules_range_up[5] = +360;



            // link_offset : to motor center (mm)
            // link_mass_C : from motor center -> that motor's link com (mm)

            ret.link_ee_offset = Eigen::Vector3d(0, -96.7, 0);

            ret.link_mass_m[0] = 4.1;
            ret.link_mass_C[0] = Eigen::Vector3d(0, -4, -40);
            ret.link_offset[0] = Eigen::Vector3d(0, 0, 169.2);
            ret.link_rotation[0] = Eigen::Matrix3d::Identity();

            ret.link_mass_m[1] = 9.6;
            ret.link_mass_C[1] = Eigen::Vector3d(0, 30, 283);
            ret.link_offset[1] = Eigen::Vector3d(0, -148.4, 0);
            ret.link_rotation[1] = Eigen::Matrix3d::Identity();

            ret.link_mass_m[2] = 3.4;
            ret.link_mass_C[2] = Eigen::Vector3d(0, 130, 240);
            ret.link_offset[2] = Eigen::Vector3d(0, 0, 566.9);
            ret.link_rotation[2] = Eigen::Matrix3d::Identity();

            ret.link_mass_m[3] = 1.3;
            ret.link_mass_C[3] = Eigen::Vector3d(0, -100, 30);
            ret.link_offset[3] = Eigen::Vector3d(0, 148.4, 522.4);
            ret.link_rotation[3] = Eigen::Matrix3d::Identity();

            ret.link_mass_m[4] = 1.3;
            ret.link_mass_C[4] = Eigen::Vector3d(0, -30, 100);
            ret.link_offset[4] = Eigen::Vector3d(0, -110.7, 0);
            ret.link_rotation[4] = Eigen::Matrix3d::Identity();

            ret.link_mass_m[5] = 0.45;
            ret.link_mass_C[5] = Eigen::Vector3d(0, -75, 0);
            ret.link_offset[5] = Eigen::Vector3d(0, 0, 110.7);
            ret.link_rotation[5] = Eigen::Matrix3d::Identity();

            int k;
            k = 0;
            ret.link_inerti[k](0, 0) = 22109;       ret.link_inerti[k](0, 1) = 0;           ret.link_inerti[k](0, 2) = 0;
            ret.link_inerti[k](1, 0) = 0;           ret.link_inerti[k](1, 1) = 21201;       ret.link_inerti[k](1, 2) = -2548;
            ret.link_inerti[k](2, 0) = 0;           ret.link_inerti[k](2, 1) = -2548;       ret.link_inerti[k](2, 2) = 12767;
            k = 1;
            ret.link_inerti[k](0, 0) = 705727;      ret.link_inerti[k](0, 1) = 0;           ret.link_inerti[k](0, 2) = 0;
            ret.link_inerti[k](1, 0) = 0;           ret.link_inerti[k](1, 1) = 702933;      ret.link_inerti[k](1, 2) = 0;
            ret.link_inerti[k](2, 0) = 0;           ret.link_inerti[k](2, 1) = 0;           ret.link_inerti[k](2, 2) = 20402;
            k = 2;
            ret.link_inerti[k](0, 0) = 217429;      ret.link_inerti[k](0, 1) = 0;           ret.link_inerti[k](0, 2) = 0;
            ret.link_inerti[k](1, 0) = 0;           ret.link_inerti[k](1, 1) = 216994;      ret.link_inerti[k](1, 2) = -7154;
            ret.link_inerti[k](2, 0) = 0;           ret.link_inerti[k](2, 1) = -7154;       ret.link_inerti[k](2, 2) = 7274;
            k = 3;
            ret.link_inerti[k](0, 0) = 1337;        ret.link_inerti[k](0, 1) = 0;           ret.link_inerti[k](0, 2) = 0;
            ret.link_inerti[k](1, 0) = 0;           ret.link_inerti[k](1, 1) = 1299;        ret.link_inerti[k](1, 2) = -142;
            ret.link_inerti[k](2, 0) = 0;           ret.link_inerti[k](2, 1) = -142;        ret.link_inerti[k](2, 2) = 1076;
            k = 4;
            ret.link_inerti[k](0, 0) = 1337;        ret.link_inerti[k](0, 1) = 0;           ret.link_inerti[k](0, 2) = 0;
            ret.link_inerti[k](1, 0) = 0;           ret.link_inerti[k](1, 1) = 1076;        ret.link_inerti[k](1, 2) = -142;
            ret.link_inerti[k](2, 0) = 0;           ret.link_inerti[k](2, 1) = -142;        ret.link_inerti[k](2, 2) = 1299;
            k = 5;
            ret.link_inerti[k](0, 0) = 124;         ret.link_inerti[k](0, 1) = 0;           ret.link_inerti[k](0, 2) = 0;
            ret.link_inerti[k](1, 0) = 0;           ret.link_inerti[k](1, 1) = 213;         ret.link_inerti[k](1, 2) = 0;
            ret.link_inerti[k](2, 0) = 0;           ret.link_inerti[k](2, 1) = 0;           ret.link_inerti[k](2, 2) = 128;

        }else if(r_code == 500920){//7DOF
            #if NO_OF_JOINT == 7
            valid_number = true;
            ret.arm_name = "SB5-920E";
            ret.arm_max_payload = 5;
            ret.redundancy_type = 2;

            ret.arm_max_pos_vel = 1000;//mm/s
            ret.arm_max_pos_acc = 20000;
            ret.arm_max_rot_vel = 360;//deg/s
            ret.arm_max_rot_acc = 720;
            

            ret.arm_coll_max = 10;
            ret.arm_coll_min = 0;

            ret.can_Ch[0] = ret.can_Ch[1] = ret.can_Ch[2] = ret.can_Ch[3] = 1;
            ret.can_Ch[4] = ret.can_Ch[5] = ret.can_Ch[6] = ret.can_Ch[7] = 0;

            ret.modules_axis[0] = 2;
            ret.modules_axis[1] = 1;
            ret.modules_axis[2] = 2;
            ret.modules_axis[3] = 1;
            ret.modules_axis[4] = 1;
            ret.modules_axis[5] = 2;
            ret.modules_axis[6] = 1;

            ret.modules_type[0] = ret.modules_type[1] = ret.modules_type[2] = ret.modules_type[3] =rb_module::MCODE::M250;
            ret.modules_type[4] = ret.modules_type[5] = ret.modules_type[6] =rb_module::MCODE::M140;

            ret.modules_range_low[0] = ret.modules_range_low[1] = ret.modules_range_low[2] = ret.modules_range_low[3]
            = ret.modules_range_low[4] = ret.modules_range_low[5] = ret.modules_range_low[6] = -360;

            ret.modules_range_up[0] = ret.modules_range_up[1] = ret.modules_range_up[2] = ret.modules_range_up[3]
            = ret.modules_range_up[4] = ret.modules_range_up[5] = ret.modules_range_up[6] = +360;



            // link_offset : to motor center (mm)
            // link_mass_C : from motor center -> that motor's link com (mm)
            // link_capsule : capsule from prev_P to motor center (mm)
            ret.link_ee_offset = Eigen::Vector3d(0, -96.7, 0);
            ret.link_ee_capsule_dw = Eigen::Vector3d(0, 10, 0);
            ret.link_ee_capsule_up = Eigen::Vector3d(0, 0, 0);
            ret.link_ee_capsule_radi = 45;

            ret.link_mass_m[0] = 3.6;
            ret.link_mass_C[0] = Eigen::Vector3d(0, -5, -40);
            ret.link_offset[0] = Eigen::Vector3d(0, 0, 169.2);
            ret.link_rotation[0] = Eigen::Matrix3d::Identity();
            ret.link_capsule_dw[0] = Eigen::Vector3d(0, 0, 0);
            ret.link_capsule_up[0] = Eigen::Vector3d(0, 0, 10);
            ret.link_capsule_radi[0] = 60;

            ret.link_mass_m[1] = 8.4;
            ret.link_mass_C[1] = Eigen::Vector3d(0, 25, 200);
            ret.link_offset[1] = Eigen::Vector3d(0, -148.4, 0);
            ret.link_rotation[1] = Eigen::Matrix3d::Identity();
            ret.link_capsule_dw[1] = Eigen::Vector3d(0, 0, 0);
            ret.link_capsule_up[1] = Eigen::Vector3d(0, 10, 0);
            ret.link_capsule_radi[1] = 60;

            ret.link_mass_m[2] = 0.0;
            ret.link_mass_C[2] = Eigen::Vector3d(0, 0, 0);
            ret.link_offset[2] = Eigen::Vector3d(0, 0, 100.0);
            ret.link_rotation[2] = Eigen::Matrix3d::Identity();
            ret.link_capsule_dw[2] = Eigen::Vector3d(0, 0, 0);
            ret.link_capsule_up[2] = Eigen::Vector3d(0, 0, 0);
            ret.link_capsule_radi[2] = 60;

            ret.link_mass_m[3] = 3.1;
            ret.link_mass_C[3] = Eigen::Vector3d(0, 120, 200);
            ret.link_offset[3] = Eigen::Vector3d(0, 0, 325.0);
            ret.link_rotation[3] = Eigen::Matrix3d::Identity();
            ret.link_capsule_dw[3] = Eigen::Vector3d(0, 0, 0);
            ret.link_capsule_up[3] = Eigen::Vector3d(0, 0, 0);
            ret.link_capsule_radi[3] = 60;

            ret.link_mass_m[4] = 1.3;
            ret.link_mass_C[4] = Eigen::Vector3d(0, -70, 12);
            ret.link_offset[4] = Eigen::Vector3d(0, 148.4, 392.0);
            ret.link_rotation[4] = Eigen::Matrix3d::Identity();
            ret.link_capsule_dw[4] = Eigen::Vector3d(0, 148.4, 0);
            ret.link_capsule_up[4] = Eigen::Vector3d(0, 0, 0);
            ret.link_capsule_radi[4] = 50;

            ret.link_mass_m[5] = 1.3;
            ret.link_mass_C[5] = Eigen::Vector3d(0, -12, 70);
            ret.link_offset[5] = Eigen::Vector3d(0, -110.7, 0);
            ret.link_rotation[5] = Eigen::Matrix3d::Identity();
            ret.link_capsule_dw[5] = Eigen::Vector3d(0, 10, 0);
            ret.link_capsule_up[5] = Eigen::Vector3d(0, 0, 0);
            ret.link_capsule_radi[5] = 45;

            ret.link_mass_m[6] = 0.45;
            ret.link_mass_C[6] = Eigen::Vector3d(0, -71, 0);
            ret.link_offset[6] = Eigen::Vector3d(0, 0, 110.7);
            ret.link_rotation[6] = Eigen::Matrix3d::Identity();
            ret.link_capsule_dw[6] = Eigen::Vector3d(0, 0, -10);
            ret.link_capsule_up[6] = Eigen::Vector3d(0, 0, 0);
            ret.link_capsule_radi[6] = 45;

            int k;
            k = 0;
            ret.link_inerti[k](0, 0) = 6646;        ret.link_inerti[k](0, 1) = 0;           ret.link_inerti[k](0, 2) = 0;
            ret.link_inerti[k](1, 0) = 0;           ret.link_inerti[k](1, 1) = 6623;        ret.link_inerti[k](1, 2) = -718;
            ret.link_inerti[k](2, 0) = 0;           ret.link_inerti[k](2, 1) = -718;        ret.link_inerti[k](2, 2) = 6643;
            k = 1;
            ret.link_inerti[k](0, 0) = 335482;      ret.link_inerti[k](0, 1) = 0;           ret.link_inerti[k](0, 2) = 0;
            ret.link_inerti[k](1, 0) = 0;           ret.link_inerti[k](1, 1) = 333883;      ret.link_inerti[k](1, 2) = 0;
            ret.link_inerti[k](2, 0) = 0;           ret.link_inerti[k](2, 1) = 0;           ret.link_inerti[k](2, 2) = 17261;
            k = 2;
            ret.link_inerti[k](0, 0) = 0;           ret.link_inerti[k](0, 1) = 0;           ret.link_inerti[k](0, 2) = 0;
            ret.link_inerti[k](1, 0) = 0;           ret.link_inerti[k](1, 1) = 0;           ret.link_inerti[k](1, 2) = 0;
            ret.link_inerti[k](2, 0) = 0;           ret.link_inerti[k](2, 1) = 0;           ret.link_inerti[k](2, 2) = 0;
            k = 3;
            ret.link_inerti[k](0, 0) = 81979;       ret.link_inerti[k](0, 1) = 0;           ret.link_inerti[k](0, 2) = 0;
            ret.link_inerti[k](1, 0) = 0;           ret.link_inerti[k](1, 1) = 81808;       ret.link_inerti[k](1, 2) = -2503;
            ret.link_inerti[k](2, 0) = 0;           ret.link_inerti[k](2, 1) = -2503;       ret.link_inerti[k](2, 2) = 5735;
            k = 4;
            ret.link_inerti[k](0, 0) = 1013;        ret.link_inerti[k](0, 1) = 0;           ret.link_inerti[k](0, 2) = 0;
            ret.link_inerti[k](1, 0) = 0;           ret.link_inerti[k](1, 1) = 984;         ret.link_inerti[k](1, 2) = -79;
            ret.link_inerti[k](2, 0) = 0;           ret.link_inerti[k](2, 1) = -79;         ret.link_inerti[k](2, 2) = 726;
            k = 5;
            ret.link_inerti[k](0, 0) = 1013;        ret.link_inerti[k](0, 1) = 0;           ret.link_inerti[k](0, 2) = 0;
            ret.link_inerti[k](1, 0) = 0;           ret.link_inerti[k](1, 1) = 726;         ret.link_inerti[k](1, 2) = -79;
            ret.link_inerti[k](2, 0) = 0;           ret.link_inerti[k](2, 1) = -79;         ret.link_inerti[k](2, 2) = 984;
            k = 6;
            ret.link_inerti[k](0, 0) = 124;         ret.link_inerti[k](0, 1) = 0;           ret.link_inerti[k](0, 2) = 0;
            ret.link_inerti[k](1, 0) = 0;           ret.link_inerti[k](1, 1) = 213;         ret.link_inerti[k](1, 2) = 0;
            ret.link_inerti[k](2, 0) = 0;           ret.link_inerti[k](2, 1) = 0;           ret.link_inerti[k](2, 2) = 128;


            SELF_COLL_CHECK_COMBI temp_combination;
            temp_combination.ind_A = 0;     temp_combination.ind_B = 4;
            ret.self_coll_check_list.push_back(temp_combination);
            temp_combination.ind_A = 0;     temp_combination.ind_B = 5;
            ret.self_coll_check_list.push_back(temp_combination);
            temp_combination.ind_A = 0;     temp_combination.ind_B = 6;
            ret.self_coll_check_list.push_back(temp_combination);
            temp_combination.ind_A = 0;     temp_combination.ind_B = 7;
            ret.self_coll_check_list.push_back(temp_combination);

            temp_combination.ind_A = 1;     temp_combination.ind_B = 5;
            ret.self_coll_check_list.push_back(temp_combination);
            temp_combination.ind_A = 1;     temp_combination.ind_B = 6;
            ret.self_coll_check_list.push_back(temp_combination);
            temp_combination.ind_A = 1;     temp_combination.ind_B = 7;
            ret.self_coll_check_list.push_back(temp_combination);

            temp_combination.ind_A = 2;     temp_combination.ind_B = 6;
            ret.self_coll_check_list.push_back(temp_combination);
            temp_combination.ind_A = 2;     temp_combination.ind_B = 7;
            ret.self_coll_check_list.push_back(temp_combination);
            temp_combination.ind_A = 3;     temp_combination.ind_B = 6;
            ret.self_coll_check_list.push_back(temp_combination);
            temp_combination.ind_A = 3;     temp_combination.ind_B = 7;
            ret.self_coll_check_list.push_back(temp_combination);

            temp_combination.ind_A = 4;     temp_combination.ind_B = 7;
            ret.self_coll_check_list.push_back(temp_combination);

            #endif
        }else if(r_code == 500880 || r_code == 501880){//7DOF
            #if NO_OF_JOINT == 7
            valid_number = true;
            if(r_code == 500880){
                ret.arm_name = "RY2_588R";
            }else{
                ret.arm_name = "RY2_588L";
            }            
            ret.arm_max_payload = 5;
            ret.redundancy_type = 2;

            ret.arm_max_pos_vel = 1000;//mm/s
            ret.arm_max_pos_acc = 20000;
            ret.arm_max_rot_vel = 360;//deg/s
            ret.arm_max_rot_acc = 720;
            

            ret.arm_coll_max = 10;
            ret.arm_coll_min = 0;

            ret.can_Ch[0] = ret.can_Ch[1] = ret.can_Ch[2] = ret.can_Ch[3] = 1;
            ret.can_Ch[4] = ret.can_Ch[5] = ret.can_Ch[6] = ret.can_Ch[7] = 0;

            ret.modules_axis[0] = 1;
            ret.modules_axis[1] = 0;
            ret.modules_axis[2] = 2;
            ret.modules_axis[3] = 1;
            ret.modules_axis[4] = 2;
            ret.modules_axis[5] = 1;
            ret.modules_axis[6] = 0;

            ret.modules_type[0] = ret.modules_type[1] = rb_module::MCODE::M250;
            ret.modules_type[2] = ret.modules_type[3] = rb_module::MCODE::M200;
            ret.modules_type[4] = ret.modules_type[5] = ret.modules_type[6] =rb_module::MCODE::M140;

            ret.modules_range_low[0] = -180;
            ret.modules_range_up[0]  = +180;

            if(r_code == 500880){
                ret.modules_range_low[1] = -180;
                ret.modules_range_up[1]  = +4;
            }else{
                ret.modules_range_low[1] = -4;
                ret.modules_range_up[1]  = +180;
            }

            ret.modules_range_low[2] = -180;
            ret.modules_range_up[2]  = +180;

            ret.modules_range_low[3] = -120;
            ret.modules_range_up[3]  = +120;

            ret.modules_range_low[4] = -360;
            ret.modules_range_up[4]  = +360;

            ret.modules_range_low[5] = -100;
            ret.modules_range_up[5]  = +100;

            ret.modules_range_low[6] = -95;
            ret.modules_range_up[6]  = +95;

            // link_offset : to motor center (mm)
            // link_mass_C : from motor center -> that motor's link com (mm)
            // link_capsule : capsule from prev_P to motor center (mm)
            ret.link_ee_offset = Eigen::Vector3d(0, 0, -98.0);
            ret.link_ee_capsule_dw = Eigen::Vector3d(0, 0, 0);
            ret.link_ee_capsule_up = Eigen::Vector3d(0, 0, 0);
            ret.link_ee_capsule_radi = 45;

            if(r_code == 500880){
                ret.link_mass_m[0] = 1.0;
                ret.link_mass_C[0] = Eigen::Vector3d(0, 0, 0);
                ret.link_offset[0] = Eigen::Vector3d(0, -275.0, -275.0 * tan(7.0 * M_PI / 180.0));
                ret.link_rotation[0] = rb_math::Rx(-7.0 * M_PI / 180.0);
                ret.link_capsule_dw[0] = Eigen::Vector3d(0, 0, 0);
                ret.link_capsule_up[0] = Eigen::Vector3d(0, 0, 0);
                ret.link_capsule_radi[0] = 60;

                ret.link_mass_m[1] = 1.0;
                ret.link_mass_C[1] = Eigen::Vector3d(0, 0, 0);
                ret.link_offset[1] = Eigen::Vector3d(0, 0, 0);
                ret.link_rotation[1] = rb_math::Rx(+7.0 * M_PI / 180.0);
                ret.link_capsule_dw[1] = Eigen::Vector3d(0, 0, 0);
                ret.link_capsule_up[1] = Eigen::Vector3d(0, 0, 0);
                ret.link_capsule_radi[1] = 60;
            }else{
                ret.link_mass_m[0] = 1.0;
                ret.link_mass_C[0] = Eigen::Vector3d(0, 0, 0);
                ret.link_offset[0] = Eigen::Vector3d(0, +275.0, -275.0 * tan(7.0 * M_PI / 180.0));
                ret.link_rotation[0] = rb_math::Rx(+7.0 * M_PI / 180.0);
                ret.link_capsule_dw[0] = Eigen::Vector3d(0, 0, 0);
                ret.link_capsule_up[0] = Eigen::Vector3d(0, 0, 0);
                ret.link_capsule_radi[0] = 60;

                ret.link_mass_m[1] = 1.0;
                ret.link_mass_C[1] = Eigen::Vector3d(0, 0, 0);
                ret.link_offset[1] = Eigen::Vector3d(0, 0, 0);
                ret.link_rotation[1] = rb_math::Rx(-7.0 * M_PI / 180.0);
                ret.link_capsule_dw[1] = Eigen::Vector3d(0, 0, 0);
                ret.link_capsule_up[1] = Eigen::Vector3d(0, 0, 0);
                ret.link_capsule_radi[1] = 60;
            }

            ret.link_mass_m[2] = 1.0;
            ret.link_mass_C[2] = Eigen::Vector3d(0, 0, 0);
            ret.link_offset[2] = Eigen::Vector3d(0, 0, -100.0);
            ret.link_rotation[2] = Eigen::Matrix3d::Identity();
            ret.link_capsule_dw[2] = Eigen::Vector3d(0, 0, 0);
            ret.link_capsule_up[2] = Eigen::Vector3d(0, 0, 0);
            ret.link_capsule_radi[2] = 60;

            ret.link_mass_m[3] = 1.0;
            ret.link_mass_C[3] = Eigen::Vector3d(0, 0, 0);
            ret.link_offset[3] = Eigen::Vector3d(0, 0, -325.0);
            ret.link_rotation[3] = Eigen::Matrix3d::Identity();
            ret.link_capsule_dw[3] = Eigen::Vector3d(0, 0, 0);
            ret.link_capsule_up[3] = Eigen::Vector3d(0, 0, 0);
            ret.link_capsule_radi[3] = 60;

            ret.link_mass_m[4] = 1.0;
            ret.link_mass_C[4] = Eigen::Vector3d(0, 0, 0);
            ret.link_offset[4] = Eigen::Vector3d(0, 0, -100.0);
            ret.link_rotation[4] = Eigen::Matrix3d::Identity();
            ret.link_capsule_dw[4] = Eigen::Vector3d(0, 0, 0);
            ret.link_capsule_up[4] = Eigen::Vector3d(0, 0, 0);
            ret.link_capsule_radi[4] = 50;

            ret.link_mass_m[5] = 1.0;
            ret.link_mass_C[5] = Eigen::Vector3d(0, 0, 0);
            ret.link_offset[5] = Eigen::Vector3d(0, 0, -265.0);
            ret.link_rotation[5] = Eigen::Matrix3d::Identity();
            ret.link_capsule_dw[5] = Eigen::Vector3d(0, 0, 0);
            ret.link_capsule_up[5] = Eigen::Vector3d(0, 0, 0);
            ret.link_capsule_radi[5] = 45;

            ret.link_mass_m[6] = 0.5;
            ret.link_mass_C[6] = Eigen::Vector3d(0, 0, 0);
            ret.link_offset[6] = Eigen::Vector3d(0, 0, 0);
            ret.link_rotation[6] = Eigen::Matrix3d::Identity();
            ret.link_capsule_dw[6] = Eigen::Vector3d(0, 0, 0);
            ret.link_capsule_up[6] = Eigen::Vector3d(0, 0, 0);
            ret.link_capsule_radi[6] = 45;

            int k;
            k = 0;
            ret.link_inerti[k](0, 0) = 6646;        ret.link_inerti[k](0, 1) = 0;           ret.link_inerti[k](0, 2) = 0;
            ret.link_inerti[k](1, 0) = 0;           ret.link_inerti[k](1, 1) = 6623;        ret.link_inerti[k](1, 2) = -718;
            ret.link_inerti[k](2, 0) = 0;           ret.link_inerti[k](2, 1) = -718;        ret.link_inerti[k](2, 2) = 6643;
            k = 1;
            ret.link_inerti[k](0, 0) = 335482;      ret.link_inerti[k](0, 1) = 0;           ret.link_inerti[k](0, 2) = 0;
            ret.link_inerti[k](1, 0) = 0;           ret.link_inerti[k](1, 1) = 333883;      ret.link_inerti[k](1, 2) = 0;
            ret.link_inerti[k](2, 0) = 0;           ret.link_inerti[k](2, 1) = 0;           ret.link_inerti[k](2, 2) = 17261;
            k = 2;
            ret.link_inerti[k](0, 0) = 0;           ret.link_inerti[k](0, 1) = 0;           ret.link_inerti[k](0, 2) = 0;
            ret.link_inerti[k](1, 0) = 0;           ret.link_inerti[k](1, 1) = 0;           ret.link_inerti[k](1, 2) = 0;
            ret.link_inerti[k](2, 0) = 0;           ret.link_inerti[k](2, 1) = 0;           ret.link_inerti[k](2, 2) = 0;
            k = 3;
            ret.link_inerti[k](0, 0) = 81979;       ret.link_inerti[k](0, 1) = 0;           ret.link_inerti[k](0, 2) = 0;
            ret.link_inerti[k](1, 0) = 0;           ret.link_inerti[k](1, 1) = 81808;       ret.link_inerti[k](1, 2) = -2503;
            ret.link_inerti[k](2, 0) = 0;           ret.link_inerti[k](2, 1) = -2503;       ret.link_inerti[k](2, 2) = 5735;
            k = 4;
            ret.link_inerti[k](0, 0) = 1013;        ret.link_inerti[k](0, 1) = 0;           ret.link_inerti[k](0, 2) = 0;
            ret.link_inerti[k](1, 0) = 0;           ret.link_inerti[k](1, 1) = 984;         ret.link_inerti[k](1, 2) = -79;
            ret.link_inerti[k](2, 0) = 0;           ret.link_inerti[k](2, 1) = -79;         ret.link_inerti[k](2, 2) = 726;
            k = 5;
            ret.link_inerti[k](0, 0) = 1013;        ret.link_inerti[k](0, 1) = 0;           ret.link_inerti[k](0, 2) = 0;
            ret.link_inerti[k](1, 0) = 0;           ret.link_inerti[k](1, 1) = 726;         ret.link_inerti[k](1, 2) = -79;
            ret.link_inerti[k](2, 0) = 0;           ret.link_inerti[k](2, 1) = -79;         ret.link_inerti[k](2, 2) = 984;
            k = 6;
            ret.link_inerti[k](0, 0) = 124;         ret.link_inerti[k](0, 1) = 0;           ret.link_inerti[k](0, 2) = 0;
            ret.link_inerti[k](1, 0) = 0;           ret.link_inerti[k](1, 1) = 213;         ret.link_inerti[k](1, 2) = 0;
            ret.link_inerti[k](2, 0) = 0;           ret.link_inerti[k](2, 1) = 0;           ret.link_inerti[k](2, 2) = 128;


            SELF_COLL_CHECK_COMBI temp_combination;
            temp_combination.ind_A = 0;     temp_combination.ind_B = 4;
            ret.self_coll_check_list.push_back(temp_combination);
            temp_combination.ind_A = 0;     temp_combination.ind_B = 5;
            ret.self_coll_check_list.push_back(temp_combination);
            temp_combination.ind_A = 0;     temp_combination.ind_B = 6;
            ret.self_coll_check_list.push_back(temp_combination);
            temp_combination.ind_A = 0;     temp_combination.ind_B = 7;
            ret.self_coll_check_list.push_back(temp_combination);

            temp_combination.ind_A = 1;     temp_combination.ind_B = 5;
            ret.self_coll_check_list.push_back(temp_combination);
            temp_combination.ind_A = 1;     temp_combination.ind_B = 6;
            ret.self_coll_check_list.push_back(temp_combination);
            temp_combination.ind_A = 1;     temp_combination.ind_B = 7;
            ret.self_coll_check_list.push_back(temp_combination);

            temp_combination.ind_A = 2;     temp_combination.ind_B = 6;
            ret.self_coll_check_list.push_back(temp_combination);
            temp_combination.ind_A = 2;     temp_combination.ind_B = 7;
            ret.self_coll_check_list.push_back(temp_combination);
            temp_combination.ind_A = 3;     temp_combination.ind_B = 6;
            ret.self_coll_check_list.push_back(temp_combination);
            temp_combination.ind_A = 3;     temp_combination.ind_B = 7;
            ret.self_coll_check_list.push_back(temp_combination);

            temp_combination.ind_A = 4;     temp_combination.ind_B = 7;
            ret.self_coll_check_list.push_back(temp_combination);

            #endif
        }
        return ret;
    }

    int READ_Robot_Model(){
        return Call_Value_I(DB_ADDR_LIST::ARM_CODE, 500920);
    }
    bool WRITE_Robot_Model(int mcode){
        return Write_Value_I(DB_ADDR_LIST::ARM_CODE, mcode);
    }

    std::tuple<int, int, float> READ_Out_Collision_Para(){
        int onoff = Call_Value_I(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_OUT_COLL_ONOFF), 1);
        int react = Call_Value_I(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_OUT_COLL_REACT), 0);
        float th = Call_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_OUT_COLL_LIMIT), 0.5);
        return {onoff, react, th};
    }
    bool WRITE_Out_Collision_Para(int onoff, int react, float th){
        if(!Write_Value_I(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_OUT_COLL_ONOFF), onoff))   return false;
        if(!Write_Value_I(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_OUT_COLL_REACT), react))   return false;
        if(!Write_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_OUT_COLL_LIMIT), th))      return false;

        return true;
    }

    std::tuple<int, float, float> READ_Self_Collision_Para(){
        int mode = Call_Value_I(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_SELF_COLL_MODE), 0);
        float dist_int = Call_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_SELF_COLL_PARA_INT), 0.0);
        float dist_ext = Call_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_SELF_COLL_PARA_EXT), 0.0);
        return {mode, dist_int, dist_ext};
    }
    bool WRITE_Self_Collision_Para(int mode, float dist_int, float dist_ext){
        if(!Write_Value_I(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_SELF_COLL_MODE), mode))           return false;
        if(!Write_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_SELF_COLL_PARA_INT), dist_int))   return false;
        if(!Write_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_SELF_COLL_PARA_EXT), dist_ext))   return false;
        return true;
    }

    std::tuple<int, float, float, float> READ_Gravity_Vector(){
        int mode = Call_Value_I(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_GRAVITY_MODE), 0);
        float gx = Call_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_GRAVITY_X), 0.0);
        float gy = Call_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_GRAVITY_Y), 0.0);
        float gz = Call_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_GRAVITY_Z), -1.0);
        return {mode, gx, gy, gz};
    }
    bool WRITE_Gravity_Vector(int mode, float gx, float gy, float gz){
        if(!Write_Value_I(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_GRAVITY_MODE), mode))         return false;
        if(!Write_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_GRAVITY_X), gx))              return false;
        if(!Write_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_GRAVITY_Y), gy))              return false;
        if(!Write_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_GRAVITY_Z), gz))              return false;
        return true;
    }

    float READ_Direct_Teach_Sensitivity(unsigned int j_no){
        if(j_no >= NO_OF_JOINT){
            return 0;
        }
        return Call_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_DIRECT_SENSITIVITY_0 + j_no), 1.);
    }
    bool WRITE_Direct_Teach_Sensitivity(unsigned int j_no, float f_value){
        if(j_no >= NO_OF_JOINT){
            return false;
        }
        if(f_value < 0.)    f_value = 0.;
        if(f_value > 2.)    f_value = 2.;
        return Write_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_DIRECT_SENSITIVITY_0 + j_no), f_value);
    }

    TCP_CONFIG READ_Tcp_Parameter(unsigned int t_num){
        if(t_num >= NO_OF_TOOL){
            t_num = 0;
        }

        int tool_item_n = 21;

        std::string t_name = Call_Value_S(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_TOOL_0_NAME + t_num * tool_item_n), "TOOL_" + std::to_string(t_num));
        float t_tcp_x = Call_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_TOOL_0_TCP_X + t_num * tool_item_n), 0);
        float t_tcp_y = Call_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_TOOL_0_TCP_Y + t_num * tool_item_n), 0);
        float t_tcp_z = Call_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_TOOL_0_TCP_Z + t_num * tool_item_n), 0);
        float t_tcp_rx = Call_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_TOOL_0_TCP_RX + t_num * tool_item_n), 0);
        float t_tcp_ry = Call_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_TOOL_0_TCP_RY + t_num * tool_item_n), 0);
        float t_tcp_rz = Call_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_TOOL_0_TCP_RZ + t_num * tool_item_n), 0);
        float t_com_m = Call_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_TOOL_0_MASS_M + t_num * tool_item_n), 0);
        float t_com_x = Call_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_TOOL_0_MASS_X + t_num * tool_item_n), 0);
        float t_com_y = Call_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_TOOL_0_MASS_Y + t_num * tool_item_n), 0);
        float t_com_z = Call_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_TOOL_0_MASS_Z + t_num * tool_item_n), 0);

        int t_box_type = Call_Value_I(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_TOOL_0_BOX_TYPE + t_num * tool_item_n), 0);
        float t_box_para[9];
        for(int i = 0; i < 9; ++i){
            t_box_para[i] = Call_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_TOOL_0_BOX_PARA0 + i + t_num * tool_item_n), 0);
        }

        TCP_CONFIG ret;
        ret.tool_name = t_name;

        ret.tcp_offset = Eigen::Vector3d(t_tcp_x, t_tcp_y, t_tcp_z);
        ret.tcp_rotation = rb_math::RPY_to_R(t_tcp_rx, t_tcp_ry, t_tcp_rz);

        ret.com_mass = t_com_m;
        ret.com_offset = Eigen::Vector3d(t_com_x, t_com_y, t_com_z);

        ret.box_type = t_box_type;
        for(int i = 0; i < 9; ++i){
            ret.box_parameter[i] = t_box_para[i];
        }

        return ret;
    }
    bool WRITE_Tcp_Parameter(unsigned int t_num, TCP_CONFIG t_conf){
        if(t_num >= NO_OF_TOOL){
            return false;
        }
        int tool_item_n = 21;

        Eigen::Vector3d temp_euler= rb_math::R_to_RPY(t_conf.tcp_rotation);

        if(!Write_Value_S(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_TOOL_0_NAME + t_num * tool_item_n), t_conf.tool_name)) return false;
        
        if(!Write_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_TOOL_0_TCP_X + t_num * tool_item_n), t_conf.tcp_offset(0))) return false;
        if(!Write_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_TOOL_0_TCP_Y + t_num * tool_item_n), t_conf.tcp_offset(1))) return false;
        if(!Write_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_TOOL_0_TCP_Z + t_num * tool_item_n), t_conf.tcp_offset(2))) return false;
        if(!Write_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_TOOL_0_TCP_RX + t_num * tool_item_n), temp_euler(0))) return false;
        if(!Write_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_TOOL_0_TCP_RY + t_num * tool_item_n), temp_euler(1))) return false;
        if(!Write_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_TOOL_0_TCP_RZ + t_num * tool_item_n), temp_euler(2))) return false;

        if(!Write_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_TOOL_0_MASS_M + t_num * tool_item_n), t_conf.com_mass)) return false;
        if(!Write_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_TOOL_0_MASS_X + t_num * tool_item_n), t_conf.com_offset(0))) return false;
        if(!Write_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_TOOL_0_MASS_Y + t_num * tool_item_n), t_conf.com_offset(1))) return false;
        if(!Write_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_TOOL_0_MASS_Z + t_num * tool_item_n), t_conf.com_offset(2))) return false;

        if(!Write_Value_I(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_TOOL_0_BOX_TYPE + t_num * tool_item_n), t_conf.box_type)) return false;
        for(int i = 0; i < 9; ++i){
            if(!Write_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::ARM_TOOL_0_BOX_PARA0 + i + t_num * tool_item_n), t_conf.box_parameter[i])) return false;
        }

        return true;
    }

    USERF_CONFIG READ_User_Frame(unsigned int u_num){
        if(u_num >= NO_OF_USERF){
            u_num = 0;
        }

        int userf_item_n = 7;

        std::string u_name  = Call_Value_S(DB_ADDR_LIST((int)DB_ADDR_LIST::USER_FRAME_0_NAME + u_num * userf_item_n), "USER_" + std::to_string(u_num));
        float u_x           = Call_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::USER_FRAME_0_X    + u_num * userf_item_n), 0);
        float u_y           = Call_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::USER_FRAME_0_Y    + u_num * userf_item_n), 0);
        float u_z           = Call_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::USER_FRAME_0_Z    + u_num * userf_item_n), 0);
        float u_rx          = Call_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::USER_FRAME_0_RX   + u_num * userf_item_n), 0);
        float u_ry          = Call_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::USER_FRAME_0_RY   + u_num * userf_item_n), 0);
        float u_rz          = Call_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::USER_FRAME_0_RZ   + u_num * userf_item_n), 0);

        USERF_CONFIG ret;
        ret.userf_offset = Eigen::Vector3d(u_x, u_y, u_z);
        ret.userf_rotation = rb_math::RPY_to_R(u_rx, u_ry, u_rz);
        ret.userf_name = u_name;
        return ret;
    }
    bool WRITE_User_Frame(unsigned int u_num, USERF_CONFIG u_conf){
        if(u_num >= NO_OF_USERF){
            return false;
        }

        Eigen::Vector3d temp_euler= rb_math::R_to_RPY(u_conf.userf_rotation);

        int userf_item_n = 7;

        if(!Write_Value_S(DB_ADDR_LIST((int)DB_ADDR_LIST::USER_FRAME_0_NAME + u_num * userf_item_n), u_conf.userf_name)) return false;
        if(!Write_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::USER_FRAME_0_X    + u_num * userf_item_n), u_conf.userf_offset(0))) return false;
        if(!Write_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::USER_FRAME_0_Y    + u_num * userf_item_n), u_conf.userf_offset(1))) return false;
        if(!Write_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::USER_FRAME_0_Z    + u_num * userf_item_n), u_conf.userf_offset(2))) return false;
        if(!Write_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::USER_FRAME_0_RX   + u_num * userf_item_n), temp_euler(0))) return false;
        if(!Write_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::USER_FRAME_0_RY   + u_num * userf_item_n), temp_euler(1))) return false;
        if(!Write_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::USER_FRAME_0_RZ   + u_num * userf_item_n), temp_euler(2))) return false;

        return true;
    }

    AREA_CONFIG READ_Area_Parameter(unsigned int a_num){
        if(a_num >= NO_OF_AREA){
            a_num = 0;
        }

        int area_item_n = 11;

        std::string a_name  = Call_Value_S(DB_ADDR_LIST((int)DB_ADDR_LIST::AREA_0_NAME + a_num * area_item_n), "AREA_" + std::to_string(a_num));
        int a_type          = Call_Value_I(DB_ADDR_LIST((int)DB_ADDR_LIST::AREA_0_TYPE + a_num * area_item_n), 0);
        float a_x           = Call_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::AREA_0_X    + a_num * area_item_n), 0);
        float a_y           = Call_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::AREA_0_Y    + a_num * area_item_n), 0);
        float a_z           = Call_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::AREA_0_Z    + a_num * area_item_n), 0);
        float a_rx          = Call_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::AREA_0_RX   + a_num * area_item_n), 0);
        float a_ry          = Call_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::AREA_0_RY   + a_num * area_item_n), 0);
        float a_rz          = Call_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::AREA_0_RZ   + a_num * area_item_n), 0);
        float a_pA          = Call_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::AREA_0_PARA_A + a_num * area_item_n), 0);
        float a_pB          = Call_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::AREA_0_PARA_B + a_num * area_item_n), 0);
        float a_pC          = Call_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::AREA_0_PARA_C + a_num * area_item_n), 0);

        AREA_CONFIG ret;
        ret.area_name = a_name;
        ret.area_type = a_type;
        ret.area_offset = Eigen::Vector3d(a_x, a_y, a_z);
        ret.area_rotation = rb_math::RPY_to_R(a_rx, a_ry, a_rz);
        ret.area_parameter[0] = a_pA;
        ret.area_parameter[1] = a_pB;
        ret.area_parameter[2] = a_pC;
        return ret;
    }
    bool WRITE_Area_Parameter(unsigned int a_num, AREA_CONFIG a_conf){
        if(a_num >= NO_OF_AREA){
            return false;
        }

        Eigen::Vector3d temp_euler= rb_math::R_to_RPY(a_conf.area_rotation);

        int area_item_n = 11;

        if(!Write_Value_S(DB_ADDR_LIST((int)DB_ADDR_LIST::AREA_0_NAME + a_num * area_item_n), a_conf.area_name)) return false;
        if(!Write_Value_I(DB_ADDR_LIST((int)DB_ADDR_LIST::AREA_0_TYPE + a_num * area_item_n), a_conf.area_type)) return false;
        if(!Write_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::AREA_0_X    + a_num * area_item_n), a_conf.area_offset(0))) return false;
        if(!Write_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::AREA_0_Y    + a_num * area_item_n), a_conf.area_offset(1))) return false;
        if(!Write_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::AREA_0_Z    + a_num * area_item_n), a_conf.area_offset(2))) return false;
        if(!Write_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::AREA_0_RX   + a_num * area_item_n), temp_euler(0))) return false;
        if(!Write_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::AREA_0_RY   + a_num * area_item_n), temp_euler(1))) return false;
        if(!Write_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::AREA_0_RZ   + a_num * area_item_n), temp_euler(2))) return false;
        if(!Write_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::AREA_0_PARA_A + a_num * area_item_n), a_conf.area_parameter[0])) return false;
        if(!Write_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::AREA_0_PARA_B + a_num * area_item_n), a_conf.area_parameter[1])) return false;
        if(!Write_Value_F(DB_ADDR_LIST((int)DB_ADDR_LIST::AREA_0_PARA_C + a_num * area_item_n), a_conf.area_parameter[2])) return false;

        return true;
    }

    int READ_IO_Special_BOX(int in_or_out, unsigned int p_no){
        if(p_no >= 32){
            return 0;
        }
        if(in_or_out == 1){
            ;//input
            return Call_Value_I(DB_ADDR_LIST((int)DB_ADDR_LIST::DIN_SPECIAL_FUNC_0 + p_no), 0);
        }else{
            ;//output
            return Call_Value_I(DB_ADDR_LIST((int)DB_ADDR_LIST::DOUT_SPECIAL_FUNC_0 + p_no), 0);
        }
    }
    bool WRITE_IO_Special_BOX(int in_or_out, unsigned int p_no, int function_no){
        if(p_no >= 32){
            return false;
        }

        if(in_or_out == 1){
            ;//input
            return Write_Value_I(DB_ADDR_LIST((int)DB_ADDR_LIST::DIN_SPECIAL_FUNC_0 + p_no), function_no);
        }else{
            ;//output
            return Write_Value_I(DB_ADDR_LIST((int)DB_ADDR_LIST::DOUT_SPECIAL_FUNC_0 + p_no), function_no);
        }
    }

    int READ_DIN_Filter_Count(unsigned int p_no){
        if(p_no >= 32){
            return 1;
        }

        return Call_Value_I(DB_ADDR_LIST((int)DB_ADDR_LIST::DIN_FILTER_CNT_0 + p_no), 1);
    }
    bool WRITE_DIN_Filter_Count(unsigned int p_no, int f_count){
        if(p_no >= 32){
            return false;
        }

        return Write_Value_I(DB_ADDR_LIST((int)DB_ADDR_LIST::DIN_FILTER_CNT_0 + p_no), f_count);
    }
}


