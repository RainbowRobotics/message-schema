#include "sqlite_function.h"

RainbowDataBase::RainbowDataBase(){
    name_db = "";
    name_table = "";
    is_init = false;
    db_handle = NULL;
}

void RainbowDataBase::RBDB_Init(std::string t_name){
    name_db = t_name;
    name_table = t_name +"_table";
    is_init = false;

    void *lib_handle;
    lib_handle = dlopen("libsqlite3.so", RTLD_LAZY);

    if(!lib_handle){
        //std::cout<<"RBDB : Fail to open LIB so"<<std::endl;
        lib_handle = NULL;
        lib_handle = dlopen("libsqlite3.so.0", RTLD_LAZY);
        if(!lib_handle){
            //std::cout<<"RBDB : Fail to open LIB so0"<<std::endl;
            std::cout<<"-----------------------------------------Fail to Open DB"<<std::endl;
            return;
        }
    }

    my_sqlite3_open = (sqlite3_open_t)dlsym(lib_handle, "sqlite3_open");
    my_sqlite3_close = (sqlite3_close_t)dlsym(lib_handle, "sqlite3_close");
    my_sqlite3_exec = (sqlite3_exec_t)dlsym(lib_handle, "sqlite3_exec");
    my_sqlite3_finalize = (sqlite3_finalize_t)dlsym(lib_handle, "sqlite3_finalize");
    my_sqlite3_errmsg = (sqlite3_errmsg_t)dlsym(lib_handle, "sqlite3_errmsg");
    my_sqlite3_free = (sqlite3_free_t)dlsym(lib_handle, "sqlite3_free");
    my_sqlite3_prepare_v2 = (sqlite3_prepare_v2_t)dlsym(lib_handle, "sqlite3_prepare_v2");
    my_sqlite3_step = (sqlite3_step_t)dlsym(lib_handle, "sqlite3_step");
    my_sqlite3_column_int = (sqlite3_column_int_t)dlsym(lib_handle, "sqlite3_column_int");
    my_sqlite3_column_double = (sqlite3_column_double_t)dlsym(lib_handle, "sqlite3_column_double");
    my_sqlite3_column_text = (sqlite3_column_text_t)dlsym(lib_handle, "sqlite3_column_text");

    if(!my_sqlite3_open || !my_sqlite3_close || !my_sqlite3_exec || !my_sqlite3_finalize || !my_sqlite3_errmsg || !my_sqlite3_free
            || !my_sqlite3_prepare_v2 || !my_sqlite3_step
            || !my_sqlite3_column_int || !my_sqlite3_column_double || !my_sqlite3_column_text){
        std::cout<<"RBDB : Fail to symbolize LIB"<<std::endl;
        dlclose(lib_handle);
        return;
    }

    int open_rc = my_sqlite3_open((name_db+ ".db").c_str(), &db_handle);
    if(open_rc){
        std::cout<<"RBDB : Fail to Open"<<std::endl;
        dlclose(lib_handle);
        return;
    }



    char *errMsg = 0;

    // (0) name TEXT
    // (1) value_i INT
    // (2) value_f DOUBLE
    // (3) value_s TEXT
    std::string cmd_create_table ="CREATE TABLE IF NOT EXISTS " + name_table + "(name TEXT UNIQUE NOT NULL, value_i INT, value_f DOUBLE, value_s TEXT);";
    int create_rc = my_sqlite3_exec(db_handle, cmd_create_table.c_str(), 0, 0, &errMsg);
    if(create_rc != SQLITE_OK){
        std::cout<<"RBDB : Table Creation Error !!---"<<std::endl;
        std::cout<<errMsg<<std::endl;
        std::cout<<"RBDB : --------------------------"<<std::endl;
        my_sqlite3_free(errMsg);

        my_sqlite3_close(db_handle);
        dlclose(lib_handle);

        return;
    }

    is_init = true;
}

void RainbowDataBase::RBDB_Close(){
    if(is_init && db_handle != NULL){
        my_sqlite3_close(db_handle);
    }
    is_init = false;
}

int RainbowDataBase::RBDB_IsWorking(void){
    if(!is_init || db_handle == NULL || name_db == "" || name_table == ""){
        return 0;
    }else{
        return 1;
    }
}

int RainbowDataBase::RBDB_Write_F(std::string v_name, float value_f){
    if(!is_init || db_handle == NULL || name_db == "" || name_table == ""){
        return 1;
    }

    char *errMsg = 0;
    std::string tar_num_str = std::to_string(value_f);
    std::string cmd_insert = "INSERT OR REPLACE INTO " + name_table + " (name, value_f) VALUES('" + v_name + "', " + tar_num_str + ");";
    //std::cout<<cmd_insert<<std::endl;
    int ins_rc_f = my_sqlite3_exec(db_handle, cmd_insert.c_str(), 0, 0, &errMsg);
    if(ins_rc_f != SQLITE_OK){
        std::cout<<"RBDB : INSERT F Error !!---------"<<std::endl;
        std::cout<<errMsg<<std::endl;
        std::cout<<"RBDB : --------------------------"<<std::endl;
        my_sqlite3_free(errMsg);
        return 2;
    }

    return 0;
}

int RainbowDataBase::RBDB_Write_I(std::string v_name, int value_i){
    if(!is_init || db_handle == NULL || name_db == "" || name_table == ""){
        return 1;
    }

    char *errMsg = 0;
    std::string tar_num_str = std::to_string(value_i);
    std::string cmd_insert = "INSERT OR REPLACE INTO " + name_table + " (name, value_i) VALUES('" + v_name + "', " + tar_num_str + ");";
    int ins_rc_f = my_sqlite3_exec(db_handle, cmd_insert.c_str(), 0, 0, &errMsg);
    if(ins_rc_f != SQLITE_OK){
        std::cout<<"RBDB : INSERT I Error !!---------"<<std::endl;
        std::cout<<errMsg<<std::endl;
        std::cout<<"RBDB : --------------------------"<<std::endl;
        my_sqlite3_free(errMsg);
        return 2;
    }

    return 0;
}

int RainbowDataBase::RBDB_Write_S(std::string v_name, std::string value_s){
    if(!is_init || db_handle == NULL || name_db == "" || name_table == ""){
        return 1;
    }

    char *errMsg = 0;
    std::string cmd_insert = "INSERT OR REPLACE INTO " + name_table + " (name, value_s) VALUES('" + v_name + "', '" + value_s + "');";
    //std::cout<<cmd_insert<<std::endl;
    int ins_rc_f = my_sqlite3_exec(db_handle, cmd_insert.c_str(), 0, 0, &errMsg);
    if(ins_rc_f != SQLITE_OK){
        std::cout<<"RBDB : INSERT S Error !!-----------"<<std::endl;
        std::cout<<errMsg<<std::endl;
        std::cout<<"RBDB : --------------------------"<<std::endl;
        my_sqlite3_free(errMsg);
        return 2;
    }
    return 0;
}

RBDB_RET RainbowDataBase::RBDB_Read_LineNum(void){
    RBDB_RET ret;
    ret.is_success = 0;

    if(!is_init || db_handle == NULL || name_db == "" || name_table == ""){
        ret.is_success = -1;
        return ret;
    }

    std::string cmd_sql_count = "SELECT COUNT(*) FROM "+ name_table + ";";
    sqlite3_stmt *stmt_c;
    int rc = my_sqlite3_prepare_v2(db_handle, cmd_sql_count.c_str(), -1, &stmt_c, 0);
    if(rc != SQLITE_OK){
        ret.is_success = -2;
        std::cout<<"RBDB : SELECT COUNT Error !!-----------"<<std::endl;
        std::cout<<my_sqlite3_errmsg(db_handle)<<std::endl;
        std::cout<<"RBDB : --------------------------"<<std::endl;
    }else{
        if(my_sqlite3_step(stmt_c) == SQLITE_ROW){
            ret.ret_val_i = ret.ret_val_f = my_sqlite3_column_int(stmt_c, 0);
            ret.is_success = 1;
        }
    }

    my_sqlite3_finalize(stmt_c);
    return ret;
}

RBDB_RET RainbowDataBase::RBDB_Read_Item_List(void){
    RBDB_RET ret;
    ret.is_success = 0;

    if(!is_init || db_handle == NULL || name_db == "" || name_table == ""){
        ret.is_success = -1;
        return ret;
    }

    std::string cmd_sql_list = "SELECT * FROM "+ name_table + ";";
    sqlite3_stmt *stmt_c;
    int rc = my_sqlite3_prepare_v2(db_handle, cmd_sql_list.c_str(), -1, &stmt_c, 0);
    if(rc != SQLITE_OK){
        ret.is_success = -2;
        std::cout<<"RBDB : LIST SHOW Error !!--------"<<std::endl;
        std::cout<<my_sqlite3_errmsg(db_handle)<<std::endl;
        std::cout<<"RBDB : --------------------------"<<std::endl;
    }else{
        std::string string_sum = "";
        int num_of_item = 0;
        while(my_sqlite3_step(stmt_c) == SQLITE_ROW){
            if(num_of_item != 0){
                string_sum += " / ";
            }
            const unsigned char *temp_ch = my_sqlite3_column_text(stmt_c, 0);
            std::string temp_ch_str = (temp_ch ? reinterpret_cast<const char *>(temp_ch) : "");
            string_sum += temp_ch_str;
            num_of_item++;
        }
        string_sum += (" : TOTAL = " + std::to_string(num_of_item) + " itmes");
        ret.ret_str = string_sum;
        ret.is_success = 1;
    }

    my_sqlite3_finalize(stmt_c);
    return ret;
}

RBDB_RET RainbowDataBase::RBDB_Read_F(std::string v_name){
    RBDB_RET ret;
    ret.is_success = 0;
    if(!is_init || db_handle == NULL || name_db == "" || name_table == ""){
        ret.is_success = -1;
        return ret;
    }

    std::string cmd_read = "SELECT value_f FROM " + name_table + " WHERE name = '" + v_name + "';";
    sqlite3_stmt *stmt_c;
    int rc = my_sqlite3_prepare_v2(db_handle, cmd_read.c_str(), -1, &stmt_c, 0);
    if(rc != SQLITE_OK){
        std::cout<<"RBDB : READ F Error !!-----------"<<std::endl;
        std::cout<<my_sqlite3_errmsg(db_handle)<<std::endl;
        std::cout<<"RBDB : --------------------------"<<std::endl;
        ret.is_success = -2;
    }else{
        ret.is_success = -3;
        while(my_sqlite3_step(stmt_c) == SQLITE_ROW){
            ret.ret_val_f = my_sqlite3_column_double(stmt_c, 0);
            ret.ret_val_i = ret.ret_val_f;
            ret.is_success = 1;
        }
    }
    my_sqlite3_finalize(stmt_c);
    return ret;
}

RBDB_RET RainbowDataBase::RBDB_Read_I(std::string v_name){
    RBDB_RET ret;
    ret.is_success = 0;
    if(!is_init || db_handle == NULL || name_db == "" || name_table == ""){
        ret.is_success = -1;
        return ret;
    }

    std::string cmd_read = "SELECT value_i FROM " + name_table + " WHERE name = '" + v_name + "';";
    sqlite3_stmt *stmt_c;
    int rc = my_sqlite3_prepare_v2(db_handle, cmd_read.c_str(), -1, &stmt_c, 0);
    if(rc != SQLITE_OK){
        std::cout<<"RBDB : READ I Error !!-----------"<<std::endl;
        std::cout<<my_sqlite3_errmsg(db_handle)<<std::endl;
        std::cout<<"RBDB : --------------------------"<<std::endl;
        ret.is_success = -2;
    }else{
        ret.is_success = -3;
        while(my_sqlite3_step(stmt_c) == SQLITE_ROW){
            ret.ret_val_i = my_sqlite3_column_double(stmt_c, 0);
            ret.ret_val_f = ret.ret_val_i;
            ret.is_success = 1;
        }
    }
    my_sqlite3_finalize(stmt_c);
    return ret;
}

RBDB_RET RainbowDataBase::RBDB_Read_S(std::string v_name){
    RBDB_RET ret;
    ret.is_success = 0;
    if(!is_init || db_handle == NULL || name_db == "" || name_table == ""){
        ret.is_success = -1;
        return ret;
    }

    std::string cmd_read = "SELECT value_s FROM " + name_table + " WHERE name = '" + v_name + "';";
    sqlite3_stmt *stmt_c;
    int rc = my_sqlite3_prepare_v2(db_handle, cmd_read.c_str(), -1, &stmt_c, 0);
    if(rc != SQLITE_OK){
        std::cout<<"RBDB : READ S Error !!-----------"<<std::endl;
        std::cout<<my_sqlite3_errmsg(db_handle)<<std::endl;
        std::cout<<"RBDB : --------------------------"<<std::endl;
        ret.is_success = -2;
    }else{
        ret.is_success = -3;
        while(my_sqlite3_step(stmt_c) == SQLITE_ROW){
            const unsigned char *temp_ch = my_sqlite3_column_text(stmt_c, 0);
            ret.ret_str = (temp_ch ? reinterpret_cast<const char *>(temp_ch) : "");
            ret.is_success = 1;
        }
    }
    my_sqlite3_finalize(stmt_c);
    return ret;
}
