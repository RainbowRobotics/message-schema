#ifndef SQLITE_FUNCTION_H
#define SQLITE_FUNCTION_H

#include <sqlite3.h>
#include <dlfcn.h>
#include <iostream>

typedef int (*sqlite3_open_t)(const char *, sqlite3 **);
typedef int (*sqlite3_close_t)(sqlite3 *);
typedef int (*sqlite3_exec_t)(sqlite3 *, const char *, int (*)(void*,int,char**,char**),void*, char**);
typedef int (*sqlite3_finalize_t)(sqlite3_stmt *);
typedef const char *(*sqlite3_errmsg_t)(sqlite3 *);
typedef void (*sqlite3_free_t)(void *);
typedef int (*sqlite3_prepare_v2_t)(sqlite3 *, const char *, int, sqlite3_stmt **, const char **);
typedef int (*sqlite3_step_t)(sqlite3_stmt *);
typedef int (*sqlite3_column_int_t)(sqlite3_stmt *, int);
typedef double (*sqlite3_column_double_t)(sqlite3_stmt *, int);
typedef const unsigned char *(*sqlite3_column_text_t)(sqlite3_stmt *, int);

typedef struct{
    char            is_success;
    int             ret_val_i;
    float           ret_val_f;
    std::string     ret_str;
}RBDB_RET;

class RainbowDataBase
{
public:
    RainbowDataBase();

    void  RBDB_Init(std::string t_name);
    void  RBDB_Close();
    int   RBDB_IsWorking(void);

    int   RBDB_Write_F(std::string v_name, float value_f);
    int   RBDB_Write_I(std::string v_name, int value_i);
    int   RBDB_Write_S(std::string v_name, std::string value_s);

    RBDB_RET RBDB_Read_LineNum(void);
    RBDB_RET RBDB_Read_Item_List(void);
    RBDB_RET RBDB_Read_F(std::string v_name);
    RBDB_RET RBDB_Read_I(std::string v_name);
    RBDB_RET RBDB_Read_S(std::string v_name);


private:
    std::string     name_db;
    std::string     name_table;

    unsigned char   is_init;
    sqlite3         *db_handle;

    sqlite3_open_t          my_sqlite3_open;
    sqlite3_close_t         my_sqlite3_close;
    sqlite3_exec_t          my_sqlite3_exec;
    sqlite3_finalize_t      my_sqlite3_finalize;
    sqlite3_errmsg_t        my_sqlite3_errmsg;
    sqlite3_free_t          my_sqlite3_free;
    sqlite3_prepare_v2_t    my_sqlite3_prepare_v2;
    sqlite3_step_t          my_sqlite3_step;

    sqlite3_column_int_t    my_sqlite3_column_int;
    sqlite3_column_double_t my_sqlite3_column_double;
    sqlite3_column_text_t   my_sqlite3_column_text;
};

#endif // SQLITE_FUNCTION_H
