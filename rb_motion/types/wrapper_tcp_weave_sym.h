#ifndef WRAPPER_TCP_WEAVE_SYM_H
#define WRAPPER_TCP_WEAVE_SYM_H

#include "rmath.h"
struct tcp_weave_shm_ret {
    VectorCd L_output;
    int      Right_Left;
};
class wrapper_tcp_weave_sym
{
public:
    wrapper_tcp_weave_sym();
    ~wrapper_tcp_weave_sym();

    int Start(double magnitude, double time);
    int Finish();
    bool IsWorking();

    tcp_weave_shm_ret Control(VectorCd original_path);

    void Update_Timer(double dt);
    double Get_Timer();
    void Set_Timer(double _time);
private:
    bool            onoff;
    double          timer;
    double          T[8];
    double          L;
    int             sym_type;
};

#endif // WRAPPER_TCP_WEAVE_SYM_H
