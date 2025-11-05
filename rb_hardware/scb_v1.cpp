#include "scb_v1.h"

#define P_NAME  "SCB_V1"

scb_v1::scb_v1()
{
    ;
}

scb_v1::~scb_v1(){
    ;
}

void scb_v1::onCANMessage(int ch, int id, const unsigned char* data, int dlc) {
    // id 기반으로 본인의 메시지만 필터링하고 처리
    (void)ch;
    (void)id;
    (void)dlc;
    (void)data;

    if(id == 0x3F5 || id == 0x3F9){
        //std::cout << P_NAME << " can : "<<ch<<std::endl;
    }
}