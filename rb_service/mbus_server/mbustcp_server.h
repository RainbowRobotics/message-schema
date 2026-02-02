#ifndef MBUSTCP_SERVER_H
#define MBUSTCP_SERVER_H

#include "shareddata.h"

// Create Modbus mapping
// modbus_mapping_t *modbus_mapping_new(
//     int nb_bits,             // Coils (0xxxx)            — 읽기/쓰기 가능한 비트 (출력)    // 00001–09999
//     int nb_input_bits,       // Discrete Inputs (1xxxx)  — 읽기 전용 비트 (입력)         // 10001–19999
//     int nb_registers,        // Holding Registers (4xxxx) — 읽기/쓰기 가능한 워드 (출력)     // 40001–49999
//     int nb_input_registers   // Input Registers (3xxxx) — 읽기 전용 워드 (입력)          // 30001–39999
// );

// 0x01 Read Coils 출력 비트                                        (0xxxx) 읽기
// 0x02 Read Discrete Inputs 입력 비트                              (1xxxx) 읽기
// 0x05 Write Single Coil 단일 Coil ON/OFF                         (0xxxx) 쓰기
// 0x0F Write Multiple Coils여러 개의 Coil 쓰기                      (0xxxx) 쓰기

// 0x03 Read Holding Registers 출력 워드                            (4xxxx) 읽기
// 0x04 Read Input Registers 입력 워드                              (3xxxx) 읽기
// 0x06 Write Single Register 단일 Holding Register 쓰기            (4xxxx) 쓰기
// 0x10 Write Multiple Registers 여러 개의 Holding Register 쓰기     (4xxxx) 쓰기

// ---------------------
// bits (Coils)
// ---------------------
// 주소 영역: 0xxxx (00001–09999)
// 읽기: FC01
// 쓰기: FC05/15
// ---------------------
// input_bits (Discrete Inputs)
// ---------------------
// 주소 영역: 1xxxx (10001–19999)
// 읽기: FC02
// ---------------------
//  registers (Holding Registers)
// ---------------------
// 주소 영역: 4xxxx (40001–49999)
// 읽기: FC03
// 쓰기: FC06/16
// ---------------------
// input_registers (Input Registers)
// ---------------------
// 주소 영역: 3xxxx (30001–39999)
// 읽기: FC04
// ---------------------

enum MBUS_MAP_bits{// R/W
    MBUS_BITS_NUM
};
enum MBUS_MAP_input_bits{// R
    MBUS_INPUT_BITS_NUM
};
enum MBUS_MAP_registers{// R/W
    MBUS_REGS_NUM
};
enum MBUS_MAP_input_registers{// R
    MBUS_INPUT_REGS_ARM_CODE,
    MBUS_INPUT_REGS_SW_VERSION,
    MBUS_INPUT_REGS_HEART_BEAT,
    MBUS_INPUT_REGS_NUM
};

struct ST_SHM_TO_MBUS{
    int     shm_index;
    float   scaler;
};

const ST_SHM_TO_MBUS MBUS_MAP_INPUT_REGS[MBUS_INPUT_REGS_NUM] = {
    // {SHM_ARM_CODE, 1.0},
    // {SHM_SYSTEM_VERSION, 1.0},
    // {SHM_HEART_BEAT, 1.0},
};

namespace rb_mbus_server {
    bool initialize(std::string domain, int th_cpu, int port_no);
    void shutdown();
}

#endif // MBUSTCP_SERVER_H
