#ifndef LEDLIGHT_H
#define LEDLIGHT_H

#include "canobserver.h"
#include "lan2can.h"

struct lCANS{
    int             CAN_CH;    
    int             CAN_ID_LED; 
};

enum class ARMLED_CMD{
    ARM_LED_STATUS_NONE = 0,
    ARM_LED_BEFORE_ACTIVE,
    ARM_LED_AFTER_ACTIVE,
    ARM_LED_AFTER_REAL_MODE,
    ARM_LED_ARM_MOVING,
    ARM_LED_ARM_GRAVITY_COM,
    ARM_LED_COLLISION,
    ARM_LED_STATUS_NUMBER
};

struct led_Data_Struct{
    unsigned char cycle;
    unsigned char duty[3];
};


class ledlight : public ICANObserver
{
public:
    ledlight(int ch);
    ~ledlight();

    void onCANMessage(int ch, int id, const unsigned char* data, int dlc) override;

    CAN_MSG CmdLED(ARMLED_CMD tCMD);


private:
    lCANS           cans;

    int             firmware_version;

    led_Data_Struct led_DB[(int)ARMLED_CMD::ARM_LED_STATUS_NUMBER];
};
#endif // LEDLIGHT_H
