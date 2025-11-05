
#define P_NAME  "LEDLIGHT"

#include "ledlight.h"

ledlight::ledlight(int ch)
{
    cans.CAN_CH = ch;
    cans.CAN_ID_LED = 0x2FF;

    // cycle(ms) = cycle / 255 * 2000 ms
    // duty(ms) = duty / 255 * 2000 ms
    led_DB[(int)ARMLED_CMD::ARM_LED_STATUS_NONE].cycle = 255;
    led_DB[(int)ARMLED_CMD::ARM_LED_STATUS_NONE].duty[0] = 0;
    led_DB[(int)ARMLED_CMD::ARM_LED_STATUS_NONE].duty[1] = 0;
    led_DB[(int)ARMLED_CMD::ARM_LED_STATUS_NONE].duty[2] = 0;

    led_DB[(int)ARMLED_CMD::ARM_LED_BEFORE_ACTIVE].cycle = 255;
    led_DB[(int)ARMLED_CMD::ARM_LED_BEFORE_ACTIVE].duty[0] = 0;
    led_DB[(int)ARMLED_CMD::ARM_LED_BEFORE_ACTIVE].duty[1] = 0;
    led_DB[(int)ARMLED_CMD::ARM_LED_BEFORE_ACTIVE].duty[2] = 255;

    led_DB[(int)ARMLED_CMD::ARM_LED_AFTER_ACTIVE].cycle = 255;
    led_DB[(int)ARMLED_CMD::ARM_LED_AFTER_ACTIVE].duty[0] = 0;
    led_DB[(int)ARMLED_CMD::ARM_LED_AFTER_ACTIVE].duty[1] = 255;
    led_DB[(int)ARMLED_CMD::ARM_LED_AFTER_ACTIVE].duty[2] = 255;

    led_DB[(int)ARMLED_CMD::ARM_LED_AFTER_REAL_MODE].cycle = 255;
    led_DB[(int)ARMLED_CMD::ARM_LED_AFTER_REAL_MODE].duty[0] = 255;
    led_DB[(int)ARMLED_CMD::ARM_LED_AFTER_REAL_MODE].duty[1] = 255;
    led_DB[(int)ARMLED_CMD::ARM_LED_AFTER_REAL_MODE].duty[2] = 255;

    led_DB[(int)ARMLED_CMD::ARM_LED_ARM_MOVING].cycle = 200;
    led_DB[(int)ARMLED_CMD::ARM_LED_ARM_MOVING].duty[0] = 100;
    led_DB[(int)ARMLED_CMD::ARM_LED_ARM_MOVING].duty[1] = 100;
    led_DB[(int)ARMLED_CMD::ARM_LED_ARM_MOVING].duty[2] = 100;

    led_DB[(int)ARMLED_CMD::ARM_LED_ARM_GRAVITY_COM].cycle = 80;
    led_DB[(int)ARMLED_CMD::ARM_LED_ARM_GRAVITY_COM].duty[0] = 40;
    led_DB[(int)ARMLED_CMD::ARM_LED_ARM_GRAVITY_COM].duty[1] = 40;
    led_DB[(int)ARMLED_CMD::ARM_LED_ARM_GRAVITY_COM].duty[2] = 40;

    led_DB[(int)ARMLED_CMD::ARM_LED_COLLISION].cycle = 24;
    led_DB[(int)ARMLED_CMD::ARM_LED_COLLISION].duty[0] = 12;
    led_DB[(int)ARMLED_CMD::ARM_LED_COLLISION].duty[1] = 12;
    led_DB[(int)ARMLED_CMD::ARM_LED_COLLISION].duty[2] = 12;
}

ledlight::~ledlight(){
    ;
}

void ledlight::onCANMessage(int ch, int id, const unsigned char* data, int dlc) {
    (void)ch;
    (void)id;
    (void)dlc;
    (void)data;
    if(ch == cans.CAN_CH){
        ;
    }
}

CAN_MSG ledlight::CmdLED(ARMLED_CMD tCMD){
    CAN_MSG can_m;
    can_m.id = cans.CAN_ID_LED;
    can_m.channel = cans.CAN_CH;
    can_m.data[0] = led_DB[(int)tCMD].cycle;
    can_m.data[1] = led_DB[(int)tCMD].duty[0];
    can_m.data[2] = led_DB[(int)tCMD].duty[1];
    can_m.data[3] = led_DB[(int)tCMD].duty[2];
    can_m.dlc = 4;

    return can_m;
}