#ifndef DOF_H
#define DOF_H

#define NO_OF_JOINT         7
#define NO_OF_CARTE         7

#define NO_OF_AIN       4
#define NO_OF_AOUT      4
#define NO_OF_DIN       16
#define NO_OF_DOUT      16

#define RT_PERIOD_MS        2
#define RT_PERIOD_SEC       0.002
#define RT_FREQUENCY        500

#define SYSTEM_VERSION      "1.0.0"
#define SYSTEM_CATEGORY     "manipulate"
#define SYSTEM_NAME         "cobot"
#define SYSTEM_L2C_IP       "10.0.1.1"
#define SYSTEM_L2C_PORT     1977

#define NO_OF_TOOL          8
#define NO_OF_USERF         8
#define NO_OF_AREA          8
#define NO_OF_SHIFT         8

#define COUT_RED            "\033[1;31m"
#define COUT_GREEN          "\033[1;32m"
#define COUT_YELLOW         "\033[1;33m"
#define COUT_BLUE           "\033[1;34m"
#define COUT_MAGENTA        "\033[1;35m"
#define COUT_CYAN           "\033[1;36m"
#define COUT_WHITE          "\033[1;37m"
#define COUT_NORMAL         "\033[0m"

#endif // DOF_H
