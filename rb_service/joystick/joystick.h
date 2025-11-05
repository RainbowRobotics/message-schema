#ifndef JOYSTICK_H
#define JOYSTICK_H

namespace rb_joystick {
    bool initialize(std::string domain, int th_cpu);
    bool connect();
    bool disconnect();
    bool refresh();
}
#endif // JOYSTICK_H
