#ifndef U_CONTROL_H
#define U_CONTROL_H

struct u_Control
{
        // Control u from controller and its respective time stamps
        double u;
        double ts_trigger;
        double ts_elapsed;
};

#endif

