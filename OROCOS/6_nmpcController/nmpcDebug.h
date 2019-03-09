#ifndef NMPCDEBUG_H
#define NMPCDEBUG_H

struct nmpcDebug
{

    double u_carousel_speed;
    double u_carousel_acc;
    double u_winch_speed;
    double tether_speedMeas;
    double Rref;
    double Href;
    double ref_alpha;
    double ref_ddelta;
    // In case of choosing EXP 4, tether length reference is also reported
    double ref_tether_length;
    double time_feedback;
    double time_preparation;
    
    double ts_trigger;
    double ts_elapsed;
};


#endif
