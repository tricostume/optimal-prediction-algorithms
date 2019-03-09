#ifndef STATE2_H
#define STATE2_H

struct State2
{
	// Our linearised state vector
	double ddelta_arm;
	double alpha;
	double beta;
	double dddelta_arm;
	double dalpha;
	double dbeta;
	double ddelta_motor_setpoint;

	double ts_trigger;
	double ts_elapsed;
};

#endif
