#ifndef AUG_STATE_H
#define AUG_STATE_H

struct augState
{
	// Augmented state definition
	double ddelta_arm;
	double alpha;
	double beta;
	double dddelta_arm;
	double dalpha;
	double dbeta;
	double ddelta_motor_setpoint;
	double s;

	double ts_trigger;
	double ts_elapsed;
};

#endif
