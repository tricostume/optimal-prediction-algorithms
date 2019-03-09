#pragma once
// NOT USED IN THE FINAL IMPLEMENTATION
struct ControllerParameterization2
{
	double R_control;
	double Q_alpha;
	double Q_dalpha;
	double stateNoiseVariance_ddelta_motor;
	double stateNoiseVariance_ddelta_arm;
	double stateNoiseVariance_dalpha;
	double stateNoiseVariance_dbeta;
	double sensorNoiseVariance_alpha;
	double sensorNoiseVariance_beta;
	double ts_trigger;
	double ts_elapsed;
};
