#ifndef LQR_REGULATOR_DEBUG_H
#define LQR_REGULATOR_DEBUG_H


struct lqrRegulatorssDebug
{
	double X_est_ddelta_arm;
	double X_est_alpha;
	double X_est_beta;
	double X_est_dddelta_arm;
	double X_est_dalpha;
	double X_est_dbeta;
	double X_est_ddelta_motor_setpoint;
	double X_est_s;

        double X_ref_ddelta_arm;
        double X_ref_alpha;
        double X_ref_beta;
        double X_ref_dddelta_arm;
        double X_ref_dalpha;
        double X_ref_dbeta;
        double X_ref_ddelta_motor_setpoint;
	double X_ref_s;

	double u; 

	double ts_trigger;
	double ts_elapsed;
};

#endif
