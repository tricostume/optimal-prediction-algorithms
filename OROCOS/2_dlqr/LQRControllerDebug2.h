#ifndef LQR_CONTROLLER_DEBUG2_H
#define LQR_CONTROLLER_DEBUG2_H
struct LQRControllerDebug2
{	
	// Debug structure
	double X_est_ddelta_arm;
	double X_est_alpha;
	double X_est_beta;
	double X_est_dddelta_arm;
	double X_est_dalpha;
	double X_est_dbeta;
	double X_est_ddelta_motor_setpoint;
        double X_ref_ddelta_arm;
        double X_ref_alpha;
        double X_ref_beta;
        double X_ref_dddelta_arm;
        double X_ref_dalpha;
        double X_ref_dbeta;
        double X_ref_ddelta_motor_setpoint;
	double u; // Coming from LQR
	double ts_trigger;
	double ts_elapsed;
};

#endif
