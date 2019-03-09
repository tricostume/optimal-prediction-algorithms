#ifndef KALMAN_FILTER2_DEBUG_H
#define KALMAN_FILTER2_DEBUG_H


struct kalmanFilter2Debug
{
        // State estimates
	double X_est_ddelta_arm;
        double X_est_alpha;
        double X_est_beta;
        double X_est_dddelta_arm;
        double X_est_dalpha;
        double X_est_dbeta;
        double X_est_ddelta_motor_setpoint;
	// Measurements to visualiser
        double X_meas_ddelta_arm;
        double X_meas_alpha;
        double X_meas_beta;
        double X_meas_ddelta_motor_setpoint;
	// Time stamps
        double ts_trigger;
        double ts_elapsed;
};

#endif

