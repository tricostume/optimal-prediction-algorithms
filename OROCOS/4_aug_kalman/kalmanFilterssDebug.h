#ifndef KALMAN_FILTERSS_DEBUG_H
#define KALMAN_FILTERSS_DEBUG_H


struct kalmanFilterssDebug
{
        double X_est_ddelta_arm;
        double X_est_alpha;
        double X_est_beta;
        double X_est_dddelta_arm;
        double X_est_dalpha;
        double X_est_dbeta;
        double X_est_ddelta_motor_setpoint;
	double X_est_s;

        double X_meas_ddelta_arm;
        double X_meas_alpha;
        double X_meas_beta;
        double X_meas_ddelta_motor_setpoint;
	
        double ts_trigger;
        double ts_elapsed;
};

#endif
