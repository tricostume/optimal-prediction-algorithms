#include "kalmanFilterss.hpp"
#include <rtt/Logger.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>
using namespace std;
using namespace RTT;
using namespace RTT::os;
typedef uint64_t TIME_TYPE;
#include <Eigen/Dense>
using Eigen::Matrix;
KalmanFilterss::KalmanFilterss(std::string name):TaskContext(name,PreOperational) 
{
	addPort("lineAngles",portLineAngles).doc("Line Angle Sensor Measurements");	
	addPort("xss2",portxss2);
	addPort("xss02",portxss02);
	addPort("xss12",portxss12);
	addPort("M12",portM12);
	addPort("M22",portM22);
	addPort("driveState",portDriveState).doc("The state of the simens drives");
	addPort("resampledMeasurements",portResampledMeasurements).doc("Resampled measurements from all sensors");
	addPort("stateEstimate2",portStateEstimate2).doc("The estimated state of the system");
	addPort("debug",portDebug).doc("Kalman filter debug info");
	addPort("Control",portControl).doc("Control Debug port is also connected to receive control u as acceleration from it");
	memset(&lineAngles, 0, sizeof( lineAngles ));
	memset(&driveState, 0, sizeof( driveState ));
	memset(&xss2, 0, sizeof( xss2 ));
	memset(&xss02, 0, sizeof( xss02 ));
	memset(&xss12, 0, sizeof( xss12 ));
	memset(&M12, 0, sizeof( M12 ));
	memset(&M22, 0, sizeof( M22 ));
	memset(&stateEstimate2, 0, sizeof( stateEstimate2 ));
	memset(&resampledMeasurements, 0, sizeof(resampledMeasurements));
	memset(&sensorEstimate2, 0, sizeof( sensorEstimate2 ));
	memset(&debug_, 0, sizeof( kalmanFilterssDebug ));
	memset(&Control, 0, sizeof( u_Control ));
	trigger = TimeService::Instance()->getTicks();
	trigger_last = TimeService::Instance()->getTicks();
}

// This function reads the PRECOMPUTED gains and reference state vectors xss0 and xss1 as well as the linearization state vector xss.
// The Kalman matrix M (divided here in two matrices) is also read.

bool KalmanFilterss::update_gains2()
{
	FlowStatus status;
	augState xss_temp;	
	augState xss0_temp;	
	augState xss1_temp;	
	augState M1_temp;
	augState M2_temp;
	bool all_loaded = true;

	status = portxss2.read(xss_temp);
	if(status != NewData) all_loaded = false;
	status = portxss02.read(xss0_temp);
	if(status != NewData) all_loaded = false;
	status = portxss12.read(xss1_temp);
	if(status != NewData) all_loaded = false;
	status = portM12.read(M1_temp);
	if(status != NewData) all_loaded = false;
	status = portM22.read(M2_temp);
	if(status != NewData) all_loaded = false;

	if (all_loaded)
	{
		xss2 = xss_temp;	
		xss02 = xss0_temp;	
		xss12 = xss1_temp;	
		M12 = M1_temp;
		M22 = M2_temp;
	} else {
		log(Error) << "kalmanFilter: update_gains: NewData not available on one of the ports!" << endlog();
	}
	
	return all_loaded;

}

bool KalmanFilterss::configureHook()
{
	bool havegains = update_gains2();
	if (!havegains)
	{
		log(Error) << "kalmanFilterss: refusing to configure cause there was trouble loading gains!" << endlog();
		return false;
	}
	//Load carousel model linearization
	A << 0.813101762586754, 0, 0, 0.085045031905651, 0, 0, 0.186898237330979, 0,
             0.011964096824357, 0.982861824372350, -0.000210267359080, 0.000390636634151, 0.097816146515939, 0.002457904102471, 0.000508363351415, 0.005240751832503,
             0.012612179862981, 0.001049622312677, 0.987276481942412, -0.008807955760581, -0.012755259473640, 0.077089269426289, -0.013803915836774, -0.000500786440094,
             -3.312327821850077, 0, 0, 0.676333955144180, 0, 0, 3.312327821586087, 0,
             0.217489668623435, -0.319841827956458, -0.005808728909641, 0.009022036721478, 0.952308098076149, 0.042478414878823, 0.015214774805813, 0.097763739107454,
             0.312705533863665, 0.031450492060853, -0.221569861471667, -0.143480904011484, -0.220534027645812, 0.594803510530469, -0.343040267327870, -0.012750253722864,
             0, 0, 0, 0, 0, 0, 0.999999999917733, 0,
	     0, 0, 0, 0, 0, 0, 0, 0.99000531641659;	

	B << 0.007240538169029,
             0.000014118040070,
             -0.000430846879007,
             0.186946075434281,
             0.000508559550625,
            -0.013808668507758,
             0.099999999392253,
	     0;
	
	// OFFSET CONFIGURATION
	OFFSET=0.746196;
	FlowStatus resampledMeasurementsStatus = portResampledMeasurements.read(resampledMeasurements);
	if (resampledMeasurementsStatus != NewData) 
	{
		log(Info) << "kalmanFilterss: First read to resampledMeasurements port was indeed not newData!" << endlog();
	}	
	FlowStatus lineAnglesStatus = portLineAngles.read(lineAngles);
        if (lineAnglesStatus != NewData) 
        {
                log(Info) << "kalmanFilterss: First read to lineAngles port was indeed not newData!" << endlog();
        }
        FlowStatus driveStateStatus = portDriveState.read(driveState);

        if (driveStateStatus != NewData)

        {
                log(Info) << "kalmanFilterss: No new driveState data!! " << endlog();
        }
        FlowStatus controlStatus = portControl.read(Control);
      if (controlStatus != NewData)
        {
                log(Info) << "kalmanFilterss: No new control u data!! " << endlog();
        }
	return true;
}

bool  KalmanFilterss::startHook()
{
	// 1. FIRST STATE A PRIORI ESTIMATE------------------------------------------------------------------------
	stateEstimate2.ddelta_arm = 0.000001061080370;
        stateEstimate2.alpha = -1.570796326794897;
        stateEstimate2.beta = 0.086948735822950;
        stateEstimate2.dddelta_arm = 0.000000000000057;
        stateEstimate2.dalpha = 0;
        stateEstimate2.dbeta = 0;
        stateEstimate2.ddelta_motor_setpoint = 0.000001061080372;
	stateEstimate2.s = 0;
	return true;	
}

void  KalmanFilterss::updateHook()
{
	TIME_TYPE trigger = TimeService::Instance()->getTicks();
	FlowStatus resampledMeasurementsStatus = portResampledMeasurements.read(resampledMeasurements);
       if (resampledMeasurementsStatus != NewData)

        {
                log(Warning) << "kalmanFilter2: No new resampledMeasurement data!! " << endlog();
                return;
        }
        FlowStatus lineAnglesStatus = portLineAngles.read(lineAngles);
        if (lineAnglesStatus != NewData)
        {
                log(Warning) << "kalmanFilter2: No new lineAngles data!! " << endlog();
                return;
        }
      FlowStatus driveStateStatus = portDriveState.read(driveState);
      if (driveStateStatus != NewData)
        {
                log(Warning) << "kalmanFilter2: No new driveState data!! " << endlog();
              return;
        }
      FlowStatus controlStatus = portControl.read(Control);
      if (controlStatus != NewData)
        {
                log(Info) << "kalmanFilterss: No new control u data!! " << endlog();
        }
	stateEstimate2.ddelta_motor_setpoint = driveState.carouselSpeedSetpoint;
         M1 << M12.ddelta_arm,
               M12.alpha,
               M12.beta,
               M12.dddelta_arm,
               M12.dalpha,
               M12.dbeta,
               M12.ddelta_motor_setpoint,
	       M12.s;
         M2 << M22.ddelta_arm,
               M22.alpha,
               M22.beta,
               M22.dddelta_arm,
               M22.dalpha,
               M22.dbeta,
               M22.ddelta_motor_setpoint,
	       M22.s;
	xss << xss2.ddelta_arm,
	       xss2.alpha,
	       xss2.beta,
	       xss2.dddelta_arm,
 	       xss2.dalpha,
	       xss2.dbeta,
               xss2.ddelta_motor_setpoint,
	       xss2.s;

	double u = Control.u;

	//1. STATE A PRIORI ESTIMATE (x_k-1 | k-1)^
	X_est << stateEstimate2.ddelta_arm,
               	 stateEstimate2.alpha,
               	 stateEstimate2.beta,
               	 stateEstimate2.dddelta_arm,
                 stateEstimate2.dalpha,
               	 stateEstimate2.dbeta,
               	 stateEstimate2.ddelta_motor_setpoint, //driveState.carouselSpeedSetpoint;	
		 stateEstimate2.s;
	//2. PREDICTION STEP FOR STATE (x_k | k-1)^ AND OUTPUT y_k^ AND MEASUREMENTS RETRIEVAL y_k-----------------------------
	X_est = xss + A*(X_est - xss) + B*u; //+ B*u;
	Y_est << X_est(1,0),
		 X_est(2,0);
	Y_meas << lineAngles.elevation-OFFSET,
		  lineAngles.azimuth;
	//3. UPDATE STEP: PREDICTION TO MEASUREMENTS COMPARISON y_k-(y_k)^------------------------------------------------------
	sensorError = Y_meas-Y_est;
	X_est += M1*sensorError(0,0) + M2*sensorError(1,0);
	// Filling port out so that other components can access to the estimate
	stateEstimate2.ddelta_arm = X_est(0,0);
        stateEstimate2.alpha = X_est(1,0);
        stateEstimate2.beta = X_est(2,0);
        stateEstimate2.dddelta_arm = X_est(3,0);
        stateEstimate2.dalpha = X_est(4,0);
        stateEstimate2.dbeta = X_est(5,0);
	stateEstimate2.ddelta_motor_setpoint = X_est(6,0);
	stateEstimate2.s = X_est(7,0); 
	stateEstimate2.ts_trigger = trigger;
	// Filling debug port to show in telemetry
	debug_.X_est_ddelta_arm =X_est(0,0);
	debug_.X_est_alpha = X_est(1,0);
        debug_.X_est_beta = X_est(2,0);
        debug_.X_est_dddelta_arm = X_est(3,0);
        debug_.X_est_dalpha = X_est(4,0);
        debug_.X_est_dbeta = X_est(5,0);
        debug_.X_est_ddelta_motor_setpoint = X_est(6,0);
	debug_.X_est_s = X_est(7,0);
	double debug_speed = resampledMeasurements.carouselSpeedSmoothed;
        double debug_elevation = resampledMeasurements.elevation-OFFSET;
        double debug_azimuth = resampledMeasurements.azimuth;
	debug_.X_meas_ddelta_arm = debug_speed;
        debug_.X_meas_alpha = debug_elevation;
        debug_.X_meas_beta = debug_azimuth;
        debug_.X_meas_ddelta_motor_setpoint = driveState.carouselSpeedSetpoint;
	debug_.ts_trigger = trigger;
	double ts_elapsed = TimeService::Instance()->secondsSince( trigger );
	stateEstimate2.ts_elapsed = ts_elapsed;
	debug_.ts_elapsed = ts_elapsed;
/* Uncomment for logging state estimate

      cout << "Kalmanfilter2--> What estimator sends to LQR" << endl;
       
        cout << "Kalmanfilter2--> X_est11=" << stateEstimate2.ddelta_arm << endl;
        cout << "Kalmanfilter2--> X_est21=" << stateEstimate2.alpha << endl;
        cout << "Kalmanfilter2--> X_est31=" << stateEstimate2.beta << endl;
        cout << "Kalmanfilter2--> X_est41=" << stateEstimate2.dddelta_arm << endl;
        cout << "Kalmanfilter2--> X_est51=" << stateEstimate2.dalpha << endl;
        cout << "Kalmanfilter2--> X_est61=" << stateEstimate2.dbeta << endl;
        cout << "Kalmanfilter2--> X_est71=" << stateEstimate2.ddelta_motor_setpoint << endl;
	cout << "Kalmanfilter2--> X_est81=" << stateEstimate2.s << endl;
	cout << "REAL ACTUAL MEASUREMENTS =" << endl;
	cout << "REAL_SPEED-----:" << debug_speed << endl; 
	cout << "REAL_ELEVATION-:" << debug_elevation << endl;
	cout << "REAL_BETA------:" << debug_azimuth << endl;

*/
	portStateEstimate2.write(stateEstimate2);
	portDebug.write(debug_);
}

void  KalmanFilterss::stopHook()
{}

void  KalmanFilterss::cleanupHook()
{}

void  KalmanFilterss::errorHook()
{}

ORO_CREATE_COMPONENT( KalmanFilterss )
