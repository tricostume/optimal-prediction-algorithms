#include "kalmanFilter2.hpp"
#include <rtt/Logger.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>

using namespace std;
using namespace RTT;
using namespace RTT::os;

typedef uint64_t TIME_TYPE;
#include <Eigen/Dense>
using Eigen::Matrix;

KalmanFilter2::KalmanFilter2(std::string name):TaskContext(name,PreOperational) 
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
	// Erase memory slots
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
	memset(&debug_, 0, sizeof( kalmanFilter2Debug ));
	memset(&Control, 0, sizeof( u_Control ));
	// Define time marks
	trigger = TimeService::Instance()->getTicks();
	trigger_last = TimeService::Instance()->getTicks();
}

// This function reads the PRECOMPUTED gains and reference state vectors xss0 and xss1 as well as the linearization state vector xss.
// The Kalman matrix M (divided here in two matrices) is also read.

bool KalmanFilter2::update_gains2()
{
	// Function used for reading the tuning parameters and the steady states
	FlowStatus status;
	State2 xss_temp;	
	State2 xss0_temp;	
	State2 xss1_temp;	
	State2 M1_temp;
	State2 M2_temp;
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
	{	xss2 = xss_temp;	
		xss02 = xss0_temp;	
		xss12 = xss1_temp;	
		M12 = M1_temp;
		M22 = M2_temp;
	} else {
		log(Error) << "kalmanFilter: update_gains: NewData not available on one of the ports!" << endlog();
	}
	return all_loaded;
}

bool KalmanFilter2::configureHook()
{
	bool havegains = update_gains2();
	if (!havegains)
	{
		log(Error) << "kalmanFilter2: refusing to configure cause there was trouble loading gains!" << endlog();
		return false;
	}
	//Load carousel model
	//Matrix<double,7,7> A;
	A << 0.813101762586754, 0, 0, 0.085045031905651, 0, 0, 0.186898237330979,
	     0.011964097712536, 0.982861824372350, -0.000210267359080, 0.000390636634151, 0.097816146515939, 0.002457904102471, 0.000508363795504,
	     0.012612198958817, 0.001049622312677, 0.987276481942412, -0.008807944838762, -0.012755259473640, 0.077089269426289, -0.013803892105757,
	     -3.312327821850077, 0, 0, 0.676333955144180, 0, 0, 3.312327821683083,
	     0.217489679210100, -0.319841827954846, -0.005808728909530, 0.009022042802874, 0.952308098078041, 0.042478414880326, 0.015214787958453,
	     0.312705830121245, 0.031450492059851, -0.221569861471537, -0.143480722189639, -0.220534027645331, 0.594803510531393, -0.343049883179328,
	     0, 0, 0, 0, 0, 0, 0.999999999917733;
	//Matrix<double,7,1> B;
	B << 0.007240538169029,
 	     0.000014118040070,
	     -0.000430845782662,
	     0.186946075439572,
	     0.000508560173188,
	    -0.013808644317131,
	     0.099999999392253;	
	// OFFSET CONFIGURATION
	OFFSET=0.746196;
	// Read Kalman filter ports and resampled Measurements port
	FlowStatus resampledMeasurementsStatus = portResampledMeasurements.read(resampledMeasurements);
	if (resampledMeasurementsStatus != NewData) 
	{
		log(Info) << "kalmanFilter2: First read to resampledMeasurements port was indeed not newData!" << endlog();
	}	
	FlowStatus lineAnglesStatus = portLineAngles.read(lineAngles);
        if (lineAnglesStatus != NewData) 
        {
                log(Info) << "kalmanFilter2: First read to lineAngles port was indeed not newData!" << endlog();
        }
        FlowStatus driveStateStatus = portDriveState.read(driveState);
        if (driveStateStatus != NewData)
        {
                log(Info) << "kalmanFilter2: No new driveState data!! " << endlog();
        }
	return true;
}

bool  KalmanFilter2::startHook()
{
	// 1. FIRST STATE A PRIORI ESTIMATE------------------------------------------------------
	stateEstimate2.ddelta_arm = 0.000001061080370;
	stateEstimate2.alpha = -1.570796326794897;
	stateEstimate2.beta = 0.086948735822950;
	stateEstimate2.dddelta_arm = 0.000000000000057;
	stateEstimate2.dalpha = 0;
	stateEstimate2.dbeta = 0;
	stateEstimate2.ddelta_motor_setpoint = 0.000001061080372;	
	return true;
}


void  KalmanFilter2::updateHook()
{
	TIME_TYPE trigger = TimeService::Instance()->getTicks();
	FlowStatus resampledMeasurementsStatus = portResampledMeasurements.read(resampledMeasurements);
	// Checking functionality of ports
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
	stateEstimate2.ddelta_motor_setpoint = driveState.carouselSpeedSetpoint;
	// Loading important matrices for DLQE, xss will be used in the linearized state
	// prediction
         M1 << M12.ddelta_arm,
               M12.alpha,
               M12.beta,
               M12.dddelta_arm,
               M12.dalpha,
               M12.dbeta,
               M12.ddelta_motor_setpoint;
         M2 << M22.ddelta_arm,
               M22.alpha,
               M22.beta,
               M22.dddelta_arm,
               M22.dalpha,
               M22.dbeta,
               M22.ddelta_motor_setpoint;
	xss << xss2.ddelta_arm,
	       xss2.alpha,
	       xss2.beta,
	       xss2.dddelta_arm,
 	       xss2.dalpha,
	       xss2.dbeta,
               xss2.ddelta_motor_setpoint;
	// Receiving u from Controller
	FlowStatus controlStatus = portControl.read(Control);
	double u = Control.u;

	//1. STATE A PRIORI ESTIMATE (x_k-1 | k-1)^
	X_est << stateEstimate2.ddelta_arm,
               	 stateEstimate2.alpha,
               	 stateEstimate2.beta,
               	 stateEstimate2.dddelta_arm,
                 stateEstimate2.dalpha,
               	 stateEstimate2.dbeta,
               	 stateEstimate2.ddelta_motor_setpoint;//driveState.carouselSpeedSetpoint;	
	
	//2. PREDICTION STEP FOR STATE (x_k | k-1)^ AND OUTPUT y_k^ AND MEASUREMENTS RETRIEVAL y_k-----------------------------
	X_est = xss + A*(X_est - xss) + B*u;
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
	stateEstimate2.ts_trigger = trigger;
	
	// Filling debug port to show in telemetry
	debug_.X_est_ddelta_arm =X_est(0,0);
	debug_.X_est_alpha = X_est(1,0);
        debug_.X_est_beta = X_est(2,0);
        debug_.X_est_dddelta_arm = X_est(3,0);
        debug_.X_est_dalpha = X_est(4,0);
        debug_.X_est_dbeta = X_est(5,0);
        debug_.X_est_ddelta_motor_setpoint = X_est(6,0);
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
	portStateEstimate2.write(stateEstimate2);
	portDebug.write(debug_);
}

void  KalmanFilter2::stopHook()
{}

void  KalmanFilter2::cleanupHook()
{}

void  KalmanFilter2::errorHook()
{}

ORO_CREATE_COMPONENT( KalmanFilter2 )
