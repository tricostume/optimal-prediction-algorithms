#include "LQR.hpp"

#include <rtt/Logger.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>

using namespace std;
using namespace RTT;
using namespace RTT::os;

typedef uint64_t TIME_TYPE;

#include <Eigen/Dense>
using Eigen::Matrix;
// DO NOT FORGET: FOR THIS EXPERIMENT, THE CAROUSEL SHOULD ALREADY BE MOVING AND LOCATED AT STEADY STATE xss0
// THIS IS WHY THE CAROUSEL IS ACCELERATED AT THE BEGINNING
bool clamp(double & x, double lb, double ub)
{
	bool didClamp = false;
	if (x < lb) { x = lb; didClamp = true;};
	if (x > ub) { x = ub; didClamp = true;};
	return didClamp;
}
//////////////////////////////////////////////////////////////////////////////////////////////////
void LQR::ramp(double targetSpeed)
{	
	stepheight = dt*acceleration;
    if (fabs(targetSpeed) > softlimit) {
		targetSpeed = softlimit;
    }	
	FlowStatus driveStateStatus = portDriveState.read(driveState);
        if (driveStateStatus != NewData)
        {
                log(Warning) << "kalmanFilter2: No new driveState data!! " << endlog();
        	return;
	}
		portDriveState.read(driveState);
		currentSetpoint = driveState.carouselSpeedSetpoint;
		currentSpeed = driveState.carouselSpeedSmoothed;
		nextSetpoint = currentSetpoint;
		if (fabs(currentSetpoint - currentSpeed) < threshold) {
			//check if targetspeed is reached
        		if (fabs(currentSetpoint - targetSpeed) < stepheight) {
                		nextSetpoint = targetSpeed;
				state = 2;	
        		}
			else if (currentSetpoint > targetSpeed) {
				state = 3;
				nextSetpoint = max(targetSpeed, currentSetpoint - stepheight );
			}
			else {
				state = 4;
				nextSetpoint = min(targetSpeed, currentSetpoint + stepheight);
			}
	        	retrys = 10;
		}
		// check if current speed is more than a stepheight closer to target speed than current setpoint is.
        	else if ( fabs(currentSetpoint - targetSpeed) - fabs(currentSpeed - targetSpeed) > stepheight) {
			nextSetpoint = currentSpeed;
		}
		else {
			state = 5;
               		retrys--;
        	}
        	// check if ramp got stuck
        	if (retrys < -1) {               		
		 	state = 6;
			targetSpeed = 0;	
                       	acceleration = 0.1;
			nextSetpoint = max(targetSpeed, currentSetpoint - stepheight);			
		}
        	else {}	   	   
	clamp(nextSetpoint,0,1.9);		
        driveCommand.carouselSpeedSetpoint = nextSetpoint; // rad/s
        driveCommand.ts_trigger = trigger;
        ts_elapsed = TimeService::Instance()->secondsSince( trigger );
        driveCommand.ts_elapsed = ts_elapsed;
        portDriveCommand.write(driveCommand);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void LQR::acc_ramp(double targetAcceleration)
{
	if (targetAcceleration > 0)
	{
		targetSpeed = softlimit;
	}
	else if (targetAcceleration < 0)
	{
		targetSpeed = 0;
	}
	stepheight = dt*fabs(targetAcceleration);
        FlowStatus driveStateStatus = portDriveState.read(driveState);
        if (driveStateStatus != NewData)
        {
                log(Warning) << "kalmanFilter2: No new driveState data!! " << endlog();
                return;
        }
                portDriveState.read(driveState);
                currentSetpoint = driveState.carouselSpeedSetpoint;
                currentSpeed = driveState.carouselSpeedSmoothed;
                nextSetpoint = currentSetpoint;
                if (fabs(currentSetpoint - currentSpeed) < threshold) {
                        //check if targetspeed is reached
                        if (fabs(currentSetpoint - targetSpeed) < stepheight) {
                                nextSetpoint = targetSpeed;
                                state = 2;
                        }
                        else if (currentSetpoint > targetSpeed) {
                                state = 3;
                                nextSetpoint = max(targetSpeed, currentSetpoint - stepheight );
                        }
                        else {
                                state = 4;
                                nextSetpoint = min(targetSpeed, currentSetpoint + stepheight);
                        }
                        retrys = 10;
                }
                // check if current speed is more than a stepheight closer to target speed than current setpoint is.
                else if ( fabs(currentSetpoint - targetSpeed) - fabs(currentSpeed - targetSpeed) > stepheight) {
                        nextSetpoint = currentSpeed;
                }
                else {
                        cout << "Current setpoint not reached! Retrying(" << retrys << ")" << endl;
                        state = 5;
                        retrys--;
                }
                // check if ramp got stuck
                if (retrys < -1) {
                        cout << "Aborting ramp! Current setpoint = " << currentSetpoint << ", current speed = " << currentSpeed << endl;
                        state = 6;
                        targetSpeed = 0;
                        acceleration = 0.1;
                        nextSetpoint = max(targetSpeed, currentSetpoint - stepheight);
                }
                else {}
        clamp(nextSetpoint,0,1.9); 
        driveCommand.carouselSpeedSetpoint = nextSetpoint; // rad/s
        driveCommand.ts_trigger = trigger;
        ts_elapsed = TimeService::Instance()->secondsSince( trigger );
        driveCommand.ts_elapsed = ts_elapsed;
        portDriveCommand.write(driveCommand);
	
}
///////////////////////////////////////////////////////////////////////////////////////////////////
LQR::LQR(std::string name):TaskContext(name,PreOperational) 
{
	addPort("resampledMeasurements",portResampledMeasurements).doc("Resampled measurements from all sensors");
	addPort("K2",portK2).doc("LQR  Gains");
	addPort("debug",portDebug).doc("LQR controller debug info");
	addPort("data",portDriveCommand).doc("Command to the Siemens Drives");
	addEventPort("stateEstimate2",portStateEstimate2).doc("Reference elevation");
	addPort("xss1",portxss1).doc("Steady state linearization point");
        addPort("xss01",portxss01).doc("Reference 0");
        addPort("xss11",portxss11).doc("Reference 1");
	addPort("driveState",portDriveState).doc("The state of the simens drives");
	addPort("Control",portControl).doc("Control u sent from this LQR controller");	
	// Cleaning memory
	memset(&resampledMeasurements, 0, sizeof(resampledMeasurements));
	memset(&driveCommand, 0, sizeof(driveCommand));
	memset(&K2, 0, sizeof(K2));
	memset(&stateEstimate2, 0, sizeof(stateEstimate2));
	memset(&xss1, 0, sizeof(xss1));
	memset(&xss01, 0, sizeof(xss01));
	memset(&xss11, 0, sizeof(xss11));
	memset(&driveState, 0, sizeof( driveState ));
	memset(&debug_, 0, sizeof( LQRControllerDebug2 ));
	memset(&Control, 0, sizeof( u_Control ));
	// Defininf time variables
	trigger = TimeService::Instance()->getTicks();
	trigger_last = TimeService::Instance()->getTicks();
	trigger_last_is_valid = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
bool LQR::configureHook()
{
FlowStatus gainsStatus = portK2.read(K2);
	if(gainsStatus != NewData) 
	{
		log(Error) << "lqrController2: Cannot configure; gains are needed!" << endlog();
		return false;
	}
	
	// CHOOSE EXPERIMENT TO BE EXECUTED
	// 0. NORMAL EXPERIMENT: CONSISTS OF A STEP IN THE ELEVATION ANGLE FROM 60 TO 70 DEGREES, THIS MEANING A STEP IN CAROUSEL SPEED FROM 1.18 TO 1.42 RAD/S 
	// 1. BIG STEP : CONSISTS OF A STEP IN THE ELEVATION ANGLE FROM 52 TO 70 DEGREES, THIS MEANING A STEP IN CAROUSEL SPEED FROM 1.18 TO 1.60 RAD/S
	////////////////////////////////////////////
	bigStep = 1;//////////////////////////////// If you choose 1, please go downwards and choose a weak or aggresive tuning
	////////////////////////////////////////////
	// Defining extra variables to be used in the ramp iterations	

	acceleration = 0.1;
	softlimit = 3.1415;
	currentSetpoint = 0;
	nextSetpoint = 0;
    	dt = 0.1; // s
    	retrys = 10;
    	threshold = 0.05; // Rad/s
	state = 0;
	counter = 0;

	// Check all ports for the first time to be able to determine if they send new data or not in the future
        FlowStatus measurementStatus = portResampledMeasurements.read(resampledMeasurements);
        if (measurementStatus != NewData)
        {
                log(Info) << "lqrController2: First read to measurements port was indeed not newData!" << endlog();
        }
        FlowStatus xss0Status = portxss01.read(xss01);
        if (xss0Status != NewData)
        {
                log(Info) << "lqrController2: No x00 arrived at configure loop!! " << endlog();
                
        }
        FlowStatus xss1Status = portxss11.read(xss11);
        if (xss1Status != NewData)
        {
                log(Info) << "lqrController2: No x11 arrived at configure loop!! " << endlog();
                
        }
        FlowStatus K2Status = portK2.read(K2);
        if (K2Status != NewData)
        {
                log(Info) << "lqrController2: No K2 arrived at configure loop!! " << endlog();
                
        }
        FlowStatus stateEstimate2Status = portStateEstimate2.read(stateEstimate2);
        if (stateEstimate2Status != NewData)
        {
                log(Info) << "lqrController2: No stateEstimate2 arrived at configure loop!! " << endlog();

        }
     FlowStatus driveStateStatus = portDriveState.read(driveState);
        if (driveStateStatus != NewData)

        {
                log(Info) << "kalmanFilter2: No new driveState data!! " << endlog();
        }
	return true;
}

bool  LQR::startHook()
{
// For initializing, we try to get as close as possible to the XSS0 by rotating the carousel
	speed0 = 1.180050285609923;
	ss0_ready = 0;
	ramp(speed0);
	return true;
}

void  LQR::updateHook()
{
	trigger_last = trigger; 
	trigger = TimeService::Instance()->getTicks();
	if(!trigger_last_is_valid)
	{
		trigger_last_is_valid = true;
		log(Info) << "lqrController2: returning early because need to initialize dt" << endlog();
		return;
	}
// REACH THE INITIAL SPEED (PROCESS STARTED IN START HOOK OF THIS COMPONENT) and make sure it was reached
if (ss0_ready == 0) 
{
	ramp(speed0);
	portResampledMeasurements.read(resampledMeasurements);
	currentSpeed = resampledMeasurements.carouselSpeedSmoothed;
	if (currentSpeed >= speed0)
		{
			ss0_ready = 1;
			usleep(30000000);//Waiting for stabilisation at the given set point
		}
}
else
{
	FlowStatus estimateStatus = portStateEstimate2.read(stateEstimate2);
        if (estimateStatus != NewData)
        {
//                log(Warning) << "lqrController2: No new estimated data!! " << endlog();
                //return;
        }
	//LQR is a full state feedback regulator. We acquire the full state from the estimator
	double & arm_speed = stateEstimate2.ddelta_arm;
	double & elevation = stateEstimate2.alpha;
        double & azimuth = stateEstimate2.beta;
	double & arm_acc = stateEstimate2.dddelta_arm;
	double & elevation_speed = stateEstimate2.dalpha;
	double & azimuth_speed = stateEstimate2.dbeta;
	double & speed_sp = stateEstimate2.ddelta_motor_setpoint;
	// We also reread the gains
	State2 & gains = K2;

// Uncomment for visualizing the state in the terminal (as well as K_LQR)        
//	cout << "LQR--> Received K2 =" << endl;
//        cout << "LQR--> K11=" << K2.ddelta_arm << endl;
//        cout << "LQR--> K21=" << K2.alpha << endl;
//        cout << "LQR--> K31=" << K2.beta << endl;
//        cout << "LQR--> K41=" << K2.dddelta_arm << endl;
//        cout << "LQR--> K51=" << K2.dalpha << endl;
//        cout << "LQR--> K61=" << K2.dbeta << endl;
//        cout << "LQR--> K71=" << K2.ddelta_motor_setpoint << endl;


//        cout << "LQR--> Received stateEstimate2(kf) =" << endl;
//        cout << "LQR--> kf11=" << arm_speed << endl;
//        cout << "LQR--> kf21=" << elevation << endl;
//        cout << "LQR--> kf31=" << azimuth << endl;
//        cout << "LQR--> kf41=" << arm_acc << endl;
//        cout << "LQR--> kf51=" << elevation_speed << endl;
//        cout << "LQR--> kf61=" << azimuth_speed << endl;
//        cout << "LQR--> kf71=" << speed_sp << endl;

// PERFORMING GENERAL DEMONSTRATION EXPERIMENT, STEP FROM 60 TO 70 DEGREES IN ALPHA CORRESPONDING TO A
// STEP FROM 1.18 TO 1.42 rad/s IN CAROUSEL SPEED. 
if  (bigStep == 0)
{
	// As well as the initial reference vector (FIRST COMMAND) and the step reference
        State2 & ss0 = xss01;
        State2 & ss1 = xss11;
	// Getting out necessary matrices
	K2_mat << gains.ddelta_arm, gains.alpha, gains.beta, gains.dddelta_arm, gains.dalpha, gains.dbeta, gains.ddelta_motor_setpoint;
	X_est << arm_speed,
		 elevation,
		 azimuth,
		 arm_acc,
		 elevation_speed,
		 azimuth_speed,
		 speed_sp;

	// These conditions change the reference dynamically from xss0 to xss1 and back every 20 seconds
	        if(counter>=200 && counter<=201)
        {

                 X_ref << ss0.ddelta_arm,
                 ss0.alpha,
                 ss0.beta,
                 ss0.dddelta_arm,
                 ss0.dalpha,
                 ss0.dbeta,
                 ss0.ddelta_motor_setpoint;
                
        }
        else if (counter < 200)
        {
                 X_ref << ss1.ddelta_arm,
                 ss1.alpha,
                 ss1.beta,
                 ss1.dddelta_arm,
                 ss1.dalpha,
                 ss1.dbeta,
                 ss1.ddelta_motor_setpoint;
                
        }

        else if(counter >= 400)
        {
                counter = 0;
                 X_ref << ss1.ddelta_arm,
                 ss1.alpha,
                 ss1.beta,
                 ss1.dddelta_arm,
                 ss1.dalpha,
                 ss1.dbeta,
                 ss1.ddelta_motor_setpoint;               
        }
}
else if (bigStep == 1)
{
	State2 & ss0 = xss01;
	// Getting out necessarz matrices
	// Less aggresive tuning: Max carousel acceleration = 0.2 rad/s2 (peak)
        //K2_mat << -0.023132282831026, -0.642325057895780, -0.120158098423393, 0.011781954157341, 0.052016911580369, 0.006372720390084, 1.044555020128350;
	// More aggresive tuning: Max carousel acceleration = 0.4 rad/s2(peak)        
	 K2_mat << 0.035379313799455, -1.207305467270903, -0.226682539635779, 0.037038314949464, 0.281645516479225, 0.044741979761324, 1.791599794207368;
	X_est << arm_speed,
                 elevation,
                 azimuth,
                 arm_acc,
                 elevation_speed,
                 azimuth_speed,
                 speed_sp;

                if(counter>=200 && counter<=201)
        {

                 X_ref << ss0.ddelta_arm,
                 ss0.alpha,
                 ss0.beta,
                 ss0.dddelta_arm,
                 ss0.dalpha,
                 ss0.dbeta,
                 ss0.ddelta_motor_setpoint;

        }
        else if (counter < 200)
        {
                 X_ref << 1.605155872584968,
                 -0.907571211037051,
                 -0.055943416126512,
                 0,
                 0,
                 0,
                 1.605155872584968;

        }

        else if(counter >= 400)
        {
                counter = 0;
                 X_ref << 1.605155872584968,
                 -0.907571211037051,
                 -0.055943416126512,
                 0,
                 0,
                 0,
                 1.605155872584968;

        }
}

// Close the loop with full state control law

        Matrix<double,7,1> ERROR = X_est-X_ref;
	u =-K2_mat*(ERROR); 
	// Sending the feedback to debug port before clamping
	debug_.u = u;
	Control.u = u;		
	clamp(u,-0.42,0.42);
	// Apply immediately control to system
//////////////////////////////////////////////////////////
	acc_ramp(u);//////////////////////////////////////
//////////////////////////////////////////////////////////
	// Write out missing debug info
	debug_.X_est_ddelta_arm = X_est(0,0);
	debug_.X_est_alpha = X_est(1,0);
	debug_.X_est_beta = X_est(2,0);
        debug_.X_est_dddelta_arm = X_est(3,0);
	debug_.X_est_dalpha = X_est(4,0);
        debug_.X_est_dbeta = X_est(5,0);
        debug_.X_est_ddelta_motor_setpoint = X_est(6,0);
        debug_.X_ref_ddelta_arm = X_ref(0,0);
	debug_.X_ref_alpha = X_ref(1,0);
        debug_.X_ref_beta = X_ref(2,0);
        debug_.X_ref_dddelta_arm = X_ref(3,0);
        debug_.X_ref_dalpha = X_ref(4,0);
        debug_.X_ref_dbeta = X_ref(5,0);
        debug_.X_ref_ddelta_motor_setpoint = X_ref(6,0);
	debug_.ts_trigger = trigger;
	Control.ts_trigger = trigger;
	// The elapsed time is sent to the debug port in the end
	double ts_elapsed = TimeService::Instance()->secondsSince( trigger );
	debug_.ts_elapsed = ts_elapsed;
	Control.ts_elapsed = ts_elapsed;
	portDebug.write(debug_);
	portControl.write(Control);
	counter++;

}

}

void  LQR::stopHook()
{}

void  LQR::cleanupHook()
{}

void  LQR::errorHook()
{}

ORO_CREATE_COMPONENT( LQR )
