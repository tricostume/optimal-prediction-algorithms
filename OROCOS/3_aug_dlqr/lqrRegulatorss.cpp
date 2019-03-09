#include "lqrRegulatorss.hpp"
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
void LqrRegulatorss::ramp(double targetSpeed)
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

void LqrRegulatorss::acc_ramp(double targetAcceleration)
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
LqrRegulatorss::LqrRegulatorss(std::string name):TaskContext(name,PreOperational) 
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
	// Empty memory slots
	memset(&resampledMeasurements, 0, sizeof(resampledMeasurements));
	memset(&driveCommand, 0, sizeof(driveCommand));
	memset(&K2, 0, sizeof(K2));
	memset(&stateEstimate2, 0, sizeof(stateEstimate2));
	memset(&xss1, 0, sizeof(xss1));
	memset(&xss01, 0, sizeof(xss01));
	memset(&xss11, 0, sizeof(xss11));
	memset(&driveState, 0, sizeof( driveState ));
	memset(&debug_, 0, sizeof( lqrRegulatorssDebug ));
	memset(&Control, 0, sizeof( u_Control ));
	trigger = TimeService::Instance()->getTicks();
	trigger_last = TimeService::Instance()->getTicks();
	trigger_last_is_valid = false;
}

bool LqrRegulatorss::configureHook()
{
	FlowStatus gainsStatus = portK2.read(K2);
	if(gainsStatus != NewData) 
	{
		log(Error) << "lqrController2: Cannot configure; gains are needed!" << endlog();
		return false;
	}
	acceleration = 0.1;
	softlimit = 3.1415;
	currentSetpoint = 0;
	nextSetpoint = 0;
    	dt = 0.1; // s
    	retrys = 10;
    	threshold = 0.05; // Rad/s
	state = 0;
	counter=0;
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

bool  LqrRegulatorss::startHook()
{
	
// For initializing, we try to get as close as possible to the XSS0 by rotating the carousel
////////////////////////////////////////
////////////////////////////////////////
	speed0 = 1.180050285609923;////////////////
	ss0_ready = 0;//////////////////
////////////////////////////////////////
	ramp(speed0);
	return true;
}

void  LqrRegulatorss::updateHook()
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
//	cout << "LQR-->currentSpeed=" << currentSpeed << endl;
	
	if (currentSpeed >= speed0)
		{
			ss0_ready = 1;
			usleep(30000000);//Waiting also for accuracy of estimator
			cout << "Initial setpoint reached" << endl;
		}
}
else
{	
	Matrix<double,8,1> X_ref;
	augState & ss1 = xss11;
	augState & ss0 = xss01;	
	if(counter>=200 && counter<=201)
	{

		 X_ref << ss0.ddelta_arm,
                 ss0.alpha,
                 ss0.beta,
                 ss0.dddelta_arm,
                 ss0.dalpha,
                 ss0.dbeta,
                 ss0.ddelta_motor_setpoint,
                 ss0.s;
		
		
	}
	else if (counter < 200)
	{
		 X_ref << ss1.ddelta_arm,
                 ss1.alpha,
                 ss1.beta,
                 ss1.dddelta_arm,
                 ss1.dalpha,
                 ss1.dbeta,
                 ss1.ddelta_motor_setpoint,
                 ss1.s;
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
                 ss1.ddelta_motor_setpoint,
                 ss1.s;

	}

	FlowStatus estimateStatus = portStateEstimate2.read(stateEstimate2);

	//LQR is a full state feedback regulator. We acquire the full state from the estimator
	double & arm_speed = stateEstimate2.ddelta_arm;
	double & elevation = stateEstimate2.alpha;
        double & azimuth = stateEstimate2.beta;
	double & arm_acc = stateEstimate2.dddelta_arm;
	double & elevation_speed = stateEstimate2.dalpha;
	double & azimuth_speed = stateEstimate2.dbeta;
	double & speed_sp = stateEstimate2.ddelta_motor_setpoint;
	double & s = stateEstimate2.s;
	// We also reread the gains
	augState & gains = K2;
        
	// Getting out necessary matrices
	Matrix<double,1,8> K2_mat;
	Matrix<double,8,1> X_est;
	
	K2_mat << gains.ddelta_arm, gains.alpha, gains.beta, gains.dddelta_arm, gains.dalpha, gains.dbeta, gains.ddelta_motor_setpoint, gains.s;
	X_est << arm_speed,
		 elevation,
		 azimuth,
		 arm_acc,
		 elevation_speed,
		 azimuth_speed,
		 speed_sp,
		 s;

        Matrix<double,8,1> ERROR = X_est-X_ref;

	// Closing the loop with full state controller
	u =-K2_mat*(ERROR); 

	// Sending the feedback to debug port before clamping
	debug_.u = u;
	Control.u = u;		
	clamp(u,-0.3,0.3);
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
	debug_.X_est_s = X_est(7,0);
        debug_.X_ref_ddelta_arm = X_ref(0,0);
	debug_.X_ref_alpha = X_ref(1,0);
        debug_.X_ref_beta = X_ref(2,0);
        debug_.X_ref_dddelta_arm = X_ref(3,0);
        debug_.X_ref_dalpha = X_ref(4,0);
        debug_.X_ref_dbeta = X_ref(5,0);
        debug_.X_ref_ddelta_motor_setpoint = X_ref(6,0);
	debug_.X_ref_s = X_ref(7,0);	
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

void  LqrRegulatorss::stopHook()
{}

void  LqrRegulatorss::cleanupHook()
{}

void  LqrRegulatorss::errorHook()
{}

ORO_CREATE_COMPONENT( LqrRegulatorss )
