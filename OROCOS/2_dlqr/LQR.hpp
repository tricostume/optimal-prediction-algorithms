#ifndef __LQR__
#define __LQR__
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/Property.hpp>
#include <rtt/Port.hpp>
#include <stdint.h>
#include "State2.h"
#include "SiemensDriveCommand.h"
#include "ResampledMeasurements.h"
#include "Reference.h"
#include "LQRControllerDebug2.h"
#include "SiemensDriveState.h"
#include "u_Control.h"
#include <Eigen/Dense>

typedef uint64_t TIME_TYPE;

class LQR : public RTT::TaskContext
{
public:
	LQR(std::string name);
	virtual ~LQR(){};
	virtual bool configureHook();
	virtual bool startHook();
	virtual void updateHook();
	virtual void stopHook();
	virtual void cleanupHook();
	virtual void errorHook();
	void ramp(double speed);
	void acc_ramp(double acceleration);
protected:

	RTT::InputPort< ResampledMeasurements > portResampledMeasurements;
        RTT::InputPort< SiemensDriveState > portDriveState;
	RTT::InputPort< State2 > portStateEstimate2; // The state estimate from the state estimator (kalman filter for now)
	RTT::InputPort< State2 > portxss1;
	RTT::InputPort< State2 > portxss01;
	RTT::InputPort< State2 > portxss11;
	RTT::InputPort< State2 > portK2; // The LQR controller feedback gains
	RTT::OutputPort< u_Control > portControl;
	RTT::OutputPort< SiemensDriveCommand > portDriveCommand;
	RTT::OutputPort< LQRControllerDebug2 > portDebug;
private:
	ResampledMeasurements resampledMeasurements;
	State2 stateEstimate2;
	State2 xss1;	
	State2 xss01;	
	State2 xss11;	
	State2 K2;
	SiemensDriveCommand driveCommand;
	SiemensDriveState driveState;
	bool trigger_last_is_valid;
	TIME_TYPE trigger_last;	
	TIME_TYPE trigger;
	double speed0;
	double ts_elapsed;
	double u;
	double acceleration;
	double dt;
	double softlimit;
	double currentSetpoint;
	double currentSpeed;
	double nextSetpoint;
	double threshold;
	double stepheight;
	int state;
	int retrys;
	int ss0_ready;
	// ramp acceleration
	double targetSpeed;
	LQRControllerDebug2 debug_;
	u_Control Control;
	double counter;
	double bigStep; // Experiment selector
	Eigen::Matrix<double,1,7> K2_mat;
        Eigen::Matrix<double,7,1> X_est;
        Eigen::Matrix<double,7,1> X_ref;

};

#endif
