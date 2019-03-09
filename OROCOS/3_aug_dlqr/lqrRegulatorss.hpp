#ifndef __LQRREGULATORSS__
#define __LQRREGULATORSS__
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/Property.hpp>
#include <rtt/Port.hpp>
#include <stdint.h>
#include "augState.h"
#include "SiemensDriveCommand.h"
#include "ResampledMeasurements.h"
#include "Reference.h"
#include "lqrRegulatorssDebug.h"
#include "SiemensDriveState.h"
#include "u_Control.h"

typedef uint64_t TIME_TYPE;

class LqrRegulatorss : public RTT::TaskContext
{
public:
	LqrRegulatorss(std::string name);
	virtual ~LqrRegulatorss(){};
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
	RTT::InputPort< augState > portStateEstimate2;
	RTT::InputPort< augState > portxss1;
	RTT::InputPort< augState > portxss01;
	RTT::InputPort< augState > portxss11;
	RTT::InputPort< augState > portK2; // The LQR controller feedback gains
	RTT::OutputPort< u_Control > portControl;
	RTT::OutputPort< SiemensDriveCommand > portDriveCommand;
	RTT::OutputPort< lqrRegulatorssDebug > portDebug;

private:

ResampledMeasurements resampledMeasurements;

	augState stateEstimate2;
	augState xss1;	
	augState xss01;	
	augState xss11;	
	augState K2;
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
	double state;
	int retrys;
	int ss0_ready;
	double targetSpeed;
	lqrRegulatorssDebug debug_;
	u_Control Control;
	double counter;

};

#endif
