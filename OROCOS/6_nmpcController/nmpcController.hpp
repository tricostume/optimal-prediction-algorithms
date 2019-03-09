#ifndef __NMPCCONTROLLER__
#define __NMPCCONTROLLER__

#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/Property.hpp>
#include <rtt/Port.hpp>

#include <stdint.h>

#include <queue>

#include "SiemensDriveCommand.h"

#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>


#include <stdio.h>
#include <math.h>
#include <mheEstimate.h>
#include <nmpcDebug.h>
#include <ResampledMeasurements.h>
#include <controlNMPC.h>

#include "export_MPC/acado_common.h"
#include "export_MPC/acado_auxiliary_functions.h"

class NmpcController : public RTT::TaskContext
{
public:
	NmpcController(std::string name);
	virtual ~NmpcController(){};

	virtual bool configureHook();
	virtual bool startHook();
	virtual void updateHook();
	virtual void stopHook();
	virtual void cleanupHook();
	virtual void errorHook();
protected:
    RTT::InputPort< mheEstimate > portStateEstimate;
    RTT::InputPort< ResampledMeasurements > portResampledMeasurements;
    RTT::OutputPort< SiemensDriveCommand > portDriveCommand;
    RTT::OutputPort< nmpcDebug > portDebug;
    RTT::OutputPort< controlNMPC > portControl;
private:
//	SiemensDriveState driveState;
//	LineAngles lineAngles;
//	ResampledMeasurements resampledMeasurements;
	double experimentNumber;
	double system_initialized;
	double R0;
	double h0;
	double Rf;
	double hf;
	double A;
	double w;
	double OFFSET;
	double ellapsed_time;
	double total_time;
	double max_time;
	double period;
	int iterations;
	double T_helix;
	double a;
	double b;
	double Rref;
	double Href;
	double state;
	double actualLength;
	double OFFSET_TETHER;
	double tether_length0;
	double arm_speed0;
	double winchSpeedSetpoint;
	double carouselSpeedSetpoint;
	double controlling;
	SiemensDriveCommand driveCommand;
	mheEstimate stateEstimate;
	controlNMPC controlNMPC;
	nmpcDebug nmpcDebug;
	ResampledMeasurements Sensors;
	double l_tether_precalc[ 4501 ];
};

#endif
