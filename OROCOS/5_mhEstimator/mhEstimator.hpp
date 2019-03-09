#ifndef __MHESTIMATOR__
#define __MHESTIMATOR__

#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/Property.hpp>
#include <rtt/Port.hpp>

#include <stdint.h>
#include <queue>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>
#include <stdio.h>
#include <math.h>

#include "export_MHE/acado_common.h"
#include "export_MHE/acado_auxiliary_functions.h"
#include <SiemensDriveState.h>
#include <ResampledMeasurements.h>
#include <controlNMPC.h>
#include <mheEstimate.h>
#include <nmpcDebug.h>
#include <ImuAccel.h>

class MhEstimator : public RTT::TaskContext
{
public:
	MhEstimator(std::string name);
	virtual ~MhEstimator(){};

	virtual bool configureHook();
	virtual bool startHook();
	virtual void updateHook();
	virtual void stopHook();
	virtual void cleanupHook();
	virtual void errorHook();
protected:
	RTT::InputPort< SiemensDriveState > portDriveState;
	RTT::InputPort< ResampledMeasurements > portResampledMeasurements;
	RTT::InputPort< nmpcDebug > portnmpcDebug;
	RTT::InputPort< controlNMPC > portControl;
	RTT::InputPort< ImuAccel > portArmbone;
	RTT::OutputPort< mheEstimate > portStateEstimate;
private:
	SiemensDriveState DriveState;
	ResampledMeasurements Sensors;
	controlNMPC Controls;
	mheEstimate Estimates;
	nmpcDebug nmpcDebug;
	ImuAccel armbone;

	double workMode;
	double R0;
	double h0;
	double factor;
	double v_l_tether;
	double v_delta_arm;
	double v_alpha;
	double v_beta;		 
	double v_ddelta_motor_sp;
	double v_dl_tether;
	double ellapsed_time;
	double total_time;
	double max_time;
	double period;
	double iterations;
	double Rest;
	double Hest;
	double uIN1;
	double uIN2;
	double OFFSET_TETHER;
	double OFFSET_DELTA_ARM;
};

#endif
