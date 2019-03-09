#ifndef __KALMANFILTERSS__
#define __KALMANFILTERSS__
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/Property.hpp>
#include <rtt/Port.hpp>
#include <stdint.h>
#include "augState.h"
#include "u_Control.h"
#include "SiemensDriveState.h"
#include <Eigen/Dense>
#include "ResampledMeasurements.h"
#include "kalmanFilterssDebug.h"
using Eigen::Matrix;
#include "LineAngles.h"
typedef uint64_t TIME_TYPE;
class KalmanFilterss : public RTT::TaskContext
{
public:
	KalmanFilterss(std::string name);
	virtual ~KalmanFilterss(){};
	virtual bool configureHook();
	virtual bool startHook();
	virtual void updateHook();
	virtual void stopHook();
	virtual void cleanupHook();
	virtual void errorHook();
protected:

	RTT::InputPort< ResampledMeasurements > portResampledMeasurements;
	RTT::InputPort< SiemensDriveState > portDriveState;
	// This Kalman Filter operates purely from line angle measurements.
	RTT::InputPort< LineAngles > portLineAngles; // Our actual measurements
	RTT::InputPort< u_Control > portControl;
	RTT::InputPort< augState > portxss2;
	RTT::InputPort< augState > portxss02;
	RTT::InputPort< augState > portxss12;
	RTT::InputPort< augState > portM12; // The first row of the Kalman Filter feedback gains
	RTT::InputPort< augState > portM22; // The second row of the Kalman Filter feedback gains
	RTT::OutputPort< augState > portStateEstimate2;
	RTT::OutputPort< kalmanFilterssDebug > portDebug;

private:

	LineAngles lineAngles;
	augState xss2;	
	augState xss02;	
	augState xss12;	
	augState M12;
	augState M22;
	ResampledMeasurements resampledMeasurements;
	SiemensDriveState driveState;

	// Storage for the output ports...
	augState stateEstimate2;
	LineAngles sensorEstimate2;
	TIME_TYPE trigger_last;	
	TIME_TYPE trigger;
	Matrix<double,8,8> A;
	Matrix<double,8,1> B;
	Matrix<double,8,1> X_est;
	Matrix<double,2,1> Y_est;
	Matrix<double,2,1> Y_meas;
	Matrix<double,8,1> M1;
	Matrix<double,8,1> M2;
	Matrix<double,8,1> xss;
	Matrix<double,2,1> sensorError;
	double OFFSET;

	//LQRControllerDebug debug;
	
	bool update_gains2();
	kalmanFilterssDebug debug_;
	u_Control Control;

};

#endif
