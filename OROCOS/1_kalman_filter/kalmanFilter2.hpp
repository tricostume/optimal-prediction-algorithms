#ifndef __KALMANFILTER2__
#define __KALMANFILTER2__
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/Property.hpp>
#include <rtt/Port.hpp>
#include <stdint.h>
#include "../LQR/State2.h"
#include "../LQR/u_Control.h"
#include "SiemensDriveState.h"
#include <Eigen/Dense>
#include "ResampledMeasurements.h"
#include "kalmanFilter2Debug.h"

using Eigen::Matrix;

#include "LineAngles.h"
typedef uint64_t TIME_TYPE;
class KalmanFilter2 : public RTT::TaskContext
{
public:
	KalmanFilter2(std::string name);
	virtual ~KalmanFilter2(){};
	virtual bool configureHook();
	virtual bool startHook();
	virtual void updateHook();
	virtual void stopHook();
	virtual void cleanupHook();
	virtual void errorHook();
	
protected:
	RTT::InputPort< ResampledMeasurements > portResampledMeasurements;
	RTT::InputPort< SiemensDriveState > portDriveState;
	RTT::InputPort< LineAngles > portLineAngles; 
	RTT::InputPort< u_Control > portControl;
	RTT::InputPort< State2 > portxss2;
	RTT::InputPort< State2 > portxss02;
	RTT::InputPort< State2 > portxss12;
	RTT::InputPort< State2 > portM12; 
	RTT::InputPort< State2 > portM22; 
	RTT::OutputPort< State2 > portStateEstimate2;
	RTT::OutputPort< kalmanFilter2Debug > portDebug;

private:
	LineAngles lineAngles;
	State2 xss2;	
	State2 xss02;	
	State2 xss12;	
	State2 M12;
	State2 M22;
	ResampledMeasurements resampledMeasurements;
	SiemensDriveState driveState;
	State2 stateEstimate2;
	LineAngles sensorEstimate2;
	TIME_TYPE trigger_last;	
	TIME_TYPE trigger;
	Matrix<double,7,7> A;
	Matrix<double,7,1> B;
	Matrix<double,7,1> X_est;
	Matrix<double,2,1> Y_est;
	Matrix<double,2,1> Y_meas;
	Matrix<double,7,1> M1;
	Matrix<double,7,1> M2;
	Matrix<double,7,1> xss;
	Matrix<double,2,1> sensorError;
	double OFFSET;	
	bool update_gains2();
	kalmanFilter2Debug debug_;
	u_Control Control;

};

#endif
