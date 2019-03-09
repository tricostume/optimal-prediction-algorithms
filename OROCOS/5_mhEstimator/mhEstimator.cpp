#include "mhEstimator.hpp"

#include <rtt/Logger.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>

#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <string>

using namespace std;
using namespace RTT;
using namespace RTT::os;

typedef uint64_t TIME_TYPE;


/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define VERBOSE     0         /* Show iterations: 1, silent: 0.  */

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;




MhEstimator::MhEstimator(std::string name):TaskContext(name,PreOperational) 
{

	addPort("stateEstimate",portStateEstimate).doc("The estimated state of the system OUT");
	addPort("driveState",portDriveState).doc("Actuator setpoints are retrieved here");
	addPort("control",portControl).doc("MHE receives control from MPC (carousel and winch speeds)");
	addPort("nmpcDebug",portnmpcDebug).doc("MHE sends the reference imposed by nmpc to visualiser");
	addPort("armbone",portArmbone).doc("Not taken in MHE but in order to compare arm acceleration (y accelerometer)");
	addEventPort("resampledMeasurements",portResampledMeasurements).doc("Resampled measurements from all sensors");
	addProperty("workMode",workMode).doc("1. Works with NMPC together 2. Works with lower level DriveState together");
  
	
	/* INITIALIZE MHE PROBLEM */

	/*Some temporary variables*/
	int i;

	/* Initialize the solver*/	

	initializeSolver();

	// Reset all solver memory
	memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
	memset(&acadoVariables, 0, sizeof( acadoVariables ));
	memset(&Estimates, 0, sizeof( Estimates ));
	memset(&Controls, 0, sizeof( Controls ));	

	/* Initialize the states and controls according to selected experiment */
	//X0_MHE = [1.35,0,0.000000887416239,-pi/2,-0.086168493208539,0.000000000000106,0,0];
	// R0 = 2.05; h0 = 1.35;
	// Initial radius and height
	R0 = 2.05;
	h0 = 1.30;
	
	for (i = 0; i < N+1; ++i)
		{
			acadoVariables.x[i*NX+0] = h0;
			acadoVariables.x[i*NX+1] = 0.0;
			acadoVariables.x[i*NX+2] = 0.000000887416239;
			acadoVariables.x[i*NX+3] = -M_PI/2;
			acadoVariables.x[i*NX+4] = -0.086168493208539;
			acadoVariables.x[i*NX+5] = 0.000000000000106;
			acadoVariables.x[i*NX+6] = 0.0;
			acadoVariables.x[i*NX+7] = 0.0;
		}

		for (i = 0; i < N; ++i)
		{
			acadoVariables.u[i*NU+0] = 0.0;
			acadoVariables.u[i*NU+1] = 0.0;
		}


		/* Initialize the measurements/reference. */
		// Y0_MHE = [X0_MHE(1),X0_MHE(2),X0_MHE(4),X0_MHE(5)];
		for (i = 0; i < N; ++i)
		{
			acadoVariables.y[i*NY+0] = h0;  		
			acadoVariables.y[i*NY+1] = 0.0; 		
			acadoVariables.y[i*NY+2] = -M_PI/2; 		
			acadoVariables.y[i*NY+3] = -0.086168493208539;
			acadoVariables.y[i*NY+4] = 0.0;
			acadoVariables.y[i*NY+5] = 0.0;

		}


			acadoVariables.yN[0] = h0;  		
			acadoVariables.yN[1] = 0.0; 		
			acadoVariables.yN[2] = -M_PI/2; 		
			acadoVariables.yN[3] = -0.086168493208539;		


		/* Weighting Matrices */
	
		
		//Sensor variances
		  factor=1;
		  v_l_tether = factor*1.75e-3;
		  v_delta_arm = factor*1.75e-3;
		  v_alpha = factor*1.75e-3;
		  v_beta = factor*1.75e-4;
		  //Control variances
		  v_ddelta_motor_sp = factor*1e-5;
		  v_dl_tether = factor*1e-5;

		  //RE_sim = diag([v_l_tether, v_delta_arm, v_alpha, v_beta, v_ddelta_motor_sp].^2);
		  //inputMHE.W = diag(1./[v_l_tether, v_delta_arm, v_alpha, v_beta, v_ddelta_motor_sp, v_dl_tether].^2);
		

		for (i = 0; i < NY*NY; i++)
		  acadoVariables.W[i] = 1e-8;
		
		acadoVariables.W[0*NY+0] = (1/v_l_tether)*(1/v_l_tether); 
		acadoVariables.W[1*NY+1] = (1/v_delta_arm)*(1/v_delta_arm);
		acadoVariables.W[2*NY+2] = (1/v_alpha)*(1/v_alpha);
		acadoVariables.W[3*NY+3] = (1/v_beta)*(1/v_beta);
		acadoVariables.W[4*NY+4] = (1/v_ddelta_motor_sp)*(1/v_ddelta_motor_sp);
		acadoVariables.W[5*NY+5] = (1/v_dl_tether)*(1/v_dl_tether);


		for (i = 0; i < NYN*NYN; i++)
		  acadoVariables.WN[i] = 1e-8;

		acadoVariables.WN[0*NYN+0] = (1/v_l_tether)*(1/v_l_tether); 
		acadoVariables.WN[1*NYN+1] = (1/v_delta_arm)*(1/v_delta_arm);
		acadoVariables.WN[2*NYN+2] = (1/v_alpha)*(1/v_alpha);
		acadoVariables.WN[3*NYN+3] = (1/v_beta)*(1/v_beta);

	
	/* Prepare first step */
	preparationStep();
	
	
	
	
}

bool MhEstimator::configureHook()
{
	workMode = 1;
	return true;
}

bool  MhEstimator::startHook()
{
    ellapsed_time = 0.0;
    total_time = 0.0;
    max_time = 0.0;
    period = 0.02;
    iterations = 0;
    Rest = R0;
    Hest = -h0;
    uIN1 = 0;
    uIN2 = 0;
    portResampledMeasurements.read(Sensors);
    OFFSET_TETHER = Sensors.winchEncoderPosition;        
    OFFSET_DELTA_ARM = Sensors.carouselEncoderPosition;
    cout << "MHE started succesfully! OFFSET_TETHER =" << OFFSET_TETHER << endl;
    
	return true;
}

void  MhEstimator::updateHook()
{
	TIME_TYPE start_trigger = TimeService::Instance()->getTicks();
	int i;
	double Ts = 0.02;
	timer t;
	/* Get the time before start of the loop. */
	tic( &t );
	
	// 1. Shift MHE reference, incorporate measurements and solve
	
	for (i = 0; i < N-1; ++i)
	      {
		     // inputMHE.y = [inputMHE.y(2:end,:); inputMHE.yN.', uNMPC1,uNMPC2];
		      acadoVariables.y[i*NY] = acadoVariables.y[(i+1)*NY+0];  		
		      acadoVariables.y[i*NY+1] = acadoVariables.y[(i+1)*NY+1]; 	
		      acadoVariables.y[i*NY+2] = acadoVariables.y[(i+1)*NY+2]; 		
		      acadoVariables.y[i*NY+3] = acadoVariables.y[(i+1)*NY+3];		
		      acadoVariables.y[i*NY+4] = uIN1;
		      acadoVariables.y[i*NY+5] = uIN2;
	      }

	acadoVariables.y[(N-1)*NY] = acadoVariables.yN[0];  		
	acadoVariables.y[(N-1)*NY+1] = acadoVariables.yN[1]; 	
	acadoVariables.y[(N-1)*NY+2] = acadoVariables.yN[2]; 		
	acadoVariables.y[(N-1)*NY+3] = acadoVariables.yN[3];		
	acadoVariables.y[(N-1)*NY+4] = uIN1;
	acadoVariables.y[(N-1)*NY+5] = uIN2;
	
	portResampledMeasurements.read(Sensors);
	portArmbone.read(armbone);	

	acadoVariables.yN[0] = Sensors.winchEncoderPosition-OFFSET_TETHER+h0;//Considering winch calibration  		
	acadoVariables.yN[1] = Sensors.carouselEncoderPosition-OFFSET_DELTA_ARM; 		
	acadoVariables.yN[2] = Sensors.elevation-0.746196;// Considering sensor offset 		
	acadoVariables.yN[3] = Sensors.azimuth;	
	
	feedbackStep( );
	
	// 2. Shift MHE state inputs and controls
	
	shiftStates(2, 0, 0);
	shiftControls( 0 );
			
	// 3. Step 3 is occupied by MPC. 
	
	// 4.Immediately send last MHE horizon state (actual state of every control problem)
	
	Estimates.l_tetherEst = acadoVariables.x[N*NX+0];
	Estimates.delta_armEst = acadoVariables.x[N*NX+1];
	Estimates.ddelta_armEst = acadoVariables.x[N*NX+2];
	Estimates.alphaEst = acadoVariables.x[N*NX+3];
	Estimates.betaEst = acadoVariables.x[N*NX+4];
	Estimates.dddelta_armEst = acadoVariables.x[N*NX+5];
	Estimates.dalphaEst = acadoVariables.x[N*NX+6];
	Estimates.dbetaEst = acadoVariables.x[N*NX+7];
	Estimates.ddelta_carousel_spEst = acadoVariables.u[(N-1)*NU+0];
	Estimates.ddelta_winch_spEst = acadoVariables.u[(N-1)*NU+1];
	Estimates.Rest = sqrt(acadoVariables.x[N*NX+0]*acadoVariables.x[N*NX+0]*cos(acadoVariables.x[N*NX+3])*cos(acadoVariables.x[N*NX+3]) + 2.05*2.05 + 2*acadoVariables.x[N*NX+0]*2.05*cos(acadoVariables.x[N*NX+3])*cos(acadoVariables.x[N*NX+4]));
	Estimates.Hest = acadoVariables.x[N*NX+0]*sin(acadoVariables.x[N*NX+3]);
	Estimates.l_tetherMeas = Sensors.winchEncoderPosition-OFFSET_TETHER+h0; // Considering calibration
	Estimates.delta_armMeas = Sensors.carouselEncoderPosition-OFFSET_DELTA_ARM;
	Estimates.alphaMeas = Sensors.elevation-0.746196; // Considering sensor offset
	Estimates.betaMeas = Sensors.azimuth;
	Estimates.arm_speedMeas = Sensors.carouselSpeedSmoothed;
	Estimates.delta_armMeas_Raw = Sensors.carouselEncoderPosition;
	Estimates.tether_speedMeas = Sensors.winchSpeedSmoothed;
	Estimates.arm_accelMeas = armbone.ay;	

	// Measure nmpcDebug port to acquire references
	portnmpcDebug.read(nmpcDebug);
	Estimates.ref_ltether = 1.6;
	Estimates.ref_armspeed = nmpcDebug.ref_ddelta;
	Estimates.ref_alpha = nmpcDebug.ref_alpha;
	Estimates.ref_beta = -0.045261843850114;
	Estimates.ref_armacc = 0;
	Estimates.ref_alphaspeed = 0;
	Estimates.ref_betaspeed = 0;
	Estimates.ref_R = nmpcDebug.Rref;
	Estimates.ref_H = nmpcDebug.Href;	
	real_t te_feedback = toc( &t );
	 tic( &t );
	 if( VERBOSE ) printf("\tReal-Time Iteration time %f, time for feedback %f:  KKT Tolerance = %.9e\n", ellapsed_time, te_feedback, getKKT() );	
	 
	 //5. and 6. Steps 5 and 6 are occupied my MPC and real system respectively (measurements)
	 
	 // 7. Receive MPC (or normal lower level controller) inputs for further estimation
	 
	 if (workMode == 1)
	   
	 {
	   portControl.read(Controls);
	   uIN1 = Controls.u_carousel_speed;
	   uIN2 = Controls.u_winch_speed;
	   
	 }
	 
	 else if (workMode == 2)
	   
	 {
	   portDriveState.read(DriveState);
	   uIN1 = DriveState.carouselSpeedSetpoint; 
	   uIN2 = DriveState.winchSpeedSetpoint;
	 }
	 
	 
	 // Prepare for the next iteration
	  
	 /* Prepare for the next step. */
	  preparationStep();
	 /* Read the elapsed time. */
	  real_t te_preparation = toc( &t );

//	  if( VERBOSE ) printf("\tTime for preparation: %f\n\n", te_preparation );
	  if( VERBOSE ) printf("\tReal-Time Iteration time %f, time for feedback %f:  KKT Tolerance = %.9e\n", ellapsed_time, te_feedback, getKKT() );
	  double te = te_preparation + te_feedback;

	  if (te > max_time) max_time = te;

	  total_time+= te;

	  // prepare for next iteration
	  ellapsed_time += period;
	
	  Estimates.time_feedback = te_feedback;
	  Estimates.time_preparation = te_preparation;
	  Estimates.ts_trigger = TimeService::Instance()->getTicks();
	  Estimates.ts_elapsed = TimeService::Instance()->secondsSince( start_trigger );
	  portStateEstimate.write(Estimates);	

}

void  MhEstimator::stopHook()
{
  
  log(RealTime) << "stophook"<< endlog();

    printf("\n\n Average time of one real-time iteration:   %.5g milliseconds\n", 1e3 * total_time / iterations );
    printf("\n\n Maximum time of one real-time iteration:   %.5g milliseconds\n", 1e3 * max_time);
  
}

void  MhEstimator::cleanupHook()
{
  
  log(RealTime) << "cleanuphook"<< endlog();
  
}

void  MhEstimator::errorHook()
{
  
  log(RealTime) << "errorhook"<< endlog();
  
}

ORO_CREATE_COMPONENT( MhEstimator )
