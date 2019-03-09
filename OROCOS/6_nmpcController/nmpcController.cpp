#include "nmpcController.hpp"

#include <rtt/Logger.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>

#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <string>
#include <stdio.h>
#include <stdlib.h>

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

#define NUM_STEPS   10        /* Number of real-time iterations. */
#define VERBOSE     0         /* Show iterations: 1, silent: 0.  */

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;



bool clamp(double & x, double lb, double ub)
{
	bool didClamp = false;
	if (x < lb) { x = lb; didClamp = true;};
	if (x > ub) { x = ub; didClamp = true;};
	return didClamp;
}




NmpcController::NmpcController(std::string name):TaskContext(name,PreOperational) 
{

	addEventPort("stateEstimate",portStateEstimate).doc("The estimated state of the system");
	addPort("driveCommand",portDriveCommand).doc("The actuators command");
	addPort("debug",portDebug).doc("NMPController Debug info u");
	addPort("control",portControl).doc("NMPController sends control to MHE");
	addPort("resampledMeasurements",portResampledMeasurements).doc("Resampled measurements from all sensors");
	addProperty("system_initialized",system_initialized).doc("This must be 0 if the initial optimization state has not been reached");
	/* INITIALIZE MPC PROBLEM */
	// Select experiment: 1. Constant radius 2. Constant height 3. Helix tracking 4. Constant radius improvement
	experimentNumber = 4;
	controlling = 1; // 0. Simulate what ACADO would give to the carousel
			 // 1. Control by sending the control commands to actuators
	/*Some temporary variables*/
	int i;

	/* Initialize the solver*/	

	initializeSolver();

	// Reset all solver memory
	memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
	memset(&acadoVariables, 0, sizeof( acadoVariables ));
	memset(&stateEstimate, 0, sizeof( stateEstimate ));


	// If experiment 4 was chosen, the precalculated tether length trajectory has to be loaded into the array l_tether_precalc

        if (experimentNumber == 4)
        {
		// Uncomment if want to try EXP4. Have to find a way to write relative paths in here without getting an error
/*                FILE *myFile;
                myFile = fopen("/home/klarl/planepower/components/carousel2/nmpcController/precalc.txt","r");

                if (myFile == NULL)
                {
                        printf("Error Reading File \n");
                }
                // Read file into double l_tether_precalc[] array
                for (i = 0; i < 4501; ++i)
                        {
                                l_tether_precalc[i] = 0.0;
                        }

                for (i = 0; i<4501; i++)
                        {
                                fscanf(myFile, "%lf\n", &l_tether_precalc[i]);
                        }

		printf("NMPC:EXP 4: Proving file reading, 1.6022 has to be displayed next:\n");
		printf("%lf\n\n", l_tether_precalc[4500]);
*/
/*                for (i = 0; i < 4501; i++)
                        {
                                printf("%lf\n\n", l_tether_precalc[i]);
                        }
*/
        }



	/* Initialize the states and controls according to selected experiment */

	if ( experimentNumber == 3 )
		{
			cout << "Experiment chosen: " << experimentNumber << endl;
			//Initial state -----> X0 = [1.8, 1.186347700078462, -1.201023842411613, -0.046154929792893, 0.000000000000000, 0, 0, 1.186347700078462 ];
			// State description:      l_tether     d_arm                alpha              beta              dd_arm    dalpha dbeta   d_arm_sp
			
			for (i = 0; i < N+1; ++i)
			{
        			acadoVariables.x[i*NX+0] = 1.8;
        			acadoVariables.x[i*NX+1] = 1.186347700078462;
        			acadoVariables.x[i*NX+2] = -1.201023842411613;
       	 			acadoVariables.x[i*NX+3] = -0.046154929792893;
        			acadoVariables.x[i*NX+4] = 0.000000000000000;
        			acadoVariables.x[i*NX+5] = 0.0;
        			acadoVariables.x[i*NX+6] = 0.0;
        			acadoVariables.x[i*NX+7] = 1.186347700078462;
    			}

    			for (i = 0; i < N; ++i)
    			{
        			acadoVariables.u[i*NU+0] = 0.0;
				acadoVariables.u[i*NU+1] = 0.0;
    			}


			/* Initialize the measurements/reference. */
			//Initial tracked variables radius and height ----> Y0 = [2.7,1.678337270339664];
    			for (i = 0; i < N; ++i)
    			{
        			acadoVariables.y[i*NY+0] = 1.8;  		
        			acadoVariables.y[i*NY+1] = 1.186347700078462; 		
        			acadoVariables.y[i*NY+2] = -1.201023842411613; 		
        			acadoVariables.y[i*NY+3] = -0.046154929792893;		
        			acadoVariables.y[i*NY+4] = 0.000000000000000;
        			acadoVariables.y[i*NY+5] = 0.0;
        			acadoVariables.y[i*NY+6] = 0.0;
        			acadoVariables.y[i*NY+7] = 1.186347700078462; 	
        			acadoVariables.y[i*NY+8] = 2.7;
				acadoVariables.y[i*NY+9] = -1.678337270339664;
        			acadoVariables.y[i*NY+10] = 0.0; 		
				acadoVariables.y[i*NY+11] = 0.0;
    			}


				acadoVariables.yN[0] = 1.8;  		
				acadoVariables.yN[1] = 1.186347700078462; 		
				acadoVariables.yN[2] = -1.201023842411613; 		
				acadoVariables.yN[3] = -0.046154929792893;		
				acadoVariables.yN[4] = 0.000000000000000;
				acadoVariables.yN[5] = 0.0;
				acadoVariables.yN[6] = 0.0;
				acadoVariables.yN[7] = 1.186347700078462;    
				acadoVariables.yN[8] = 2.7;
				acadoVariables.yN[9] = -1.678337270339664;

			/* Weighting Matrices */
			//input.W = diag([10 0.0001 100 100 0.0001 0.0001 10 1 20000 20000 1 1]);
			//input.WN = input.W(1:10,1:10);
			
			for (i = 0; i < NY*NY; i++)
			  acadoVariables.W[i] = 1e-8;
			
			acadoVariables.W[0*NY+0] = 0.5; 
			acadoVariables.W[1*NY+1] = 0.5;
			acadoVariables.W[2*NY+2] = 0.001;
			acadoVariables.W[3*NY+3] = 0.001;
			acadoVariables.W[4*NY+4] = 0.001;
			acadoVariables.W[5*NY+5] = 0.001;
			acadoVariables.W[6*NY+6] = 0.001;
			acadoVariables.W[7*NY+7] = 1;
			acadoVariables.W[8*NY+8] = 400;
			acadoVariables.W[9*NY+9] = 400;
			acadoVariables.W[10*NY+10] = 10;
			acadoVariables.W[11*NY+11] = 100;       
			


			for (i = 0; i < NYN*NYN; i++)
			  acadoVariables.WN[i] = 1e-8;
      
			acadoVariables.WN[0*NYN+0] = acadoVariables.W[0*NY+0];
                        acadoVariables.WN[1*NYN+1] = acadoVariables.W[1*NY+1];
                        acadoVariables.WN[2*NYN+2] = acadoVariables.W[2*NY+2];
                        acadoVariables.WN[3*NYN+3] = acadoVariables.W[3*NY+3];
                        acadoVariables.WN[4*NYN+4] = acadoVariables.W[4*NY+4];
                        acadoVariables.WN[5*NYN+5] = acadoVariables.W[5*NY+5];
                        acadoVariables.WN[6*NYN+6] = acadoVariables.W[6*NY+6];
                        acadoVariables.WN[7*NYN+7] = acadoVariables.W[7*NY+7];
                        acadoVariables.WN[8*NYN+8] = acadoVariables.W[8*NY+8];
                        acadoVariables.WN[9*NYN+9] = acadoVariables.W[9*NY+9];  


			/* MPC: initialize the current state feedback. */

			for (i = 0; i < NX; ++i)
			acadoVariables.x0[ i ] = acadoVariables.x[ i ];
			
			R0 = 2.7;
			h0 = 1.678337270339664;
			Rf = 3;
			hf = 1.159954745795368;
			// REMEMBER : Xf = [1.5, 1.636784578436209, -0.884034465885497, -0.056983803558526, -0.000000000000000, 0, 0, 1.636784578436209 ];

		}

        else 
                {	cout << "Experiment chosen: " << experimentNumber << endl;
			//Initial state -----> X0 = [1.6, 1.246490265867967, -1.17, -0.045261843850114, 0.000000000000001, 0, 0, 1.246490265867967];
			// State description:      l_tether     d_arm         alpha        beta              dd_arm    dalpha dbeta   d_arm_sp
			
			for (i = 0; i < N+1; ++i)
			{
        			acadoVariables.x[i*NX+0] = 1.6;
        			acadoVariables.x[i*NX+1] = 1.246490265867967;
        			acadoVariables.x[i*NX+2] = -1.17;
       	 			acadoVariables.x[i*NX+3] = -0.045261843850114;
        			acadoVariables.x[i*NX+4] = 0.000000000000001;
        			acadoVariables.x[i*NX+5] = 0.0;
        			acadoVariables.x[i*NX+6] = 0.0;
        			acadoVariables.x[i*NX+7] = 1.246490265867967;
    			}

    			for (i = 0; i < N; ++i)
    			{
        			acadoVariables.u[i*NU+0] = 0.0;
				acadoVariables.u[i*NU+1] = 0.0;
    			}


			/* Initialize the measurements/reference. */
			//Initial tracked variables radius and height ----> Y0 = [2.673752570118138,1.473200956377817]; 
    			for (i = 0; i < N; ++i)
    			{
        			acadoVariables.y[i*NY+0] = 1.6;  		
        			acadoVariables.y[i*NY+1] = 1.246490265867967; 		
        			acadoVariables.y[i*NY+2] = -1.17; 		
        			acadoVariables.y[i*NY+3] = -0.045261843850114;		
        			acadoVariables.y[i*NY+4] = 0.000000000000000;
        			acadoVariables.y[i*NY+5] = 0.0;
        			acadoVariables.y[i*NY+6] = 0.0;
        			acadoVariables.y[i*NY+7] = 1.246490265867967; 	
        			acadoVariables.y[i*NY+8] = 2.673752570118138;
				acadoVariables.y[i*NY+9] = -1.473200956377817;
        			acadoVariables.y[i*NY+10] = 0.0; 		
				acadoVariables.y[i*NY+11] = 0.0;
    			}


				acadoVariables.yN[0] = 1.6;  		
				acadoVariables.yN[1] = 1.246490265867967; 		
				acadoVariables.yN[2] = -1.17; 		
				acadoVariables.yN[3] = -0.045261843850114;		
				acadoVariables.yN[4] = 0.000000000000000;
				acadoVariables.yN[5] = 0.0;
				acadoVariables.yN[6] = 0.0;
     				acadoVariables.yN[7] = 1.246490265867967;    
				acadoVariables.yN[8] = 2.673752570118138;
				acadoVariables.yN[9] = -1.473200956377817;

			
			/* MPC: initialize the current state feedback. */

			for (i = 0; i < NX; ++i)
			acadoVariables.x0[ i ] = acadoVariables.x[ i ];	
			R0 = 2.673752570118138;
			h0 = 1.473200956377817;
			A = 0.05;
			w = 2*M_PI*0.1;
			OFFSET = -1.17;
				
			 /* Weighting Matrices */
			//For experiment 1:
			//input.W = diag([10 0.0001 24000 1000 0.0001 0.0001 0.001 1 20000 0 10 1]);
			//input.WN = input.W(1:10,1:10);
			if ( experimentNumber == 1  || experimentNumber == 4)
			{
				cout << "Preparing W matrices for experiment 1" << endl;
				 for (i = 0; i < NY*NY; i++)
				acadoVariables.W[i] = 1e-8;
				
				acadoVariables.W[0*NY+0] = 150;	//l_tether 
				acadoVariables.W[1*NY+1] = 0.5;	//arm_speed
				acadoVariables.W[2*NY+2] = 600;		//alpha
				acadoVariables.W[3*NY+3] = 0.0001;		//beta
				acadoVariables.W[4*NY+4] = 0.0001;		//arm_acceleration
				acadoVariables.W[5*NY+5] = 0.0001;		//alpha_speed
				acadoVariables.W[6*NY+6] = 0.0001;		//beta_speed
				acadoVariables.W[7*NY+7] = 1;	//arm_speed_SP
				acadoVariables.W[8*NY+8] = 300;	//radius
				acadoVariables.W[9*NY+9] = 0.001;	//height
				acadoVariables.W[10*NY+10] = 10;	//Control: arm acc
				acadoVariables.W[11*NY+11] = 100;       	//Control: tether speed
			


				for (i = 0; i < NYN*NYN; i++)
				    acadoVariables.WN[i] = 1e-8;
      
				acadoVariables.WN[0*NYN+0] = acadoVariables.W[0*NY+0]; 
				acadoVariables.WN[1*NYN+1] = acadoVariables.W[1*NY+1];
				acadoVariables.WN[2*NYN+2] = acadoVariables.W[2*NY+2];
				acadoVariables.WN[3*NYN+3] = acadoVariables.W[3*NY+3];
				acadoVariables.WN[4*NYN+4] = acadoVariables.W[4*NY+4];
				acadoVariables.WN[5*NYN+5] = acadoVariables.W[5*NY+5];
				acadoVariables.WN[6*NYN+6] = acadoVariables.W[6*NY+6];
				acadoVariables.WN[7*NYN+7] = acadoVariables.W[7*NY+7];
				acadoVariables.WN[8*NYN+8] = acadoVariables.W[8*NY+8];
				acadoVariables.WN[9*NYN+9] = acadoVariables.W[9*NY+9];  
			}

			
			//For experiment 2:
			//input.W = diag([10 0.0001 24000 1000 0.0001 0.0001 0.001 1 0 20000 10 1]);
			//input.WN = input.W(1:10,1:10);
			
			if ( experimentNumber == 2 )
			{
				cout << "Preparing W matrices for experiment 2" << endl;
				 for (i = 0; i < NY*NY; i++)	
				    acadoVariables.W[i] = 1e-8;
			
				acadoVariables.W[0*NY+0] = 0.5; 
				acadoVariables.W[1*NY+1] = 0.5;
				acadoVariables.W[2*NY+2] = 500;
				acadoVariables.W[3*NY+3] = 0.001;
				acadoVariables.W[4*NY+4] = 0.001;
				acadoVariables.W[5*NY+5] = 0.001;
				acadoVariables.W[6*NY+6] = 0.001;
				acadoVariables.W[7*NY+7] = 1;
				acadoVariables.W[8*NY+8] = 0.001;
				acadoVariables.W[9*NY+9] = 250;
				acadoVariables.W[10*NY+10] = 10;
				acadoVariables.W[11*NY+11] = 100;       
			


				for (i = 0; i < NYN*NYN; i++)
				    acadoVariables.WN[i] = 1e-8;
      
				acadoVariables.WN[0*NYN+0] = acadoVariables.W[0*NY+0]; 
				acadoVariables.WN[1*NYN+1] = acadoVariables.W[1*NY+1];
				acadoVariables.WN[2*NYN+2] = acadoVariables.W[2*NY+2];
				acadoVariables.WN[3*NYN+3] = acadoVariables.W[3*NY+3];
				acadoVariables.WN[4*NYN+4] = acadoVariables.W[4*NY+4];
				acadoVariables.WN[5*NYN+5] = acadoVariables.W[5*NY+5];
				acadoVariables.WN[6*NYN+6] = acadoVariables.W[6*NY+6];
				acadoVariables.WN[7*NYN+7] = acadoVariables.W[7*NY+7];
				acadoVariables.WN[8*NYN+8] = acadoVariables.W[8*NY+8];
				acadoVariables.WN[9*NYN+9] = acadoVariables.W[9*NY+9];  
			}




                }


	/* Prepare first step */
	preparationStep();

}

bool NmpcController::configureHook()
{



	FlowStatus measurementStatus = portResampledMeasurements.read(Sensors);
        if (measurementStatus != NewData)
        {
                log(Info) << "winchController: First read to measurements port was indeed not newData!" << endlog();
        }

	
	        /* WINCH CALIBRATION ROUTINE*/
        cout << "==========================" << endl;
        cout << "== Winch Calibrating... ==" << endl;
        cout << "==========================" << endl;

        double speed = 0.1;

        for (int i=0; i<8 ;i++)
                {

                double trigger = TimeService::Instance()->getTicks();
                driveCommand.winchSpeedSetpoint =-speed; // m/s
                driveCommand.ts_trigger = trigger;
                double ts_elapsed = TimeService::Instance()->secondsSince( trigger );
                driveCommand.ts_elapsed = ts_elapsed;
                portDriveCommand.write(driveCommand);
                usleep(500000);
                speed=-speed;

                }
                portResampledMeasurements.read(Sensors);
                double trigger = TimeService::Instance()->getTicks();
                driveCommand.winchSpeedSetpoint = 0.1; // m/s
                driveCommand.ts_trigger = trigger;
                double ts_elapsed = TimeService::Instance()->secondsSince( trigger );
                driveCommand.ts_elapsed = ts_elapsed;
                portDriveCommand.write(driveCommand);
                cout << "Velocity value was sent to winch controller v_winch=0.1m/s" << endl;
                cout << "Limit Switch " << Sensors.limitSwitch << endl;
                while(Sensors.limitSwitch < 1 || Sensors.limitSwitch > 1)
                        {

                        portResampledMeasurements.read(Sensors);

                        }
	        cout << "Calibration succes!" << endl;

		controlNMPC.u_carousel_speed = 0;
        	controlNMPC.u_winch_speed = 0;
        	controlNMPC.ts_trigger = trigger;
        	controlNMPC.ts_elapsed = ts_elapsed;
        	portDriveCommand.write(driveCommand);
        	portControl.write(controlNMPC);


    		cout << "MPC configured succesfully!" << endl;

	state = 0; // (used for the system initialization, look in initialize function)
	OFFSET_TETHER = Sensors.winchEncoderPosition;
	system_initialized = 0;
	return true;
}



bool  NmpcController::startHook()
{   
    
    ellapsed_time = 0.0;
    total_time = 0.0;
    max_time = 0.0;
    //period = this->getActivity()->getPeriod();
    period = 0.02;
    cout << "CONTROL PERIOD =" << period << endl;
    iterations = 0;
    // These 3 parameters work in the case of having chosen experiment 3
    T_helix = 8; // Time of arrival
    a = 0.303147675172103; // Manifold coefficients
    b = -3.888283822344296;

    Rref = R0;
    	
	return true;
}

void  NmpcController::updateHook()
{


        if (system_initialized == 0)
        {
                TIME_TYPE trigger = TimeService::Instance()->getTicks();
                if (experimentNumber ==3)
                {
                tether_length0 = 1.8;
                arm_speed0 = 1.186347700078462;
                }
                else
                {
                tether_length0 = 1.6;
                arm_speed0 = 1.246490265867967;
                }

                portResampledMeasurements.read(Sensors);

                // Reaching desired initial tether length

                actualLength = 1.3 - OFFSET_TETHER + Sensors.winchEncoderPosition;
                double error = tether_length0 - actualLength;
                double epsilon = 0.01;
                if (error > epsilon)

                {
                        winchSpeedSetpoint = -0.1;
                }

                else if (error < -epsilon)

                {

                        winchSpeedSetpoint = 0.1;
                }
		else

                {
                        winchSpeedSetpoint = 0;
                }


                // Reaching desired initial carousel speed

                double stepheight = 0.02*0.1; // Sampling time times acceleration 
                if(fabs(state-arm_speed0)>stepheight)
                {
                        if(state > arm_speed0)
                        {
                                state = max(arm_speed0, state - stepheight );
                        }
                        else
                        {
                        state = min(arm_speed0, state + stepheight);
                        }
                }
                else
                {
                        state = arm_speed0;
                }

        driveCommand.winchSpeedSetpoint = winchSpeedSetpoint;
        driveCommand.carouselSpeedSetpoint = state; // rad/s
        driveCommand.ts_trigger = trigger;
        double ts_elapsed = TimeService::Instance()->secondsSince( trigger );
        driveCommand.ts_elapsed = ts_elapsed;
        controlNMPC.u_carousel_speed = state;
	controlNMPC.u_winch_speed = winchSpeedSetpoint;
	controlNMPC.ts_trigger = trigger;
	controlNMPC.ts_elapsed = ts_elapsed;
	portDriveCommand.write(driveCommand);
	portControl.write(controlNMPC);
        } // END : Loop for system not yet initialized
        
	else

	{


		  TIME_TYPE start_trigger = TimeService::Instance()->getTicks();

		  int i;
		  double Ts = 0.02;
		  timer t;
		  /* Get the time before start of the loop. */
		  tic( &t );
		  double cur_time = ellapsed_time;
		  // STEPS 1 AND 2 ARE OCCUPIED BY MHE
		  
		  if ( experimentNumber == 3 )
		  {
		  //cout << "EXP 3" << endl;
		    // 3 . Prepare MPC Reference Experiment 3
		    
			for (i = 0; i < N; ++i)
			{
				double t_ref = cur_time+i*Ts;
				Rref = (t_ref*(Rf-R0)/T_helix)+R0;
				
				if (Rref >= Rf)
				  Rref = Rf;
				Href = (a*Rref*Rref)+b; 
				
				// Xf = [1.5, 1.636784578436209, -0.884034465885497, -0.056983803558526, -0.000000000000000, 0, 0, 1.636784578436209 ];
				acadoVariables.y[i*NY] = 1.5;  		
				acadoVariables.y[i*NY+1] = 1.636784578436209; 	
				acadoVariables.y[i*NY+2] = -0.884034465885497; 		
				acadoVariables.y[i*NY+3] = -0.056983803558526;		
				acadoVariables.y[i*NY+4] = 0.000000000000000;
				acadoVariables.y[i*NY+5] = 0.0;
				acadoVariables.y[i*NY+6] = 0.0;
				acadoVariables.y[i*NY+7] = 1.636784578436209; 	
				acadoVariables.y[i*NY+8] = Rref;
				acadoVariables.y[i*NY+9] = Href;
				acadoVariables.y[i*NY+10] = 0.0;
				acadoVariables.y[i*NY+11] = 0.0;
			}

			double t_ref = cur_time+N*Ts;
			Rref = (t_ref*(Rf-R0)/T_helix)+R0;
			if (Rref >= Rf)
			  Rref = Rf;
			Href = (a*Rref*Rref)+b;
			
			acadoVariables.yN[0] = 1.5;  		
			acadoVariables.yN[1] = 1.636784578436209; 		
			acadoVariables.yN[2] = -0.884034465885497; 		
			acadoVariables.yN[3] = -0.056983803558526;		
			acadoVariables.yN[4] = 0.000000000000000;
			acadoVariables.yN[5] = 0.0;
			acadoVariables.yN[6] = 0.0;
			acadoVariables.yN[7] = 1.636784578436209;  
			acadoVariables.yN[8] = Rref;
			acadoVariables.yN[9] = Href;	  
	  
		}
	
		else if (experimentNumber == 4)
	
		{	  
	  
		  // 3 . Prepare MPC Reference Experiments 1, 2 or 4
      

			for (i = 0; i < N; ++i)
                        {
                                double t_ref = cur_time+i*Ts;
                                double alpha_ref = OFFSET + A*sin(w*t_ref);

                                acadoVariables.y[i*NY] = l_tether_precalc[iterations+i];
                                acadoVariables.y[i*NY+1] = 1.246490265867967;
                                acadoVariables.y[i*NY+2] = alpha_ref;
                                acadoVariables.y[i*NY+3] = -0.045261843850114;
                                acadoVariables.y[i*NY+4] = 0.000000000000001;
                                acadoVariables.y[i*NY+5] = 0.0;
                                acadoVariables.y[i*NY+6] = 0.0;
                                acadoVariables.y[i*NY+7] = 1.246490265867967;
                                acadoVariables.y[i*NY+8] = R0;
                                acadoVariables.y[i*NY+9] = -h0;
                                acadoVariables.y[i*NY+10] = 0.0;
                                acadoVariables.y[i*NY+11] = 0.0;
                        }

                        double t_ref = cur_time+N*Ts;
                        double alpha_ref = OFFSET + A*sin(w*t_ref);

                        acadoVariables.yN[0] = l_tether_precalc[iterations+N];
                        acadoVariables.yN[1] = 1.246490265867967;
                        acadoVariables.yN[2] = alpha_ref;
                        acadoVariables.yN[3] = -0.045261843850114;
                        acadoVariables.yN[4] = 0.000000000000001;
                        acadoVariables.yN[5] = 0.0;
                        acadoVariables.yN[6] = 0.0;
                        acadoVariables.yN[7] = 1.246490265867967;
                        acadoVariables.yN[8] = R0;
                        acadoVariables.yN[9] = -h0;



		}
		

		else
		{	 
	      		for (i = 0; i < N; ++i)
	      		{
		      		double t_ref = cur_time+i*Ts;
		      		double alpha_ref = OFFSET + A*sin(w*t_ref);
		  
		      		acadoVariables.y[i*NY] = 1.6;  		
		     	 	acadoVariables.y[i*NY+1] = 1.246490265867967; 	
		      		acadoVariables.y[i*NY+2] = alpha_ref; 		
		      		acadoVariables.y[i*NY+3] = -0.045261843850114;		
		      		acadoVariables.y[i*NY+4] = 0.000000000000001;
		      		acadoVariables.y[i*NY+5] = 0.0;
		      		acadoVariables.y[i*NY+6] = 0.0;
		      		acadoVariables.y[i*NY+7] = 1.246490265867967; 	
		      		acadoVariables.y[i*NY+8] = R0;
		      		acadoVariables.y[i*NY+9] = -h0;
		      		acadoVariables.y[i*NY+10] = 0.0;
		      		acadoVariables.y[i*NY+11] = 0.0;
	      		}

	      		double t_ref = cur_time+N*Ts;
	      		double alpha_ref = OFFSET + A*sin(w*t_ref);

	      		acadoVariables.yN[0] = 1.6;  		
	      		acadoVariables.yN[1] = 1.246490265867967; 		
	      		acadoVariables.yN[2] = alpha_ref; 		
	      		acadoVariables.yN[3] = -0.045261843850114;		
	      		acadoVariables.yN[4] = 0.000000000000001;
	      		acadoVariables.yN[5] = 0.0;
	      		acadoVariables.yN[6] = 0.0;
	      		acadoVariables.yN[7] = 1.246490265867967;  
	      		acadoVariables.yN[8] = R0;
	      		acadoVariables.yN[9] = -h0;
		}	      
	  
	
	
	// 4. Pass the state estimate from MHE and perform feedback step
	//
	//
	// 
	//
	portStateEstimate.read(stateEstimate);
	
	// UNCOMMENT TO SEE OPEN LOOP RESPONSE	
//	for (i = 0; i < NX; ++i)
//	      acadoVariables.x0[ i ] = acadoVariables.x[NX + i];
	
	acadoVariables.x0[ 0 ] = stateEstimate.l_tetherEst;
	acadoVariables.x0[ 1 ] = stateEstimate.ddelta_armEst;
	acadoVariables.x0[ 2 ] = stateEstimate.alphaEst;
	acadoVariables.x0[ 3 ] = stateEstimate.betaEst;
	acadoVariables.x0[ 4 ] = stateEstimate.dddelta_armEst;
	acadoVariables.x0[ 5 ] = stateEstimate.dalphaEst;
	acadoVariables.x0[ 6 ] = stateEstimate.dbetaEst;
	acadoVariables.x0[ 7 ] = stateEstimate.ddelta_carousel_spEst;


	feedbackStep( );
	
	 // 5. Apply the new control immediately to the process, first NU components.
	 // Also shift MPC inputs and controls
	 double u_carousel_speed = acadoVariables.x[1*NX + 7];
	 double u_carousel_acc = acadoVariables.u[0];
	 double u_winch_speed = acadoVariables.u[1];
	//cout << "CONTROLS: carousel= " << u_carousel_speed <<"    winch= "<< u_winch_speed << endl;
	 clamp(u_carousel_speed,1,2);
	 clamp(u_winch_speed,-0.4,0.4);
	 //driveCommand.carouselSpeedSetpoint = clamped_carousel_speed;
	if (controlling == 1)
		{
	 	driveCommand.carouselSpeedSetpoint = u_carousel_speed;
	 	driveCommand.winchSpeedSetpoint = u_winch_speed;
	 	}
	else
		{	
	       	driveCommand.carouselSpeedSetpoint = arm_speed0;
        	driveCommand.winchSpeedSetpoint = 0;
		}

	 driveCommand.ts_trigger = TimeService::Instance()->getTicks();
	 driveCommand.ts_elapsed = TimeService::Instance()->secondsSince( start_trigger );
	 
	 portDriveCommand.write(driveCommand);	
	 real_t te_feedback = toc( &t );
	 tic( &t );
	 if( VERBOSE ) printf("\tReal-Time Iteration time %f, time for feedback %f:  KKT Tolerance = %.9e\n", ellapsed_time, te_feedback, getKKT() );	
	 //
	 // Prepare for the next iteration
	 //

	  shiftStates(2, 0, 0);
	  shiftControls( 0 );
	  
	 // 6. Step 6 depends on the real system (to be captured by MHE)
	 /* Prepare for the next step. */
	  preparationStep();
	 /* Read the elapsed time. */
	  real_t te_preparation = toc( &t );

	  if( VERBOSE ) printf("\tTime for preparation: %f\n\n", te_preparation );

	  double te = te_preparation + te_feedback;

	  if (te > max_time) max_time = te;

	  total_time+= te;

	  
	  ellapsed_time += period; // count ellapsed time
	  
	  
	  
	  // 7. Pass the controls to the MHE for further estimation
	 
	  controlNMPC.u_carousel_speed = u_carousel_speed;
	  controlNMPC.u_carousel_acc = u_carousel_acc;
	  controlNMPC.u_winch_speed = u_winch_speed;
	  controlNMPC.ts_trigger = driveCommand.ts_trigger;
	  controlNMPC.ts_elapsed = driveCommand.ts_elapsed;
	  portControl.write(controlNMPC);
	  
	  // Debug important information
	portResampledMeasurements.read(Sensors);	  
	  if ( experimentNumber == 1 )
	  {
	    
	  nmpcDebug.u_carousel_speed = u_carousel_speed;
	  nmpcDebug.u_carousel_acc = u_carousel_acc;
	  nmpcDebug.u_winch_speed = u_winch_speed;
	  nmpcDebug.tether_speedMeas = Sensors.winchSpeedSmoothed;
	  nmpcDebug.Rref = R0;
	  nmpcDebug.Href = -h0;
	  nmpcDebug.ref_tether_length = acadoVariables.y[0];
	  nmpcDebug.ref_alpha = acadoVariables.y[2];
	  nmpcDebug.ref_ddelta = 1.246490265867967;
	  nmpcDebug.time_feedback = te_feedback;
	  nmpcDebug.time_preparation = te_preparation;
	  nmpcDebug.ts_trigger = driveCommand.ts_trigger;
	  nmpcDebug.ts_elapsed = driveCommand.ts_elapsed;
	  portDebug.write(nmpcDebug);
	  

	  }
	  
	  else if ( experimentNumber == 2 )
	    
	  {
	  
	  nmpcDebug.u_carousel_speed = u_carousel_speed;
	  nmpcDebug.u_carousel_acc = u_carousel_acc;
	  nmpcDebug.u_winch_speed = u_winch_speed;
	  nmpcDebug.tether_speedMeas = Sensors.winchSpeedSmoothed;
	  nmpcDebug.Rref = R0;
	  nmpcDebug.Href = -h0;
	  nmpcDebug.ref_tether_length = acadoVariables.y[0];
	  nmpcDebug.ref_alpha = acadoVariables.y[2];
	  nmpcDebug.ref_ddelta = 1.246490265867967;
	  nmpcDebug.time_feedback = te_feedback;
	  nmpcDebug.time_preparation = te_preparation;
	  nmpcDebug.ts_trigger = driveCommand.ts_trigger;
	  nmpcDebug.ts_elapsed = driveCommand.ts_elapsed;
	  portDebug.write(nmpcDebug);

	  
	  }
	  
	  else if (experimentNumber == 4)
	  {

		nmpcDebug.u_carousel_speed = u_carousel_speed;
	        nmpcDebug.u_carousel_acc = u_carousel_acc;
        	nmpcDebug.u_winch_speed = u_winch_speed;
          	nmpcDebug.tether_speedMeas = Sensors.winchSpeedSmoothed;
          	nmpcDebug.Rref = R0;
          	nmpcDebug.Href = -h0;
		nmpcDebug.ref_tether_length = l_tether_precalc[iterations];
          	nmpcDebug.ref_alpha = acadoVariables.y[2];
          	nmpcDebug.ref_ddelta = 1.246490265867967;
          	nmpcDebug.time_feedback = te_feedback;
          	nmpcDebug.time_preparation = te_preparation;
          	nmpcDebug.ts_trigger = driveCommand.ts_trigger;
          	nmpcDebug.ts_elapsed = driveCommand.ts_elapsed;
          	portDebug.write(nmpcDebug);

	  }


	  else
	  {
	    
	  nmpcDebug.u_carousel_speed = u_carousel_speed;
	  nmpcDebug.u_carousel_acc = u_carousel_acc;
	  nmpcDebug.u_winch_speed = u_winch_speed;
	  nmpcDebug.tether_speedMeas = Sensors.winchSpeedSmoothed;
	  nmpcDebug.Rref = acadoVariables.y[8];
	  nmpcDebug.Href = acadoVariables.y[9];
	  nmpcDebug.ref_tether_length = acadoVariables.y[0];
	  nmpcDebug.ref_alpha = -0.884034465885497;
	  nmpcDebug.ref_ddelta = 1.636784578436209;
	  nmpcDebug.time_feedback = te_feedback;
	  nmpcDebug.time_preparation = te_preparation;
	  nmpcDebug.ts_trigger = driveCommand.ts_trigger;
	  nmpcDebug.ts_elapsed = driveCommand.ts_elapsed;
	  portDebug.write(nmpcDebug);
	    
	  }

    iterations++;

	if (experimentNumber == 4)

	{
		if (iterations == 4461)
		{
			this->stop();
		}

	}

    if (iterations >= 5000)
        this->stop();
	}	
}


void  NmpcController::stopHook()
{
  log(RealTime) << "stophook"<< endlog();

    printf("\n\n Average time of one real-time iteration:   %.5g milliseconds\n", 1e3 * total_time / iterations );
    printf("\n\n Maximum time of one real-time iteration:   %.5g milliseconds\n", 1e3 * max_time);
  
}

void  NmpcController::cleanupHook()
{
  log(RealTime) << "cleanuphook"<< endlog();
  
}

void  NmpcController::errorHook()
{
  
  log(RealTime) << "errorhook"<< endlog();
  
}

ORO_CREATE_COMPONENT( NmpcController )
