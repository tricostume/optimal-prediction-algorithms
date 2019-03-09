#ifndef MHEESTIMATE_H
#define MHEESTIMATE_H

struct mheEstimate
{

	// Estimated states    
    double l_tetherEst;
    double delta_armEst;
    double ddelta_armEst;
    double alphaEst;
    double betaEst;
    double dddelta_armEst;
    double dalphaEst;
    double dbetaEst;
	// Estimated controls
    double ddelta_carousel_spEst;
    double ddelta_winch_spEst;
	//Estimated radius and height
    double Rest;
    double Hest;
	// Direct measurements
    double l_tetherMeas;
    double delta_armMeas;
    double alphaMeas;
    double betaMeas;
	// Extra measurements
    double delta_armMeas_Raw;
    double arm_speedMeas;
    double tether_speedMeas;
    double arm_accelMeas;
    	// References
    double ref_ltether;
    double ref_armspeed;
    double ref_alpha;
    double ref_beta;
    double ref_armacc;
    double ref_alphaspeed;
    double ref_betaspeed;
    double ref_R;
    double ref_H;
	
	// Important measured times
    double time_feedback;
    double time_preparation;

	// General time stamps for actualization fo the structure
	double ts_trigger;
	double ts_elapsed;
};


#endif
