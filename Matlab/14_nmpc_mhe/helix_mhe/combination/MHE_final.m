clc;
clear all;
close all;

EXPORT = 1;
Ts = 0.02;      % sampling time

DifferentialState l_tetherMHE delta_armMHE ddelta_armMHE alphaMHE betaMHE dddelta_armMHE dalphaMHE dbetaMHE;      % provide proper names for all differential states and controls etc.
Control ddelta_motor_spMHE dl_tetherMHE;

%% Differential Equations
m_ball = 0.57;
I_arm = 205;
I_LAS = 0.03203;
r_arm = 2.05;
g = 9.81;
wn = 6.2409;
d = 0.1288;
my_alphaMHE_LA = 0.4876;
my_betaMHE_LA = 2.1774;
mu_air = 0.0074;
I_tether = 1.0006;

% LAST QUESTION: HOW TO DEFINE A LINEAR INPUT COMBINED WITH THE ODE?
odeMHE = [ dot(l_tetherMHE)==-dl_tetherMHE;
        dot(delta_armMHE)==-ddelta_armMHE;
        dot(ddelta_armMHE)==dddelta_armMHE;
        dot(alphaMHE)==dalphaMHE;
        dot(betaMHE)==dbetaMHE;
        dot(dddelta_armMHE)==wn*wn*ddelta_motor_spMHE-2*d*wn*dddelta_armMHE-wn*wn*ddelta_armMHE;
        dot(dalphaMHE)==-(dalphaMHE*my_alphaMHE_LA + (I_tether*dbetaMHE^2*sin(2*alphaMHE))/2 + g*l_tetherMHE*m_ball*cos(alphaMHE) + (dbetaMHE^2*l_tetherMHE^2*m_ball*sin(2*alphaMHE))/2 + (ddelta_armMHE^2*l_tetherMHE^2*m_ball*sin(2*alphaMHE))/2 + dalphaMHE*l_tetherMHE^2*mu_air*(dalphaMHE^2*l_tetherMHE^2 + ddelta_armMHE^2*r_arm^2 + dbetaMHE^2*l_tetherMHE^2*cos(alphaMHE)^2 + ddelta_armMHE^2*l_tetherMHE^2*cos(alphaMHE)^2 + 2*dbetaMHE*ddelta_armMHE*l_tetherMHE^2*cos(alphaMHE)^2 + 2*ddelta_armMHE^2*l_tetherMHE*r_arm*cos(alphaMHE)*cos(betaMHE) + 2*dbetaMHE*ddelta_armMHE*l_tetherMHE*r_arm*cos(alphaMHE)*cos(betaMHE) - 2*dalphaMHE*ddelta_armMHE*l_tetherMHE*r_arm*sin(alphaMHE)*sin(betaMHE))^(1/2) + dbetaMHE*ddelta_armMHE*l_tetherMHE^2*m_ball*sin(2*alphaMHE) + ddelta_armMHE^2*l_tetherMHE*m_ball*r_arm*cos(betaMHE)*sin(alphaMHE) - dddelta_armMHE*l_tetherMHE*m_ball*r_arm*sin(alphaMHE)*sin(betaMHE) - ddelta_armMHE*l_tetherMHE*mu_air*r_arm*sin(alphaMHE)*sin(betaMHE)*(dalphaMHE^2*l_tetherMHE^2 + ddelta_armMHE^2*r_arm^2 + dbetaMHE^2*l_tetherMHE^2*cos(alphaMHE)^2 + ddelta_armMHE^2*l_tetherMHE^2*cos(alphaMHE)^2 + 2*dbetaMHE*ddelta_armMHE*l_tetherMHE^2*cos(alphaMHE)^2 + 2*ddelta_armMHE^2*l_tetherMHE*r_arm*cos(alphaMHE)*cos(betaMHE) + 2*dbetaMHE*ddelta_armMHE*l_tetherMHE*r_arm*cos(alphaMHE)*cos(betaMHE) - 2*dalphaMHE*ddelta_armMHE*l_tetherMHE*r_arm*sin(alphaMHE)*sin(betaMHE))^(1/2))/(m_ball*l_tetherMHE^2 + I_tether);
        dot(dbetaMHE)==-(dbetaMHE*my_betaMHE_LA + dddelta_armMHE*l_tetherMHE^2*m_ball*cos(alphaMHE)^2 - I_tether*dalphaMHE*dbetaMHE*sin(2*alphaMHE) - dalphaMHE*dbetaMHE*l_tetherMHE^2*m_ball*sin(2*alphaMHE) - dalphaMHE*ddelta_armMHE*l_tetherMHE^2*m_ball*sin(2*alphaMHE) + dbetaMHE*l_tetherMHE^2*mu_air*cos(alphaMHE)^2*(dalphaMHE^2*l_tetherMHE^2 + ddelta_armMHE^2*r_arm^2 + dbetaMHE^2*l_tetherMHE^2*cos(alphaMHE)^2 + ddelta_armMHE^2*l_tetherMHE^2*cos(alphaMHE)^2 + 2*dbetaMHE*ddelta_armMHE*l_tetherMHE^2*cos(alphaMHE)^2 + 2*ddelta_armMHE^2*l_tetherMHE*r_arm*cos(alphaMHE)*cos(betaMHE) + 2*dbetaMHE*ddelta_armMHE*l_tetherMHE*r_arm*cos(alphaMHE)*cos(betaMHE) - 2*dalphaMHE*ddelta_armMHE*l_tetherMHE*r_arm*sin(alphaMHE)*sin(betaMHE))^(1/2) + ddelta_armMHE*l_tetherMHE^2*mu_air*cos(alphaMHE)^2*(dalphaMHE^2*l_tetherMHE^2 + ddelta_armMHE^2*r_arm^2 + dbetaMHE^2*l_tetherMHE^2*cos(alphaMHE)^2 + ddelta_armMHE^2*l_tetherMHE^2*cos(alphaMHE)^2 + 2*dbetaMHE*ddelta_armMHE*l_tetherMHE^2*cos(alphaMHE)^2 + 2*ddelta_armMHE^2*l_tetherMHE*r_arm*cos(alphaMHE)*cos(betaMHE) + 2*dbetaMHE*ddelta_armMHE*l_tetherMHE*r_arm*cos(alphaMHE)*cos(betaMHE) - 2*dalphaMHE*ddelta_armMHE*l_tetherMHE*r_arm*sin(alphaMHE)*sin(betaMHE))^(1/2) + ddelta_armMHE^2*l_tetherMHE*m_ball*r_arm*cos(alphaMHE)*sin(betaMHE) + dddelta_armMHE*l_tetherMHE*m_ball*r_arm*cos(alphaMHE)*cos(betaMHE) + ddelta_armMHE*l_tetherMHE*mu_air*r_arm*cos(alphaMHE)*cos(betaMHE)*(dalphaMHE^2*l_tetherMHE^2 + ddelta_armMHE^2*r_arm^2 + dbetaMHE^2*l_tetherMHE^2*cos(alphaMHE)^2 + ddelta_armMHE^2*l_tetherMHE^2*cos(alphaMHE)^2 + 2*dbetaMHE*ddelta_armMHE*l_tetherMHE^2*cos(alphaMHE)^2 + 2*ddelta_armMHE^2*l_tetherMHE*r_arm*cos(alphaMHE)*cos(betaMHE) + 2*dbetaMHE*ddelta_armMHE*l_tetherMHE*r_arm*cos(alphaMHE)*cos(betaMHE) - 2*dalphaMHE*ddelta_armMHE*l_tetherMHE*r_arm*sin(alphaMHE)*sin(betaMHE))^(1/2))/(m_ball*l_tetherMHE^2*cos(alphaMHE)^2 + I_tether*cos(alphaMHE)^2 + I_LAS)
        ];          % <-- TODO: define the set of differential equations
    

%% SIMULATOR
acadoSet('problemname', 'sim');

sim = acado.SIMexport( Ts ); %DISCRETE SIMULATION ROUTINE SAMPLED AT Ts???

sim.setModel(odeMHE);      % pass the ODE model

sim.set( 'INTEGRATOR_TYPE',             'INT_RK4'   );  % RK4 method
sim.set( 'NUM_INTEGRATOR_STEPS',        2           );

if EXPORT
    sim.exportCode( 'export_SIM' );
    
    cd export_SIM
    make_acado_integrator('../simulate_system')
    cd ..
end
    
    
    %% Export of an optimization routine:
acadoSet('problemname', 'mhe');

N = 40;     % number of shooting intervals
ocpMHE = acado.OCP( 0.0, N*Ts, N );

r_cyl = (l_tetherMHE^2*cos(alphaMHE)^2 + r_arm^2 + 2*l_tetherMHE*r_arm*cos(alphaMHE)*cos(betaMHE))^(1/2);

index = [1,2,4,5];
h = [l_tetherMHE; delta_armMHE; alphaMHE; betaMHE; controls];
hN = [l_tetherMHE; delta_armMHE; alphaMHE; betaMHE];

W = acado.BMatrix(eye(length(h)));  % <-- TODO: weighting matrix for the stage cost
WN = acado.BMatrix(eye(length(hN))) ;

ocpMHE.minimizeLSQ( W, h );            % stage cost--> 
ocpMHE.minimizeLSQEndTerm( WN, hN );   % terminal cost

ocpMHE.setModel(odeMHE);      % pass the ODE model

mhe = acado.OCPexport( ocpMHE );
mhe.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
mhe.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
mhe.set( 'SPARSE_QP_SOLUTION',          'CONDENSING_N2');
mhe.set( 'FIX_INITIAL_STATE',           'NO');
mhe.set( 'INTEGRATOR_TYPE',             'INT_RK4'           );  % RK4 method
mhe.set( 'NUM_INTEGRATOR_STEPS',        2*N                 );
mhe.set( 'QP_SOLVER',                   'QP_QPOASES'    	);

if EXPORT
    mhe.exportCode( 'export_MHE' );
    
    global ACADO_;
    copyfile([ACADO_.pwd '/../../external_packages/qpoases'], 'export_MHE/qpoases')
    
    cd export_MHE
    make_acado_solver('../acado_MHEstep')
    cd ..
end

