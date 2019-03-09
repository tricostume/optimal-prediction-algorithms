%------------------Albert-Ludwigs-Universitaet Freiburg--------------------
%___________________M. Sc. in Microsystems Engineering_____________________
%Thesis: 
%Period of preparation: April-September 2015
%Author: Ernesto Denicia
%Script: Final Lagrange formalism approach for the obtention of a SISO
%model of the HIGHWIND carousel.
%Description: The formulation uses the terms described in the thesis
%Comments: Be aware of the fact that this formalism already includes
%simplifications applied to it.
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%% 1. Definition of symbolic variables
clear all; close all; clc;
%Important positions
syms p_ball real %position of the ball in inertial frame, [m]
syms p_LAS real %position of LAS in inertial frame [m]

% Carousel parameters and variables
syms r_A real %arm length [m]
syms delta_arm ddelta_arm dddelta_arm ddddelta_arm real %rotation angle around arm dimension (NED reference) [rad]
syms I_A real %moment of inertia of the arm [kg m^2]

%LAS parameters and variables
syms l_t real %ltether length [m]
syms m_ball real %ball mass [kg]
syms I_LAS I_ball real % central moment of inertia of LAS and ball [kg m^2]
syms i_t real % constant part of the tether inertia [kg/m^2]
syms alpha dalpha ddalpha real %elevation angle measured from horizontal up [rad]
syms beta dbeta ddbeta real %azimuth angle of rope,lead positive[rad]
syms mu_beta mu_alpha real %linear friction coefficients of LAS  [Nms/rad]
syms mu_air real %air friction constant condensed [kg/m]

%Important constants
syms g real %gravity (kgm/s^2)

%Control
syms ddelta_motor_sp real %setpoint for the motor controller [rad/s]
assume(l_t>0)

%% Complete set of independen generalized coordinates
E = [alpha; beta];
dE = [dalpha; dbeta];
ddE = [ddalpha; ddbeta];

%% Expression for ball position and velocity
p_LAS = r_A*[sin(delta_arm);
                  cos(delta_arm);
                  0];
p_ball = p_LAS + l_t*[cos(alpha)*sin(delta_arm + beta); 
                      cos(alpha)*cos(delta_arm + beta);
                      -sin(alpha)];

v_ball = jacobian(p_ball, E)*dE + jacobian(p_ball, delta_arm)*ddelta_arm;

%% Energy flow 

%Kinetic coenergy
T = 0;
T = T + 1/2*m_ball*(v_ball.'*v_ball);
T = T + 1/2*I_A*ddelta_arm^2;
T = T + 1/2*I_LAS*dbeta^2;
T = T + 1/2*i_t*l_t*l_t*(dalpha^2+(cos(alpha)*cos(alpha))*dbeta^2);

%Potential energy
V = m_ball*g*(l_t*sin(alpha));


%% Construction of the Lagrangian
L = simplify(T - V);

%% Generalized forces formulation
%1. Define the non conservative variational work in the system. Here
%transformation from the reference frame to the generalized coordinates.
J = simplify(jacobian(p_ball,E));
%2. Calculation of the air torque on the ball as explained in the thesis
f_airfriction = simplify(-mu_air*v_ball*simplify(sqrt(v_ball.'*v_ball)));
gen_airfriction = simplify(J.'*f_airfriction);
%3. Determination of the friction torques of the LAS
T_alpha = simplify(-mu_alpha*dalpha);
T_beta = simplify(-mu_beta*dbeta);
%4. Vector of generalized forces
gen_forces = simplify([
                        T_alpha; 
                        T_beta;
                                             ] + gen_airfriction);
              

%% Obtention of implicit ODE
LE = simplify(jacobian(L, E).');
LdE = simplify(jacobian(L, dE));
LdEt = simplify(jacobian(LdE, E)*dE                 ...
        + jacobian(LdE, dE)*ddE                     ...
        + jacobian(LdE,delta_arm)*ddelta_arm        ...
        + jacobian(LdE,ddelta_arm)*dddelta_arm );   
        
implicit = simplify(LdEt - LE - gen_forces);

%% Obtention of the explicit dd_alpha_s and dd_beta_s
solution = solve(implicit(1), implicit(2), ddalpha, ddbeta);
xdot = simplify(expand([solution.ddalpha; solution.ddbeta])); 

