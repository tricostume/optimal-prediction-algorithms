%------------------Albert-Ludwigs-Universitaet Freiburg--------------------
%___________________M. Sc. in Microsystems Engineering_____________________
%Thesis: 
%Period of preparation: April-September 2015
%Author: Ernesto Denicia
%Script: DLQE + DLQR Simulation with augmented states
%Description: This script deals with the robustness problem of the DLQG by
%improving its performance with the use of the IMP method. This script
%needs an augmented state model and the auxiliary functions linearising it
%and discretising it directly from the integration step with MATLAB's
%ode15s. 
%Comments: Notice that in order to ESTIMATE use the linearised model at a
%tether length of 1.5 m (or the non linear model as well). In order to
%SIMULATE the real model, use the model at a tether length of 1.7 meters,
%otherwise there is no point on using the script.
%NOTICE: The real experiments are presented HERE.
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%%
clear all; close all; clc; 
% Choosse a mode: 1. See robustness at varied l_t=1.3m
%                 2. See robustness at varied l_t=1.7m
mode=1;
%1.Specify the changing references
alpha0 = -70*pi/180;
alpha1 = -60*pi/180;
%2.Map references into state space
xss0=lsq_alpha(alpha0);
xss1=lsq_alpha(alpha1);
xss=lsq_alpha((alpha1+alpha0)/2);

[A,B,C] = discretise_model(xss, 0);
%% DLQG general preparation
%4.Number of samples to be considered and sampling time
N=600;
Ts=0.1;
nx = length(xss0);
%5.Preparation of debugging matrices and time vector
X = zeros(nx, N + 1);
U = zeros(1, N);
T = 0:Ts:(N*Ts);
X(:,1) = xss0;
Xref = zeros(nx,N);
%% Full state feedback matrix
%3. Get feedback matrix from dlqr function
%Propose weighting matrix for optimal control problem
q_ddelta_arm = 0.1;
q_alpha = 10;
q_beta = 0.1;
q_dddelta_arm = 0.1;
q_dalpha = 5;
q_dbeta = 0.1;
q_ddelta_motor_sp = 0.1;
q_gamma = 0.1;

Q = diag([q_ddelta_arm,           ...
          q_alpha,                ...
          q_beta,                 ...
          q_dddelta_arm,          ... 
          q_dalpha,               ...
          q_dbeta,                ...
          q_ddelta_motor_sp,  ...
          q_gamma].^2);               ...

 R = (8).^2;

 %Retrieve K: Notice that discrete system is used
 [K, ~, ~] = dlqr(A, B, Q, R);
 %% Estimator correction matrix
% Process noise description
wn=0.001;
qe_ddelta_arm = 0;
qe_alpha = 0;
qe_beta = 0;
qe_dddelta_arm = wn;
qe_dalpha = wn;
qe_dbeta = wn;
qe_ddelta_motor_sp = 0;
qe_gamma = 0.001;

QE = diag([qe_ddelta_arm,           ...
           qe_alpha,                ...
           qe_beta,                 ...
           qe_dddelta_arm,          ... 
           qe_dalpha,               ...
           qe_dbeta,                ...
           qe_ddelta_motor_sp,      ...
           qe_gamma].^2);           ...
           
QEs = diag(2*[qe_ddelta_arm,        ...
           qe_alpha,                ...
           qe_beta,                 ...
           qe_dddelta_arm,          ... 
           qe_dalpha,               ...
           qe_dbeta,                ...
           qe_ddelta_motor_sp,      ...
           qe_gamma].^2);           ...

vn=0.0004; 
%Sensor noise description
RE = diag([vn,vn].^2);
REs = diag(1*[vn,vn].^2);
G=eye(nx);
%Notice that the discrete time system is used!
[L, P, Z, E] = dlqe(A, G, C, QE, RE);
%% Estimation preparation
ny = size(C, 1);
Xest = X;
Yout = zeros(ny, N);
Xest(:,1) = xss0;
%% Control loop
for k = 1:N
    %Estimator steps
    %1. Output prediction step
    yest = output(Xest(:, k));
    %2. Measurement step
    Yout(:, k) = output(X(:, k));%+ mvnrnd(zeros([2, 1]), REs)';
    %3. Update step, map errors to states with correction matrix L
    Xest(:, k) = Xest(:, k) + L*(Yout(:, k) - yest);
    
    
    % Prepare reference
    if mod(T(k),40) < 20
        xref = xss0;
    else
        xref = xss1;
    end
    Xref(:, k) = xref;
    
    %Close the loop by applying full state feedback. NOTICE: Use estimates!
    U(k) = -K*(Xest(:, k)-Xref(:, k));
    
    % Simulation of the system

     X(:, k + 1) = simulate_step(X(:,k),U(k),mode);%+ mvnrnd(zeros(nx, 1), QEs)';       
    
    %4. Estimator: State prediction step
    %Choose one of the two ways to estimate, linearly or nonlinearly.
    %Xest(:, k + 1) = simulate_step15(Xest(:,k),U(k));
    Xest(:, k + 1) = xss + A*(Xest(:, k) - xss) + B*U(k);
    
end
%% Plotting section
if mode == 1
read_estimator_data13
plot_results13
else if mode == 2
read_estimator_data17
plot_results17
    end
end

