%------------------Albert-Ludwigs-Universitaet Freiburg--------------------
%___________________M. Sc. in Microsystems Engineering_____________________
%Thesis: 
%Period of preparation: April-September 2015
%Author: Ernesto Denicia
%Script: Implementation of closed loop on the carousel
%Description: This script only retrieves the data got from the carousel
% and shows it in graphs for further analysis in the thesis.
%Comments: This is the small initial problem to then continue to the 
%bigger step.
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%% Load feedback and correction matrices as well as model
clear all; close all; clc; 
 load('K_matrix.mat')
 load('L_matrix.mat')
 load('linear_model.mat')
  %% The new setpoint xss1 has to be stablished
 xss1 = lsq_alpha(-0.907571211037051);
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
%% Estimation preparation
ny = size(C, 1);
Xest = X;
Yout = zeros(ny, N);
Xest(:,1) = xss0;
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

Q = diag([q_ddelta_arm,           ...
          q_alpha,                ...
          q_beta,                 ...
          q_dddelta_arm,          ... 
          q_dalpha,               ...
          q_dbeta,                ...
          q_ddelta_motor_sp].^2); ...

 R = (12).^2;

 %Retrieve K: Notice that discrete system is used
 [K, ~, ~] = dlqr(sysd.A, sysd.B, Q, R);
%% Control loop
for k = 1:N
    %Estimator steps
    %1. Output prediction step
    yest = output(Xest(:, k));
    %2. Measurement step
    Yout(:, k) = output(X(:, k));
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
    X(:, k + 1) = simulate_step(X(:,k),U(k),2);
    
    %4. Estimator: State prediction step
    Xest(:, k + 1) = xss + sysd.A*(Xest(:, k) - xss) + sysd.B*U(k);
         
end
%% Plotting section
read_estimator_data
plot_results
simulate_only