%------------------Albert-Ludwigs-Universitaet Freiburg--------------------
%___________________M. Sc. in Microsystems Engineering_____________________
%Thesis: 
%Period of preparation: April-September 2015
%Author: Ernesto Denicia
%Script: DLQE Carousel measurement as input (not yet on carousel)
%Description: This script uses carousel measurements to check the
%functionality of the Kalman Filter.
%Comments: None
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%% Read nc files and load model
clear all; close all; clc; 
nc_estimator
load('linear_model.mat');
load('L_matrix.mat');

%% Estimator matrices initialization
% Initial estimate generation
xss0=lsq_alpha(-pi/2);
%Number of samples to be considered and sampling time
N=length(alpha_real);
Ts=0.1;
nx = length(xss0);
ny = size(C, 1);
%5.Preparation of debugging matrices and time vector
U = zeros(1, N);
T = 0:Ts:(N*Ts);
Yout = zeros(ny, N);
Xest(:,1) = xss0;
Xmod = zeros(nx, N + 1);
Xmod(:,1) = xss0;
%% Estimator loop

for k = 1:N
   % Save the motor speed sent for graphing and pass it to estimate
   % This step is necessary because the carousel is normally 
   % controlled with speed and not with acceleration. As known
   % this Kalman filter was designed to work with acceleration as
   % input.
    U(k) = u(k);
    %Estimator steps
    %1. Output prediction step
    yest = output(Xest(:, k));
    %2. Measurement step
    Yout(:, k) = [alpha_real(k),beta_real(k)]';
    %3. Update step, map errors to states with correction matrix L
    Xest(:, k) = [Xest(1:end-1, k);U(k)] + L*(Yout(:, k) - yest);
    %4. Estimator: State prediction step
    Xest(:, k + 1) = xss + sysd.A*(Xest(:, k) - xss);% + sysd.B*U(k);
    %5. Simulate what the pure model would have predicted
    Xmod(1:6,k + 1) = simulate_step(Xmod(1:6,k),U(k),1);
end
%% Plotting section
plot_results