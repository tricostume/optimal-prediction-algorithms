%------------------Albert-Ludwigs-Universitaet Freiburg--------------------
%___________________M. Sc. in Microsystems Engineering_____________________
%Thesis: 
%Period of preparation: April-September 2015
%Author: Ernesto Denicia
%Script: Implementation of integral action and state feedback together
%Description: This implementation refers to chapter 5 and includes integral
%action and a full state matrix approach.
%Comments: Note that the system is now augmented in one state corresponding
%to the integral error of the elevation angle alpha.
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%% Load feedback and correction matrices as well as model
clear all; close all; clc; 
 load('K.mat')
 load('L_matrix.mat')
 load('linear_model.mat')
 %% Augmentate model
 alpha0 = -69*pi/180;
alpha1 = -61*pi/180;
%2.Map references into state space
xss0=lsq_alpha(alpha0);
xss1=lsq_alpha(alpha1);
xss=lsq_alpha((alpha1+alpha0)/2);

%%
 %%Modify K and L matrices => K=[K_LQR,K_i] L=[L;0,0]

 Ki=1.3;
 K=[K,Ki];
 K(7)=0.08;
  K(2)=-1.1;
  K(5)=0.005;
  K(1)=0.005;
  K(4)=0.005;

 L=[L;0,0];
%% DLQG general preparation
%4.Number of samples to be considered and sampling time
% Account for integral action in the structures
N=1000;
Ts=0.1;
nx = length(xss0)+1;
%5.Preparation of debugging matrices and time vector
X = zeros(nx, N + 1);
U = zeros(1, N);
T = 0:Ts:(N*Ts);
X(:,1) = [xss0;0];
Xref = zeros(nx,N);
%% Estimation preparation
ny = size(C, 1);
Xest = X;
Yout = zeros(ny, N);
Xest(:,1) = [xss0;0];
%% Control loop
for k = 1:N
    %Estimator steps
    %1. Output prediction step
    yest = output(Xest(:, k));
    %2. Measurement step
    Yout(:, k) = output(X(:, k));
    %3. Update step, map errors to states with correction matrix L
    Xest(:, k) = Xest(:, k) + L*(Yout(:, k) - yest);
    
    
    % Prepare reference: Modify reference vectors
    if mod(T(k),40) < 20
        xref = [xss0;0];
    else
        xref = [xss1;0];
    end
    Xref(:, k) = xref;
    %Calculate error
    alpha_ierror = X(8,k);
    %Close the loop by applying full state feedback. NOTICE: Use estimates!
    U(k) = -K*([Xest(1:7, k);alpha_ierror]-Xref(:, k));
    
    % Simulation of the system: Modified system
    X(8,k)=X(2,k)-xref(2,1);
    X(:, k + 1) = simulate_step(X(:,k),U(k),2);

    %4. Estimator: State prediction step
    Xest(:, k + 1) = [xss;0] + [sysd.A,zeros(7,1);zeros(1,7),1]*(Xest(:, k) - [xss;0]) + [sysd.B;0]*U(k);
         
end
%% Plotting section
simulate_only