%------------------Albert-Ludwigs-Universitaet Freiburg--------------------
%___________________M. Sc. in Microsystems Engineering_____________________
%Thesis: 
%Period of preparation: April-September 2015
%Author: Ernesto Denicia
%Script: DLQR Simulation
%Description: This script simulates possible results on LQR to be
%implemented on the carousel. The infinite horizon problem is solved with
%MATLAB's function dlqr.
%Comments: Further comments on performance are made in the thesis work.
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%%
clear all; close all; clc; 
load('linear_model.mat')
%1.Specify the changing references
alpha0 = -70*pi/180;
alpha1 = -60*pi/180;
%2.Map references into state space
xss0=lsq_alpha(alpha0);
xss1=lsq_alpha(alpha1);
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

 R = (6.5).^2;

 %Retrieve K: Notice that discrete system is used
 [K, ~, ~] = dlqr(sysd.A, sysd.B, Q, R);
%% Control preparation
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
%% Control loop
for k = 1:N
    % Prepare reference
    if mod(T(k),40) < 20
        xref = xss0;
    else
        xref = xss1;
    end
    Xref(:, k) = xref;
    
    %Close the loop by applying full state feedback
    U(k) = -K*(X(:, k)-Xref(:, k));
    
    % Simulation of the system
    X(:, k + 1) = simulate_step(X(:,k),U(k));    
         
end
%% Plotting section
%plots
figure(1);
clf;

%Elevation angle
ax(1) = subplot(2, 2, 1);
hold on;
plot(T(1:end-1), Xref(2, :), 'k')
xlabel('Time [s]')
ylabel('Elevation [rad]')
plot(T, X(2, :), 'b');
%legend('Reference', 'Simulation')
axis([0 60 -1.25 -1])
grid on;

%Arm speed
ax(2) = subplot(2, 2, 2);
hold on;
plot(T(1:end - 1), Xref(1, :), 'k')
plot(T, X(1, :),'b');
grid on;
xlabel('Time [s]')
ylabel('Arm Speed [rad/s]')
%legend('Reference','Simulation')

%Control u
ax(4) = subplot(2, 2, 4);
hold on;
grid on;
plot(T(1:end - 1), U,'Color',[0.6,0,0.8]);
legend('Simulation')
xlabel('Time [s]')
ylabel('Motor acceleration [rad/s^2]')

%Azimuth angle
ax(5) = subplot(2, 2, 3);
hold on;
grid on;
plot(T(1:end - 1), Xref(3, :), 'k')
plot(T, X(3, :), 'b')
xlabel('Time [s]')
ylabel('Azimuth angle [rad]')
legend('Reference', 'Simulation')

linkaxes(ax, 'x')
