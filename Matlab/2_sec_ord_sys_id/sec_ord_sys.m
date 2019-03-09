%------------------Albert-Ludwigs-Universitaet Freiburg--------------------
%___________________M. Sc. in Microsystems Engineering_____________________
%Thesis: 
%Period of preparation: April-September 2015
%Author: Ernesto Denicia
%Script: Second order identification
%Description: A system with input ddelta_motor_sp and output ddelta_arm is
%designed with MATLAB's function lsqnonlin.
%Comments: Notice that the NetCD-Files are needed in order to see the
%results.
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%% Development section
clear all;close all; clc;
%1. Retrieve measurements from NetCD-Files
multisteps
%2. Call the function in charge of finding K,w and d.
%theta: Values of optimization problem
%confidence: Confidence of interval with alpha=0.95
%Q_xx: Covariance matrix of the parameters
%std_dev: Standar deviation of the parameters (sqrt(diag(Q_xx)))
[theta,confidence,Q_xx,std_dev]=sec_ord_optimisation(t,u,y);
theta
confidence
Q_xx
std_dev

%3. Name each of the parameters
K=theta(1);
wn=theta(2);
d=theta(3);
%4. Generate the system and simulate it
sos=tf([wn*wn],[1 2*d*wn wn*wn]);
sossim=lsim(sos,u,t);
%% Plotting section
figure;plot(t,y,'r')
hold on
plot(t,sossim)
axis([98 120 01.6 2.3])
grid on
legend('Measured arm velocity','Simulated arm velocity')
title('Second order system approximation')
xlabel('Time (s)')
ylabel('Arm speed [rad/s]')



