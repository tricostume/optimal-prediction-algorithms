%------------------Albert-Ludwigs-Universitaet Freiburg--------------------
%___________________M. Sc. in Microsystems Engineering_____________________
%Thesis: 
%Period of preparation: April-September 2015
%Author: Ernesto Denicia
%Script: Linearisation of the system
%Description: This script produces the state space matrices corresponding
%to the next formulation:
%x =[ddelta_arm alpha beta dddelta_arm dalpha dbeta]
%u = ddelta_motorsp
%y = [alpha]
%Comments: Not yet system for controlling
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%% Preparation of linearisation state from alpha
clear all; close all; clc;
% 1. Propose elevation angle
alphass = -1.1345;
% 2. Compute the steady state related to this elevation angle and check its
% optimality
xss = lsq_alpha(-1.1345)
xdotss = model(xss,xss(1))
%3. Get state space description for selected parameters
[A,B,C,D] = continuous_linearisation(xss, xss(1))
%% Debrief section: Matrices are shown here
% A =
% 
%          0         0         0    1.0000         0         0
%          0         0         0         0    1.0000         0
%          0         0         0         0         0    1.0000
%   -38.9488         0         0   -1.6077         0         0
%    -1.5369   -2.1103   -0.0083   -0.0205   -0.1545   -0.3631
%    -0.0000    0.0879   -1.9046   -1.4612    1.9325   -3.3151

% B =
% 
%          0
%          0
%          0
%    38.9488
%          0
%          0

% C =
% 
%          0    1.0000         0         0         0         0
% 

% D =
% 
%      0

%% Matrix exponential descomposition
%4. Generate continuous time state space representation
sys=ss(A,B,C,D);
%5. Drawing root locus diagram of the system
rlocus(sys);
%6. Obtaining poles and zeros of the system
[sysVec,sysVal]=eig(A);
sysVal=diag(sysVal);
axis([-12 6 -30 30]);
grid on;