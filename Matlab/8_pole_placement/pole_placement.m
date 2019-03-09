%------------------Albert-Ludwigs-Universitaet Freiburg--------------------
%___________________M. Sc. in Microsystems Engineering_____________________
%Thesis: 
%Period of preparation: April-September 2015
%Author: Ernesto Denicia
%Script: Pole placement
%Description: This script generates a control law accomplishing with
%requirements. It is demonstrated though that pole placement loses track of
%the controls needed dor its accomplishment.
%Comments: Matrix K_pp is saved in the workspace to use it for closed loop
%simulations afterwards
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%%
clear all; close all; clc;
%1. Load extended linear model with u=dddelta_motor_sp and analyse its
% eigenvalues
load('linear_model.mat');

%% DISCRETE TIME
openloop_polesdis=eig(sysd)
%2. Choose proposed closed loop poles
dis_desired_poles=[    0.74 + 0.53i...
                       0.74 - 0.53i...
                       0.65 + 0.00i...
                       0.84 + 0.21i...
                       0.84 - 0.21i...
                       0.86 + 0.00i...
                       0.94 + 0.00i];   

%3. Generate the new characteristic polynomial with the linear control law
Kppd = place(sysd.A,sysd.B,dis_desired_poles);
%4. Quick proof, generate a new system and see its impulse response.
sysnew_dis=ss(sysd.A-sysd.B*Kppd,sysd.B,sysd.C,sysd.D,0.1);
impulse(sysnew_dis)




