%------------------Albert-Ludwigs-Universitaet Freiburg--------------------
%___________________M. Sc. in Microsystems Engineering_____________________
%Thesis: 
%Period of preparation: April-September 2015
%Author: Ernesto Denicia
%Script: Parameter estimation: Non linear least-square error minimisation
%Description: Script in charge of finding the optimal parameters minimising
%real system measurements and model predictions.
%Comments: Results are easily reachable in Results.mat file.
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%% Load the data to be used in the routine
clear all; close all; clc
%Choose experiment: "initial experiment" (initial guess generation) or 
                    %"multisteps" (full range optimisation)
initial_experiment
data(:,1)=alpha_real;
data(:,2)=beta_real;
data(:,3)=y;
%% Define initial guesses and program lsqnonlin
%theta0=[0.0099,0.4867,2.1715,1.0343];
% Initial guess initial_experiment
theta0=[0.007,0.1,3,0.4];
options=optimoptions(@lsqnonlin, 'display','iter');
[theta,resnorm,residual,exitflag,output,lambda,jacobian]=lsqnonlin(@(theta)residuals( t, u, data, theta),theta0,[],[],options);
%% Calculate variability measures
confidence = nlparci(theta,residual,'jacobian',jacobian);
[rows,cols]=size(jacobian);
% A posteriori variance factor
Sigma_o = sqrt((residual'*residual) / (rows-cols));
% Unscaled Covariance Matrix (Inverse of the Normal Equation matrix)
Q_xx  = inv(jacobian'*jacobian);
%  Precision measures ,i.e. the standard deviation of each parameter
diagonals_of_Q_xx = diag(Q_xx);
std_dev = Sigma_o .* sqrt(diagonals_of_Q_xx);