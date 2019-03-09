%------------------Albert-Ludwigs-Universitaet Freiburg--------------------
%___________________M. Sc. in Microsystems Engineering_____________________
%Thesis: 
%Period of preparation: April-September 2015
%Author: Ernesto Denicia
%Script: DLQE + DLQR Simulation
%Description: This script simulates possible results on DLQR to be
%implemented on the carousel with DLQE as feedback. The infinite horizon 
%problem is solved with MATLAB's functions dlqr and dlqe for both blocks.
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

QE = diag([qe_ddelta_arm,           ...
           qe_alpha,                ...
           qe_beta,                 ...
           qe_dddelta_arm,          ... 
           qe_dalpha,               ...
           qe_dbeta,                ...
           qe_ddelta_motor_sp].^2); ...
           
QEs = diag(2*[qe_ddelta_arm,           ...
           qe_alpha,                ...
           qe_beta,                 ...
           qe_dddelta_arm,          ... 
           qe_dalpha,               ...
           qe_dbeta,                ...
           qe_ddelta_motor_sp].^2); ...

vn=0.0004; 
%Sensor noise description
RE = diag([vn,vn].^2);
REs = diag(1*[vn,vn].^2);
G=eye(nx);
%Notice that the discrete time system is used!
[L, P, Z, E] = dlqe(sysd.A, G, sysd.C, QE, RE);
variance = diag(Z);
std_dev = sqrt(variance);
% Joined root locus
zgrid on
hold on
rlocus(sysd.A-L*sysd.C,sysd.B,sysd.C(1,:),sysd.D(1));
hold on
rlocus(sysd.A-sysd.B*K,sysd.B,sysd.C(1,:),sysd.D(1));
legend('DLQE','DLQR')
axis([-1.1 1.1 -1.1 1.1])
% Obtain eigenvalues and eigenvectors
[eigVec_est,eigVal_est] = eig(sysd.A-L*sysd.C);
[eigVec_cont,eigVal_cont] = eig(sysd.A-sysd.B*K);
eigVal_est=diag(eigVal_est);
eigVal_cont=diag(eigVal_cont);
% Controller poles
%   0.751160053840272 + 0.535310509836703i
%   0.751160053840272 - 0.535310509836703i
%   0.929131405548966 + 0.157130141190946i
%   0.929131405548966 - 0.157130141190946i
%   0.798765492095064 + 0.000000000000000i
%   0.942385051618461 + 0.000000000000000i
%   0.898692769303838 + 0.000000000000000i

%Observer poles

%   0.743285295014140 + 0.538020322000352i
%   0.743285295014140 - 0.538020322000352i
%   0.696234866836936 + 0.250604518447589i
%   0.696234866836936 - 0.250604518447589i
%   0.719168510985670 + 0.360423419077775i
%   0.719168510985670 - 0.360423419077775i
%   0.999999999917733 + 0.000000000000000i
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
    Yout(:, k) = output(X(:, k))+ mvnrnd(zeros([2, 1]), REs)';
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
    X(:, k + 1) = simulate_step(X(:,k),U(k),2)+ mvnrnd(zeros(nx, 1), QEs)';
    
    %4. Estimator: State prediction step
    Xest(:, k + 1) = xss + sysd.A*(Xest(:, k) - xss) + sysd.B*U(k);
         
end
%% Plotting section
%plots
figure();
clf;

%Elevation angle
ax(1) = subplot(2, 2, 1);
hold on;
plot(T(1:end-1), Xref(2, :), 'k')
xlabel('Time [s]')
ylabel('Elevation [rad]')
plot(T, X(2, :), 'b');
plot(T, Xest(2, :), 'g');
%legend('Reference', 'Simulation', 'Estimate')
axis([0 60 -1.25 -1])
grid on;

%Arm speed
ax(2) = subplot(2, 2, 2);
hold on;
plot(T(1:end - 1), Xref(1, :), 'k')
plot(T, X(1, :),'b');
plot(T, Xest(1, :), 'g');
grid on;
xlabel('Time [s]')
ylabel('Arm Speed [rad/s]')
%legend('Reference','Simulation','Estimate')

%Control u
ax(4) = subplot(2, 2, 4);
hold on;
grid on;
plot(T(1:end - 1), U,'Color',[0.6,0,0.8]);
legend('Simulation')
xlabel('Time [s]')
ylabel('Motor acceleration [rad/s^2]')
axis([0 60 -0.4 0.4])

%Azimuth angle
ax(5) = subplot(2, 2, 3);
hold on;
grid on;
plot(T(1:end - 1), Xref(3, :), 'k')
plot(T, X(3, :), 'b')
plot(T, Xest(3, :), 'g')
xlabel('Time [s]')
ylabel('Azimuth angle [rad]')
legend('Reference', 'Simulation','Estimate')

linkaxes(ax, 'x')
