%------------------Albert-Ludwigs-Universitaet Freiburg--------------------
%___________________M. Sc. in Microsystems Engineering_____________________
%Thesis: 
%Period of preparation: April-September 2015
%Author: Ernesto Denicia
%Script: Pole placement simulation
%Description: This script generates a control law accomplishing with
%requirements. It is demonstrated though that pole placement loses track of
%the controls needed dor its accomplishment.
%Comments: Matrix K_pp from pole placement is used in order to conclude
%about its performance
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
%3. Get feedback matrix
load('Kppd.mat')
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
    U(k) = -Kppd*(X(:, k)-Xref(:, k));
    
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
axis([-inf inf -0.8 0.8])

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
