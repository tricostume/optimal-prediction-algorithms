%------------------Albert-Ludwigs-Universitaet Freiburg--------------------
%___________________M. Sc. in Microsystems Engineering_____________________
%Thesis: 
%Period of preparation: April-September 2015
%Author: Ernesto Denicia
%Script: Model validation: Simulation examples
%Description: The model is proved with the same experiment from which the
%parameters were estimated and an extra model with less agressive
%excitation signals (ramps).
%Comments: None
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%%
clear all; close all; clc;
%1. Choose between experiments, write 'multisteps' for optimisation
%experiment or 'ramp_steps_experiment' for validation model.
multisteps
%2. Start simulation of the data packages
[T, X, U] = simulate_series(u);
%% Plotting section
figure(4);
clf;
%3. Plot carousel velocity
ax(1) = subplot(3, 1, 1);
plot(t, u, 'k')
hold on;
plot(t, y, 'r')
plot(T, X(1, :), 'b')
axis([0 560 0 3.8])
xlabel('Time [s]')
ylabel('Arm speed [rad/s]')
legend('Reference arm speed','Measured arm speed','Simulated arm speed')
grid on;
%4. Plot elevation angle
ax(2) = subplot(3, 1, 2);
plot(t, alpha_real, 'r')
hold on;
plot(T, X(2, :), 'b');
%axis([60 100 -70 -30]);
legend('Measured Elevation','Simulated Elevation')
xlabel('Time [s]')
ylabel('Elevation angle [rad]')
grid on
axis([0 560 -2 0.5])
%5 Plot azimuth angle
ax(3) = subplot(3, 1, 3);
plot(t, beta_real, 'r')
hold on;
plot(T, X(3, :), 'b')
legend('Measured azimuth angle','Simulated azimuth angle')
xlabel('Time [s]')
ylabel('Azimuth [rad]')
grid on;
linkaxes(ax, 'x')
%Plot detail on alpha
figure(5);
clf; hold on;
plot(t, alpha_real, 'r')
hold on;
plot(T, X(2, :), 'b');
%axis([60 100 -70 -30]);
legend('Measured Elevation','Simulated Elevation')
xlabel('Time [s]')
ylabel('Elevation angle [rad]')
grid on
axis([220 476 -3.2 1])
