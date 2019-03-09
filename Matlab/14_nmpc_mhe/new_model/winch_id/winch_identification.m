%Script dedicated to plitting the results on winch identification
clear all; close all;clc;
w3
[T,X,U] = simulate_series(u);
Tshift=6;
%% Plotting section
figure(4);
clf;
ax(1) = subplot(3, 1, 1);
plot(t, motor_speed, 'r','lineWidth',1.8)
plot(t, u(:,1), 'Color',[0.6,0,0.8],'lineWidth',1.5)
hold on;
plot(t, u(:,2), 'c','lineWidth',1.3)
plot(T + Tshift, X(1, :), 'b','lineWidth',1)
xlabel('Time [s]')
ylabel('Arm/Winch Speeds [rad/s]')
axis([110 170 -1 2.5])
legend('Control sent','Control sent')
grid on;

ax(2) = subplot(3, 1, 2);
plot(T + Tshift, X(2, :), 'b','lineWidth',1.3);
hold on;
plot(t, alpha_real, 'r','lineWidth',1.3)
xlabel('Time [s]')
ylabel('Elevation [rad]')
grid on;

ax(3) = subplot(3, 1, 3);
plot(T + Tshift, X(3, :), 'b','lineWidth',1.3)
hold on;
plot(t, beta_real, 'r','lineWidth',1.3)
%axis([60 100 -3 2]);
legend('Simulated', 'Measured')
xlabel('Time [s]')
ylabel('Azimuth [rad]')
axis([110 170 -inf inf])
linkaxes(ax, 'x')
axis([110 170 -inf inf])
grid on;

figure(5);
plot(t,winch_encoder,'r','lineWidth',1.5)
xlabel('Time [s]')
ylabel('Tether length [m]')
hold on
plot(T+Tshift,X(7, :), 'b','lineWidth',1.3)
grid on
legend('Measured','Simulated')
axis([110 170 1.3 1.8])

