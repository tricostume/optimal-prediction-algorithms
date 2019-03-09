%% Plotting section
figure(1);
clf;
delay=172.4; %53.9
%plot elevation
ax(1) = subplot(2, 2, 1);
hold on;
plot(T(1:end-1), Xref(2, :), 'k')
plot(t-21.9-delay, alphaEst, 'g','lineWidth',2.5);
plot(T, X(2, :), 'b', 'lineWidth',1.3);
plot(t-21.9-delay, elevation,'r','lineWidth',1.25)
axis([5 55 -1.3 -0.97])
xlabel('Time [s]')
ylabel('Elevation [rad]')
legend('Reference','Estimator', 'Simulation','Experiment measurement')
grid on;

% plot(alphaEst)
% hold on
% plot(Xref(2,:))
error1 = Xref(2,1:600)-alphaEst(424:1023)';
error2 = Xref(2,1:600)-alphaEst(824:1423)';
error3 = Xref(2,1:600)-alphaEst(1224:1823)';

%plot arm speed
ax(2) = subplot(2, 2, 2);
plot(T(1:end - 1), Xref(1, :), 'k')
hold on;
plot(t-21.9-delay, motor_speedEst, 'g','lineWidth',2.5);
plot(T, X(1, :),'b', 'lineWidth',1.3);
plot(t-21.9-delay, carousel_speed,'r')
grid on;
xlabel('Time [s]')
ylabel('Arm Speed [rad/s]')
%legend('Simulation','Reference','Real experiment','Estimator')
axis([5 55 1.07 1.6])

%plot u
ax(4) = subplot(2, 2, 4);
hold on;
grid on;
plot(T(1:end - 1), U,'b','lineWidth',1.3);
h1=plot(t-22.8-131.6+80,u,'Color',[0.6,0,0.8],'lineWidth',1.3);
%plot(20+T(1,1:788)+0.3,u(124:911,1),'m');
legend([h1],{'Sent control'});
xlabel('Time [s]')
ylabel('Motor acceleration setpoint [rad/s^2]')
axis([-inf inf -0.3 0.3])

%plot azimuth
ax(5) = subplot(2, 2, 3);
hold on;
grid on;
plot(T(1:end - 1), Xref(3, :), 'k')
plot(t-21.9-delay, betaEst, 'g','lineWidth',2.5)
plot(T, X(3, :), 'b','lineWidth',1.3)
plot(t-21.9-delay, azimuth,'r','lineWidth',1.25)
xlabel('Time [s]')
ylabel('Azimuth [rad]')
%legend('Reference', 'Simulation', 'Estimation', 'Real experiment')
axis([5 55 -0.6 0.6])

linkaxes(ax, 'x')
%%
% Plotting disturbances
figure();
subplot(2,1,1)
hold on
plot(T(1:end-1), Xref(2, :), 'k')
plot(t-21.9-delay, alphaEst, 'g','lineWidth',2.5);
plot(T, X(2, :), 'b', 'lineWidth',1.3);
plot(t-21.9-delay, elevation,'r','lineWidth',1.25)
axis([5 55 -1.3 -0.97])
xlabel('Time [s]')
ylabel('Elevation [rad]')
legend('Reference','Estimator', 'Simulation','Experiment measurement')
grid on;
subplot(2,1,2)
plot(t-21.9-delay,gammaEst(1:end-1))
ylabel(['Elevation disturbance [rad/s^2]'])
xlabel('Time (s)')
axis([5 55 -0.03 0.12])