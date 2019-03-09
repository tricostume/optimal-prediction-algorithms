%% Plotting section
figure;
clf;
delay=80.6;
%plot elevation
ax(1) = subplot(2, 2, 1);
hold on;
plot(T(1:end-1), Xref(2, :), 'k')
%plot(t-21.9-delay, alphaEst, 'g','lineWidth',2.5);
%plot(T, X(2, :), 'b', 'lineWidth',1.3);
plot(T, X(2, :), 'b');
%plot(t-21.9-delay, elevation,'r','lineWidth',1.25)
axis([5 55 -1.3 -0.8])
xlabel('Time [s]')
ylabel('Elevation [rad]')
%legend('Reference','Estimator', 'Simulation','Experiment measurement')
legend('Reference', 'Simulation')
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
%plot(t-21.9-delay, motor_speedEst, 'g','lineWidth',2.5);
plot(T, X(1, :),'b');
%plot(t-21.9-delay, carousel_speed,'r')
grid on;
xlabel('Time [s]')
ylabel('Arm Speed [rad/s]')
%legend('Simulation','Reference','Real experiment','Estimator')
axis([5 55 1.1 1.75])

%plot u
ax(4) = subplot(2, 2, 4);
hold on;
grid on;
plot(T(1:end - 1), U);
%h1=plot(t-22.1-120,u,'Color',[0.6,0,0.8],'lineWidth',1.3);
%legend([h1],{'Sent control'});
xlabel('Time [s]')
ylabel('Motor acceleration setpoint [rad/s^2]')
axis([5 55 -0.35 0.35])

%plot azimuth
ax(5) = subplot(2, 2, 3);
hold on;
grid on;
plot(T(1:end - 1), Xref(3, :), 'k')
%plot(t-21.9-delay, beta_speedEst, 'g','lineWidth',2.5)
plot(T, X(3, :), 'b')
%plot(t-21.9-delay, azimuth,'r','lineWidth',1.25)
xlabel('Time [s]')
ylabel('Azimuth [rad]')
%legend('Reference', 'Simulation', 'Estimation', 'Real experiment')
axis([5 55 -1 1])

linkaxes(ax, 'x')
