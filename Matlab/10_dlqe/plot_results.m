%% Plotting section
% Prepare measurements to be plotted (stay with one every three)
filter=3;
alpha_graph = alpha_real(filter);
beta_graph = beta_real(filter);
y_graph = y(filter);
t_k = T(filter);
for k=1:size(t)
    if(mod(k,filter) == 0)
        alpha_graph = [alpha_graph,alpha_real(k)];
        beta_graph = [beta_graph,beta_real(k)];
        y_graph = [y_graph,y(k)];
        t_k = [t_k,T(k)];
    end
end
%plots
figure(1);
clf;
%Elevation angle
ax(1) = subplot(2, 2, 1);
hold on;
plot(t_k, alpha_graph, 'r.')
xlabel('Time [s]')
ylabel('Elevation [rad]')
plot(T, Xmod(2, :), 'b');
plot(T, Xest(2, :), 'g');
%legend('Reference', 'Simulation', 'Estimate')
axis([134 154 -1.6 0])
grid on;

%Arm speed
ax(2) = subplot(2, 2, 2);
hold on;
plot(t_k, y_graph, 'r.')
plot(T, Xmod(1, :), 'b');
plot(T, Xest(1, :), 'g');
grid on;
xlabel('Time [s]')
ylabel('Arm Speed [rad/s]')
axis([134 154 0.5 3.5])
%legend('Measured','Simulation','Estimate')

%Control u
ax(4) = subplot(2, 2, 4);
hold on;
grid on;
plot(T(1:end - 1), U,'Color',[0.6,0,0.8]);
legend('Sent to carousel')
xlabel('Time [s]')
ylabel('Motor speed setpoint [rad/s^2]')
axis([134 154 0.5 3.5])

%Azimuth angle
ax(5) = subplot(2, 2, 3);
hold on;
grid on;
plot(t_k, beta_graph, 'r.')
plot(T, Xmod(3, :), 'b')
plot(T, Xest(3, :), 'g')
xlabel('Time [s]')
ylabel('Azimuth angle [rad]')
legend('Measurement','Simulation','Estimate')
axis([134 154 -0.9 0.9])

linkaxes(ax, 'x')
