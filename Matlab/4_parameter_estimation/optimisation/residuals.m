function [ res ] = residuals( t,u,data,theta )
% Function needed by lsqnonlin to be minimised, residuals are returned from
% the function

%1. Retrieve data
alpha_real=data(:,1);
beta_real=data(:,2);
y=data(:,3);
%2. Simulate data
[T, X, U] = simulate_series(u, t,theta);
res=[X(2,1:end-1)'-alpha_real;X(3,1:end-1)'-beta_real];
%3. Plot while optimising
figure (1)
ax(1)=subplot(2,1,1);
plot(T,alpha_real,'r',T,X(2,1:end-1),'b')
legend('Measured Elevation','Simulated Elevation')
xlabel('Time [s]')
ylabel('Elevation angle [rad/s]')
grid on;
ax(2)=subplot(2,1,2);
plot(T,beta_real,'r',T,X(3,1:end-1),'b')
legend('Measured Azimuth','Simulated Azimuth')
xlabel('Time [s]')
ylabel('Azimuth angle [rad/s]')
grid on;
end
