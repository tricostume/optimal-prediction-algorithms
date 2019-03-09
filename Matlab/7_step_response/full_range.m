% Function for retrieving the measurements out of the NetCD-Files
%1. Open the NetCD-Files and get variables from them
ncid1 = netcdf.open('full_range_siemens.nc');
time=netcdf.getVar(ncid1,0);
motor_speedsp=netcdf.getVar(ncid1,10);
motor_speed=netcdf.getVar(ncid1,7);
ncid2 = netcdf.open('full_range_armbone.nc');
time2=netcdf.getVar(ncid2,0);
arm_speed=netcdf.getVar(ncid2,3);
ncid3 = netcdf.open('full_range_LAS.nc');
time3=netcdf.getVar(ncid3,0);
alpha=netcdf.getVar(ncid3,2);
OFFSET=0.75049; %Offset for the elevation angle after reconfiguration
                %of LAS.
alpha_real=alpha-OFFSET;
beta_real=netcdf.getVar(ncid3,1);
%2. Generate a fixed sampling time vector for all measurements
t=0:0.1:time(end);
t=t';
%Step up
init=3889;
final=4084;
% Step down
%init=3763;
%final=3938;
%3. Interpolate them with splines
y=interp1(time2,arm_speed,t,'spline');
i1=interp1(time,motor_speed,t,'spline');
u=interp1(time,motor_speedsp,t,'spline');
alpha_real=interp1(time3,alpha_real,t,'spline');
beta_real=interp1(time3,beta_real,t,'spline');
y=y(init:final);
u=u(init:final);
alpha_real=alpha_real(init:final);
beta_real=beta_real(init:final);
t=0:0.1:(final-init)*0.1;
%% Plotting section
figure(4);
clf;
%3. Plot carousel velocity
ax(1) = subplot(3, 1, 1);
plot(t, u, 'k')
hold on;
plot(t, y, 'r')
%axis([0 20 1 2.2])
xlabel('Time [s]')
ylabel('Arm speed [rad/s]')
legend('Reference arm speed','Measured arm speed')
grid on;
%4. Plot elevation angle
ax(2) = subplot(3, 1, 2);
plot(t, alpha_real, 'r')
hold on;
%axis([0 100 -70 -30]);
legend('Measured Elevation')
xlabel('Time [s]')
ylabel('Elevation angle [rad]')
grid on
%axis([0 560 -2 0.5])
%5 Plot azimuth angle
ax(3) = subplot(3, 1, 3);
plot(t, beta_real, 'r')
hold on;
legend('Measured azimuth angle')
xlabel('Time [s]')
ylabel('Azimuth [rad]')
grid on;
linkaxes(ax, 'x')
