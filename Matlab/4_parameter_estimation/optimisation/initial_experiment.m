% Function for retrieving the measurements out of the NetCD-Files
%1. Open the NetCD-Files and get variables from them
ncid1 = netcdf.open('base_siemens.nc');
time=netcdf.getVar(ncid1,0);
motor_speedsp=netcdf.getVar(ncid1,10);
motor_speed=netcdf.getVar(ncid1,7);
ncid2 = netcdf.open('base_armbone.nc');
time2=netcdf.getVar(ncid2,0);
arm_speed=netcdf.getVar(ncid2,3);
ncid3 = netcdf.open('base_LAS.nc');
time3=netcdf.getVar(ncid3,0);
alpha=netcdf.getVar(ncid3,2);
OFFSET=0.75049; %Offset for the elevation angle after reconfiguration
                %of LAS.
alpha_real=alpha-OFFSET;
beta_real=netcdf.getVar(ncid3,1);
%2. Generate a fixed sampling time vector for all measurements
t=0:0.1:time(end);
t=t';
init=346;
final=907;
%3. Interpolate them with splines
y=interp1(time2,arm_speed,t,'spline');
i1=interp1(time,motor_speed,t,'spline');
u=interp1(time,motor_speedsp,t,'spline');
alpha_real=interp1(time3,alpha_real,t,'spline');
beta_real=interp1(time3,beta_real,t,'spline');
%figure; ax(1)=subplot(3,1,1); plot(u);hold on; plot(y,'r'); ax(2)=subplot(3,1,2); plot(beta_real); ax(3)=subplot(3,1,3); plot(alpha_real); linkaxes(ax,'x');

y=y(init:final);
i1=i1(init:final);
u=u(init:final);
alpha_real=alpha_real(init:final);
beta_real=beta_real(init:final);





