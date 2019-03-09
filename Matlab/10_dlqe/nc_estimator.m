%% Reading nc files
% Function for reading nc files corresponding to the
% measurements of the carousel
ncid1 = netcdf.open('multiramps_siemens.nc');
time=netcdf.getVar(ncid1,0);
motor_speedsp=netcdf.getVar(ncid1,10);
motor_speed=netcdf.getVar(ncid1,7);
ncid2 = netcdf.open('multiramps_armbone.nc');
time2=netcdf.getVar(ncid2,0);
arm_speed=netcdf.getVar(ncid2,3);
ncid3 = netcdf.open('multiramps_LAS.nc');
time3=netcdf.getVar(ncid3,0);
alpha=netcdf.getVar(ncid3,2);
OFFSET=0.75049;
alpha_real=alpha-OFFSET;
beta_real=netcdf.getVar(ncid3,1);
t=0:0.1:time(end);
t=t';
y=interp1(time2,arm_speed,t,'spline');
i1=interp1(time,motor_speed,t,'spline');
u=interp1(time,motor_speedsp,t,'spline');
alpha_real=interp1(time3,alpha_real,t,'spline');
beta_real=interp1(time3,beta_real,t,'spline');