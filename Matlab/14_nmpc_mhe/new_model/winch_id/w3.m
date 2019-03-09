clear
clc
cut=300;
ncid1 = netcdf.open('3_siemens.nc');
time=netcdf.getVar(ncid1,0);
motor_speedsp=netcdf.getVar(ncid1,10);
motor_speed=netcdf.getVar(ncid1,7);

winchspeed_sp = netcdf.getVar(ncid1, 5);
winchspeed_smoothed = netcdf.getVar(ncid1, 2);
winch_encoder = netcdf.getVar(ncid1, 3);

ncid2 = netcdf.open('3_armbone.nc');
time2=netcdf.getVar(ncid2,0);
arm_speed=netcdf.getVar(ncid2,3);

ncid3 = netcdf.open('3_LAS.nc');
time3=netcdf.getVar(ncid3,0);
alpha=netcdf.getVar(ncid3,2);
OFFSET=0.75049;
alpha_real=alpha-OFFSET;
beta_real=netcdf.getVar(ncid3,1);


t=0:0.02:time(end);
t=t';
arm_speed=interp1(time2,arm_speed,t,'spline');
arm_speed=arm_speed(cut:end);

motor_speed=interp1(time,motor_speed,t,'spline');
motor_speed=motor_speed(cut:end);

motor_speedsp=interp1(time,motor_speedsp,t,'spline');
motor_speedsp=motor_speedsp(cut:end);

alpha_real=interp1(time3,alpha_real,t,'spline');
alpha_real=alpha_real(cut:end);

beta_real=interp1(time3,beta_real,t,'spline');
beta_real=beta_real(cut:end);

winchspeed_sp=interp1(time,winchspeed_sp,t,'spline');
winchspeed_sp=winchspeed_sp(cut:end);

winchspeed_smoothed=interp1(time,winchspeed_smoothed,t,'spline');
winchspeed_smoothed=winchspeed_smoothed(cut:end);

winch_encoder=interp1(time,winch_encoder,t,'spline');
winch_encoder=winch_encoder-winch_encoder(cut)+1.36;
winch_encoder=winch_encoder(cut:end);

u=[motor_speedsp,winchspeed_smoothed];
t=t(cut:end);
figure; ax(1)=subplot(3,1,1); plot(u);hold on; plot(arm_speed,'r'); ax(2)=subplot(3,1,2); plot(beta_real); ax(3)=subplot(3,1,3); plot(alpha_real); linkaxes(ax,'x');
