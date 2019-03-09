
clc
cut=265;
ncid1 = netcdf.open('2_siemens.nc');
time=netcdf.getVar(ncid1,0);
motor_speedsp_=netcdf.getVar(ncid1,10);
motor_speed_=netcdf.getVar(ncid1,7);
arm_encoder_=netcdf.getVar(ncid1,8);


ncid2 = netcdf.open('2_armbone.nc');
time2=netcdf.getVar(ncid2,0);
arm_speed_=netcdf.getVar(ncid2,3);

winchspeed_sp_ = netcdf.getVar(ncid1, 5);
winchspeed_smoothed_ = netcdf.getVar(ncid1, 2);
winch_encoder_ = netcdf.getVar(ncid1, 3);

ncid3 = netcdf.open('2_LAS.nc');
time3=netcdf.getVar(ncid3,0);
alpha_=netcdf.getVar(ncid3,2);
OFFSETalpha=0.75049;
alpha_real_=alpha_-OFFSETalpha;
beta_real_=netcdf.getVar(ncid3,1);


t=0:0.02:time(end);
t=t';

motor_speed_=interp1(time,motor_speed_,t,'spline');
motor_speed_=motor_speed_(cut:end);

motor_speedsp_=interp1(time,motor_speedsp_,t,'spline');
motor_speedsp_=motor_speedsp_(cut:end);

arm_speed_=interp1(time2,arm_speed_,t,'spline');
arm_speed_=arm_speed_(cut:end);

arm_encoder_=interp1(time,arm_encoder_,t,'spline');
arm_encoder_=arm_encoder_(cut:end);
arm_encoder_=arm_encoder_-arm_encoder_(1);

alpha_real_=interp1(time3,alpha_real_,t,'spline');
alpha_real_=alpha_real_(cut:end);

beta_real_=interp1(time3,beta_real_,t,'spline');
beta_real_=beta_real_(cut:end);

winchspeed_sp_=interp1(time,winchspeed_sp_,t,'spline');
winchspeed_sp_=winchspeed_sp_(cut:end);

winchspeed_smoothed_=interp1(time,winchspeed_smoothed_,t,'spline');
winchspeed_smoothed_=winchspeed_smoothed_(cut:end);

winch_encoder_=interp1(time,winch_encoder_,t,'spline');
winch_encoder_=winch_encoder_-winch_encoder_(cut)+1.35;
winch_encoder_=winch_encoder_(cut:end);

u=[motor_speedsp_,winchspeed_sp_];

%figure; ax(1)=subplot(3,1,1); plot(u);hold on; plot(y,'r'); ax(2)=subplot(3,1,2); plot(beta_real); ax(3)=subplot(3,1,3); plot(alpha_real); linkaxes(ax,'x');
