load('presimul.mat')
winch_encoder_= state_sim(:,1);
motor_speed_= state_sim(:,2);
alpha_real_= state_sim(:,3);
beta_real_= state_sim(:,4);
motor_speedsp_= state_sim(:,8);
radius_ = state_sim(:,9);
ddmotor_speed_sp_= state_sim(:,10);
dl_tether_ = state_sim(:,11);