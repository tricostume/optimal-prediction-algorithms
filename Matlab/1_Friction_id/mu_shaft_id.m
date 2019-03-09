%------------------Albert-Ludwigs-Universitaet Freiburg---------------------
%___________________M. Sc. in Microsystems Engineering_____________________
%Thesis: 
%Period of preparation: April-September 2015
%Author: Ernesto Denicia
%Script: Linear friction parameter identification (mu_shaft)
%Description: A least square estimation is performed for finding the linear
%friction parameter of the shaft in the arm generalized coordinate. A filter
%is applied to the torque measurements for comparison of the model with the
%filtered approximation. 
%Comments: Notice that the simplification explained in the thesis is used
%reason for which I_A is unrealiable.
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%%
clear all; close all; clc
%1. Set some cutoff regions for applying the algorithm
Beginning=86156;
End=106797;

%2. Due to filtering, also some data is lost at the beginning. Security
%cutoff.
Filter_cut=2208;

%3. Read data from .nc files adn cut them off
[time, winch_speed_sp, winch_speed, winch_angle, ...
 winch_torque,winch_current,motor_speed_sp, motor_speed, ...
 motor_angle, motor_torque,motor_current]=...
                                       read_siemens('increasing_steps.nc');
w=motor_speed(Beginning:End);
t=time(Beginning:End);
T=motor_torque(Beginning:End);

%4. Design and apply low pass filter for speed w in the frequency
        f = 2*pi*0.4; % cut frequency in Hz
        H = zpk([],[-f -f],f^2);
        TS = 0.015; % sampling time
        HD = c2d(H,TS,'tustin');
        [N,D] = tfdata(HD,'v');
        w_filt = filter(N,D,w);

        %For drawing purposes, T is also filtered
        f = 2*pi*0.05; % cut frequency in Hz
        H = zpk([],[-f -f],f^2);
        TS = 0.015; % sampling time
        HD = c2d(H,TS,'tustin');
        [N,D] = tfdata(HD,'v');
        T_filt=filter(N,D,T);

 %% Start programming linear Least-Squares algorithm
 %5. Propose a Phi matrix containing the model
Phi=ones(length(t)-1-Filter_cut,2);

%6. According to the simplification, finite differences have to be
% performed in order to obtain the angular acceleration.
alpha=diff(w_filt)./(diff(t));

%7. Populate Phi matrix with respective vectors, here [alpha-w_filt-1] for
%the linear coefficients I, mu_shaft and a constant coefficient.
Phi(1:length(t)-1-Filter_cut+1,1:1)=alpha(Filter_cut:end);
Phi(1:length(t)-1-Filter_cut+1,2:2)=w_filt(Filter_cut:length(t)-1);
Phi(1:length(t)-1-Filter_cut+1,3:3)=1;

%8. Obtain the pseudoinverse to solve the problem, display mu_shaft
Phi_min=pinv(Phi)*T(Filter_cut:length(t)-1);
Phi_min(2)

%9. Calculate the model back with the use of the obtained parameters,
%conserve I even though we know it does not make sense, as the optimization
%was achieved with it.
Regression=Phi_min(1)*alpha(Filter_cut:end)+Phi_min(2)*...
                                w_filt(Filter_cut:length(t)-1)+Phi_min(3);

%% Plotting section

figure(1); 
clf
subplot(3,1,1)
plot(t,T,'r',t,[zeros(Filter_cut,1);Regression],'b',t,T_filt,'k');
legend('Measured torque','Regression: Torque','Filtered torque');
xlabel('Time [s]');
ylabel('Torque [Nm]');
grid on;
axis([1100 1300 0 45])
i(1)=subplot(3,1,2);
plot(t,w,'r',t,w_filt,'k');
legend('Measured carousel speed','Filtered carousel speed');
xlabel('Time[s]');
ylabel('Arm speed [rad/s]');
grid on;
axis([1100 1300 0 1.5])
i(2)=subplot(3,1,3);
plot(t(1:end-1),alpha,'r');
xlabel('Time [s]');
ylabel('Acceleration [rad/s^2]');
grid on;
linkaxes(i,'x');
axis([1100 1300 0 2])



