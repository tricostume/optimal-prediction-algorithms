%------------------Albert-Ludwigs-Universitaet Freiburg--------------------
%___________________M. Sc. in Microsystems Engineering_____________________
%Thesis: 
%Period of preparation: April-September 2015
%Author: Ernesto Denicia
%Script: MHE tested on measurements from the carousel
%Comments: This MHE will be ccompanying the experiments along the whole 
% chapter
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
clc;
clear all;
close all;

EXPORT = 0;
Ts = 0.02;      % sampling time

DifferentialState l_tether delta_arm ddelta_arm alpha beta dddelta_arm dalpha dbeta;      % provide proper names for all differential states and controls etc.
Control ddelta_motor_sp dl_tether;

%% Differential Equations
m_ball = 0.57;
I_arm = 205;
I_LAS = 0.03203;
r_arm = 2.05;
g = 9.81;
wn = 6.2409;
d = 0.1288;
my_alpha_LA = 0.4876;
my_beta_LA = 2.1774;
mu_air = 0.0074;
I_tether = 1.0006;

ode = [ dot(l_tether)==-dl_tether;
        dot(delta_arm)==-ddelta_arm;
        dot(ddelta_arm)==dddelta_arm;
        dot(alpha)==dalpha;
        dot(beta)==dbeta;
        dot(dddelta_arm)==wn*wn*ddelta_motor_sp-2*d*wn*dddelta_arm-wn*wn*ddelta_arm;
        dot(dalpha)==-(dalpha*my_alpha_LA + (I_tether*dbeta^2*sin(2*alpha))/2 + g*l_tether*m_ball*cos(alpha) + (dbeta^2*l_tether^2*m_ball*sin(2*alpha))/2 + (ddelta_arm^2*l_tether^2*m_ball*sin(2*alpha))/2 + dalpha*l_tether^2*mu_air*(dalpha^2*l_tether^2 + ddelta_arm^2*r_arm^2 + dbeta^2*l_tether^2*cos(alpha)^2 + ddelta_arm^2*l_tether^2*cos(alpha)^2 + 2*dbeta*ddelta_arm*l_tether^2*cos(alpha)^2 + 2*ddelta_arm^2*l_tether*r_arm*cos(alpha)*cos(beta) + 2*dbeta*ddelta_arm*l_tether*r_arm*cos(alpha)*cos(beta) - 2*dalpha*ddelta_arm*l_tether*r_arm*sin(alpha)*sin(beta))^(1/2) + dbeta*ddelta_arm*l_tether^2*m_ball*sin(2*alpha) + ddelta_arm^2*l_tether*m_ball*r_arm*cos(beta)*sin(alpha) - dddelta_arm*l_tether*m_ball*r_arm*sin(alpha)*sin(beta) - ddelta_arm*l_tether*mu_air*r_arm*sin(alpha)*sin(beta)*(dalpha^2*l_tether^2 + ddelta_arm^2*r_arm^2 + dbeta^2*l_tether^2*cos(alpha)^2 + ddelta_arm^2*l_tether^2*cos(alpha)^2 + 2*dbeta*ddelta_arm*l_tether^2*cos(alpha)^2 + 2*ddelta_arm^2*l_tether*r_arm*cos(alpha)*cos(beta) + 2*dbeta*ddelta_arm*l_tether*r_arm*cos(alpha)*cos(beta) - 2*dalpha*ddelta_arm*l_tether*r_arm*sin(alpha)*sin(beta))^(1/2))/(m_ball*l_tether^2 + I_tether);
        dot(dbeta)==-(dbeta*my_beta_LA + dddelta_arm*l_tether^2*m_ball*cos(alpha)^2 - I_tether*dalpha*dbeta*sin(2*alpha) - dalpha*dbeta*l_tether^2*m_ball*sin(2*alpha) - dalpha*ddelta_arm*l_tether^2*m_ball*sin(2*alpha) + dbeta*l_tether^2*mu_air*cos(alpha)^2*(dalpha^2*l_tether^2 + ddelta_arm^2*r_arm^2 + dbeta^2*l_tether^2*cos(alpha)^2 + ddelta_arm^2*l_tether^2*cos(alpha)^2 + 2*dbeta*ddelta_arm*l_tether^2*cos(alpha)^2 + 2*ddelta_arm^2*l_tether*r_arm*cos(alpha)*cos(beta) + 2*dbeta*ddelta_arm*l_tether*r_arm*cos(alpha)*cos(beta) - 2*dalpha*ddelta_arm*l_tether*r_arm*sin(alpha)*sin(beta))^(1/2) + ddelta_arm*l_tether^2*mu_air*cos(alpha)^2*(dalpha^2*l_tether^2 + ddelta_arm^2*r_arm^2 + dbeta^2*l_tether^2*cos(alpha)^2 + ddelta_arm^2*l_tether^2*cos(alpha)^2 + 2*dbeta*ddelta_arm*l_tether^2*cos(alpha)^2 + 2*ddelta_arm^2*l_tether*r_arm*cos(alpha)*cos(beta) + 2*dbeta*ddelta_arm*l_tether*r_arm*cos(alpha)*cos(beta) - 2*dalpha*ddelta_arm*l_tether*r_arm*sin(alpha)*sin(beta))^(1/2) + ddelta_arm^2*l_tether*m_ball*r_arm*cos(alpha)*sin(beta) + dddelta_arm*l_tether*m_ball*r_arm*cos(alpha)*cos(beta) + ddelta_arm*l_tether*mu_air*r_arm*cos(alpha)*cos(beta)*(dalpha^2*l_tether^2 + ddelta_arm^2*r_arm^2 + dbeta^2*l_tether^2*cos(alpha)^2 + ddelta_arm^2*l_tether^2*cos(alpha)^2 + 2*dbeta*ddelta_arm*l_tether^2*cos(alpha)^2 + 2*ddelta_arm^2*l_tether*r_arm*cos(alpha)*cos(beta) + 2*dbeta*ddelta_arm*l_tether*r_arm*cos(alpha)*cos(beta) - 2*dalpha*ddelta_arm*l_tether*r_arm*sin(alpha)*sin(beta))^(1/2))/(m_ball*l_tether^2*cos(alpha)^2 + I_tether*cos(alpha)^2 + I_LAS)
        ];          
    

%% Export of an optimization routine:
acadoSet('problemname', 'mhe');

N = 40;     
ocp = acado.OCP( 0.0, N*Ts, N );

r_cyl = (l_tether^2*cos(alpha)^2 + r_arm^2 + 2*l_tether*r_arm*cos(alpha)*cos(beta))^(1/2);

index = [1,2,4,5];
h = [l_tether; delta_arm; alpha; beta; controls];
hN = [l_tether; delta_arm; alpha; beta];

W = acado.BMatrix(eye(length(h)));  
WN = acado.BMatrix(eye(length(hN))) ;

ocp.minimizeLSQ( W, h );            
ocp.minimizeLSQEndTerm( WN, hN );   

ocp.setModel(ode);      

mhe = acado.OCPexport( ocp );
mhe.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
mhe.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
mhe.set( 'SPARSE_QP_SOLUTION',          'CONDENSING_N2');
mhe.set( 'FIX_INITIAL_STATE',           'NO');
mhe.set( 'INTEGRATOR_TYPE',             'INT_RK4'           );  
mhe.set( 'NUM_INTEGRATOR_STEPS',        2*N                 );
mhe.set( 'QP_SOLVER',                   'QP_QPOASES'    	);

if EXPORT
    mhe.exportCode( 'export_MHE' );
    
    global ACADO_;
    copyfile([ACADO_.pwd '/../../external_packages/qpoases'], 'export_MHE/qpoases')
    
    cd export_MHE
    make_acado_solver('../acado_MHEstep')
    cd ..
end

%% PARAMETERS SIMULATION
%Initial estimate to be passed to the MHE 
winchid
X0_MHE = [1.35,0,0.000000887416239,-pi/2,-0.086168493208539,0.000000000000106,0,0];       % <-- TODO: initial state (downward position)
Y0_MHE = [X0_MHE(1),X0_MHE(2),X0_MHE(4),X0_MHE(5)];
Rref = 2.05;

% %% CALIBRATION FOR STEADY STATE
% l_tether = X0_MHE(1);
% ddelta_arm = X0_MHE(2);
% alpha = X0_MHE(3);
% beta = X0_MHE(4);
% dddelta_arm = X0_MHE(5);
% dalpha = 0;
% dbeta = 0;
% ddelta_motor_sp = X0_MHE(8);      % provide proper names for all differential states and controls etc.
% dddelta_motor_sp = 0;
% dl_tether = 0;
% 
% (l_tether^2*cos(alpha)^2 + r_arm^2 + 2*l_tether*r_arm*cos(alpha)*cos(beta))^(1/2)
% 
% ode = [ dl_tether;
%         dddelta_arm;
%         dalpha;
%         dbeta;
%         wn*wn*ddelta_motor_sp-2*d*wn*dddelta_arm-wn*wn*ddelta_arm;
%         -(dalpha*my_alpha_LA + (I_tether*dbeta^2*sin(2*alpha))/2 + g*l_tether*m_ball*cos(alpha) + (dbeta^2*l_tether^2*m_ball*sin(2*alpha))/2 + (ddelta_arm^2*l_tether^2*m_ball*sin(2*alpha))/2 + dalpha*l_tether^2*mu_air*(dalpha^2*l_tether^2 + ddelta_arm^2*r_arm^2 + dbeta^2*l_tether^2*cos(alpha)^2 + ddelta_arm^2*l_tether^2*cos(alpha)^2 + 2*dbeta*ddelta_arm*l_tether^2*cos(alpha)^2 + 2*ddelta_arm^2*l_tether*r_arm*cos(alpha)*cos(beta) + 2*dbeta*ddelta_arm*l_tether*r_arm*cos(alpha)*cos(beta) - 2*dalpha*ddelta_arm*l_tether*r_arm*sin(alpha)*sin(beta))^(1/2) + dbeta*ddelta_arm*l_tether^2*m_ball*sin(2*alpha) + ddelta_arm^2*l_tether*m_ball*r_arm*cos(beta)*sin(alpha) - dddelta_arm*l_tether*m_ball*r_arm*sin(alpha)*sin(beta) - ddelta_arm*l_tether*mu_air*r_arm*sin(alpha)*sin(beta)*(dalpha^2*l_tether^2 + ddelta_arm^2*r_arm^2 + dbeta^2*l_tether^2*cos(alpha)^2 + ddelta_arm^2*l_tether^2*cos(alpha)^2 + 2*dbeta*ddelta_arm*l_tether^2*cos(alpha)^2 + 2*ddelta_arm^2*l_tether*r_arm*cos(alpha)*cos(beta) + 2*dbeta*ddelta_arm*l_tether*r_arm*cos(alpha)*cos(beta) - 2*dalpha*ddelta_arm*l_tether*r_arm*sin(alpha)*sin(beta))^(1/2))/(m_ball*l_tether^2 + I_tether);
%         -(dbeta*my_beta_LA + dddelta_arm*l_tether^2*m_ball*cos(alpha)^2 - I_tether*dalpha*dbeta*sin(2*alpha) - dalpha*dbeta*l_tether^2*m_ball*sin(2*alpha) - dalpha*ddelta_arm*l_tether^2*m_ball*sin(2*alpha) + dbeta*l_tether^2*mu_air*cos(alpha)^2*(dalpha^2*l_tether^2 + ddelta_arm^2*r_arm^2 + dbeta^2*l_tether^2*cos(alpha)^2 + ddelta_arm^2*l_tether^2*cos(alpha)^2 + 2*dbeta*ddelta_arm*l_tether^2*cos(alpha)^2 + 2*ddelta_arm^2*l_tether*r_arm*cos(alpha)*cos(beta) + 2*dbeta*ddelta_arm*l_tether*r_arm*cos(alpha)*cos(beta) - 2*dalpha*ddelta_arm*l_tether*r_arm*sin(alpha)*sin(beta))^(1/2) + ddelta_arm*l_tether^2*mu_air*cos(alpha)^2*(dalpha^2*l_tether^2 + ddelta_arm^2*r_arm^2 + dbeta^2*l_tether^2*cos(alpha)^2 + ddelta_arm^2*l_tether^2*cos(alpha)^2 + 2*dbeta*ddelta_arm*l_tether^2*cos(alpha)^2 + 2*ddelta_arm^2*l_tether*r_arm*cos(alpha)*cos(beta) + 2*dbeta*ddelta_arm*l_tether*r_arm*cos(alpha)*cos(beta) - 2*dalpha*ddelta_arm*l_tether*r_arm*sin(alpha)*sin(beta))^(1/2) + ddelta_arm^2*l_tether*m_ball*r_arm*cos(alpha)*sin(beta) + dddelta_arm*l_tether*m_ball*r_arm*cos(alpha)*cos(beta) + ddelta_arm*l_tether*mu_air*r_arm*cos(alpha)*cos(beta)*(dalpha^2*l_tether^2 + ddelta_arm^2*r_arm^2 + dbeta^2*l_tether^2*cos(alpha)^2 + ddelta_arm^2*l_tether^2*cos(alpha)^2 + 2*dbeta*ddelta_arm*l_tether^2*cos(alpha)^2 + 2*ddelta_arm^2*l_tether*r_arm*cos(alpha)*cos(beta) + 2*dbeta*ddelta_arm*l_tether*r_arm*cos(alpha)*cos(beta) - 2*dalpha*ddelta_arm*l_tether*r_arm*sin(alpha)*sin(beta))^(1/2))/(m_ball*l_tether^2*cos(alpha)^2 + I_tether*cos(alpha)^2 + I_LAS);
%         dddelta_motor_sp]



inputMHE.x = repmat(X0_MHE,N+1,1);      
inputMHE.u = zeros(N,2);     
inputMHE.y = repmat([Y0_MHE 0 0],N,1);   
inputMHE.yN = Y0_MHE.';  

% l_tether, ddelta_arm, alpha, beta, dddelta_arm, dalpha, dbeta, ddelta_motor_sp, rcyl, dddelta_motor_sp, dl_tether
% %Process noise description
% w_l_tether = 1e-12;
% w_ddelta_arm = 1e-12; 
% w_alpha = 1e-12;
% w_beta = 1e-12;
% w_dddelta_arm = 0.001;
% w_dalpha = 0.01;
% w_dbeta = 0.01;
% w_ddelta_motor_sp = 0;

%Process noise description was simulted in the MPC_noise routine
%Sensor noise description
%TRIAL 1:
% factor=29;
% v_l_tether = 1.1662e-06*factor;
% v_delta_arm = 6e-5*factor;
% v_alpha = 6e-5*factor;
% v_beta = 6e-5*factor;
% v_ddelta_motor_sp =1e-07*factor;
% %v_dddelta_motor_sp = 1e-12;
% v_dl_tether = 1e-7*factor;

%Sensor variances
factor=1e6;
v_l_tether = factor*1.75e-3;
v_delta_arm = factor*1.75e-3;
v_alpha = factor*1.75e-3;
v_beta = factor*1.75e-3;
%Control variances
v_ddelta_motor_sp =factor*1e-04;
v_dl_tether = factor*1e-4;
RE_sim = diag([v_l_tether, v_delta_arm, v_alpha, v_beta, v_ddelta_motor_sp].^2);
inputMHE.W = diag(1./[v_l_tether, v_delta_arm, v_alpha, v_beta, v_ddelta_motor_sp, v_dl_tether].^2);
inputMHE.WN = inputMHE.W(1:4,1:4);
inputMHE.shifting.strategy = 1;      

%% SIMULATION LOOP
display('------------------------------------------------------------------')
display('               Simulation Loop'                                    )
display('------------------------------------------------------------------')

iter = 0; time = 0;
Tf = 70;
INFO_MHE = [];
controls_MHE = [];
X_est = X0_MHE;
R_est = Rref;
uNMPC1 = 0;
uNMPC2 = 0;
radius=Rref;

%% 


while time(end) < Tf
    tic
    
    inputMHE.y = [inputMHE.y(2:end,:); inputMHE.yN.', uNMPC1,uNMPC2];                                            
    inputMHE.yN = [winch_encoder_(iter+1), arm_encoder_(iter+1), alpha_real_(iter+1), beta_real_(iter+1)].';
    outputMHE = acado_MHEstep(inputMHE); 
    INFO_MHE = [INFO_MHE; outputMHE.info];
    controls_MHE = [controls_MHE; outputMHE.u(end,:)]; 
    inputMHE.x = [outputMHE.x(2:end,:); outputMHE.x(end,:)]; 
    inputMHE.u = [outputMHE.u(2:end,:); outputMHE.u(end,:)]; 
    X_est = [X_est;outputMHE.x(end,:)];
    R_est = [R_est; (X_est(end,1)^2*cos(X_est(end,4))^2 + r_arm^2 + 2*X_est(end,1)*r_arm*cos(X_est(end,4))*cos(X_est(end,5)))^(1/2)];   
    uNMPC1 = motor_speedsp_(iter+1);
    uNMPC2 = winchspeed_sp_(iter+1);
    iter = iter+1;
    nextTime = iter*Ts; 
    disp(['current time: ' num2str(nextTime) '   ' char(9) ' (RTI step -- QP status: ' num2str(outputMHE.info.status) ',' ' ' char(2) ' KKT val: ' num2str(outputMHE.info.kktValue,'%1.2e') ',' ' ' char(2) ' CPU time: ' num2str((outputMHE.info.cpuTime*1e3)) ' ms)'])
    time = [time nextTime]; % 

end
%%
figure(1);
%Plotting alpha and beta
ax(1)=subplot(3,2,[1 2]);
[hAx1,hLine11,hLine21]=plotyy(time,X_est(:,4),time,X_est(:,5));hold (hAx1(1),'on'); hold (hAx1(2),'on');
plot(hAx1(1),time+0.02,alpha_real_(1:iter+1), 'r--','lineWidth',1.3);
plot(hAx1(2),time+0.02,beta_real_(1:iter+1), 'k--','lineWidth',1.3);
ylabel(hAx1(1),'Elevation [rad]')
ylabel(hAx1(2),'Azimuth [rad]') 
set(hLine11,'color','g','lineWidth',2.3)
set(hLine21,'color','g','lineWidth',2.3)
set(hAx1,{'ycolor'},{'r';'k'})
legend(hAx1(2),{'Estimation','Measurement'});
xlabel('Time [s]');

%Plotting carousel speed and Radius to check equality constraints
ax(2)=subplot(3,2,[3 4]);
[hAx2,hLine12,hLine22]=plotyy(time,X_est(:,3),time,R_est);hold (hAx2(1),'on'); hold (hAx2(2),'on');
plot(hAx2(1),time+0.02,arm_speed_(1:iter+1), 'r--','lineWidth',1.3);
set(hAx2,{'ycolor'},{'r';'g'})
set(hLine12,'color','g','lineWidth',2.3)
set(hLine22,'color','g','lineWidth',2.3)
xlabel('Time [s]');
legend(hAx2(1),{'Estimation','Measurement'});
ylabel(hAx2(1),'Carousel speed [rad/s]') 
ylabel(hAx2(2),'Radius [m]') 

%Plotting controls
ax(3)=subplot(3,2,[5 6]);
[hAx,hLine1,hLine2]=plotyy(time(1:end-1),controls_MHE(:,1),time(1:end-1),controls_MHE(:,2),'stairs');hold (hAx(1),'on'); hold (hAx(2),'on');
plot(hAx(1),time,motor_speedsp_(1:iter+1), 'k--','lineWidth',1.3);
plot(hAx(2),time,winchspeed_sp_(1:iter+1), 'k--','lineWidth',1.3);
xlabel('Time [s]');
ylabel(hAx(1),'Carousel speed [rad/s]') 
ylabel(hAx(2),'Winch velocity [m/s]') 
set(hAx,{'ycolor'},{[0.6,0,0.8];'c'})
set(hLine1,'color',[0.6,0,0.8],'lineWidth',2.3)
set(hLine2,'color','c','lineWidth',2.3)
linkaxes(ax,'x');
legend([hLine1 hLine2],{'Estimated arm speed','Estimated winch speed'});
%Plot extra tether length
figure(2)
plot(time+0.02,winch_encoder_(1:iter+1),'r')
hold on
plot(time,X_est(:,1),'g--')
xlabel('Time [s]')
ylabel('Tether length [m]')







