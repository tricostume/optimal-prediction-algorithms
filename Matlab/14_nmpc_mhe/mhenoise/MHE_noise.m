clc;
clear all;
close all;

EXPORT = 0;
Ts = 0.02;      % sampling time

DifferentialState l_tether ddelta_arm alpha beta dddelta_arm dalpha dbeta  ddelta_motor_sp;      % provide proper names for all differential states and controls etc.
Control dddelta_motor_sp dl_tether;

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

% LAST QUESTION: HOW TO DEFINE A LINEAR INPUT COMBINED WITH THE ODE?
ode = [ dot(l_tether)==dl_tether;
        dot(ddelta_arm)==dddelta_arm;
        dot(alpha)==dalpha;
        dot(beta)==dbeta;
        dot(dddelta_arm)==wn*wn*ddelta_motor_sp-2*d*wn*dddelta_arm-wn*wn*ddelta_arm;
        dot(dalpha)==-(dalpha*my_alpha_LA + (I_tether*dbeta^2*sin(2*alpha))/2 + g*l_tether*m_ball*cos(alpha) + (dbeta^2*l_tether^2*m_ball*sin(2*alpha))/2 + (ddelta_arm^2*l_tether^2*m_ball*sin(2*alpha))/2 + dalpha*l_tether^2*mu_air*(dalpha^2*l_tether^2 + ddelta_arm^2*r_arm^2 + dbeta^2*l_tether^2*cos(alpha)^2 + ddelta_arm^2*l_tether^2*cos(alpha)^2 + 2*dbeta*ddelta_arm*l_tether^2*cos(alpha)^2 + 2*ddelta_arm^2*l_tether*r_arm*cos(alpha)*cos(beta) + 2*dbeta*ddelta_arm*l_tether*r_arm*cos(alpha)*cos(beta) - 2*dalpha*ddelta_arm*l_tether*r_arm*sin(alpha)*sin(beta))^(1/2) + dbeta*ddelta_arm*l_tether^2*m_ball*sin(2*alpha) + ddelta_arm^2*l_tether*m_ball*r_arm*cos(beta)*sin(alpha) - dddelta_arm*l_tether*m_ball*r_arm*sin(alpha)*sin(beta) - ddelta_arm*l_tether*mu_air*r_arm*sin(alpha)*sin(beta)*(dalpha^2*l_tether^2 + ddelta_arm^2*r_arm^2 + dbeta^2*l_tether^2*cos(alpha)^2 + ddelta_arm^2*l_tether^2*cos(alpha)^2 + 2*dbeta*ddelta_arm*l_tether^2*cos(alpha)^2 + 2*ddelta_arm^2*l_tether*r_arm*cos(alpha)*cos(beta) + 2*dbeta*ddelta_arm*l_tether*r_arm*cos(alpha)*cos(beta) - 2*dalpha*ddelta_arm*l_tether*r_arm*sin(alpha)*sin(beta))^(1/2))/(m_ball*l_tether^2 + I_tether);
        dot(dbeta)==-(dbeta*my_beta_LA + dddelta_arm*l_tether^2*m_ball*cos(alpha)^2 - I_tether*dalpha*dbeta*sin(2*alpha) - dalpha*dbeta*l_tether^2*m_ball*sin(2*alpha) - dalpha*ddelta_arm*l_tether^2*m_ball*sin(2*alpha) + dbeta*l_tether^2*mu_air*cos(alpha)^2*(dalpha^2*l_tether^2 + ddelta_arm^2*r_arm^2 + dbeta^2*l_tether^2*cos(alpha)^2 + ddelta_arm^2*l_tether^2*cos(alpha)^2 + 2*dbeta*ddelta_arm*l_tether^2*cos(alpha)^2 + 2*ddelta_arm^2*l_tether*r_arm*cos(alpha)*cos(beta) + 2*dbeta*ddelta_arm*l_tether*r_arm*cos(alpha)*cos(beta) - 2*dalpha*ddelta_arm*l_tether*r_arm*sin(alpha)*sin(beta))^(1/2) + ddelta_arm*l_tether^2*mu_air*cos(alpha)^2*(dalpha^2*l_tether^2 + ddelta_arm^2*r_arm^2 + dbeta^2*l_tether^2*cos(alpha)^2 + ddelta_arm^2*l_tether^2*cos(alpha)^2 + 2*dbeta*ddelta_arm*l_tether^2*cos(alpha)^2 + 2*ddelta_arm^2*l_tether*r_arm*cos(alpha)*cos(beta) + 2*dbeta*ddelta_arm*l_tether*r_arm*cos(alpha)*cos(beta) - 2*dalpha*ddelta_arm*l_tether*r_arm*sin(alpha)*sin(beta))^(1/2) + ddelta_arm^2*l_tether*m_ball*r_arm*cos(alpha)*sin(beta) + dddelta_arm*l_tether*m_ball*r_arm*cos(alpha)*cos(beta) + ddelta_arm*l_tether*mu_air*r_arm*cos(alpha)*cos(beta)*(dalpha^2*l_tether^2 + ddelta_arm^2*r_arm^2 + dbeta^2*l_tether^2*cos(alpha)^2 + ddelta_arm^2*l_tether^2*cos(alpha)^2 + 2*dbeta*ddelta_arm*l_tether^2*cos(alpha)^2 + 2*ddelta_arm^2*l_tether*r_arm*cos(alpha)*cos(beta) + 2*dbeta*ddelta_arm*l_tether*r_arm*cos(alpha)*cos(beta) - 2*dalpha*ddelta_arm*l_tether*r_arm*sin(alpha)*sin(beta))^(1/2))/(m_ball*l_tether^2*cos(alpha)^2 + I_tether*cos(alpha)^2 + I_LAS);
        dot(ddelta_motor_sp)==dddelta_motor_sp];          % <-- TODO: define the set of differential equations
    

%% Export of an optimization routine:
acadoSet('problemname', 'mhe');

N = 40;     % number of shooting intervals
ocp = acado.OCP( 0.0, N*Ts, N );

r_cyl = (l_tether^2*cos(alpha)^2 + r_arm^2 + 2*l_tether*r_arm*cos(alpha)*cos(beta))^(1/2);

index = [1,2,3,4,8];
h = [l_tether; ddelta_arm; alpha; beta; ddelta_motor_sp; controls];
hN = [l_tether; ddelta_arm; alpha; beta; ddelta_motor_sp];

W = acado.BMatrix(eye(length(h)));  % <-- TODO: weighting matrix for the stage cost
WN = acado.BMatrix(eye(length(hN))) ;

ocp.minimizeLSQ( W, h );            % stage cost--> 
ocp.minimizeLSQEndTerm( WN, hN );   % terminal cost

ocp.setModel(ode);      % pass the ODE model

mhe = acado.OCPexport( ocp );
mhe.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
mhe.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
mhe.set( 'SPARSE_QP_SOLUTION',          'CONDENSING_N2');
mhe.set( 'FIX_INITIAL_STATE',           'NO');
mhe.set( 'INTEGRATOR_TYPE',             'INT_RK4'           );  % RK4 method
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
%winchid
presimulation
X0_MHE = [1.6,1.246490265867967,-1.17,-0.045261843850114,0.000000000000001,0,0,1.246490265867967];       % <-- TODO: initial state (downward position)
%h = [l_tether; ddelta_arm; alpha; beta; ddelta_motor_sp; controls];
Y0_MHE = [1.6,1.246490265867967,-1.17,-0.045261843850114,1.246490265867967];
Rref = 2.673752570118138;

inputMHE.x = repmat(X0_MHE,N+1,1);      % <-- initialization of the state trajectory
inputMHE.u = zeros(N,2);     % <-- initialization of the control trajectory
inputMHE.y = repmat([Y0_MHE 0 0],N,1);   % <-- reference trajectory for the stage cost
inputMHE.yN = Y0_MHE.';  % <-- reference trajectory for the terminal cost WHY TRANSPOSED? DO WE HAVE TO GIVE A COLUMN VECTOR FOR THIS? 


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
v_l_tether = 1.1662e-05;
v_ddelta_arm = 0.004;
v_alpha = 6e-5;
v_beta = 6e-5;
v_ddelta_motor_sp =1e-12;

RE_sim = diag([v_l_tether, v_ddelta_arm, v_alpha, v_beta, v_ddelta_motor_sp].^2);

inputMHE.W = diag([1, 1, 1, 1, 1, 1, 1]);
inputMHE.WN = inputMHE.W(1:5,1:5);
inputMHE.shifting.strategy = 1;    % shifting is optional but highly recommended with RTI!
                                %      1: use xEnd, 2: integrate

%% SIMULATION LOOP
display('------------------------------------------------------------------')
display('               Simulation Loop'                                    )
display('------------------------------------------------------------------')

iter = 0; time = 0;
Tf = 10;
INFO_MHE = [];
controls_MHE = [];
%state_sim = X0;
X_est = X0_MHE;
R_est = Rref;
uNMPC1 = 0;
uNMPC2 = 0;
radius=Rref;
noisy_measurements = Y0_MHE;

%% 


while time(end) < Tf
    tic
    
    inputMHE.y = [inputMHE.y(2:end,:); inputMHE.yN.', uNMPC1,uNMPC2];
    %               measurement                   measurement     measurement                            measurement
    %                   |                              |               |                                                
    inputMHE.yN = [winch_encoder_(iter+1), motor_speed_(iter+1), alpha_real_(iter+1), beta_real_(iter+1), motor_speedsp_(iter+1)].'+ mvnrnd(zeros([5, 1]),RE_sim).';
    noisy_measurements = [noisy_measurements; inputMHE.yN.'];
    % Solve MHE OCP
    
    outputMHE = acado_MHEstep(inputMHE); 
    INFO_MHE = [INFO_MHE; outputMHE.info];
    controls_MHE = [controls_MHE; outputMHE.u(end,:)]; % Append first control of the horizon
    
    inputMHE.x = [outputMHE.x(2:end,:); outputMHE.x(end,:)]; %shift--- notice: the last two will be repeated
    inputMHE.u = [outputMHE.u(2:end,:); outputMHE.u(end,:)]; %shift--- notice: the last two will be repeated

    X_est = [X_est;outputMHE.x(end,:)];
    %(l_tether^2*cos(alpha)^2 + r_arm^2 + 2*l_tether*r_arm*cos(alpha)*cos(beta))^(1/2);
    R_est = [R_est; (X_est(end,1)^2*cos(X_est(end,3))^2 + r_arm^2 + 2*X_est(end,1)*r_arm*cos(X_est(end,3))*cos(X_est(end,4)))^(1/2)];
    
    uNMPC1 = ddmotor_speed_sp_(iter+1);
    uNMPC2 = dl_tether_(iter+1);
    iter = iter+1;
    nextTime = iter*Ts; 
    disp(['current time: ' num2str(nextTime) '   ' char(9) ' (RTI step -- QP status: ' num2str(outputMHE.info.status) ',' ' ' char(2) ' KKT val: ' num2str(outputMHE.info.kktValue,'%1.2e') ',' ' ' char(2) ' CPU time: ' num2str((outputMHE.info.cpuTime*1e3)) ' ms)'])
    time = [time nextTime]; % 
    
    %visualize(time, state_sim, Xref, xmin, xmax); 
    pause(0.75*abs(Ts-toc));
end

figure(1);


%Plotting alpha and beta
ax(1)=subplot(3,2,[1 2]);
[hAx1,hLine11,hLine21]=plotyy(time,X_est(:,3),time,X_est(:,4));hold (hAx1(1),'on'); hold (hAx1(2),'on');
legend('Elevation','Azimuth')
%plot(hAx1(1),[0 time(end)], [OFFSET OFFSET], 'r:');
%plot(hAx1(1),time,alpha_graph, 'k--');
plot(hAx1(1),time+0.02,alpha_real_(1:iter+1), 'k--');
%plot(hAx1(1),time+0.02,noisy_measurements(1:iter+1,3), 'r.-');

plot(hAx1(2),time+0.02,beta_real_(1:iter+1), 'k--');
%plot(hAx1(2),time+0.02,noisy_measurements(1:iter+1,4), 'r.-');

xlabel('time(s)');
ylabel(hAx1(1),'Elevation') % left y-axis
ylabel(hAx1(2),'Azimuth') % right y-axi
title('Important states')
grid on;
%Plotting carousel speed and Radius to check equality constraints
ax(2)=subplot(3,2,[3 4]);
[hAx2,hLine12,hLine22]=plotyy(time,X_est(:,2),time,R_est);hold (hAx2(1),'on'); hold (hAx2(2),'on');
%plot(hAx2(1),[0 time(end)], [0.2*2*pi 0.2*2*pi], 'r:');
plot(hAx2(1),time+0.02,motor_speed_(1:iter+1), 'k--');
%plot(hAx2(1),time+0.02,noisy_measurements(1:iter+1,2), 'r.-');

plot(hAx2(2),time+0.02,radius_(1:iter+1), 'k--');
xlabel('time(s)');
ylabel(hAx2(1),'Carousel speed (rad/s)') % left y-axis
ylabel(hAx2(2),'Radius (m)') % right y-axi
title('Equality constraints for carousel speed and radius')
grid on;


%plotting Radius
% figure(2);
% plot(time, radius);
% title('Radius equality constraint')
% xlabel('time(s)');
% ylabel('Radius (m)')
% grid on;

%Plotting controls
ax(3)=subplot(3,2,[5 6]);
[hAx,hLine1,hLine2]=plotyy(time(1:end-1),controls_MHE(:,1),time(1:end-1),controls_MHE(:,2),'stairs');hold (hAx(1),'on'); hold (hAx(2),'on');
%plot(hAx(1),[0 time(end)], [0 0], 'r:');
%plot(hAx(2),[0 time(end)], [dlmin dlmin], 'g--');
%plot(hAx(2),[0 time(end)], [dlmax dlmax], 'g--');
%plot(hAx(1),[0 time(end)], [dddmspmin dddmspmin], 'b--');
%plot(hAx(1),[0 time(end)], [dddmspmax dddmspmax], 'b--');
plot(hAx(1),time,ddmotor_speed_sp_(1:iter+1), 'k--');
%plot(hAx(2),time,dl_tether_(1:iter+1), 'r.-');
title('Controls')
xlabel('time(s)');
ylabel(hAx(1),'Motor acceleration (rad/s^2)') % left y-axis
ylabel(hAx(2),'Winch velocity (m/s)') % right y-axi
grid on;

linkaxes(ax,'x');
%Plot extra tether length
figure(2)

plot(time,X_est(:,1),'g')
hold on
plot(time+0.02,winch_encoder_(1:iter+1),'b')

%plot(time+0.02,noisy_measurements(1:iter+1,1), 'r.-');

%suptitle('NMPC for a MIMO-modelled tethered ball')






