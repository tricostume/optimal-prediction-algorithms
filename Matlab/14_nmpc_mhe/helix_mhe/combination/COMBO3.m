%% SIMULATION INITIALIZATION
clc;
clear all;
close all;
N=40;
Ts = 0.02;
%Choose options: 1-> Activated    0-> Not activated
process_noise = 1;
sensor_noise=1;
animation=0;

%Physical parameters of the model
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

% Propose initial state to MPC and initial estimate to MHE
% Also final state of MPC

X0 = [1.8, 1.186347700078462, -1.201023842411613, -0.046154929792893, 0.000000000000000, 0, 0, 1.186347700078462 ];
Y0 = [2.7,1.678337270339664];       
Yref = Y0;
R0 = Y0(1);
h0 = Y0(2);

Xf = [1.5, 1.636784578436209, -0.884034465885497, -0.056983803558526, -0.000000000000000, 0, 0, 1.636784578436209 ];
Yf = [3,1.159954745795368];
Rf = Yf(1);
hf = Yf(2);

X0_MHE = [X0(1), 0, X0(2), X0(3), X0(4), X0(5), 0,0];
Y0_MHE = [X0_MHE(1),X0_MHE(2),X0_MHE(4),X0_MHE(5)];


%Prepare input for MPC
input.x = repmat(X0,N+1,1);     
input.u = zeros(N,2);     
input.y = repmat([X0 R0 h0 0 0],N,1);   
input.yN = [X0 R0 h0].';
                
input.W = diag([10 0.0001 100 100 0.0001 0.0001 10 1 20000 20000 1 1]);
input.WN = input.W(1:10,1:10);
input.shifting.strategy = 1; 

%Prepare input for MHE
inputMHE.x = repmat(X0_MHE,N+1,1);      
inputMHE.u = ones(N,2)*[1.246490265867967 0; 0 0];     
inputMHE.y = repmat([Y0_MHE 1.246490265867967 0],N,1);   
inputMHE.yN = Y0_MHE.';

    %Sensor noise description
    if sensor_noise == 1
        factor=1;
        v_l_tether = factor*1.75e-3;
        v_delta_arm = factor*1.75e-3;
        v_alpha = factor*1.75e-3;
        v_beta = factor*1.75e-3;
            %Control variances
        v_ddelta_motor_sp =factor*1e-04;
        v_dl_tether = factor*1e-4;
        RE_sim = diag([v_l_tether, v_delta_arm, v_alpha, v_beta].^2);
   
    else
        v_l_tether = 1;
        v_delta_arm = 1;
        v_alpha = 1;
        v_beta = 1;
            %Control variances
        v_ddelta_motor_sp =1;
        v_dl_tether = 1;
        RE_sim = diag([0, 0, 0, 0].^2);
    end
    
    

inputMHE.W = diag(1./[v_l_tether, v_delta_arm, v_alpha, v_beta, v_ddelta_motor_sp, v_dl_tether].^2);
inputMHE.WN = inputMHE.W(1:4,1:4);
inputMHE.shifting.strategy = 1;   

%% SIMULATION LOOP

%General parametrization
iter = 0; time = 0;
Tf = 10;
Thelix = 5;
Rref = R0;

%MPC Parametrization preparation
INFO_MPC = [];
controls_MPC = [];
state_sim = X0_MHE;
height = -h0;
radius = R0;

    %Generate manifold
    
    y = [-h0;-hf];
    x = [R0^2,1;Rf^2,1];
    theta = pinv(x)*y;
    a = theta(1);
    b = theta(2);
    
    %Process noise description
    
    if process_noise == 1
        w_l_tether = 1e-12;
        w_ddelta_arm = 1e-12; 
        w_alpha = 1e-12;
        w_beta = 1e-12;
        w_dddelta_arm = 1e-5;
        w_dalpha = 0.0011;
        w_dbeta = 0.0012;
        w_ddelta_motor_sp = 0;
        
    else
        w_l_tether = 0;
        w_ddelta_arm = 0; 
        w_alpha = 0;
        w_beta = 0;
        w_dddelta_arm = 0;
        w_dalpha = 0;
        w_dbeta = 0;
        w_ddelta_motor_sp = 0;
    end
    QE_sim = diag([w_l_tether, w_ddelta_arm, w_alpha, w_beta, w_dddelta_arm, w_dalpha, w_dbeta, w_ddelta_motor_sp].^2);

%MHE Parametrization preparation
INFO_MHE = [];
controls_MHE = [];
X_est = X0_MHE;
h_est = h0;
R_est = R0;
uNMPC1 = 1.246490265867967;
uNMPC2 = 0;

%% START SIMULATING

while time(end) < Tf
    tic
    
    %1. Prepare MHE matrices and solve
    inputMHE.y = [inputMHE.y(2:end,:); inputMHE.yN.', uNMPC1,uNMPC2];
    inputMHE.yN = [state_sim(end,1), state_sim(end,2), state_sim(end,4), state_sim(end,5)].'+ mvnrnd(zeros([4, 1]),RE_sim).';
    outputMHE = acado_MHEstep(inputMHE);
    %2. Log MHE Info, controls, X_est, R_est and shift its inputs
    INFO_MHE = [INFO_MHE; outputMHE.info];
    controls_MHE = [controls_MHE; outputMHE.u(end,:)]; 
    inputMHE.x = [outputMHE.x(2:end,:); outputMHE.x(end,:)]; 
    inputMHE.u = [outputMHE.u(2:end,:); outputMHE.u(end,:)]; 
    X_est = [X_est;outputMHE.x(end,:)];
    h_est = [h_est; X_est(end,1)*sin(X_est(end,4))];
    R_est = [R_est; (X_est(end,1)^2*cos(X_est(end,4))^2 + r_arm^2 + 2*X_est(end,1)*r_arm*cos(X_est(end,4))*cos(X_est(end,5)))^(1/2)];

    %3. Prepare MPC reference
    for k=1:1:40
    Rref = (((time(end)+Ts*(k-1))*(Rf-R0))/Thelix)+R0;
    
    if Rref >= Rf
        Rref=Rf;
    end
    
    href = (a*Rref^2)+b;
    input.y(k,:) = [Xf, Rref, href, 0, 0];   % <-- TODO: reference trajectory for the stage cost
    end
    Rref = (((time(end)+Ts*(40))*(Rf-R0))/Thelix)+R0;
    
    if Rref >= Rf
        Rref=Rf;
    end
    href = (a*Rref^2)+b;
    input.yN = [Xf, Rref, href].';
    
    
    %4. Pass the MHE estimate to the MPC as x0 and solve (close the loop)
    input.x0 = [X_est(end,1), X_est(end,3:8), controls_MHE(end,1)].';
    output = acado_MPCstep(input);
    %5. Log MPC Info and controls and shift its inputs
    INFO_MPC = [INFO_MPC; output.info];
    controls_MPC = [controls_MPC; output.u(1,:)];
    input.x = [output.x(2:end,:); output.x(end,:)]; %shift
    input.u = [output.u(2:end,:); output.u(end,:)];
    
    %6. Simulate real system for comparison with estimation
    sim_input.x = state_sim(end,:).'+ mvnrnd(zeros([8, 1]),QE_sim).';
        %Pass the MPC control to the simulation
    sim_input.u = [output.x(2,8),output.u(1,2)].';
    states = simulate_system(sim_input);
    state_sim = [state_sim; states.value'];
    height = [height; states.value(1)*sin(states.value(4))];
    radius = [radius; (states.value(1)^2*cos(states.value(4))^2 + r_arm^2 + 2*states.value(1)*r_arm*cos(states.value(4))*cos(states.value(5)))^(1/2)];

    %7. Pass the MPC controls to the MHE for further estimation
    uNMPC1 = output.x(2,8);
    uNMPC2 = output.u(1,2);

    %8. Increment the iteration number and let time advance
    iter = iter+1;
    nextTime = iter*Ts; 
    disp(['current time: ' num2str(nextTime) '   ' char(9) ' (RTI step -- QP status: ' num2str(output.info.status) ',' ' ' char(2) ' KKT val: ' num2str(output.info.kktValue,'%1.2e') ',' ' ' char(2) ' CPU time: ' num2str((output.info.cpuTime*1e3)) ' ms)'])
    time = [time nextTime];
    
    %Visualize if desired
    
    if animation == 1
    visualize(time(end),radius(end),height(end),R0,h0)
    end
    
end

%For comparison, the reference elevation is also computed
for k=1:1:Tf/Ts+1
    Rgraph(k) = ((((k-1)*Ts)*(Rf-R0))/Thelix)+R0;
    if Rgraph(k) >= Rf
        Rgraph(k) = Rf;
    end
    hgraph(k) = (a*Rgraph(k)^2)+b;
end

%% Ploting section

figure(1);

%Plotting elevation and azimuth angles
ax(1)=subplot(3,2,[1 2]);
[hAx1,hLine11,hLine21]=plotyy(time,X_est(:,4),time,X_est(:,5));hold (hAx1(1),'on'); hold (hAx1(2),'on');
legend('Elevation','Azimuth')
%plot(hAx1(1),[0 time(end)], [OFFSET OFFSET], 'r:');
plot(hAx1(1),time,state_sim(1:iter+1,4), 'r--');
plot(hAx1(2),time,state_sim(1:iter+1,5), 'r--');
%plot(hAx1(1),time+0.02,noisy_measurements(1:iter+1,3), 'r.');
%plot(hAx1(2),time+0.02,noisy_measurements(1:iter+1,4), 'r.');

xlabel('time(s)');
ylabel(hAx1(1),'Elevation') % left y-axis
ylabel(hAx1(2),'Azimuth') % right y-axi
title('Important states')
grid on;
%Plotting carousel speed and Radius to check equality constraints
ax(2)=subplot(3,2,[3 4]);
[hAx2,hLine12,hLine22]=plotyy(time,X_est(:,3),time,R_est);hold (hAx2(1),'on'); hold (hAx2(2),'on');
%plot(hAx2(1),[0 time(end)], [0.2*2*pi 0.2*2*pi], 'r:');
plot(hAx2(1),time+0.02,state_sim(1:iter+1,3), 'r--');
plot(hAx2(2),time+0.02,radius(1:iter+1), 'r--');
plot(hAx2(2),time,Rgraph, 'k--');
%plot(hAx2(1),time+0.02,noisy_measurements(1:iter+1,2), 'r.');
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
[hAx,hLine1,hLine2]=plotyy(time(1:end-1),controls_MPC(:,1),time(1:end-1),controls_MHE(:,2),'stairs');hold (hAx(1),'on'); hold (hAx(2),'on');
%plot(hAx(1),[0 time(end)], [0 0], 'r:');
%plot(hAx(2),[0 time(end)], [dlmin dlmin], 'g--');
%plot(hAx(2),[0 time(end)], [dlmax dlmax], 'g--');
%plot(hAx(1),[0 time(end)], [dddmspmin dddmspmin], 'b--');
%plot(hAx(1),[0 time(end)], [dddmspmax dddmspmax], 'b--');

%plot(hAx(1),time,motor_speedsp_(1:iter+1), 'k--');
plot(hAx(2),time(1:end-1),controls_MPC(:,2), 'r--');
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
plot(time+0.02,state_sim(1:iter+1,1),'b')
plot(time,hgraph, 'k--');
plot(time+0.02,height(1:iter+1), 'r--')

%plot(time+0.02,noisy_measurements(1:iter+1,1), 'r.-');

%suptitle('NMPC for a MIMO-modelled tethered ball')


