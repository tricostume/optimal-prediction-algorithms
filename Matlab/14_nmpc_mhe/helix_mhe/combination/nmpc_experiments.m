%------------------Albert-Ludwigs-Universitaet Freiburg--------------------
%___________________M. Sc. in Microsystems Engineering_____________________
%Thesis: 
%Period of preparation: April-September 2015
%Author: Ernesto Denicia
%Script: NMPC + MHE implementation using RK4 integrator of ACADO. 
%Comments: This script has the possibility to choose between experiments
%and if noise should be applied or not. 
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------

%% SIMULATION INITIALIZATION
clc;
clear all;
close all;
N=40;
Ts = 0.02;
%Choose options: 1-> Activated    0-> Not activated
process_noise = 0;
sensor_noise=0;
animation=1;
experiment_number = 2;

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

if experiment_number == 3
    X0 = [1.8, 1.186347700078462, -1.201023842411613, -0.046154929792893, 0.000000000000000, 0, 0, 1.186347700078462 ];
    Y0 = [2.7,1.678337270339664];  
       
else   
    X0 = [1.6, 1.246490265867967, -1.17, -0.045261843850114, 0.000000000000001, 0, 0, 1.246490265867967];
    Y0 = [2.673752570118138,1.473200956377817];    
    A = 0.1;
    w = 2*pi*0.1;
    OFFSET = -1.17;
    alpha_ref = zeros(40,1);
end
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
input.y = repmat([X0 R0 -h0 0 0],N,1);   
input.yN = [X0 R0 -h0].';

if experiment_number == 1
input.W = diag([10 0.0001 24000 1000 0.0001 0.0001 0.001 1 20000 0 10 1]);
                    
input.WN = input.W(1:10,1:10);
end

if experiment_number == 2
input.W = diag([10 0.0001 24000 1000 0.0001 0.0001 0.001 1 0 20000 10 1]);
input.WN = input.W(1:10,1:10);
end       

if experiment_number == 3
input.W = diag([10 0.0001 100 100 0.0001 0.0001 10 1 20000 20000 1 1]);
input.WN = input.W(1:10,1:10);
end
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
Tf = 16;
Thelix = 8; % this works in case of choosing experiment 3
Rref = R0;

%MPC Parametrization preparation
INFO_MPC = [];
controls_MPC = [];
state_sim = X0_MHE;
height = -h0;
radius = R0;
%The next constraint values are written only for graphing purposes
dddmspmin = -0.3; 
dddmspmax = 0.3;
dlmin = -0.4; 
dlmax = 0.4;
lthmin = 1.37;
lthmax = 1.9;

    %Generate manifold: Used if experiment 3 was chosen
    
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
h_est = -h0;
R_est = R0;
uNMPC1 = 1.246490265867967;
uNMPC2 = 0;

%% START SIMULATING EXPERIMENT 1
if experiment_number == 1

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
    R_est = [R_est; (X_est(end,1)^2*cos(X_est(end,4))^2 + r_arm^2 + 2*X_est(end,1)*r_arm*cos(X_est(end,4))*cos(X_est(end,5)))^(1/2)];
    h_est = [h_est; X_est(end,1)*sin(X_est(end,4))];
    
    %3. Prepare MPC reference
    for k=1:1:40
    alpha_ref(k)=OFFSET+A*sin(w*(time(end)+(k-1)*Ts));
    input.y(k,:) = [X0(1:2), alpha_ref(k), X0(4:8), R0,-h0, 0,0];
    end
    alpha_last = OFFSET+A*sin(w*(time(end)+(40)*Ts));
    input.yN = [X0(1:2), alpha_last, X0(4:8), R0, -h0].';
    
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
    radius = [radius; (states.value(1)^2*cos(states.value(4))^2 + r_arm^2 + 2*states.value(1)*r_arm*cos(states.value(4))*cos(states.value(5)))^(1/2)];
    height = [height; states.value(1)*sin(states.value(4))];
    
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
    alpha_graph(k)=OFFSET+A*sin(w*((k-1)*Ts));
end

end

%% START SIMULATING EXPERIMENT 2

if experiment_number == 2
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
    alpha_ref(k)=OFFSET+A*sin(w*(time(end)+(k-1)*Ts));
    input.y(k,:) = [X0(1:2), alpha_ref(k), X0(4:8), R0, -h0, 0,0];
    end
    alpha_last = OFFSET+A*sin(w*(time(end)+(40)*Ts));
    input.yN = [X0(1:2), alpha_last, X0(4:8), R0, -h0].';
    
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
    alpha_graph(k)=OFFSET+A*sin(w*((k-1)*Ts));
end

end
%% START SIMULATING EXPERIMENT 3

if experiment_number == 3
    
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
        visualize(time(end),R_est(end),h_est(end),R0,h0)
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
end
%% Ploting section

figure(1);
%Plotting elevation and azimuth angles
ax(1)=subplot(3,2,[1 2]);
[hAx1,hLine11,hLine21]=plotyy(time,X_est(:,4),time,X_est(:,5));hold (hAx1(1),'on'); hold (hAx1(2),'on');
legend('Simulated','Simulated')
set(hLine11,'color','b','lineWidth',1.3)
set(hLine21,'lineWidth',1.3)
if experiment_number ~= 3
 plot(hAx1(1),time,alpha_graph, 'k--');   
end
xlabel('Time [s]');
ylabel(hAx1(1),'Elevation angle [rad]') % left y-axis
ylabel(hAx1(2),'Azimuth angle [rad]') % right y-axi
grid on;



%Plotting carousel speed and Radius to check equality constraints
ax(2)=subplot(3,2,[3 4]);
[hAx2,hLine12,hLine22]=plotyy(time,X_est(:,3),time,R_est);hold (hAx2(1),'on'); hold (hAx2(2),'on');
set(hLine12,'color','b','lineWidth',1.3)
set(hLine22,'lineWidth',1.3)
if experiment_number == 3
plot(hAx2(2),time,Rgraph, 'k--');
end
xlabel('Time [s]');
ylabel(hAx2(1),'Carousel speed [rad/s]') % left y-axis
ylabel(hAx2(2),'Radius [m]') % right y-axi
grid on;


%Plotting controls
ax(3)=subplot(3,2,[5 6]);
[hAx,hLine1,hLine2]=plotyy(time(1:end-1),controls_MPC(:,1),time(1:end-1),controls_MHE(:,2),'stairs');hold (hAx(1),'on'); hold (hAx(2),'on');
set(hLine1,'color',[0.6,0,0.8],'lineWidth',1.5)
set(hLine2,'color','c','lineWidth',1.5)
plot(hAx(1),[0 time(end)], [0 0], 'r:');
plot(hAx(2),[0 time(end)], [dlmin dlmin], 'c--');
plot(hAx(2),[0 time(end)], [dlmax dlmax], 'c--');
plot(hAx(1),[0 time(end)], [dddmspmin dddmspmin], '--','color',[0.6,0,0.8]);
plot(hAx(1),[0 time(end)], [dddmspmax dddmspmax], '--', 'color',[0.6,0,0.8]);
% plot(hAx(2),time(1:end-1),controls_MPC(:,2), 'r--');
set(hAx,{'ycolor'},{[0.6,0,0.8];'c'})
xlabel('Time [s]');
ylabel(hAx(1),'Motor speed (rad/s^2)') % left y-axis
ylabel(hAx(2),'Winch velocity (m/s)') % right y-axi
grid on;
legend('Simulated','Simulated')
linkaxes(ax,'x');
legend([hLine1 hLine2],{'Simulated','Simulated'});

%Plot tether length and height
figure(2)
[hAx3,hLine31,hLine32]=plotyy(time,X_est(:,1),time+0.02,height(1:iter+1,1));hold (hAx3(1),'on'); hold (hAx3(2),'on');
% plot(hAx3(1),time+0.02,state_sim(1:iter+1,1), 'r--');
% plot(hAx3(2),time+0.02,height(1:iter+1), 'r--')
ylabel(hAx3(1),'Tether length [m]') % left y-axis
ylabel(hAx3(2),'Height [m]') % right y-axi
set(hLine31,'color','b','lineWidth',1.3)
set(hLine32,'lineWidth',1.3)
plot(hAx3(1),[0 time(end)], [lthmin lthmin], 'b--');
plot(hAx3(1),[0 time(end)], [lthmax lthmax], 'b--');
legend([hLine31 hLine32],{'Simulated','Simulated'});
if experiment_number == 3
plot(time,hgraph, 'k--');
end
grid on;


%plot(time+0.02,noisy_measurements(1:iter+1,1), 'r.-');

%suptitle('NMPC for a MIMO-modelled tethered ball')


