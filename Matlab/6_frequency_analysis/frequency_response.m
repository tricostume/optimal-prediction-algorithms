%------------------Albert-Ludwigs-Universitaet Freiburg--------------------
%___________________M. Sc. in Microsystems Engineering_____________________
%Thesis: 
%Period of preparation: April-September 2015
%Author: Ernesto Denicia
%Script: Frequency response of the system
%Description: This script compares the frequency response of the real
%system that was obtained in previous attempts with the old stand of the
%carousel and the simulated response with the tuned model. One looks for
%information on frequency regime and to distinguish what the role of the
%carbon fibre rod is in the frequency behaviour of the system.
%Comments: Experimental results were taken from the reference stated in the
%thesis.
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%% Theoretical Bode plot results
clear all; close all; clc;
%1. Load the previously obtained linearised model
load('tl_20.mat')
%2. Let MATLAB compute amplitude and phase responses in frequency
[mag, phase, wout] = bode(sys);
magdb = 20*log10(mag);
amplitudes=mag(1,1,1);
phases=phase(1,1,1);
for k=2:length(phase)
    amplitudes=[amplitudes; mag(1,1,k)];
    phases=[phases; phase(1,1,k)];
end
amplitudes_dB= 20*log10(amplitudes);
% 3. As the alpha angle is negative when the input is positive, there will
% always be a shift of pi rad between them. This has to be substracted.
% Notice this would be equivalent to multiplying the output (elevation
% angle) by -1.
phases_rad=(phases-180)*pi/180;
%% Experimental preparation step
%4. Load data and call amplitude and phase information
[u,y,t] = readnc();
w = 2*pi*[2 4 6 8 10 12 14 16]./40;
[~, ampl_u, phase_u] = fourierseries(t,u,w);
[~, ampl_y, phase_y] = fourierseries(t,y,w);

%% 5. Experimental Bode plot results
% Plot experimental amplitude values
fig_bode = figure;
set(fig_bode,'Position',[100,100,900,400])
position = [0.1 0.52 0.8 0.4];
subplot('position',position)
ampl_dB=20*log10(ampl_y./ampl_u);
semilogx(w,ampl_dB,'r','LineWidth', 2.5);
hold on;
%Plot simulated amplitude values
semilogx(wout(1:length(amplitudes_dB)),amplitudes_dB,'b','LineWidth', 2.5)
ylabel('Amplitude [dB]')
set(gca,'XTickLabel',[]);
grid on;
legend('Experiment','Simulation')
axis([min(w) max(w) -5 10])
position = [0.1 0.1 0.8 0.4];
subplot('position',position)
% Plot experimental amplitude values
phase_deg =(phase_y-phase_u)/pi*180;
ind = find(phase_deg>180);
for k=ind
    phase_deg(k) = phase_deg(k)-360;
end
semilogx(w,phase_deg*pi/180,'r','LineWidth', 2.5);
hold on;
% Plot simulated phase values 
semilogx(wout(1:length(phases_rad)),phases_rad,'b','LineWidth', 2.5);
axis([min(w) max(w) -200*pi/180 10*pi/180])
ylabel('Phase [rad]')
xlabel('\omega [rad/s]')
set(gca,'XTick',[0.4:0.1:1 2],'XTickLabel',{'';0.5;'';'';'';'';1;2});
grid on;
set(gcf, 'Color', 'w');

