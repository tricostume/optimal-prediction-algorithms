figure(1);


%Plotting alpha and beta
ax(1)=subplot(3,2,[1 2]);
[hAx1,hLine11,hLine21]=plotyy(time,X_est(:,4),time,X_est(:,5));hold (hAx1(1),'on'); hold (hAx1(2),'on');
legend('Elevation','Azimuth')
%plot(hAx1(1),[0 time(end)], [OFFSET OFFSET], 'r:');
%plot(hAx1(1),time,alpha_graph, 'k--');
plot(hAx1(1),time+0.02,alpha_real_(1:iter+1), 'k--');
%plot(hAx1(1),time+0.02,noisy_measurements(1:iter+1,3), 'r.');

plot(hAx1(2),time+0.02,beta_real_(1:iter+1), 'k--');
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
plot(hAx2(1),time+0.02,arm_speed_(1:iter+1), 'k--');
%plot(hAx2(2),time+0.02,radius_(1:iter+1), 'k--');
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
[hAx,hLine1,hLine2]=plotyy(time(1:end-1),controls_MHE(:,1),time(1:end-1),controls_MHE(:,2),'stairs');hold (hAx(1),'on'); hold (hAx(2),'on');
%plot(hAx(1),[0 time(end)], [0 0], 'r:');
%plot(hAx(2),[0 time(end)], [dlmin dlmin], 'g--');
%plot(hAx(2),[0 time(end)], [dlmax dlmax], 'g--');
%plot(hAx(1),[0 time(end)], [dddmspmin dddmspmin], 'b--');
%plot(hAx(1),[0 time(end)], [dddmspmax dddmspmax], 'b--');
plot(hAx(1),time,motor_speedsp_(1:iter+1), 'k--');
plot(hAx(2),time,winchspeed_sp_(1:iter+1), 'k--');
title('Controls')
xlabel('time(s)');
ylabel(hAx(1),'Motor velocity (rad/s^2)') % left y-axis
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
