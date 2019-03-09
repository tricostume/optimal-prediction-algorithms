function [] = visualize(time,radius,height, Rref, Zref)

xmin = 0.5;
xmax = 0.5;
ymin = 0.5;
ymax = 0.5;

figure(10); clf
whitebg([1.0 1.0 1.0])
set(gcf,'Color',[1 1 1])

%Axes manipulation, maximum and minimum for each of them
line([Rref-xmin Rref+xmax], [-Zref -Zref], 'color', 'k', 'Linewidth',1.5); hold on;
line([Rref Rref], [-Zref-ymin -Zref+ymax], 'color', 'k', 'Linewidth',1.5); hold on;

%Where the ball should be in steady states
plot(Rref, -Zref, 'gx', 'MarkerSize', 16, 'Linewidth', 2);
text(Rref+0.1,-Zref-ymin,['current time: ' num2str(time(end)) 's'],'FontSize',15);
%The ball
plot(radius,height,'kO','MarkerSize',50,'Linewidth',3,'color','r');

grid on;
xlim([Rref-xmin Rref+xmax]);
ylim([-Zref-ymin-0.05 -Zref+ymax]);