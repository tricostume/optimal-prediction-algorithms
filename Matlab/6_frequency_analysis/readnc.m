function [in_mean,out_mean,time_mean] = readnc()
%close all;
%clear all;

debug = false;

timestamp1 = ncread('lineAngleSensor2Data.nc','TimeStamp');
out_raw = ncread('lineAngleSensor2Data.nc','lineAngleSensor2.data.elevation');
time = linspace(0,timestamp1(end),length(timestamp1));
%plot(timestamp1,out_raw)
figure

timestamp2 = ncread('functionGeneratorData.nc','TimeStamp');
in_raw = ncread('functionGeneratorData.nc','functionGenerator.refData.elevation');
clock_raw = ncread('functionGeneratorData.nc','functionGenerator.refData.cycle');

out = interp1(timestamp1 ,out_raw ,time);
in = interp1(timestamp2 ,in_raw ,time);
clock = interp1(timestamp2 ,clock_raw ,time);

t1 = find(clock == 1, 1 );
t2 = find(clock(t1:end) == 0, 1 )+t1-1;
t3 = find(clock(t2:end) == 1,1)+t2-1;
t4 = find(clock(t3:end) == 0,1)+t3-1;

dt = t4-t2;
time_mean = time(1:dt);

perioden = 9;
in_matrix = reshape(in(t1:t1+perioden*dt-1),dt,[]);
in_mean = mean(in_matrix,2);

skip = 2;
out_matrix = reshape(out(t1+skip*dt:t1+(perioden-skip)*dt-1),dt,[]);
out_mean = mean(out_matrix,2);


%Ploten für signale3
%plot(time_mean,out_matrix(:,5),'LineWidth',2)
%hold on;
%plot(time_mean,out_mean,'r-','LineWidth',2)

if debug
    meanfig = figure;
    signals = figure;
    hold on;
    for k=1:8
        figure(meanfig);
        plot(time_mean,in_matrix(:,k))
        figure(signals);
        stem(time(t2+k*dt),2)
    end
    
    
    %plot(time,in,'r-',time(t1:(t1+10*dt)-1),[in_mean; in_mean; in_mean; in_mean; in_mean; in_mean; in_mean; in_mean; in_mean; in_mean],'g-')
    %hold on
    %plot(time,out,time,clock,'g.',time(t1:(t1+10*dt)-1),[out_mean;out_mean; out_mean; out_mean; out_mean; out_mean; out_mean; out_mean; out_mean; out_mean],'g-')
    %stem([time(t1),time(t2),time(t3),time(t4)],[1,0,1,0],'MarkerSize',4);
end
end
