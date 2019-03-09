function [ a0,ampl, phase ]=fourierseries(t,y,w)
debug = false;

if isrow(w)
    w=w';
end
if isrow(y)
    y=y';
end
if iscolumn(t)
    t=t';
end

n = length(t);
a0 = sum(y)/n;
a = 2/n .* cos(w*t) * y;
b = 2/n .* sin(w*t) * y;
ampl = sqrt(a.^2+b.^2);
phase = pi-atan2(b,a);


if debug 
    %signal reproduction
    %figure;
    [~,a01,a1,b1] = fitfou(t,y);
    hold on;
    wt = w*t;
    phases = (phase+-pi/2)*ones(1,length(t));
    y_rep =a0 + ampl'*sin(wt + phases);
    %plot(t,y,'b-',t,y_rep,'g-')
    
    %figure;
    %subplot(2,1,1)
    %plot([a0 a'],'bo')
    %hold on;
    %plot([a01 a1],'rx')
    %subplot(2,1,2)
    %plot(b,'bo')
    %hold on;
    %plot(b1,'rx')
   
    
end

end