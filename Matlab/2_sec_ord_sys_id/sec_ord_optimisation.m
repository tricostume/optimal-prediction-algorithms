%% Function for finding optimal values
%The initial estimate was generated through manual tuning
function [ theta,confidence,Q_xx,std_dev ] = sec_ord_optimisation(t,u,y)
theta0=[1,5.685085244,0.11];
[theta,resnorm,residual,exitflag,output,lambda,jacobian]=...
        lsqnonlin(@(theta)sec_ord_residuals( t, u, y, theta),theta0,[],[]);
confidence = nlparci(theta,residual,'jacobian',jacobian);
[rows,cols]=size(jacobian);
Sigma_o = sqrt((residual'*residual) / (rows-cols));
Q_xx  = Sigma_o*inv(jacobian'*jacobian);
diagonals_of_Q_xx = diag(Q_xx);
std_dev = Sigma_o .* sqrt(diagonals_of_Q_xx);
end
%% Residuals function
function [ res ] = sec_ord_residuals( t,u,y,theta )
K=theta(1);
wn=theta(2);
d=theta(3);
sos=tf([K*wn*wn],[1 2*d*wn wn*wn]);
sossim=lsim(sos,u,t);
res=sossim-y;
end

