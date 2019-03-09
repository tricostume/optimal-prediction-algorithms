%------------------Albert-Ludwigs-Universitaet Freiburg---------------------
%___________________M. Sc. in Microsystems Engineering_____________________
%Thesis: 
%Period of preparation: April-August 2015
%Author: Ernesto Denicia
%Script: Motor drive NetCD-Files reader
%Description: The variables of the NetCD Files are extracted and saved in
%the workspace.
%Comments: The same NetCD-Files provided in the rooth directory have to be
%used for successful extraction of the variables.
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
function [ timestamp1, winchspeed_sp,winchspeed_smoothed,winch_encoder,...
           winch_torque, winch_current,carouselspeed_sp,...
           carouselspeed_smoothed,carousel_encoder,carousel_torque, ...
           carousel_current] = read_siemens( a )
       
ncid1 = netcdf.open(a);
timestamp1 = netcdf.getVar(ncid1, 0);
winchspeed_sp = netcdf.getVar(ncid1, 1);
winchspeed_smoothed = netcdf.getVar(ncid1, 2);
winch_encoder = netcdf.getVar(ncid1, 3);
winch_torque = netcdf.getVar(ncid1, 4);
winch_current = netcdf.getVar(ncid1, 5);
carouselspeed_sp = netcdf.getVar(ncid1, 6);
carouselspeed_smoothed = netcdf.getVar(ncid1, 7);
carousel_encoder = netcdf.getVar(ncid1, 8);
carousel_torque = netcdf.getVar(ncid1, 9);
carousel_current = netcdf.getVar(ncid1, 10);
end

