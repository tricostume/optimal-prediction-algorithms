/*
*    This file is part of ACADO Toolkit.
*
*    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
*    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
*    Developed within the Optimization in Engineering Center (OPTEC) under
*    supervision of Moritz Diehl. All rights reserved.
*
*    ACADO Toolkit is free software; you can redistribute it and/or
*    modify it under the terms of the GNU Lesser General Public
*    License as published by the Free Software Foundation; either
*    version 3 of the License, or (at your option) any later version.
*
*    ACADO Toolkit is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*    Lesser General Public License for more details.
*
*    You should have received a copy of the GNU Lesser General Public
*    License along with ACADO Toolkit; if not, write to the Free Software
*    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
*/


/**
*    Author David Ariens, Rien Quirynen
*    Date 2009-2013
*    http://www.acadotoolkit.org/matlab 
*/

#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <acado/utils/matlab_acado_utils.hpp>

USING_NAMESPACE_ACADO

#include <mex.h>


void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) 
 { 
 
    MatlabConsoleStreamBuf mybuf;
    RedirectStream redirect(std::cout, mybuf);
    clearAllStaticCounters( ); 
 
    mexPrintf("\nACADO Toolkit for Matlab - Developed by David Ariens and Rien Quirynen, 2009-2013 \n"); 
    mexPrintf("Support available at http://www.acadotoolkit.org/matlab \n \n"); 

    if (nrhs != 0){ 
      mexErrMsgTxt("This problem expects 0 right hand side argument(s) since you have defined 0 MexInput(s)");
    } 
 
    TIME autotime;
    DifferentialState l_tether;
    DifferentialState ddelta_arm;
    DifferentialState alpha;
    DifferentialState beta;
    DifferentialState dddelta_arm;
    DifferentialState dalpha;
    DifferentialState dbeta;
    DifferentialState ddelta_motor_sp;
    Control dddelta_motor_sp;
    Control dl_tether;
    SIMexport ExportModule1( 1, 0.02 );
    ExportModule1.set( GENERATE_MATLAB_INTERFACE, 1 );
    uint options_flag;
    options_flag = ExportModule1.set( INTEGRATOR_TYPE, INT_RK4 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: INTEGRATOR_TYPE");
    options_flag = ExportModule1.set( NUM_INTEGRATOR_STEPS, 2 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: NUM_INTEGRATOR_STEPS");
    DifferentialEquation acadodata_f1;
    acadodata_f1 << dot(l_tether) == (-dl_tether);
    acadodata_f1 << dot(ddelta_arm) == dddelta_arm;
    acadodata_f1 << dot(alpha) == dalpha;
    acadodata_f1 << dot(beta) == dbeta;
    acadodata_f1 << dot(dddelta_arm) == (-1.607656E+00*dddelta_arm-3.894883E+01*ddelta_arm+3.894883E+01*ddelta_motor_sp);
    acadodata_f1 << dot(dalpha) == (-1.000600E+00/2.000000E+00*pow(dbeta,2.000000E+00)*sin(2.000000E+00*alpha)-1/2.000000E+00*5.700000E-01*pow(dbeta,2.000000E+00)*pow(l_tether,2.000000E+00)*sin(2.000000E+00*alpha)-1/2.000000E+00*5.700000E-01*pow(ddelta_arm,2.000000E+00)*pow(l_tether,2.000000E+00)*sin(2.000000E+00*alpha)-2.050000E+00*5.700000E-01*cos(beta)*l_tether*pow(ddelta_arm,2.000000E+00)*sin(alpha)+2.050000E+00*5.700000E-01*dddelta_arm*l_tether*sin(alpha)*sin(beta)+2.050000E+00*7.400000E-03*ddelta_arm*l_tether*sin(alpha)*sin(beta)*sqrt((2.000000E+00*2.050000E+00*cos(alpha)*cos(beta)*dbeta*ddelta_arm*l_tether+2.000000E+00*2.050000E+00*cos(alpha)*cos(beta)*l_tether*pow(ddelta_arm,2.000000E+00)-2.000000E+00*2.050000E+00*dalpha*ddelta_arm*l_tether*sin(alpha)*sin(beta)+2.000000E+00*dbeta*ddelta_arm*pow(cos(alpha),2.000000E+00)*pow(l_tether,2.000000E+00)+4.202500E+00*pow(ddelta_arm,2.000000E+00)+pow(cos(alpha),2.000000E+00)*pow(dbeta,2.000000E+00)*pow(l_tether,2.000000E+00)+pow(cos(alpha),2.000000E+00)*pow(ddelta_arm,2.000000E+00)*pow(l_tether,2.000000E+00)+pow(dalpha,2.000000E+00)*pow(l_tether,2.000000E+00)))-4.876000E-01*dalpha-5.700000E-01*9.810000E+00*cos(alpha)*l_tether-5.700000E-01*dbeta*ddelta_arm*pow(l_tether,2.000000E+00)*sin(2.000000E+00*alpha)-7.400000E-03*dalpha*pow(l_tether,2.000000E+00)*sqrt((2.000000E+00*2.050000E+00*cos(alpha)*cos(beta)*dbeta*ddelta_arm*l_tether+2.000000E+00*2.050000E+00*cos(alpha)*cos(beta)*l_tether*pow(ddelta_arm,2.000000E+00)-2.000000E+00*2.050000E+00*dalpha*ddelta_arm*l_tether*sin(alpha)*sin(beta)+2.000000E+00*dbeta*ddelta_arm*pow(cos(alpha),2.000000E+00)*pow(l_tether,2.000000E+00)+4.202500E+00*pow(ddelta_arm,2.000000E+00)+pow(cos(alpha),2.000000E+00)*pow(dbeta,2.000000E+00)*pow(l_tether,2.000000E+00)+pow(cos(alpha),2.000000E+00)*pow(ddelta_arm,2.000000E+00)*pow(l_tether,2.000000E+00)+pow(dalpha,2.000000E+00)*pow(l_tether,2.000000E+00))))/(1.000600E+00+5.700000E-01*pow(l_tether,2.000000E+00));
    acadodata_f1 << dot(dbeta) == (1.000600E+00*dalpha*dbeta*sin(2.000000E+00*alpha)-2.050000E+00*5.700000E-01*cos(alpha)*cos(beta)*dddelta_arm*l_tether-2.050000E+00*5.700000E-01*cos(alpha)*l_tether*pow(ddelta_arm,2.000000E+00)*sin(beta)-2.050000E+00*7.400000E-03*cos(alpha)*cos(beta)*ddelta_arm*l_tether*sqrt((2.000000E+00*2.050000E+00*cos(alpha)*cos(beta)*dbeta*ddelta_arm*l_tether+2.000000E+00*2.050000E+00*cos(alpha)*cos(beta)*l_tether*pow(ddelta_arm,2.000000E+00)-2.000000E+00*2.050000E+00*dalpha*ddelta_arm*l_tether*sin(alpha)*sin(beta)+2.000000E+00*dbeta*ddelta_arm*pow(cos(alpha),2.000000E+00)*pow(l_tether,2.000000E+00)+4.202500E+00*pow(ddelta_arm,2.000000E+00)+pow(cos(alpha),2.000000E+00)*pow(dbeta,2.000000E+00)*pow(l_tether,2.000000E+00)+pow(cos(alpha),2.000000E+00)*pow(ddelta_arm,2.000000E+00)*pow(l_tether,2.000000E+00)+pow(dalpha,2.000000E+00)*pow(l_tether,2.000000E+00)))-2.177400E+00*dbeta+5.700000E-01*dalpha*dbeta*pow(l_tether,2.000000E+00)*sin(2.000000E+00*alpha)+5.700000E-01*dalpha*ddelta_arm*pow(l_tether,2.000000E+00)*sin(2.000000E+00*alpha)-5.700000E-01*dddelta_arm*pow(cos(alpha),2.000000E+00)*pow(l_tether,2.000000E+00)-7.400000E-03*dbeta*pow(cos(alpha),2.000000E+00)*pow(l_tether,2.000000E+00)*sqrt((2.000000E+00*2.050000E+00*cos(alpha)*cos(beta)*dbeta*ddelta_arm*l_tether+2.000000E+00*2.050000E+00*cos(alpha)*cos(beta)*l_tether*pow(ddelta_arm,2.000000E+00)-2.000000E+00*2.050000E+00*dalpha*ddelta_arm*l_tether*sin(alpha)*sin(beta)+2.000000E+00*dbeta*ddelta_arm*pow(cos(alpha),2.000000E+00)*pow(l_tether,2.000000E+00)+4.202500E+00*pow(ddelta_arm,2.000000E+00)+pow(cos(alpha),2.000000E+00)*pow(dbeta,2.000000E+00)*pow(l_tether,2.000000E+00)+pow(cos(alpha),2.000000E+00)*pow(ddelta_arm,2.000000E+00)*pow(l_tether,2.000000E+00)+pow(dalpha,2.000000E+00)*pow(l_tether,2.000000E+00)))-7.400000E-03*ddelta_arm*pow(cos(alpha),2.000000E+00)*pow(l_tether,2.000000E+00)*sqrt((2.000000E+00*2.050000E+00*cos(alpha)*cos(beta)*dbeta*ddelta_arm*l_tether+2.000000E+00*2.050000E+00*cos(alpha)*cos(beta)*l_tether*pow(ddelta_arm,2.000000E+00)-2.000000E+00*2.050000E+00*dalpha*ddelta_arm*l_tether*sin(alpha)*sin(beta)+2.000000E+00*dbeta*ddelta_arm*pow(cos(alpha),2.000000E+00)*pow(l_tether,2.000000E+00)+4.202500E+00*pow(ddelta_arm,2.000000E+00)+pow(cos(alpha),2.000000E+00)*pow(dbeta,2.000000E+00)*pow(l_tether,2.000000E+00)+pow(cos(alpha),2.000000E+00)*pow(ddelta_arm,2.000000E+00)*pow(l_tether,2.000000E+00)+pow(dalpha,2.000000E+00)*pow(l_tether,2.000000E+00))))/(1.000600E+00*pow(cos(alpha),2.000000E+00)+3.203000E-02+5.700000E-01*pow(cos(alpha),2.000000E+00)*pow(l_tether,2.000000E+00));
    acadodata_f1 << dot(ddelta_motor_sp) == dddelta_motor_sp;

    ExportModule1.setModel( acadodata_f1 );

    uint export_flag = 0;
    ExportModule1.setTimingSteps( 0 );
    export_flag = ExportModule1.exportCode( "export_SIM" );
    if(export_flag != 0) mexErrMsgTxt("ACADO export failed because of the above error(s)!");


    clearAllStaticCounters( ); 
 
} 

