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
 
    DifferentialState l_tetherMHE;
    DifferentialState delta_armMHE;
    DifferentialState ddelta_armMHE;
    DifferentialState alphaMHE;
    DifferentialState betaMHE;
    DifferentialState dddelta_armMHE;
    DifferentialState dalphaMHE;
    DifferentialState dbetaMHE;
    Control ddelta_motor_spMHE;
    Control dl_tetherMHE;
    BMatrix acadodata_M1;
    acadodata_M1.read( "mhe_data_acadodata_M1.txt" );
    BMatrix acadodata_M2;
    acadodata_M2.read( "mhe_data_acadodata_M2.txt" );
    Function acadodata_f2;
    acadodata_f2 << l_tetherMHE;
    acadodata_f2 << delta_armMHE;
    acadodata_f2 << alphaMHE;
    acadodata_f2 << betaMHE;
    acadodata_f2 << ddelta_motor_spMHE;
    acadodata_f2 << dl_tetherMHE;
    Function acadodata_f3;
    acadodata_f3 << l_tetherMHE;
    acadodata_f3 << delta_armMHE;
    acadodata_f3 << alphaMHE;
    acadodata_f3 << betaMHE;
    OCP ocp1(0, 0.8, 40);
    ocp1.minimizeLSQ(acadodata_M1, acadodata_f2);
    ocp1.minimizeLSQEndTerm(acadodata_M2, acadodata_f3);
    DifferentialEquation acadodata_f4;
    acadodata_f4 << dot(l_tetherMHE) == (-dl_tetherMHE);
    acadodata_f4 << dot(delta_armMHE) == (-ddelta_armMHE);
    acadodata_f4 << dot(ddelta_armMHE) == dddelta_armMHE;
    acadodata_f4 << dot(alphaMHE) == dalphaMHE;
    acadodata_f4 << dot(betaMHE) == dbetaMHE;
    acadodata_f4 << dot(dddelta_armMHE) == (-1.607656E+00*dddelta_armMHE-3.894883E+01*ddelta_armMHE+3.894883E+01*ddelta_motor_spMHE);
    acadodata_f4 << dot(dalphaMHE) == (-1.000600E+00/2.000000E+00*pow(dbetaMHE,2.000000E+00)*sin(2.000000E+00*alphaMHE)-1/2.000000E+00*5.700000E-01*pow(dbetaMHE,2.000000E+00)*pow(l_tetherMHE,2.000000E+00)*sin(2.000000E+00*alphaMHE)-1/2.000000E+00*5.700000E-01*pow(ddelta_armMHE,2.000000E+00)*pow(l_tetherMHE,2.000000E+00)*sin(2.000000E+00*alphaMHE)-2.050000E+00*5.700000E-01*cos(betaMHE)*l_tetherMHE*pow(ddelta_armMHE,2.000000E+00)*sin(alphaMHE)+2.050000E+00*5.700000E-01*dddelta_armMHE*l_tetherMHE*sin(alphaMHE)*sin(betaMHE)+2.050000E+00*7.400000E-03*ddelta_armMHE*l_tetherMHE*sin(alphaMHE)*sin(betaMHE)*sqrt((2.000000E+00*2.050000E+00*cos(alphaMHE)*cos(betaMHE)*dbetaMHE*ddelta_armMHE*l_tetherMHE+2.000000E+00*2.050000E+00*cos(alphaMHE)*cos(betaMHE)*l_tetherMHE*pow(ddelta_armMHE,2.000000E+00)-2.000000E+00*2.050000E+00*dalphaMHE*ddelta_armMHE*l_tetherMHE*sin(alphaMHE)*sin(betaMHE)+2.000000E+00*dbetaMHE*ddelta_armMHE*pow(cos(alphaMHE),2.000000E+00)*pow(l_tetherMHE,2.000000E+00)+4.202500E+00*pow(ddelta_armMHE,2.000000E+00)+pow(cos(alphaMHE),2.000000E+00)*pow(dbetaMHE,2.000000E+00)*pow(l_tetherMHE,2.000000E+00)+pow(cos(alphaMHE),2.000000E+00)*pow(ddelta_armMHE,2.000000E+00)*pow(l_tetherMHE,2.000000E+00)+pow(dalphaMHE,2.000000E+00)*pow(l_tetherMHE,2.000000E+00)))-4.876000E-01*dalphaMHE-5.700000E-01*9.810000E+00*cos(alphaMHE)*l_tetherMHE-5.700000E-01*dbetaMHE*ddelta_armMHE*pow(l_tetherMHE,2.000000E+00)*sin(2.000000E+00*alphaMHE)-7.400000E-03*dalphaMHE*pow(l_tetherMHE,2.000000E+00)*sqrt((2.000000E+00*2.050000E+00*cos(alphaMHE)*cos(betaMHE)*dbetaMHE*ddelta_armMHE*l_tetherMHE+2.000000E+00*2.050000E+00*cos(alphaMHE)*cos(betaMHE)*l_tetherMHE*pow(ddelta_armMHE,2.000000E+00)-2.000000E+00*2.050000E+00*dalphaMHE*ddelta_armMHE*l_tetherMHE*sin(alphaMHE)*sin(betaMHE)+2.000000E+00*dbetaMHE*ddelta_armMHE*pow(cos(alphaMHE),2.000000E+00)*pow(l_tetherMHE,2.000000E+00)+4.202500E+00*pow(ddelta_armMHE,2.000000E+00)+pow(cos(alphaMHE),2.000000E+00)*pow(dbetaMHE,2.000000E+00)*pow(l_tetherMHE,2.000000E+00)+pow(cos(alphaMHE),2.000000E+00)*pow(ddelta_armMHE,2.000000E+00)*pow(l_tetherMHE,2.000000E+00)+pow(dalphaMHE,2.000000E+00)*pow(l_tetherMHE,2.000000E+00))))/(1.000600E+00+5.700000E-01*pow(l_tetherMHE,2.000000E+00));
    acadodata_f4 << dot(dbetaMHE) == (1.000600E+00*dalphaMHE*dbetaMHE*sin(2.000000E+00*alphaMHE)-2.050000E+00*5.700000E-01*cos(alphaMHE)*cos(betaMHE)*dddelta_armMHE*l_tetherMHE-2.050000E+00*5.700000E-01*cos(alphaMHE)*l_tetherMHE*pow(ddelta_armMHE,2.000000E+00)*sin(betaMHE)-2.050000E+00*7.400000E-03*cos(alphaMHE)*cos(betaMHE)*ddelta_armMHE*l_tetherMHE*sqrt((2.000000E+00*2.050000E+00*cos(alphaMHE)*cos(betaMHE)*dbetaMHE*ddelta_armMHE*l_tetherMHE+2.000000E+00*2.050000E+00*cos(alphaMHE)*cos(betaMHE)*l_tetherMHE*pow(ddelta_armMHE,2.000000E+00)-2.000000E+00*2.050000E+00*dalphaMHE*ddelta_armMHE*l_tetherMHE*sin(alphaMHE)*sin(betaMHE)+2.000000E+00*dbetaMHE*ddelta_armMHE*pow(cos(alphaMHE),2.000000E+00)*pow(l_tetherMHE,2.000000E+00)+4.202500E+00*pow(ddelta_armMHE,2.000000E+00)+pow(cos(alphaMHE),2.000000E+00)*pow(dbetaMHE,2.000000E+00)*pow(l_tetherMHE,2.000000E+00)+pow(cos(alphaMHE),2.000000E+00)*pow(ddelta_armMHE,2.000000E+00)*pow(l_tetherMHE,2.000000E+00)+pow(dalphaMHE,2.000000E+00)*pow(l_tetherMHE,2.000000E+00)))-2.177400E+00*dbetaMHE+5.700000E-01*dalphaMHE*dbetaMHE*pow(l_tetherMHE,2.000000E+00)*sin(2.000000E+00*alphaMHE)+5.700000E-01*dalphaMHE*ddelta_armMHE*pow(l_tetherMHE,2.000000E+00)*sin(2.000000E+00*alphaMHE)-5.700000E-01*dddelta_armMHE*pow(cos(alphaMHE),2.000000E+00)*pow(l_tetherMHE,2.000000E+00)-7.400000E-03*dbetaMHE*pow(cos(alphaMHE),2.000000E+00)*pow(l_tetherMHE,2.000000E+00)*sqrt((2.000000E+00*2.050000E+00*cos(alphaMHE)*cos(betaMHE)*dbetaMHE*ddelta_armMHE*l_tetherMHE+2.000000E+00*2.050000E+00*cos(alphaMHE)*cos(betaMHE)*l_tetherMHE*pow(ddelta_armMHE,2.000000E+00)-2.000000E+00*2.050000E+00*dalphaMHE*ddelta_armMHE*l_tetherMHE*sin(alphaMHE)*sin(betaMHE)+2.000000E+00*dbetaMHE*ddelta_armMHE*pow(cos(alphaMHE),2.000000E+00)*pow(l_tetherMHE,2.000000E+00)+4.202500E+00*pow(ddelta_armMHE,2.000000E+00)+pow(cos(alphaMHE),2.000000E+00)*pow(dbetaMHE,2.000000E+00)*pow(l_tetherMHE,2.000000E+00)+pow(cos(alphaMHE),2.000000E+00)*pow(ddelta_armMHE,2.000000E+00)*pow(l_tetherMHE,2.000000E+00)+pow(dalphaMHE,2.000000E+00)*pow(l_tetherMHE,2.000000E+00)))-7.400000E-03*ddelta_armMHE*pow(cos(alphaMHE),2.000000E+00)*pow(l_tetherMHE,2.000000E+00)*sqrt((2.000000E+00*2.050000E+00*cos(alphaMHE)*cos(betaMHE)*dbetaMHE*ddelta_armMHE*l_tetherMHE+2.000000E+00*2.050000E+00*cos(alphaMHE)*cos(betaMHE)*l_tetherMHE*pow(ddelta_armMHE,2.000000E+00)-2.000000E+00*2.050000E+00*dalphaMHE*ddelta_armMHE*l_tetherMHE*sin(alphaMHE)*sin(betaMHE)+2.000000E+00*dbetaMHE*ddelta_armMHE*pow(cos(alphaMHE),2.000000E+00)*pow(l_tetherMHE,2.000000E+00)+4.202500E+00*pow(ddelta_armMHE,2.000000E+00)+pow(cos(alphaMHE),2.000000E+00)*pow(dbetaMHE,2.000000E+00)*pow(l_tetherMHE,2.000000E+00)+pow(cos(alphaMHE),2.000000E+00)*pow(ddelta_armMHE,2.000000E+00)*pow(l_tetherMHE,2.000000E+00)+pow(dalphaMHE,2.000000E+00)*pow(l_tetherMHE,2.000000E+00))))/(1.000600E+00*pow(cos(alphaMHE),2.000000E+00)+3.203000E-02+5.700000E-01*pow(cos(alphaMHE),2.000000E+00)*pow(l_tetherMHE,2.000000E+00));

    ocp1.setModel( acadodata_f4 );


    OCPexport ExportModule2( ocp1 );
    ExportModule2.set( GENERATE_MATLAB_INTERFACE, 1 );
    uint options_flag;
    options_flag = ExportModule2.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: HESSIAN_APPROXIMATION");
    options_flag = ExportModule2.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: DISCRETIZATION_TYPE");
    options_flag = ExportModule2.set( SPARSE_QP_SOLUTION, CONDENSING_N2 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: SPARSE_QP_SOLUTION");
    options_flag = ExportModule2.set( FIX_INITIAL_STATE, NO );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: FIX_INITIAL_STATE");
    options_flag = ExportModule2.set( INTEGRATOR_TYPE, INT_RK4 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: INTEGRATOR_TYPE");
    options_flag = ExportModule2.set( NUM_INTEGRATOR_STEPS, 80 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: NUM_INTEGRATOR_STEPS");
    options_flag = ExportModule2.set( QP_SOLVER, QP_QPOASES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: QP_SOLVER");
    uint export_flag;
    export_flag = ExportModule2.exportCode( "export_MHE" );
    if(export_flag != 0) mexErrMsgTxt("ACADO export failed because of the above error(s)!");


    clearAllStaticCounters( ); 
 
} 

