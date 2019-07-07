# Non Linear Model Predictive Control  and State Estimation of Tethered Flying Objects

![alt text](carousel_now.png)

# Motivation
Could it be possible to move a flying object over a predefined trajectory between two
points in the three-dimensional space? Former modelling attempts are taken into con-
sideration and the Lagrange formalism is applied in order to find a more appropriate
model for faster simulations and control. Its parameters are then estimated with several
optimisation routines. As usual, the model is linearised and discretised and a Discrete
Linear Quadratic Regulator is used to control the elevation angle of a ball as a response
to different kinds of input excitations at the carousel. A full state controller can be
achieved only after designing and implementing a Kalman filter. Both components are
then improved with the so called Internal Model Principle so that the elevation response
of the ball keeps as unchanged as possible. A Nonlinear Model Predictive Control is then
developed along with a Moving Horizon Estimator (state observer) and two nontrivial
optimal control problems are tested.

![alt text](concept_parabola.png)

# Code
The code is separated in modular thematic checkpoints, for some of them (basically for modelling purposes) 
just Matlab implementations are available while for the ones needed for state estimation or 
predictive control real time C++ (OROCOS) versions are also attached. 

 ## Modelling section
 
 Points 1-6 in the code correspond to the modelling chain followed:
 - 1, 2: Friction parameters identification: An easy friction model from the lower motor to the shaft was found and tuned. A second order model was tuned and used as a basis for the further model chain.
 - 3: Lagrange symbolic formulation with Matlab Symbolic Tool to find nonlinear dynamic model of the system.
 - 4: Parameter estimation: Procedures followed to tune the model parameters with real measurements using non linear least squares approach.
 - 5: Linearisation and discretisation using finite differences 
 - 6: Analyzing the frequency response of the linearized model via Bode diagrams.
 
 ## Optimal control and estimation
 
 - 7: Step response: The step response of the real carousel system is presented. Steady states are also modelled via Least Square approach.
 - 8: Pole placement: The system characteristics is brought to a stable working setpoint via pole placement approach.
 - 9: DLQR Simulation: A Discrete Linear Quadratic Regulator is developed in pursuit of a cheap control strategy given the linearized version of the model.
 - 10: DLQE: Discrete Linear Quadratic Estimator (Kalman Filter) applied to the system. Several measures of robustness and techniques for mismodelling and disturbance rejection are programmed and validated with the real system.
 - 11: Real closed loop: Both previous points are brought together to close the loop. Robustness analyses are performed based on external disturbances (caused here as a longer tether length).
 - 12: LQR + Integral control: Added the integral part to the LQR to compare its results with previously used methods.
 - 13: Internal Principle Method (IMP) applied to the model in order to counteract external kinds of disturbances, here tested with a constant disturbance bias.
 - 14: Nonlinear Model Predictive Control (NMPC) and Nonlinear Moving Horizon Estimation (NMHE) are planned and applied to see their advantages against previously implemented methods.
 
 You can also find some of their real time optimized cpunterparts in the OROCOS folder.
 
 ## Installation
 Given that the code is written in Matlab, this is the only requirement. For the C++ part it would be necessary to count with the OROCOS libraries and to work with lua. These source code however should be available for anyone wanting to quickly implement one of this algorithms in any generic case.
