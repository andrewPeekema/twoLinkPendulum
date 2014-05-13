% Derive, simulate, plot, and animate a two link pendulum
% Author: Andrew Peekema

% Cleanup
clc       % Clear the command prompt
clear all % Remove all workspace variables
close all % Close all figure windows

% Solve the kinematics
k = kinematicEqns;
% Solve the velocity kinematics
z = velocityEqns(k);
% Solve the dynamics
eqs = dynamicEqns(k,z);

% TODO: Substitute constants into the dynamics

%{
% Constants
% Link 1
c.m1 = 1;  % mass (kg)
c.c1 = 1;  % damping (N*s/rad)
c.l1 = 1;  % length (m)
c.I1 = 1;  % inertia (kg*m^2)
% Link 2
c.m2 = 1;  % mass (kg)
c.c2 = 1;  % damping (N*s/rad)
c.l2 = 2;  % length (m)
c.I2 = 1;  % inertia (kg*m^2)
%}

% TODO: Integrate the time response of the system (use springMassDamperSim.m as a guide)
% Initial state conditions
X0 = [0 ...    % Angle (rad)
      0 ...    % Angular velocity (rad/s)
      pi/2 ... % Angle (rad)
      0];      % Angular velocity (rad/s)

% TODO: Plot the response
% TODO: Animate the response
