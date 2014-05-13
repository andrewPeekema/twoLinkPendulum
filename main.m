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

% Declare constants
g = 9.81; % m/s^2
% Link 1
m1 = 1;  % mass (kg)
c1 = 1;  % damping (N*s/rad)
l1 = 1;  % length (m)
I1 = 1;  % inertia (kg*m^2)
% Link 2
m2 = 1;  % mass (kg)
c2 = 1;  % damping (N*s/rad)
l2 = 2;  % length (m)
I2 = 1;  % inertia (kg*m^2)

% Substitute constants into the dynamics
ddq1 = matlabFunction(subs(eqs.ddq1));
ddq2 = matlabFunction(subs(eqs.ddq2));

% Initial state conditions
X0 = [pi/2 ...    % Angle (rad)
      0 ...    % Angular velocity (rad/s)
      0.01 ...    % Angle (rad)
      0];      % Angular velocity (rad/s)

% Integrate the time response of the system
sol = dynamicsSim(X0,ddq1,ddq2);

% Plot the response
plot(sol.X(:,1),sol.X(:,3),'.');
title('State Space Response')
xlabel('q1 (rad)')
ylabel('q2 (rad)')

% Animate the response
exportVideo = false;
c.l1 = l1;
c.m1 = m1;
c.l2 = l2;
c.m2 = m2;
animation(c,k,sol,exportVideo);
