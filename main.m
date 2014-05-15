% Derive, simulate, plot, and animate a two link pendulum
% Author: Andrew Peekema

% Cleanup
clc       % Clear the command prompt
clear all % Remove all workspace variables
close all % Close all figure windows

display('Solving the equations of motion...')

% Solve the kinematics
k = kinematicEqns;

% Solve the velocity kinematics
z = velocityEqns(k);

% Solve the dynamics
eqs = dynamicEqns(k,z);

% Add feedback linearization
eqs = feedbackLinearization(eqs);

display('...equations of motion solved')



display('Simulating the dynamics...')

% Substute constants into the dynamic equations
[c ddq1 ddq2] = subConstants(eqs);

% Initial state conditions
X0 = [0 ...    % Angle (rad)
      0 ...    % Angular velocity (rad/s)
      0 ...    % Angle (rad)
      0];      % Angular velocity (rad/s)

% Time vector (s)
t = [0:0.01:20];

% Integrate the time response of the system
sol = dynamicsSim(t,X0,ddq1,ddq2);

display('...dynamics simulated')



% Plot the response
q1 = sol.X(:,1);
q2 = sol.X(:,3);
plot(q1,q2,'.');
title('State Space Response')
xlabel('q1 (rad)')
ylabel('q2 (rad)')

% Animate the response
exportVideo = false;
animation(c,k,sol,exportVideo);
