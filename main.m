% Derive, simulate, plot, and animate a two link pendulum
% Author: Andrew Peekema

% Cleanup
clc       % Clear the command prompt
clear all % Remove all workspace variables
close all % Close all figure windows

display('Solving the dynamics...')
% Solve the kinematics
k = kinematicEqns;
% Solve the velocity kinematics
z = velocityEqns(k);
% Solve the dynamics
eqs = dynamicEqns(k,z);

% Control the system using feedback linearization
syms q1 q2 dq1 dq2 q1des q2des real
f = [dq1;
     eqs.ddq1;
     dq2;
     eqs.ddq2];
y = [q1 - q1des;
     q2 - q2des];
q = {'q1' 'dq1' 'q2' 'dq2'};
Lfy = lieDerivative(y,f,q);
Lfy2 = lieDerivative(Lfy,f,q);

% How does the torque enter the system?
syms I1m I2m real
g = [0     0;
     1/I1m 0;
     0     0;
     0     1/I2m];
LgLfy = lieDerivative(Lfy,g,q);

% Gain matricies
k1 = [1 0;
      0 1];
k2 = [1;
      1];

% The control
% TODO: Troubleshoot this control
u = -inv(LgLfy)*(Lfy2+k1*Lfy+k2);

% Put the control into the dynamic equations
eqs.ddq1 = eqs.ddq1 + g(2,1)*u(1);
eqs.ddq2 = eqs.ddq2 + g(4,2)*u(2);

% Declare constants
g = 9.81; % m/s^2
% Link 1
m1  = 1;  % mass (kg)
c1  = 1;  % damping (N*s/rad)
l1  = 1;  % length (m)
I1  = 1;  % inertia (kg*m^2)
I1m = 1;  % inertia (kg*m^2)

% Link 2
m2  = 1;  % mass (kg)
c2  = 1;  % damping (N*s/rad)
l2  = 1;  % length (m)
I2  = 1;  % inertia (kg*m^2)
I2m = 1;  % inertia (kg*m^2)
% Desired angles
q1des = pi/4;
q2des = pi/4;

% Substitute constants into the dynamics
ddq1 = matlabFunction(subs(eqs.ddq1));
ddq2 = matlabFunction(subs(eqs.ddq2));

display('...dynamics solved')



display('Simulating the dynamics...')
% Initial state conditions
X0 = [pi/4 ...    % Angle (rad)
      0 ...    % Angular velocity (rad/s)
      pi/4-0.01 ...    % Angle (rad)
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
c.l1 = l1;
c.l2 = l2;
animation(c,k,sol,exportVideo);
