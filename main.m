% Simulate, plot, and animate a two link pendulum
% Author: Andrew Peekema

% Cleanup
clc       % Clear the command prompt
clear all % Remove all workspace variables
close all % Close all figure windows

% Generate the kinematics
syms q1 l1 q2 l2 t real
k.f0   = SE3; % The base
k.f1f0 = k.f0*SE3([0 0 0 0 0 q1]); % start of the first link
k.g1f1 = SE3([l1/2 0 0]);
k.g1f0 = k.f1f0*k.g1f1; % Center of the first link
k.h1g1 = SE3([l1/2 0 0]);
k.h1f0 = k.g1f0*k.h1g1; % End of the first link
k.f2h1 = SE3([0 0 0 0 0 q2]);
k.f2f0 = k.h1f0*k.f2h1; % Start of the second link
k.f2f0.g = simplify(k.f2f0.g,'IgnoreAnalyticConstraints',true); % Clean up notation
k.g2f2 = SE3([l2/2 0 0]);
k.g2f0 = k.f2f0*k.g2f2; % Center of the second link

% Convert to time varying variables
%{
syms q1(t) q2(t) real
mySub = SE3Subs({'q1' 'q2'},{'q1(t)' 'q2(t)'});
kTemp = mySub.structSubs(k);

% Differentiate
myDiff = SE3Diff('t');
dk = myDiff.structDiff(kTemp);

% Simplify
syms dq1 dq2 real
mySub = SE3Subs({'diff(q1(t), t)' 'diff(q2(t), t)'},{'dq1' 'dq2'});
dk = mySub.structSubs(dk);
mySub = SE3Subs({'q1(t)' 'q2(t)'},{'q1' 'q2'});
dk = mySub.structSubs(dk);
%}

%% Generate the velocity kinematics
syms dq1 dq2 real
z.f0 = zeros(6,1);
z.f1 = k.f1f0.invAdj*(z.f0+[zeros(5,1); dq1]);
z.g1 = k.g1f1.invAdj*z.f1;
z.h1 = k.h1g1.invAdj*z.g1; % End of the first link
z.f2 = k.f2h1.invAdj*(z.h1+[zeros(5,1); dq2]);
z.g2 = k.g2f2.invAdj*z.f2;

%% Generate the dynamics
syms m1 I1 m2 I2 real
% Mass matrix of the first link
mm1 = m1*eye(3,3);
Im1 = [0 0 0;
      0 0 0;
      0 0 I1];
M1 = [[mm1 zeros(3,3)];
      [zeros(3,3) Im1]];
% Mass matrix of the second link
mm2 = m2*eye(3,3);
Im2 = [0 0 0;
      0 0 0;
      0 0 I2];
M2 = [[mm2 zeros(3,3)];
      [zeros(3,3) Im2]];
% Kinetic Energy of the system
T = 1/2*z.g1'*M1*z.g1 + 1/2*z.g2'*M2*z.g2;

%{
% Initial state conditions
X0 = [0 ...    % Angle (rad)
      0 ...    % Angular velocity (rad/s)
      pi/2 ... % Angle (rad)
      0];      % Angular velocity (rad/s)

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
