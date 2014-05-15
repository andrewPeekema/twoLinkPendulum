function [c, ddq1, ddq2] = subConstants(eqs)
% Substitute constants into the dynamic equations
% Input
%   eqs: Dynamic equations (ddq1, ddq2)
% Output
%   c: struct of constants
%   ddq1: Angular acceleration of q1
%   ddq2: Angular acceleration of q2

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

% Constants used for the animation
c.l1 = l1;
c.l2 = l2;

end % function
