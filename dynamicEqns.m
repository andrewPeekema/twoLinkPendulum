function sol = dynamicEqns(k,z)
% Generate the equations of motion
% Input
%   k: kinematic equations in SE3
%   z: velocity kinematics
% Output
%   sol: equations of motion

% Find the Lagrangian
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

% Kinetic energy of the system
T = 1/2*z.g1'*M1*z.g1 + 1/2*z.g2'*M2*z.g2;

% Potential energy of the system
syms m g
V = g*(m1*k.g1f0.y + m2*k.g2f0.y);

% The Lagrangian
L = T - V;

% The Euler-Lagrange equation
eqs = eulerLagrange(L,{'q1' 'q2'});

% Solve for acceleration
syms ddq1 ddq2 real
sol = solve(eqs(1),eqs(2),ddq1,ddq2);
sol.ddq1 = simplify(sol.ddq1);
sol.ddq2 = simplify(sol.ddq2);

end % dynamicEqns
