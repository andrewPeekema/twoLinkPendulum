function z = velocityEqns(k)
% Generate the velocity kinematics of the two-link pendulum
% Input
%   k: kinematic equations in SE3
% Output
%   z: velocity kinematics

syms dq1 dq2 real
z.f0 = zeros(6,1);
z.f1 = k.f1f0.invAdj*(z.f0+[zeros(5,1); dq1]);
z.g1 = k.g1f1.invAdj*z.f1;
z.h1 = k.h1g1.invAdj*z.g1; % End of the first link
z.f2 = k.f2h1.invAdj*(z.h1+[zeros(5,1); dq2]);
z.g2 = k.g2f2.invAdj*z.f2;

end % velocityEqns
