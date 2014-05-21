function k = kinematicEqns
% Output
%   k: kinematics of the two-link pendulum

syms q1 l1 q2 l2 real

%% The base
k.f0   = SE3;


%% First link
k.f1f0 = k.f0*SE3([0 0 0 0 0 q1]);

% Center
k.g1f1 = SE3([l1/2 0 0]);
k.g1f0 = k.f1f0*k.g1f1;

% End
k.h1g1 = SE3([l1/2 0 0]);
k.h1f0 = k.g1f0*k.h1g1;


%% Second link
k.f2h1 = SE3([0 0 0 0 0 q2]);
k.f2f0 = k.h1f0*k.f2h1;
k.f2f0.g = simplify(k.f2f0.g,'IgnoreAnalyticConstraints',true); % Clean up notation

% Center
k.g2f2 = SE3([l2/2 0 0]);
k.g2f0 = k.f2f0*k.g2f2;

end % kinematicEqns
