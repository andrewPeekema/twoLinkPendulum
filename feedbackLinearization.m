function eqs = feedbackLinearization(eqs)
% Add feedback linearization on top of the equations of motion
% Input
%   eqs: equations of motion (ddq1, ddq2)
% Output
%   eqs: equations of motion with the control added


% Define the natural dynamics (f)
syms q1 q2 dq1 dq2 q1des q2des real
f = [dq1;
     eqs.ddq1;
     dq2;
     eqs.ddq2];

% y will be driven to zero
y = [q1 - q1des;
     q2 - q2des];

% The state variables
q = {'q1' 'dq1' 'q2' 'dq2'};

% How does the torque enter the system?
syms I1m I2m real
g = [0     0;
     1/I1m 0;
     0     0;
     0     1/I2m];

% Gain matricies
k1 = [2 0;
      0 2];
k2 = [4 0;
      0 4];

% Take the lie derivatives necessary for the control
Lfy   = lieDerivative(y,f,q);
Lfy2  = lieDerivative(Lfy,f,q);
LgLfy = lieDerivative(Lfy,g,q);

% The control
u = -LgLfy\(Lfy2+k1*Lfy+k2*y);

% Put the control into the dynamic equations
eqs.ddq1 = eqs.ddq1 + g(2,1)*u(1);
eqs.ddq2 = eqs.ddq2 + g(4,2)*u(2);

end % function eqs
