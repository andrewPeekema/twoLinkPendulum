function sol = dynamicsSim(X0,ddq1,ddq2)
% Simulates the time response of a two link pendulum
% Given
%   X0: Initial state [q1 dq1 q2 dq2]
%   ddq1: Angular acceleration for q1
%   ddq2: Angular acceleration for q2
% Returns
%   sol.t: time vector
%   sol.X: state matrix
% Author: Andrew Peekema

% Simulation tolerances
options = odeset(...
    'RelTol', 1e-9, ...
    'AbsTol', 1e-9);

% Simulate the dynamics over a time interval
[sol.t sol.X] = ode45(@dynamics,[0:0.01:10], X0, options);

function dX = dynamics(t,X)
    % t == time
    % X == the state
    q1  = X(1); % Angular Position
    dq1 = X(2); % Angular Velocity
    q2  = X(3); % Angular Position
    dq2 = X(4); % Angular Velocity

    % Return the state derivative
    dX = zeros(4,1);
    dX(1) = dq1;                 % Angular Velocity
    dX(2) = ddq1(dq1,dq2,q1,q2); % Angular Acceleration
    dX(3) = dq2;                 % Angular Velocity
    dX(4) = ddq2(dq1,dq2,q1,q2); % Angular Acceleration
end % dynamics

end % dynamicsSim
