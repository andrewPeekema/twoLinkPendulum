function animation(c,k,sol,exportVideo)
% 3D Visualization Template
% Input
%   c: Simulation constants
%   sol: Simulation solution
%   exportVideo: Should the video be exported? (True/False)
% Output
%   An animation
% By Andrew Peekema

% Unpack constants
l1 = c.l1;
m1 = c.m1;
l2 = c.l2;
m2 = c.m2;

% Create visualization objects
link1 = CubeClass([l1 sqrt(m1/l1)*0.5 sqrt(m1/l1)*0.5]);
link2 = CubeClass([l2 sqrt(m2/l2)*0.5 sqrt(m2/l2)*0.5]);

% Create link transformations
g1f0 = matlabFunction(subs(k.g1f0.g));
g2f0 = matlabFunction(subs(k.g2f0.g));

% Create a figure handle
h.figure = figure;

% Put the shapes into a plot
link1.plot
link2.plot

% Figure properties
view(2)
title('Simulation')
xlabel('x Position (m)')
ylabel('y Position (m)')
zlabel('z Position (m)')
% Set axis limits
aLim = (l1+l2)*1.1;
axis([-aLim aLim ... % x
      -aLim aLim ... % y
      -1.0 1.0]);  % z

% Speed up if watching in realtime
if exportVideo
    frameStep = 3;
else
    frameStep = 4;
end

% Iterate over state data
for it = 1:frameStep:length(sol.t)
    q1 = sol.X(it,1);
    q2 = sol.X(it,3);

    % Link 1 position
    link1.resetFrame
    link1.globalMove(SE3(g1f0(q1)))

    % Link 2 position
    link2.resetFrame
    link2.globalMove(SE3(g2f0(q1,q2)))

    % Update data
    link1.updatePlotData
    link2.updatePlotData

    % Draw figure
    drawnow

    % Save the frames
    if exportVideo
        frame = getframe(h.figure);
        imwrite(frame.cdata, sprintf('./video/%04d.png',it));
    end
end % for it = ...

end % animation
