classdef SE3Subs < handle
    properties
        from
        to
    end

    methods

    % Constructor
    function obj = SE3Subs(from,to)
        obj.from = from;
        obj.to   = to;
    end

    function gOut = Subs(obj,g)
        % Input
        %   g: SE3 object
        % Output
        %   g: substituted SE3 object

        gOut = SE3(subs(g.g,obj.from,obj.to));
    end % function gOut

    function kOut = structSubs(obj,k)
        % Input
        %   k: struct of SE3
        % Output
        %   k: substituted struct

        kOut = structfun(@obj.Subs,k,'UniformOutput',false);
    end % function kOut

    end % methods
end % classdef
