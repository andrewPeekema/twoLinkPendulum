classdef SE3Diff < handle
    properties
        diffVar
    end

    methods

    % Constructor
    function obj = SE3Diff(diffVar)
        obj.diffVar = diffVar;
    end

    function g = Diff(obj,g)
        % Diff the SE3 element g
        g = SE3(diff(g.g,obj.diffVar));
    end % function g

    function k = structDiff(obj,k)
        % Diff each SE3 element in k
        k = structfun(@obj.Diff,k,'UniformOutput',false);
    end % function k

    end % methods
end % classdef
