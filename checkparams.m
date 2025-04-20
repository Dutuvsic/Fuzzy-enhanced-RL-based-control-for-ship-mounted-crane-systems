function [s, flag, overridden, defaulted] = checkparams(s, defaults, required)
%Verifies parameter structure and sets defaults for optional parameters
REQARGIN = 2;
if nargin < REQARGIN + 1
    required = {};
end

overridden = {}; defaulted = {};

for i = 1 : length(required)
    if ~isfield(s, required{i})
        if nargout > 1
            flag = -1;
            return;
        else 
            error(['Field ' required{i} ' is required']);
        end
    end
end

flag = 1;       % required OK

if isempty(defaults), return; end

fld = fieldnames(defaults);
for i = 1 : length(fld)
    if ~isfield(s, fld{i})            % assign defaults
        s.(fld{i}) = defaults.(fld{i});
        defaulted{end+1} = fld{i};
    else
        overridden{end+1} = fld{i};
        % stop at 1st level structures
%         % process recursively for defaults
%         if isstruct(s.(fld{i})),
%           s.(fld{i}) = checkparams(s.(fld{i}), defaults.(fld{i}));
%         end;
    end;
end;