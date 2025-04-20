function X = flat(x)
% Gives an explicit enumeration of the set of points in the cross product of n grids
if length(x) == 1, X = x{1}; 
else
    n = length(x);
    xnd = ndgridx(x);
    X = zeros(n, numel(xnd{1}));
    for i = 1:n
        X(i, :) = xnd{i}(:);
    end
end

