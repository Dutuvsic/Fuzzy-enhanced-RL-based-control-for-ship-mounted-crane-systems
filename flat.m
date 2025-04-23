function X = flat(x)
% Provide an explicit enumeration of point sets in n grid cross products
if length(x) == 1, X = x{1}; 
else
    n = length(x);
    xnd = ndgridx(x);
    X = zeros(n, numel(xnd{1}));
    for i = 1:n
        X(i, :) = xnd{i}(:);
    end
end

