function X = ndgridx(x)
% Equivalent of NDGRID for which input and output are cell arrays
N = lengthgrids(x);
n = length(N);

X = cell(n, 1);
if n == 1
    X{1} = x{1}; return; 
end

for i = 1:n
    N1 = N; N2 = ones(1, n);
    N1(i) = 1; N2(i) = N(i);
    xi = zeros(N2); xi(:) = x{i};
    X{i} = repmat(xi, N1);
end

end


