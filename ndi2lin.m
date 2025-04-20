function lin = ndi2lin(ndi,dims)
c = [1 cumprod(dims(1:length(dims)-1))];    % vector of "bases"
lin = (c*(ndi-1)')' + 1;                    % compute lin. index
