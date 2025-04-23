function [ind, mu, indp] = mdegs_p(x, c, roll, n, p, tab)

indp = zeros(bitshift(1, p), p);    
mu = indp;
% compute 1-dimensional mdegs and indices of activated 1-dimensional MFs
for ip = p:-1:1
    i = find(c{ip} <= x(ip), 1, 'last');
    if i < n(ip)          
        m  = (c{ip}(i+1) - x(ip)) / (c{ip}(i+1) - c{ip}(i));
        mv = [m;  1-m];         
        iv = [i;  i+1];        
    elseif isempty(i)
          iv = [1;  2];
          mv = [0;  1];
    else                   
        mv = [  0;  1];         
        iv = [i-1;  i];        
    end
    
    indp(:, ip) = iv(tab(:,  ip));
    mu(:, ip)  = mv(tab(:,  ip));
end

mu = prod(mu, 2);          
ind = ndi2lin(indp, n);    
