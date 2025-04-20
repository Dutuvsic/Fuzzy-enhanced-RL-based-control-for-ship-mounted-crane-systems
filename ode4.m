function Y = ode4(odefun,tspan,y0,varargin)
%ODE4  Solve differential equations with a non-adaptive method of order 4.
h = diff(tspan);
y0 = y0(:);  

neq = length(y0);
N = length(tspan);
Y = zeros(neq,N);
F = zeros(neq,4);

Y(:,1) = y0;
for i = 2:N
  ti = tspan(i-1);
  hi = h(i-1);
  yi = Y(:,i-1);
  F(:,1) = feval(odefun,ti,yi,varargin{:});
  F(:,2) = feval(odefun,ti+0.5*hi,yi+0.5*hi*F(:,1),varargin{:});
  F(:,3) = feval(odefun,ti+0.5*hi,yi+0.5*hi*F(:,2),varargin{:});  
  F(:,4) = feval(odefun,tspan(i),yi+hi*F(:,3),varargin{:});
  Y(:,i) = yi + (hi/6)*(F(:,1) + 2*F(:,2) + 2*F(:,3) + F(:,4));
end
Y = Y.';

