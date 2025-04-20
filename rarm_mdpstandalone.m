function [xplus, rplus, terminal] = rarm_mdpstandalone(m, x, u)
% Discrete-time dynamics of the ship-mounted crane
u = max(-m.maxu, min(m.maxu, u));

if  m.odemethod(1) == 'f'    % fixed-step ODE
    odey = feval(m.odesolver, @rarm_transstandalone, m.odet, x, m, u, 0);
    xplus = odey(end, :)';      
end

% Normalized state
if m.wrap  % wrap angles in [-pi, pi), bound velocities
    xplus([1 5]) = mod(xplus([1 5]) + pi, 2*pi) - pi;
    xplus([2 6]) = max(-m.maxomega([1 3]), min(m.maxomega([1 3]), xplus([2 6])));
    if xplus(4,:)>2
        xplus(4,:)=2;
    end
    if xplus(4,:)>0 &&  xplus(3,:)>0.7
        xplus(4,:)=0;
    end
else      
    xplus = max(-m.maxx, min(m.maxx, xplus));
end

% Calculate the new driving error e_a
e_a = zeros(2,1);
e_a(1) = xplus(1) - 60*pi/180;  
e_a(2) = xplus(3) - 0.4;         

% Get speed item
eta_dot = xplus(6);             
q1_dot = xplus(2);               
q2_dot = xplus(4);               
sum_q_dot_sq = q1_dot^2 + q2_dot^2;

% Compute reward
if m.rewtype(1) == 'l'
    term1 = -e_a' * m.Upsilon_a * e_a;                 % -e_a^T Υ_a e_a
    term2 = -m.Upsilon_u * (eta_dot^2) * sum_q_dot_sq; % -Υ_u η_dot^2 Σ(q_i_dot^2)
    term3 = -u' * m.Upsilon_d * u;                     % -u^T Υ_d u
    rplus = term1 + term2 + term3;
end

% e = zeros(3,1);
% e(1,1) = 2*(xplus(1,1)-100*pi/180);
% e(2,1) = 3*(xplus(3,1)-0.4);
% e(3,1) = 5*(xplus(5,1)-0);
% % e(1,1) = 2*(xplus(1,1)-53.13*pi/180);
% % e(2,1) = 3*(xplus(3,1)-0.4);
% % e(3,1) = 5*(xplus(5,1)-0);
% % e([1 3 5]) = xplus([1 3 5])-[53.13*pi/180;0.4;0];
% % e([2 4 6]) = xplus([2 4 6])-[0;0;0];

terminal = 0;       % task is continuing



function xdot = rarm_transstandalone(t, x, m, u, addWave)
% Implements the transition function of the ship-mounted crane considering new state variables.
%
% Parameters:
%   t           - the current time
%   x           - the current state (now representing η1, η2, η3 and their derivatives)
%   m           - the model structure for the ship-mounted crane
%   u           - the vector of joint torques
%   addWave     - a flag indicating whether to add wave disturbance (true or false)
%
% Returns:
%   xdot        - the derivative of the state
%

% Initialize alpha and its derivative
alpha = 0;
alpha_dot = 0;

% Check if wave disturbance should be added
if addWave
    alpha = 4*sin(0.5*t);  % Wave disturbance function
    alpha_dot = 2*cos(0.5*t);  % Derivative of wave disturbance function
end

% Extract new state variables (η1, η2, η3) from x
eta1 = x(1);
eta2 = x(3);
eta3 = x(5);

% Compute original variables (phi, L, theta) based on new states and alpha
q1 = eta1 + alpha;  % phi = η1 + α
q2 = eta2;          % L = η2
q3 = eta3 + alpha;  % theta = η3 + α

% Calculate trigonometric functions
s2 = sin(q3); 
c1 = cos(q1);

% Define indices for the derivatives
dotix = [2; 4; 6];

% Define the mass matrix M (adjusted for new variables)
M = [m.J + m.m(2)*m.len(1)^2,  -m.m(2)*m.len(1)*cos(q1 - q3), -m.m(2)*m.len(1)*q2*sin(q1 - q3); 
     -m.m(2)*m.len(1)*cos(q1 - q3), m.m(2), 0;
     -m.m(2)*m.len(1)*q2*sin(q1 - q3), 0, m.m(2)*(q2)^2];

% Define the Coriolis matrix V (adjusted for new variables and their derivatives)
V = [0,   -m.m(2)*m.len(1)*(x(6) + alpha_dot)*sin(q1 - q3), -m.m(2)*m.len(1)*(sin(q1 - q3)*(x(4)) - q2*cos(q1 - q3)*(x(6) + alpha_dot));
     m.m(2)*m.len(1)*(x(2) + alpha_dot)*sin(q1 - q3), 0, -m.m(2)*q2*(x(6) + alpha_dot);
     -m.m(2)*m.len(1)*q2*cos(q1 - q3)*(x(2) + alpha_dot), m.m(2)*q2*(x(6) + alpha_dot), m.m(2)*q2*(x(4))] * [x(2); x(4); x(6)];

% Define the gravity vector G (adjusted for new variables)
G = [(m.m(2)*m.len(1)+m.m(1)*m.d)*m.g*c1;
    -m.m(2)*m.g*cos(q3);
    m.m(2)*q2*m.g*sin(q3)];

% Identity matrix for control input
I = [1, 0; 0, 1; 0, 0];

% Initialize the state derivative vector
xdot = zeros(6, 1);

% Assign the derivatives of the new state variables (η1_dot, η2_dot, η3_dot)
xdot([1; 3; 5]) = x(dotix);

% Compute the derivatives for the new state variables considering alpha's effect
M_inv = inv(M);
xdot(dotix) = M_inv * (I*u - V - G);  % Compensate for alpha's derivative
