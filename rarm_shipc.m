function out = rarm_shipc(what, varargin)
% Ship-mounted crane parameter setup 

gamma = 0.98;

switch what
        
    case 'model'        
        if nargin >= 2 && ~isempty(varargin{1})
            cfg = varargin{1};
        else
            cfg = struct;
        end

        MODEL.type = 'default';
        MODEL.wrap = 1;                          
        MODEL.len =      [0.7     0.7];         
        MODEL.m =        [5.6    0.23];            
        MODEL.J =       0.184;
        MODEL.d =        0.05;       
        MODEL.maxomega = [pi; 5*pi/180; pi/36];         
        MODEL.maxtau =   [10; 3];         
        MODEL.orientation = 'horiz';            
        MODEL.rewtype = 'lre_ship';
        MODEL.Q = diag([1  1  1 ]);
        MODEL.R = zeros(2, 2);
        MODEL.Ts = 0.05;
        MODEL.odemethod = 'fixed-ode';
        MODEL.odesolver = 'ode4';
        MODEL.odesteps = 5;
        
        MODEL.Upsilon_a = diag([3.0, 2.12]); 
        MODEL.Upsilon_u = 1.36;               
        MODEL.Upsilon_d = diag([0.03, 0.02]); 
%         MODEL.odemethod = 'ode';                   
%         MODEL.odesolver = 'ode45';              
%         MODEL.odesteps = 1;                     
        
        % process any user-configured fields
        model = parseconfig(cfg, MODEL);
        
        % create standard model variables 
        model.det = 1;      % always deterministic
        model.p = 6;        % always 4 states
        model.q = 2;        % and 2 actions
        % set bounds in standard format
        model.maxx = [pi/3;0.4;10*pi/180;]; 
        model.maxu = model.maxtau(:); 
        model.fun = @rarm_mdpstandalone;
        model.plotfun = @rarm_plot;
        
        % compute max reward
        switch model.rewtype
            case 'lre_ship'
                model.maxr = model.maxx' * model.Q * model.maxx + model.maxu' * model.R * model.maxu;
        end
        
        % create discretization helper vars
        switch(model.odemethod)
            case 'ode'
                model.odet = [0 model.Ts/2 model.Ts];
                model.odeopt = odeset;
            case 'fixed-ode'
                model.odet = 0 : model.Ts / model.odesteps : model.Ts;
        end
 
        % compute physical helper vars
        model.g = 9.81;
        model.neglectG = strcmp(model.orientation, 'horiz');
        out = model;

        
    case 'fuzzy'
        TH1 = [5 10 ] * pi/180;
        OM = [-360 -180 -30 0 30 180 360] * pi/180;
        TAU1 = [-1.5 0 1.5];
        TAU2 = [-1 0 1];
        cfg.xgrids = {TH, OM, TH, OM};
        cfg.ugrids = {TAU1, TAU2};
        cfg.gamma = gamma;
        out = cfg;
       
    case 'X0'     % representative set of states
        if nargin < 2
            xt = 'zero';        
        else
            xt = varargin{1}; 
        end
        switch xt
            case 'zero'
                out = {0, 0, 0, 0};
            case 'pos'
                out = {-pi:pi/3:pi, 0, -pi:pi/3:pi, 0};
            otherwise
                error(['Unknown X0 type [' xt ']']);
        end
        
end

