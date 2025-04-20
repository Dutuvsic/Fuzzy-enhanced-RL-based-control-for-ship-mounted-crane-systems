function varargout = shipRL(cfg)
% Iterative algorithm 
if nargin < 1
    cfg = struct();
end

% Process configuration structure
%% default config
% script config
CFG.run = 0;                        % run learning
CFG.resume = 0;                     % resume learning 
CFG.replay = 0;                     % replay learned policy
CFG.approx = 0;                     % special mode: subsequent extensions
CFG.problem = '';                   % what problem to solve
CFG.loadapprox = '';                % load approximator data from file
CFG.datadir = '';                   % data dir 
CFG.datafile = 'fzqidata';          % save data to file
%% learning config
CFG.gamma = 0.98;                   % discount factor
CFG.eps = 0.001;                    % threshold for convergence
CFG.maxiter = 1000;  CFG.term = 'zero'; 
CFG.serial = 0;      CFG.storeact = 1;                   
                                   
% replay config
CFG.returnprecision = [];           % replay s.t. return can be evaluated with this precision
CFG.interph = 0;                    % use interpolated (averaged) policy to replay
CFG.x0 = [];                        %initial state for replay (otherwise the problem default or zeros)
CFG.tend = 30;                      % end time for replay
CFG.userew = [];                    % use different reward function

%% display config
CFG.visualize = 0;                  % visualization level (0 = none, 1 = iteration-level)
CFG.viscfg = struct;                % visualization config options
CFG.verb = 5;                       % verbosity: the higher, the more detailed the messages displayed
CFG.noplot = 0;                     % whether to suppress figure plots
CFG.silent = 0;                     % suppress all output
CFG.initstepdisp = 0.1;              % feedback every 10% of MDP init
CFG.iterdisp = 10;                  % feedback after every 10 iterations
CFG.itervis = 10;                   % visualize once every 10 iterations
CFG.itersave = 25;                  % save after each 25 iterations

%% stats & figure output config
CFG.truncatehist = [];              % whether history should be truncated at this time value (in seconds)
                                    % before outputting and using in plots
CFG.plottarget = 'screen';          % 'screen', '', 'latex', or 'beamer'. If 'screen' figures will not be closed
CFG.savetheta = 1;                  % save param history in stats
CFG.savedir = '';                   % save figure in this directory
CFG.savefig = '';                   % save figure under this name

%% Early defaults (initialized before calling problem defaults)
ECFG.fuzzy_params = {};            
ECFG.model_params = {};          

KEEPFIELDS = {'problem', 'gamma', 'storeact', 'serial'};

%%
% Fuzzy center for state and action 
q1=[10 15 20 25 30 35 40 45 50 55 60]*pi/180;
dq1=[-90 -45 0 45 90]*pi/180;
q2=[0.05 0.1 0.3 0.35 0.4];
dq2=[-5 -1 0 1 5]*pi/180;
q3=[0.1 0.5 0.7 1 2 4 5 10 20]*pi/180;
dq3=[-5 -1 0 1 5]*pi/180;
Mc = 0:0.5:10;
Fl=-3:0.3:1;

% Configure the algorithm
cfg.run = 1;                        % set the "run" flag
cfg.datadir = pwd;                  % save the data in the current directory
cfg.datafile = 'rarm_shipdemo';  % in this file
cfg.xgrids = {q1;dq1;q2;dq2;q3;dq3};
cfg.ugrids = {Mc;Fl};
cfg.maxiter = 500;                  % run at most this number of iterations
cfg.eps = 0.001;                      % stop when the difference between consecutive parameter 

% If caller provided string, parse it into a structure
if ischar(cfg)
    cfg = str2cfg(cfg, [fieldnames(CFG); fieldnames(ECFG)]);
end
cfg = checkparams(cfg, ECFG);
try     
    if ~isempty(cfg.problem)
        cfg = checkparams(cfg, feval(cfg.problem, 'fuzzy', cfg.fuzzy_params{:})); 
    end
catch
end
cfg = checkparams(cfg, CFG);

if cfg.silent 
    cfg.verb = -Inf; 
    cfg.noplot = 1; 
end

cfg.noinit = (cfg.resume || cfg.replay || cfg.approx) && exist([cfg.datafile '.mat'], 'file');

if cfg.noinit        % load data file, making sure that cfg is not overwritten
    cfg1 = cfg; 
    kf = KEEPFIELDS;
    load(cfg.datafile);
    cfg = copyfields(cfg, cfg1, kf);
    KEEPFIELDS = kf; 
    clear cfg1 kf;
    dispx(['Data loaded from [' cfg.datafile '].'], cfg.verb, 1);
end


dispx('Fuzzy-enhanced reinforcement learning for ship-mounted crane control called with the following configuration:', cfg.verb, 1);
dispx(cfg, cfg.verb, 1);

cfg.envinfo = getenvx;

if ~cfg.noinit
    model = feval(cfg.problem, 'model', cfg.model_params{:});
end

% load tiling, trans, etc. data if given a tiling data file
if cfg.loadapprox
    load(cfg.loadapprox, 'X', 'U', 'DIMS', 'XMFS', 'MDP');
    dispx(['Initialization (approximator and sample data) loaded from [' cfg.loadapprox '].'], cfg.verb, 1);
end


% -----------------------------------------------
if ~cfg.noinit && isempty(cfg.loadapprox)
    % ensure grids were supplied
    checkparams(cfg, [], {'xgrids', 'ugrids'});
    X = cfg.xgrids;  
    U = cfg.ugrids;  

    DIMS.p = length(X); 
    DIMS.q = length(U);    % # of states and outputs
    DIMS.dimx = []; 
    DIMS.dimu = [];             % dimensions of grids
    XMFS = {};
    for p = 1:DIMS.p
        XMFS{p} = gen_mfs(X{p});  
        DIMS.dimx(end+1) = length(X{p});
    end
    for q = 1:DIMS.q
        DIMS.dimu(end+1) = length(U{q});
    end

    % # of parameters
    DIMS.N = prod(DIMS.dimx);
    DIMS.M = prod(DIMS.dimu);
    disp("~cfg.noinit && isempty(cfg.loadapprox)")
end


% -----------------------------------------------
if cfg.run || cfg.resume

    if ~cfg.resume && isempty(cfg.loadapprox)
        actstorage = DIMS.N * DIMS.M * 2^DIMS.p;
        if cfg.storeact < 0      % auto
            cfg.storeact = actstorage < 1e8;
        end
        if cfg.storeact && actstorage > 1e8
            cfg.storeact = 0; 
        end
        if cfg.storeact	
            dispx(['Computing MDP and membership data for (x,u) pairs...'], cfg.verb, 0);
        else
            dispx(['Computing MDP data for (x,u) pairs...'], cfg.verb, 0);
        end

        t = cputime;
        % init MDP structures
        if cfg.storeact  
            MDP.F = sparse(DIMS.N * DIMS.M, DIMS.N);
        else
            MDP.F = zeros(DIMS.N, DIMS.M, DIMS.p);
        end
        MDP.R = zeros(DIMS.N, DIMS.M);
        MDP.T = zeros(DIMS.N, DIMS.M);

        % iterate over (xi, uj) \in (X0, U0) and populate MDP structures
        prog = cfg.initstepdisp;    % feedback every 10% of MDP init        
        tab = dec2base(0:(2^DIMS.p-1), 2) - 47; 
        roll =0 * DIMS.dimx;
        Xflat = flat(X); %给出n个网格的叉积中点集的显式枚举
        Uflat = flat(U);
        for i = 1:DIMS.N
            for j = 1:DIMS.M
                [xplus,MDP.R(i, j),MDP.T(i, j)] = feval(model.fun, model, Xflat(:, i), Uflat(:, j));
                if cfg.storeact
                    if ~MDP.T(i, j) || cfg.term(1) == 'i'
                        [ind, mu] = mdegs_p(xplus, X, roll, DIMS.dimx, DIMS.p, tab);
                       MDP.F(i+(j-1)*DIMS.N, ind) = mu;
                    end
                else    
                    MDP.F(i, j, :) = xplus;
                end
            end          % 过度离散化
            if any(any(MDP.T)) && cfg.term(1) ~= 'i' && ~cfg.storeact
                error('Handling terminal states without storeact=1 is not implemented');
            elseif any(any(MDP.T)) && cfg.term(1) == 'i'
                dispx('WARNING! Terminal states encountered & will be ignored.', cfg.verb, -1);
            end

            if i/DIMS.N > prog    % progress feedback
                dispx([num2str(prog * 100) '% completed...'], cfg.verb, 2);
                prog = prog + cfg.initstepdisp;
            end
        end              
        qistats.tinit = cputime - t;

        dispx('done.', cfg.verb, 2);
        save(cfg.datafile);
        dispx(['Initialization data saved to [' cfg.datafile ']'], cfg.verb, 1);
    end                    
    
    % -----------------------------------------------
    if ~cfg.resume
        qistats.delta = [];
        qistats.t = 0;
        theta = zeros(DIMS.N, DIMS.M);
        if cfg.savetheta
            qistats.theta = cell(cfg.maxiter+1, 1);    
            qistats.theta{1} = theta;                  
        end
        k = 1;
    end

    if cfg.visualize
        vcfg = cfg.viscfg;
        vcfg.gview = [];
        vcfg.fuzzyqiter = 1;
        vcfg.ell = 0;
        [figh,vcfg.gview] = feval(model.visualizefun, vcfg);
    end       
    
    tab = dec2base(0:(2^DIMS.p-1), 2) - 47; 
    roll =0 * DIMS.dimx;
    delta = inf;
    t = cputime;
    while k <= cfg.maxiter && delta > cfg.eps       % main loop
        
        thetaold = theta;     
        % compute policy optimal in Q

        % loop over (xi, uj) samples
        if cfg.storeact        % can use the activation matrix MDP.F
            if cfg.serial      
                for i = 1:DIMS.N
                    for j = 1:DIMS.M
                        theta(i, j) = MDP.R(i, j) + cfg.gamma * max(MDP.F(i+(j-1)*DIMS.N, :) * theta, [], 2);
                    end    % FOR j
                end        % FOR i
            else                % can get away with a matrix operation
                % This is most likely the most efficient version
                theta = MDP.R + cfg.gamma * reshape(max(MDP.F * theta, [], 2), DIMS.N, DIMS.M);
            end            
        else                
            for i = 1:DIMS.N
                for j = 1:DIMS.M
                    [ind, mu] = mdegs_p(MDP.F(i, j, :), X, roll, DIMS.dimx, DIMS.p, tab);
                    if cfg.serial
                        theta(i, j) = MDP.R(i, j) + cfg.gamma * max(mu' * theta(ind, :));
                    else
                        theta(i, j) = MDP.R(i, j) + cfg.gamma * max(mu' * thetaold(ind, :));
                    end    % IF serial
                end% FOR j
            end    % FOR i
        end        % IF cfg.storeact
        
        delta = max(max(abs(theta - thetaold)));        

        % update stats
        qistats.t = qistats.t + (cputime - t);
        qistats.delta(end + 1) = delta;
        if cfg.savetheta
            qistats.theta{k+1} = theta; 
        end     % save theta on stats
        
        if ~mod(k, cfg.iterdisp) 
            dispx(['k=' num2str(k) ' iteration completed, delta=' num2str(delta)], cfg.verb, 2);
        end
        % visualization
        if cfg.visualize && ~mod(k, cfg.itervis)
            vcfg.ell = k;
            [figh,vcfg.gview] = feval(model.visualizefun, vcfg);
        end
        
        % data backup
        if ~mod(k, cfg.itersave)
            save(cfg.datafile);
            dispx(['Intermediary data at k=' num2str(k) ' saved to [' cfg.datafile '].'], cfg.verb, 1);
        end
        t = cputime;
        k = k + 1;
    end       % while not converged and allowed more iterations

    if delta < cfg.eps	
        dispx('Convergence detected. Algorithm stopped.', cfg.verb, 0);
    else
        dispx(['maxiter=' num2str(cfg.maxiter) ' exhausted. Algorithm stopped'], cfg.verb, 0);
    end
    
    % finalize visualizer
    if cfg.visualize
        if vcfg.ell < k - 1
            vcfg.ell = k - 1;      
        else
            vcfg.fuzzyqiter = 0;   
        end
        vcfg.finalize = 1;
        [figh,vcfg.gview] = feval(model.visualizefun, vcfg);
    end
    
    varargout = {theta, qistats};
end


if cfg.replay
%     print(cfg.userew)
    % use different reward function
    if ~isempty(cfg.userew)
        model = feval(cfg.problem, 'changerew', model, cfg.userew);
    end
    
    % compute locally optimal policy (local optimal discrete action for each basis
    % function/tile)
    if cfg.interph
        [Qstar,ui] = max(theta, [], 2); 
        clear Qstar;
        ui = lin2ndi(ui, DIMS.dimu);
        hstar = zeros(DIMS.N, DIMS.q);
        for q = 1:DIMS.q
            hstar(:, q) = U{q}(ui(:, q));
        end
    end
    
% Add debugging information
    disp('Cfg structure passed to rarm_mdpstandalone:');
    disp(cfg);
    disp(['Type of model: ', class(cfg)]);

    % initial state
    if ~isempty(cfg.x0)      % specified initial state
        x0 = cfg.x0(:);
        disp("~isempty(cfg.x0) ")
    else                      % zeros
        x0 = zeros(p, 1);
        disp("what can i say")
    end
    dispx(['Controlling from x0=' num2str(reshape(x0, 1, [])) ], cfg.verb, 0);

    % history
    if ~isempty(cfg.returnprecision)
        K = ceil(log(cfg.returnprecision * (1-cfg.gamma) / model.maxr) / log(cfg.gamma));
        cfg.tend = K * model.Ts;
    end
    t = 0 : model.Ts : cfg.tend;
    Ns = length(t)-1;       % number of samples / time instances at which control is applied
    x = zeros(DIMS.p, length(t)); 
    x(:, 1) = x0;
    u = zeros(DIMS.q, length(t)); 
    u(:, end) = NaN;
    r = zeros(1, length(t)); 
    r(1) = NaN;

    tab = dec2base(0:(2^DIMS.p-1), 2) - 47; 
    roll =0 * DIMS.dimx;
    for k = 1:Ns
        % compute mdegs of current state
        [ind, mu] = mdegs_p(x(:, k), X, roll, DIMS.dimx, DIMS.p, tab);
        
        % compute optimal action
        if cfg.interph     % either interpolated
            u(:, k) = (mu' * hstar(ind, :))';
        else                % or crisp (ties broken randomly)
            Qa = mu' * theta(ind, :);
            ui = find(Qa == max(Qa)); 
            ui = ui(ceil(rand * length(ui)));
            ui = lin2ndi(ui, DIMS.dimu);
            for q = 1:DIMS.q
                u(q, k) = U{q}(ui(:, q));
            end
        end
        
        % apply to system
%        [x(:, k+1),r(k+1),term] = feval(model.fun, 7, x(:, k), u(:, k));
       [x(:, k+1), r(k+1), term] = feval(model.fun, model, x(:, k), u(:, k));
        if term 
            Ns = k; 
            u(:, k+1) = NaN; 
            break 
        end      % entered terminal state
    end
    
    R = discreturn(cfg, r, Ns, term);
    
    % plot history & optionally save figures
    hist.t = t(1:Ns+1);
    hist.x = x(:, 1:Ns+1); 
    hist.u = u(:, 1:Ns+1); 
    hist.r = r(1:Ns+1); 
    hist.R = R(1:Ns+1);
    if ~isempty(cfg.truncatehist)
        hist = truncatehist(hist, cfg.truncatehist); 
    end

    if ~cfg.noplot
        if isfield(model, 'plotfun')
            figh = feval(model.plotfun, hist);
            if isempty(figh)
                figh = plothistory(hist); 
            end
        else
            figh = plothistory(hist);
        end
        setfigprop(cfg);
        saveplot(figh(end), [cfg.savedir cfg.savefig], cfg.plottarget);
    end

    fig = ~cfg.noplot && (strcmp(cfg.plottarget, 'screen') || isempty(cfg.plottarget));
    if fig        
        varargout = {hist, figh};
    else
        varargout = {hist, []};
    end
end       % IF replay

if cfg.approx
    acfg.xgrids = X;
    acfg.ugrids = U;
    approx = triang(model, acfg);
    varargout = {approx, theta};
end

if cfg.run || cfg.resume
    % save into the same directory as the problem
    if isempty(cfg.datadir)
        datadir = fileparts(which(cfg.problem));
    else
        datadir = cfg.datadir; 
        if (datadir(end) == '\'), datadir = datadir(1:end-1); 
        end
    end
    % if default file name is used, generate a unique file name
    if strcmp(cfg.datafile, CFG.datafile)
        c = fix(clock);
        for i = 1:length(c)
            cfg.datafile = [cfg.datafile '-' num2str(c(i), '%02d')];
        end
        delete([CFG.datafile '.mat']);
    elseif ~strcmp(cfg.savedir, pwd)
        delete([cfg.datafile '.mat']);   % need to save in a different directory, so delete anyway
    end    % otherwise just overwrite
    cfg.datafile = [datadir '\' cfg.datafile];
    save(cfg.datafile);
    dispx(['Finished. Data was saved to [' cfg.datafile '].'], cfg.verb, 1);
end
