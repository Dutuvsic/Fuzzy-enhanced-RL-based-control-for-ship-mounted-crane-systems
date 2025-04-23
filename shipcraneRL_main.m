% Fuzzy enhanced reinforcement learning control method for ship-mounted crane
clear;
close all; 
clc;  

shipcfg = struct;
shipcfg.problem = 'rarm_shipc';
shipcfg.model_params = {'orientation=vert maxtau=[10,3]'};
shipcfg.gamma = 0.98;      % discount factor

if exist('rarm_shipdemo.mat', 'file')
    delete('rarm_shipdemo.mat'); 
end

shipRL(shipcfg);
