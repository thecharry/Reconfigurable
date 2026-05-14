%% 主程序
clear; clc; close all;

params = Get_params();
load('Optim_config_data1.mat', 'B_opt', 'r_opt', 'fval');

log_orig = Closedloop_sim(params, params.B_all);
log_opt = Closedloop_sim(params, B_opt);

Plot_results(log_orig, log_opt, params, B_opt, r_opt);