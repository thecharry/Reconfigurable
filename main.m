%% 主程序
clear; clc; close all;

params = Get_params();

data_files = dir('Optim_config_data_*.mat');
[~, latest_idx] = max([data_files.datenum]);
latest_file = data_files(latest_idx).name;
load(latest_file, 'B_opt', 'r_opt', 'fval');

log_orig = Closedloop_sim(params, params.B_all);
log_opt = Closedloop_sim(params, B_opt);

Plot_results(log_orig, log_opt, params, B_opt, r_opt);
