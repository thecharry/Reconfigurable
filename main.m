%% 主程序
clear; clc; close all;
params = Get_params();
% 加载最新优化结果
data_files = dir('Optim_config_data_*.mat');
[~, latest_idx] = max([data_files.datenum]);
latest_file = data_files(latest_idx).name;
load(latest_file, 'B_opt', 'r_opt', 'fval');
% load('Optim_config_data1.mat', 'B_opt', 'r_opt', 'fval');
% 闭环仿真
for i = 1:params.Num
        params.true_faults = [1];
        params.alloc_mode = 'strict_sync';
        params.reuse_mode = 'strict_sync'; % 可选: 'six_d_qp' / 'task_book' / 'strict_sync'            
        log_orig = Closedloop_sim(params,params.B_all);
        log_opt = Closedloop_sim(params,B_opt);
        Plot_results(log_orig, log_opt, params, B_opt, r_opt);
end
