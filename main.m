%% 主程序
clear; clc; close all;
params = Get_params();
% 加载两种优化布局结果
data_1 = load('Optim_data_20260603_161805_2jzm.mat', 'B_opt', 'r_opt');
% data_2 = load('Optim_data_20260603_150553_3jztlq.mat', 'B_opt', 'r_opt');
data_2 = load('Optim_data_20260614_154028.mat', 'B_opt', 'r_opt');
B_opt = data_2.B_opt;
r_opt = data_2.r_opt;
layout_set = struct('name', {'原布局', '方案一', '方案二'}, ...
                    'B', {params.B_all, data_1.B_opt, data_2.B_opt}, ...
                    'r', {params.r_all, data_1.r_opt, data_2.r_opt});
% 闭环仿真
% for i = 1:params.Num
        params.true_faults = [1];
        params.alloc_mode = 'task_book';% 可选: 'six_d_qp' / 'task_book' / 'strict_sync'
        log_orig = Closedloop_sim(params,params.B_all);
        log_opt1 = Closedloop_sim(params,B_opt);
        log_opt2 = Closedloop_sim(params,B_opt);
        Plot_results(log_orig, log_opt1, params, B_opt, r_opt, layout_set);
% end
