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
        params.true_faults = [];
        params.alloc_mode = 'synchronous_time_division';% 默认同步分时复用策略
        sim_cfg_override = struct('faulty_time', (0.2 + 0.6 * rand) * 2000);
        log_orig = Closedloop_sim(params,params.B_all,sim_cfg_override);
        log_opt1 = Closedloop_sim(params,data_1.B_opt,sim_cfg_override);
        log_opt2 = Closedloop_sim(params,data_2.B_opt,sim_cfg_override);
        log_orig.B_all = params.B_all;
        log_opt1.B_opt = data_1.B_opt;
        log_opt2.B_opt = data_2.B_opt;
        Plot_results(log_orig, log_opt1, log_opt2, params, B_opt, r_opt, layout_set);
% end
