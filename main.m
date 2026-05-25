%% 主程序
clear; clc; close all;
params = Get_params();
% 加载最新优化结果
data_files = dir('Optim_config_data_*.mat');
[~, latest_idx] = max([data_files.datenum]);
latest_file = data_files(latest_idx).name;
load(latest_file, 'B_opt', 'r_opt', 'fval');
% 闭环仿真
log_orig = Closedloop_sim(params, params.B_all);
log_opt = Closedloop_sim(params, B_opt);
% 结果绘制
Plot_results(log_orig, log_opt, params, B_opt, r_opt);
fprintf('随机故障配置: 实际故障推力器 = [%s], 故障时间 = %.2f s\n', ...
        num2str(log_orig.faulty_thrusters), log_orig.faulty_time);
fprintf('原布局诊断结果: 诊断故障推力器 = [%s], 诊断时间 = %.2f s, 诊断率 = %d\n', ...
        num2str(log_orig.estimated_faults), log_orig.diagnosis_time, log_orig.diagnosis_success);
fprintf('优化布局诊断结果: 诊断故障推力器 = [%s], 诊断时间 = %.2f s, 诊断率 = %d\n', ...
        num2str(log_opt.estimated_faults), log_opt.diagnosis_time, log_opt.diagnosis_success);