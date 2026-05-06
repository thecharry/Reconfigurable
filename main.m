%% 主程序
% clear; clc; close all;
% params = Get_params();
% load('Optim_config_data1.mat', 'B_opt', 'r_opt', 'fval');
% % load('Optim_config_data_GlobalFree.mat', 'B_opt', 'r_opt', 'fval');
% 
% 
% %% =========================================================
% % 2. 计算原布局、优化布局的底层指标矩阵
% %    每行对应一个状态：标况 + 12个单故障
% % ==========================================================
% disp('正在计算 原布局 底层指标...');
% [Z_Force_orig, Z_Torque_orig, Z_Total_orig, StateNames] = get_Z_matrix(params, params.B_all);
% 
% disp('正在计算 优化后布局 底层指标...');
% [Z_Force_opt, Z_Torque_opt, Z_Total_opt, ~] = get_Z_matrix(params, B_opt);
% 
% %% =========================================================
% % 3. 拼装全局评价矩阵（统一基准）
% % ==========================================================
% Z_Force_combined  = [Z_Force_orig;  Z_Force_opt];
% Z_Torque_combined = [Z_Torque_orig; Z_Torque_opt];
% Z_Total_combined  = [Z_Total_orig;  Z_Total_opt];
% 
% %% =========================================================
% % 4. AHP 主观判断矩阵
% %    指标顺序：
% %    [Jc, Ja, Jo, Jf]
% % ==========================================================
% % 布局层指标顺序：[Jc, Ja, Jf, Jo]
% G_AHP = [
%     1,   2,   3,   4;
%    1/2, 1,   2,   3;
%    1/3, 1/2, 1,   2;
%    1/4, 1/3, 1/2, 1
% ];
% 
% %% =========================================================
% % 5. 综合评价
% % ==========================================================
% [F_Force_combined,  W_Force,  U_Force,  V_Force]  = Comprehensive_Eval(Z_Force_combined,  G_AHP);
% [F_Torque_combined, W_Torque, U_Torque, V_Torque] = Comprehensive_Eval(Z_Torque_combined, G_AHP);
% [F_Total_combined,  W_Total,  U_Total,  V_Total]  = Comprehensive_Eval(Z_Total_combined,  G_AHP);
% 
% %% =========================================================
% % 6. 拆分结果
% % ==========================================================
% nState = params.Num + 1;
% 
% F_Force_orig  = F_Force_combined(1:nState);
% F_Force_opt   = F_Force_combined(nState+1:end);
% 
% F_Torque_orig = F_Torque_combined(1:nState);
% F_Torque_opt  = F_Torque_combined(nState+1:end);
% 
% F_Total_orig  = F_Total_combined(1:nState);
% F_Total_opt   = F_Total_combined(nState+1:end);
% 
% %% =========================================================
% % 7. 打印权重结果
% % ==========================================================
% disp('==============================================================');
% disp('最终组合权重（统一评价池下）');
% disp('--------------------------------------------------------------');
% % disp('Force 方向权重 [Jc, Ja, Jo, Jf] = ');
% % disp(W_Force');
% % 
% % disp('Torque 方向权重 [Jc, Ja, Jo, Jf] = ');
% % disp(W_Torque');
% % 
% % disp('Total 综合权重 [Jc, Ja, Jo, Jf] = ');
% % disp(W_Total');
% disp('Force 方向权重 [Jc, Ja, Jf, Jo] = ');
% disp(W_Force');
% 
% disp('Torque 方向权重 [Jc, Ja, Jf, Jo] = ');
% disp(W_Torque');
% 
% disp('Total 综合权重 [Jc, Ja, Jf, Jo] = ');
% disp(W_Total');
% disp('==============================================================');
% 
% %% =========================================================
% % 8. 打印状态对比结果
% % ==========================================================
% disp(' ');
% disp('==========================================================================');
% disp('               力控制(Force) 综合评价值 F 对比（统一基准）');
% disp('==========================================================================');
% disp('状态编号         | 原布局 F     | 优化布局 F   | 提升(%)');
% for i = 1:nState
%     improvement = (F_Force_opt(i) - F_Force_orig(i)) / (F_Force_orig(i) + 1e-12) * 100;
%     fprintf('%-14s | %10.4f | %10.4f | %+8.2f%%\n', ...
%         StateNames{i}, F_Force_orig(i), F_Force_opt(i), improvement);
% end
% disp('==========================================================================');
% 
% disp(' ');
% disp('==========================================================================');
% disp('              力矩控制(Torque) 综合评价值 F 对比（统一基准）');
% disp('==========================================================================');
% disp('状态编号         | 原布局 F     | 优化布局 F   | 提升(%)');
% for i = 1:nState
%     improvement = (F_Torque_opt(i) - F_Torque_orig(i)) / (F_Torque_orig(i) + 1e-12) * 100;
%     fprintf('%-14s | %10.4f | %10.4f | %+8.2f%%\n', ...
%         StateNames{i}, F_Torque_orig(i), F_Torque_opt(i), improvement);
% end
% disp('==========================================================================');
% 
% disp(' ');
% disp('==========================================================================');
% disp('                整体综合(Total) 评价值 F 对比（统一基准）');
% disp('==========================================================================');
% disp('状态编号         | 原布局 F     | 优化布局 F   | 提升(%)');
% for i = 1:nState
%     improvement = (F_Total_opt(i) - F_Total_orig(i)) / (F_Total_orig(i) + 1e-12) * 100;
%     fprintf('%-14s | %10.4f | %10.4f | %+8.2f%%\n', ...
%         StateNames{i}, F_Total_orig(i), F_Total_opt(i), improvement);
% end
% disp('==========================================================================');
% 
% %% =========================================================
% % 9. 打印平均性能
% % ==========================================================
% disp(' ');
% disp('======================= 平均综合性能 =======================');
% fprintf('Force  平均 F: 原布局 = %.4f, 优化布局 = %.4f, 提升 = %+6.2f%%\n', ...
%     mean(F_Force_orig), mean(F_Force_opt), ...
%     (mean(F_Force_opt)-mean(F_Force_orig))/(mean(F_Force_orig)+1e-12)*100);
% 
% fprintf('Torque 平均 F: 原布局 = %.4f, 优化布局 = %.4f, 提升 = %+6.2f%%\n', ...
%     mean(F_Torque_orig), mean(F_Torque_opt), ...
%     (mean(F_Torque_opt)-mean(F_Torque_orig))/(mean(F_Torque_orig)+1e-12)*100);
% 
% fprintf('Total  平均 F: 原布局 = %.4f, 优化布局 = %.4f, 提升 = %+6.2f%%\n', ...
%     mean(F_Total_orig), mean(F_Total_opt), ...
%     (mean(F_Total_opt)-mean(F_Total_orig))/(mean(F_Total_orig)+1e-12)*100);
% disp('============================================================');
% 
% %% =========================================================
% % 10. 闭环仿真对比：原布局 vs 优化布局
% % ==========================================================
% 
% % 为了公平对比，故障时间和故障推力器必须固定
% sim_cfg.r0 = [0;15;55];
% sim_cfg.rt = [5;25;35];
% sim_cfg.v0 = [0;0;0];
% 
% sim_cfg.euler0 = deg2rad([0;15;55]);
% sim_cfg.eulert = deg2rad([5;25;35]);
% 
% sim_cfg.T_sim = 2000;
% sim_cfg.dt = 0.005;
% 
% sim_cfg.Kp_pos = 10;
% sim_cfg.Kd_pos = 300;
% sim_cfg.Kp_att = 400 * eye(3);
% sim_cfg.Kd_att = 3200 * eye(3);
% sim_cfg.Ki_att = 20 * eye(3);
% 
% % 固定故障场景
% sim_cfg.num_faults = 1;
% sim_cfg.fault_time = 0.5 * sim_cfg.T_sim;     % 建议先固定，不要 rand
% sim_cfg.faulty_thrusters_fixed = 3;           % 例如固定第3台故障
% 
% % 如果你想随机，也要先随机一次，然后原/优化共用
% % sim_cfg.fault_time = rand() * sim_cfg.T_sim;
% % sim_cfg.faulty_thrusters_fixed = randperm(params.Num, sim_cfg.num_faults);
% 
% disp(' ');
% disp('================ 开始原布局闭环仿真 ================');
% [log_orig, Perf_orig, fault_orig] = Run_ClosedLoop_Sim(params, params.B_all, sim_cfg);
% 
% disp(' ');
% disp('================ 开始优化布局闭环仿真 ================');
% [log_opt, Perf_opt, fault_opt] = Run_ClosedLoop_Sim(params, B_opt, sim_cfg);
% 
% %% 闭环性能对比
% Compare_ClosedLoop_Performance(Perf_orig, Perf_opt);
% 
% %% 可选绘图
% [J_orig, ~] = Evaluation(params, params.B_all);
% [J_opt,  ~] = Evaluation(params, B_opt);
% 
% % Plot_results(fault_orig.faulty_thrusters, J_orig, log_orig, params, params.B_all, [], params.B_all, fault_orig.fault_time);
% % Plot_results(fault_opt.faulty_thrusters,  J_opt,  log_opt,  params, B_opt, r_opt, B_opt, fault_opt.fault_time);
% Plot_results(params, B_opt, r_opt, ...
%                    log_orig, log_opt, Perf_orig, Perf_opt, ...
%                    fault_orig, ...
%                    Z_Force_orig, Z_Force_opt, ...
%                    Z_Torque_orig, Z_Torque_opt, ...
%                    Z_Total_orig, Z_Total_opt, ...
%                    F_Force_orig, F_Force_opt, ...
%                    F_Torque_orig, F_Torque_opt, ...
%                    F_Total_orig, F_Total_opt, ...
%                    StateNames);

%% 主程序
clear; clc; close all;

params = Get_params();
load('Optim_config_data1.mat', 'B_opt', 'r_opt', 'fval');

% %% =========================================================
% % 1. 计算原布局、优化布局的底层指标矩阵
% %    指标顺序统一为：[Jc, Ja, Jf, Jo]
% % ==========================================================
% disp('正在计算 原布局 底层指标...');
% [Z_Force_orig, Z_Torque_orig, Z_Total_orig, StateNames] = get_Z_matrix(params, params.B_all);
% 
% disp('正在计算 优化后布局 底层指标...');
% [Z_Force_opt, Z_Torque_opt, Z_Total_opt, ~] = get_Z_matrix(params, B_opt);
% 
% %% =========================================================
% % 2. 拼装全局评价矩阵
% % ==========================================================
% Z_Force_combined  = [Z_Force_orig;  Z_Force_opt];
% Z_Torque_combined = [Z_Torque_orig; Z_Torque_opt];
% Z_Total_combined  = [Z_Total_orig;  Z_Total_opt];
% 
% %% =========================================================
% % 3. 布局层 AHP 判断矩阵
% %    布局层指标顺序：[Jc, Ja, Jf, Jo]
% % ==========================================================
% G_AHP_layout = [
%     1,   2,   3,   4;
%    1/2, 1,   2,   3;
%    1/3, 1/2, 1,   2;
%    1/4, 1/3, 1/2, 1
% ];
% 
% %% =========================================================
% % 4. 布局层综合评价
% % ==========================================================
% [F_Force_combined,  W_Force,  U_Force,  V_Force]  = Comprehensive_Eval(Z_Force_combined,  G_AHP_layout);
% [F_Torque_combined, W_Torque, U_Torque, V_Torque] = Comprehensive_Eval(Z_Torque_combined, G_AHP_layout);
% [F_Total_combined,  W_Total,  U_Total,  V_Total]  = Comprehensive_Eval(Z_Total_combined,  G_AHP_layout);
% 
% nState = params.Num + 1;
% 
% F_Force_orig  = F_Force_combined(1:nState);
% F_Force_opt   = F_Force_combined(nState+1:end);
% 
% F_Torque_orig = F_Torque_combined(1:nState);
% F_Torque_opt  = F_Torque_combined(nState+1:end);
% 
% F_Total_orig  = F_Total_combined(1:nState);
% F_Total_opt   = F_Total_combined(nState+1:end);
% 
% %% =========================================================
% % 5. 输出布局层综合评价结果
% % ==========================================================
% disp('==============================================================');
% disp('布局层最终组合权重（统一评价池下）');
% disp('--------------------------------------------------------------');
% 
% disp('Force 方向权重 [Jc, Ja, Jf, Jo] = ');
% disp(W_Force');
% 
% disp('Torque 方向权重 [Jc, Ja, Jf, Jo] = ');
% disp(W_Torque');
% 
% disp('Total 综合权重 [Jc, Ja, Jf, Jo] = ');
% disp(W_Total');
% 
% disp('==============================================================');
% 
% disp(' ');
% disp('==========================================================================');
% disp('                整体综合(Total) 评价值 F_layout 对比');
% disp('==========================================================================');
% disp('状态编号         | 原布局 F        | 优化布局 F        | 提升(%)');
% 
% for i = 1:nState
%     improvement = (F_Total_opt(i) - F_Total_orig(i)) / (F_Total_orig(i) + 1e-12) * 100;
%     fprintf('%-14s | %14.4f | %16.4f | %+8.2f%%\n', ...
%         StateNames{i}, F_Total_orig(i), F_Total_opt(i), improvement);
% end
% 
% disp('==========================================================================');
% 
% disp(' ');
% disp('======================= 平均布局综合性能 =======================');
% fprintf('Force  平均 F: 原布局 = %.4f, 优化布局 = %.4f, 提升 = %+6.2f%%\n', ...
%     mean(F_Force_orig), mean(F_Force_opt), ...
%     (mean(F_Force_opt)-mean(F_Force_orig))/(mean(F_Force_orig)+1e-12)*100);
% 
% fprintf('Torque 平均 F: 原布局 = %.4f, 优化布局 = %.4f, 提升 = %+6.2f%%\n', ...
%     mean(F_Torque_orig), mean(F_Torque_opt), ...
%     (mean(F_Torque_opt)-mean(F_Torque_orig))/(mean(F_Torque_orig)+1e-12)*100);
% 
% fprintf('Total  平均 F: 原布局 = %.4f, 优化布局 = %.4f, 提升 = %+6.2f%%\n', ...
%     mean(F_Total_orig), mean(F_Total_opt), ...
%     (mean(F_Total_opt)-mean(F_Total_orig))/(mean(F_Total_orig)+1e-12)*100);
% disp('===============================================================');
% 
% %% =========================================================
% % 6. 闭环仿真对比：原布局 vs 优化布局
% % ==========================================================
% 
% sim_cfg.r0 = [0;15;55];
% sim_cfg.rt = [5;25;35];
% sim_cfg.v0 = [0;0;0];
% 
% sim_cfg.euler0 = deg2rad([0;15;55]);
% sim_cfg.eulert = deg2rad([5;25;35]);
% 
% sim_cfg.T_sim = 2000;
% sim_cfg.dt = 0.005;
% 
% sim_cfg.Kp_pos = 10;
% sim_cfg.Kd_pos = 300;
% sim_cfg.Kp_att = 400 * eye(3);
% sim_cfg.Kd_att = 3200 * eye(3);
% sim_cfg.Ki_att = 20 * eye(3);
% 
% % 固定故障场景
% sim_cfg.num_faults = 1;
% sim_cfg.fault_time = 0.5 * sim_cfg.T_sim;
% sim_cfg.faulty_thrusters_fixed = 3;
% 
% disp(' ');
% disp('================ 开始原布局闭环仿真 ================');
% [log_orig, Perf_orig, fault_orig] = Run_ClosedLoop_Sim(params, params.B_all, sim_cfg);
% 
% disp(' ');
% disp('================ 开始优化布局闭环仿真 ================');
% [log_opt, Perf_opt, fault_opt] = Run_ClosedLoop_Sim(params, B_opt, sim_cfg);
% 
% Compare_ClosedLoop_Performance(Perf_orig, Perf_opt);
% 
% %% =========================================================
% % 7. 系统层综合评价
% %    系统层指标顺序：[Je, Jc, Ja, Jf, Jo]
% % ==========================================================
% 
% G_AHP_system = [
%     1,    2,    3,    4,    5;
%     1/2,  1,    2,    3,    4;
%     1/3,  1/2,  1,    2,    3;
%     1/4,  1/3,  1/2,  1,    2;
%     1/5,  1/4,  1/3,  1/2,  1
% ];
% 
% % 当前闭环仿真对应的故障状态
% if sim_cfg.num_faults == 0
%     eval_state_idx = 1;  % 标况
% else
%     eval_state_idx = sim_cfg.faulty_thrusters_fixed(1) + 1;
% end
% 
% % Z_Total 的列顺序为：[Jc, Ja, Jf, Jo]
% Jc_orig = Z_Total_orig(eval_state_idx, 1);
% Ja_orig = Z_Total_orig(eval_state_idx, 2);
% Jf_orig = Z_Total_orig(eval_state_idx, 3);
% Jo_orig = Z_Total_orig(eval_state_idx, 4);
% 
% Jc_opt_val = Z_Total_opt(eval_state_idx, 1);
% Ja_opt_val = Z_Total_opt(eval_state_idx, 2);
% Jf_opt_val = Z_Total_opt(eval_state_idx, 3);
% Jo_opt_val = Z_Total_opt(eval_state_idx, 4);
% 
% Je_orig = Perf_orig.Precision_Score;
% Je_opt  = Perf_opt.Precision_Score;
% 
% Z_System = [
%     Je_orig, Jc_orig,     Ja_orig,     Jf_orig,     Jo_orig;
%     Je_opt,  Jc_opt_val,  Ja_opt_val,  Jf_opt_val,  Jo_opt_val
% ];
% 
% [F_System, W_System, U_System, V_System] = Comprehensive_Eval(Z_System, G_AHP_system);
% 
% disp(' ');
% disp('======================= 最终系统综合评价 =======================');
% disp('评价指标：[Je, Jc, Ja, Jf, Jo]');
% disp('指标权重W = ');
% disp(W_System');
% 
% fprintf('当前系统评价对应状态：%s\n', StateNames{eval_state_idx});
% fprintf('原布局 F_System   = %.4f\n', F_System(1));
% fprintf('优化布局 F_System = %.4f\n', F_System(2));
% fprintf('系统综合提升      = %+6.2f%%\n', ...
%     (F_System(2)-F_System(1))/(F_System(1)+1e-12)*100);
% disp('===============================================================');
% 
% %% =========================================================
% % 8. 绘图
% % ==========================================================
% 
% Plot_results(params, B_opt, r_opt, ...
%              log_orig, log_opt, Perf_orig, Perf_opt, ...
%              fault_orig, ...
%              Z_Force_orig, Z_Force_opt, ...
%              Z_Torque_orig, Z_Torque_opt, ...
%              Z_Total_orig, Z_Total_opt, ...
%              F_Force_orig, F_Force_opt, ...
%              F_Torque_orig, F_Torque_opt, ...
%              F_Total_orig, F_Total_opt, ...
%              StateNames, eval_state_idx);
% 
% 
% function [log, Perf, fault_info] = Run_ClosedLoop_Sim(params, B, sim_cfg)
% 
%     % ============================================================
%     % 输入:
%     %   params  : 参数结构体
%     %   B       : 未乘 F_max 的 6×N 推力器配置矩阵
%     %   sim_cfg : 仿真配置
%     %
%     % 输出:
%     %   log       : 仿真日志
%     %   Perf      : 闭环性能指标
%     %   fault_info: 故障信息
%     % ============================================================
% 
%     Matrix_conf = params.F_max * B;
% 
%     r0 = sim_cfg.r0;
%     rt = sim_cfg.rt;
%     v0 = sim_cfg.v0;
% 
%     euler0 = sim_cfg.euler0;
%     eulert = sim_cfg.eulert;
% 
%     sigma0 = Euler_to_MRPs(euler0);
%     omega0 = [0;0;0];
% 
%     Y = [r0; v0; sigma0; omega0];
% 
%     Next_Control_Time = 0;
%     T_sim = sim_cfg.T_sim;
%     dt = sim_cfg.dt;
%     N = floor(T_sim / dt);
% 
%     int = [0;0;0];
% 
%     Kp_pos = sim_cfg.Kp_pos;
%     Kd_pos = sim_cfg.Kd_pos;
%     Kp_att = sim_cfg.Kp_att;
%     Kd_att = sim_cfg.Kd_att;
%     Ki_att = sim_cfg.Ki_att;
% 
%     faulty_thrusters = [];
%     fault_time = sim_cfg.fault_time;
%     fault_trig = false;
%     num_faults = sim_cfg.num_faults;
% 
%     Prop_Final = zeros(params.Num, 1);
% 
%     log.Y_euler = zeros(3, N);
%     log.Time = zeros(1, N);
%     log.R = zeros(3, N);
%     log.V = zeros(3, N);
%     log.E = zeros(3, N);
%     log.O = zeros(3, N);
%     log.Y = zeros(12, N);
%     log.Pulse_Widths = zeros(params.Num, N);
% 
%     log.Total_Pulse = 0;
%     log.Control_Count = 0;
% 
%     tic;
% 
%     for k = 1:N
%         t = (k-1) * dt;
% 
%         % ---------- 故障注入 ----------
%         if t >= fault_time && ~fault_trig
%             fault_trig = true;
% 
%             if num_faults == 0
%                 faulty_thrusters = [];
%             else
%                 faulty_thrusters = sim_cfg.faulty_thrusters_fixed;
%             end
%         end
% 
%         % ---------- 控制更新 ----------
%         if t >= Next_Control_Time
%             r = Y(1:3);
%             v = Y(4:6);
%             sigma = Y(7:9);
%             omega = Y(10:12);
% 
%             [r_d, v_d] = Guidance(t, r0, rt, T_sim);
%             [euler_d, euler_rate_d] = Guidance(t, euler0, eulert, T_sim);
% 
%             R = [1, 0, -sin(euler_d(2));
%                  0, cos(euler_d(1)), sin(euler_d(1))*cos(euler_d(2));
%                  0, -sin(euler_d(1)), cos(euler_d(1))*cos(euler_d(2))];
% 
%             omega_d = R * euler_rate_d;
%             sigma_d = Euler_to_MRPs(euler_d);
% 
%             % ---------- 轨道控制期望力 ----------
%             F_orbit_req = Kp_pos * (r_d - r) + Kd_pos * (v_d - v);
%             [~, ~, R_b2o] = Body_to_Orbit(sigma);
%             F_body_req = R_b2o' * F_orbit_req;
% 
%             % ---------- 姿态控制期望力矩 ----------
%             sigma_err = ((1-sigma'*sigma)*sigma_d ...
%                         -(1-sigma_d'*sigma_d)*sigma ...
%                         -2*cross(sigma_d,-sigma)) / ...
%                         (1+(sigma_d'*sigma_d)*(sigma'*sigma)-2*dot(sigma_d,-sigma));
% 
%             int = int + sigma_err * params.T;
% 
%             T_body_req = Kp_att * sigma_err ...
%                        + Kd_att * (omega_d - omega) ...
%                        + Ki_att * int;
% 
%             % ---------- 推力器调用 ----------
%             Prop_Final = Thruster_invocation( ...
%                 F_body_req, T_body_req, Matrix_conf, faulty_thrusters, params);
% 
%             % ---------- 累计真实喷气时长 ----------
%             log.Total_Pulse = log.Total_Pulse + sum(Prop_Final);
%             log.Control_Count = log.Control_Count + 1;
% 
%             Next_Control_Time = Next_Control_Time + params.T;
%         end
% 
%         % ---------- 当前控制周期内实际喷气 ----------
%         time_in_cycle = t - (Next_Control_Time - params.T);
%         u_applied = params.F_max * (time_in_cycle < Prop_Final);
% 
%         % 实际力和力矩
%         W_total = B * u_applied;
%         params.current_F = W_total(1:3);
%         params.current_T = W_total(4:6);
% 
%         % 动力学积分
%         dY = Spacecraft_dynamics(Y, params);
%         Y = Y + dY * dt;
% 
%         % ---------- 为保证每一步都有期望轨迹记录，重新计算参考 ----------
%         [r_d_log, v_d_log] = Guidance(t, r0, rt, T_sim);
%         [euler_d_log, euler_rate_d_log] = Guidance(t, euler0, eulert, T_sim);
% 
%         R_log = [1, 0, -sin(euler_d_log(2));
%                  0, cos(euler_d_log(1)), sin(euler_d_log(1))*cos(euler_d_log(2));
%                  0, -sin(euler_d_log(1)), cos(euler_d_log(1))*cos(euler_d_log(2))];
% 
%         omega_d_log = R_log * euler_rate_d_log;
% 
%         % ---------- 数据记录 ----------
%         log.Y_euler(:, k) = MRPs_to_Euler(Y(7:9));
%         log.Time(k) = t;
%         log.R(:, k) = r_d_log;
%         log.V(:, k) = v_d_log;
%         log.E(:, k) = euler_d_log;
%         log.O(:, k) = omega_d_log;
%         log.Y(:, k) = Y;
%         log.Pulse_Widths(:, k) = Prop_Final;
%     end
% 
%     toc;
% 
%     Perf = Evaluate_ClosedLoop_Performance(log);
% 
%     fault_info.fault_time = fault_time;
%     fault_info.faulty_thrusters = faulty_thrusters;
% end
% function Compare_ClosedLoop_Performance(Perf_orig, Perf_opt)
% 
%     disp(' ');
%     disp('==============================================================');
%     disp('                 闭环仿真性能对比');
%     disp('==============================================================');
% 
%     print_compare('总喷气时长 Total_Pulse', ...
%         Perf_orig.Total_Pulse, Perf_opt.Total_Pulse, false);
% 
%     print_compare('平均每周期喷气时长', ...
%         Perf_orig.Avg_Pulse_Per_Control, Perf_opt.Avg_Pulse_Per_Control, false);
% 
%     print_compare('位置 RMSE', ...
%         Perf_orig.Pos_RMSE, Perf_opt.Pos_RMSE, false);
% 
%     print_compare('姿态 RMSE', ...
%         Perf_orig.Att_RMSE, Perf_opt.Att_RMSE, false);
% 
%     print_compare('位置 MAE', ...
%         Perf_orig.Pos_MAE, Perf_opt.Pos_MAE, false);
% 
%     print_compare('姿态 MAE', ...
%         Perf_orig.Att_MAE, Perf_opt.Att_MAE, false);
% 
%     print_compare('综合控制精度得分', ...
%         Perf_orig.Precision_Score, Perf_opt.Precision_Score, true);
% 
%     disp('==============================================================');
% 
%     function print_compare(name_str, val_orig, val_opt, is_benefit)
%         if is_benefit
%             improve = (val_opt - val_orig) / (abs(val_orig) + 1e-12) * 100;
%         else
%             improve = (val_orig - val_opt) / (abs(val_orig) + 1e-12) * 100;
%         end
% 
%         fprintf('%-26s | 原布局 = %12.6f | 优化布局 = %12.6f | 改善 = %+8.2f%%\n', ...
%             name_str, val_orig, val_opt, improve);
%     end
% end

[J,Matrix_conf_opt] = Evaluation(params, B_opt);
% B = params.B_all;
% Matrix_conf = params.Matrix_conf;
B = B_opt;
Matrix_conf = Matrix_conf_opt;

% 初始化
r0 = [0;15;55];
rt = [5;25;35];
v0 = [0; 0; 0];
euler0 = deg2rad([0;15;55]);
eulert = deg2rad([5;25;35]);
sigma0 = Euler_to_MRPs(euler0);
omega0 = [0; 0; 0];
Y = [r0; v0; sigma0; omega0];
Next_Control_Time = 0;
T_sim = 2000;% 仿真时间
dt = 0.005;
N = floor(T_sim / dt);
int = [0;0;0];
Kp_pos = 10;
Kd_pos = 300;
Kp_att = 400 * eye(3);
Kd_att = 3200 * eye(3);
Ki_att = 20 * eye(3);
faulty_thrusters = [];% 空数组表示推力器无故障
falut_time = rand() * T_sim;
fault_trig = false;% 增加标志位，防止故障被重复注入
num_faults = 1; % 允许的最大同时故障台数

log.Y_euler = zeros(3, N);
log.Time = zeros(1, N);
log.R = zeros(3, N);
log.V = zeros(3, N);
log.E = zeros(3, N);
log.O = zeros(3, N);
log.Y = zeros(12, N);
log.Pulse_Widths = zeros(12, N);
log.Total_Pulse = 0;        % 整个任务累计总喷气时长
log.Control_Count = 0;      % 控制更新次数

tic;
%% 主循环
for k = 1:N
    t = (k-1) * dt;
    if t >= falut_time && ~fault_trig
        fault_trig = true;
        if num_faults == 0
            faulty_thrusters = [];% 标况
        else
            % faulty_thrusters = randperm(params.Num, num_faults);
            faulty_thrusters =3;
        end
    end
    if t >= Next_Control_Time
        % 初始和目标状态获取
        r = Y(1:3);
        v = Y(4:6);
        sigma = Y(7:9);
        omega = Y(10:12);
        [r_d, v_d] = Guidance(t, r0, rt, T_sim);
        [euler_d, euler_rate_d] = Guidance(t, euler0, eulert, T_sim);
        R = [1, 0, -sin(euler_d(2));% 欧拉角速率与角速度转换矩阵
            0, cos(euler_d(1)), sin(euler_d(1))*cos(euler_d(2));
            0, -sin(euler_d(1)), cos(euler_d(1))*cos(euler_d(2))];
        omega_d = R * euler_rate_d; % 本体系期望角速度
        sigma_d = Euler_to_MRPs(euler_d);
        % 轨道控制期望控制力
        F_orbit_req =  Kp_pos * (r_d - r) + Kd_pos * (v_d - v);
        [~, ~, R_b2o] = Body_to_Orbit(sigma);
        F_body_req = R_b2o' * F_orbit_req;
        % 姿态控制期望控制力矩
        sigma_err = ((1-sigma'*sigma)*sigma_d-(1-sigma_d'*sigma_d)*sigma-2*cross(sigma_d,-sigma))/...
                    (1+(sigma_d'*sigma_d)*(sigma'*sigma)-2*dot(sigma_d,-sigma));% MRP乘法运算法则求误差姿态
        int = int + sigma_err * params.T; 
        T_body_req = Kp_att * sigma_err + Kd_att * (omega_d - omega)+ Ki_att * int;
        % 推力器调用策略
        Prop_Final = Thruster_invocation(F_body_req,T_body_req,Matrix_conf,faulty_thrusters,params);

        % 累计真实总喷气时长
        log.Total_Pulse = log.Total_Pulse + sum(Prop_Final);
        log.Control_Count = log.Control_Count + 1;

        % 更新时间
        Next_Control_Time = Next_Control_Time + params.T;
    end
    time_in_cycle = t - (Next_Control_Time - params.T);
    u_applied = params.F_max * (time_in_cycle < Prop_Final);
    % 实际力和力矩
    W_total = B * u_applied;
    params.current_F = W_total(1:3);
    params.current_T = W_total(4:6);
    % 动力学积分
    dY = Spacecraft_dynamics(Y, params);
    Y = Y + dY * dt;  
    % 数据记录
    log.Y_euler(1:3,k) = MRPs_to_Euler(Y(7:9,:));
    log.Time(k) = t;
    log.R(1:3,k) = r_d;
    log.V(1:3,k) = v_d;
    log.E(1:3,k) = euler_d;
    log.O(1:3,k) = omega_d;
    log.Y(1:12,k) = Y;
    log.Pulse_Widths(1:12,k) = Prop_Final;
    % log.Pulse_Widths(1:12,k) = u_applied;
end
toc;

Perf = Evaluate_ClosedLoop_Performance(log);

disp('================ 闭环仿真性能 =================');
fprintf('总喷气时长 Total_Pulse           = %.6f\n', Perf.Total_Pulse);
fprintf('平均每控制周期喷气时长          = %.6f\n', Perf.Avg_Pulse_Per_Control);
fprintf('位置 RMSE                        = %.6f\n', Perf.Pos_RMSE);
fprintf('姿态 RMSE                        = %.6f\n', Perf.Att_RMSE);
fprintf('位置 MAE                         = %.6f\n', Perf.Pos_MAE);
fprintf('姿态 MAE                         = %.6f\n', Perf.Att_MAE);
fprintf('综合控制精度得分 Precision_Score = %.6f\n', Perf.Precision_Score);
disp('==============================================');

% 绘图
Plot_results(faulty_thrusters, J, log, params, B_opt, r_opt, B,falut_time);

Plot_Jo_FullVector(params, params.B_all, B_opt, faulty_thrusters);
function Plot_Jo_FullVector(params, B_orig, B_opt, faulty_thrusters)
% ============================================================
% Jo可视化：6维控制向量方向 → PCA降维到3D
% ============================================================

    if nargin < 4
        faulty_thrusters = [];
    end

    % ===== 获取6维控制向量 =====
    K_orig = params.F_max * B_orig;
    K_opt  = params.F_max * B_opt;

    idx_orig = setdiff(1:params.Num, faulty_thrusters);
    idx_opt  = setdiff(1:params.Num, faulty_thrusters);

    K_orig = K_orig(:, idx_orig);
    K_opt  = K_opt(:, idx_opt);

    % ===== 归一化（只看方向）=====
    K_orig = K_orig ./ (vecnorm(K_orig,2,1) + 1e-12);
    K_opt  = K_opt  ./ (vecnorm(K_opt,2,1) + 1e-12);

    % ===== PCA降维 =====
    [U_orig,~,~] = svd(K_orig,'econ');
    [U_opt,~,~]  = svd(K_opt,'econ');

    % 取前三个主方向
    P_orig = U_orig(:,1:3)' * K_orig;
    P_opt  = U_opt(:,1:3)'  * K_opt;

    % ===== 绘图 =====
    figure('Name','Jo可诊断性：6维向量方向（PCA投影）','Color','w','Position',[100 100 1300 550]);

    subplot(1,2,1);
    Plot_Vector3D(P_orig, idx_orig, '原布局控制向量方向');

    subplot(1,2,2);
    Plot_Vector3D(P_opt, idx_opt, '优化布局控制向量方向');
end
function Plot_Vector3D(V, idx, title_str)

    hold on; grid on; axis equal;

    N = size(V,2);
    colors = lines(N);

    for i = 1:N
        quiver3(0,0,0, V(1,i), V(2,i), V(3,i), ...
            0, 'LineWidth',1.8,'Color',colors(i,:));

        text(1.1*V(1,i),1.1*V(2,i),1.1*V(3,i), ...
            sprintf('%d', idx(i)), ...
            'FontSize',10,'FontWeight','bold');
    end

    xlabel('X');
    ylabel('Y');
    zlabel('Z');

    title(title_str);
    view(35,25);
end


function Perf = Evaluate_ClosedLoop_Performance(log)
% ============================================================
% 闭环仿真性能评价
%
% 输入:
%   log : 主程序仿真日志
%
% 输出:
%   Perf.Total_Pulse
%   Perf.Avg_Pulse_Per_Control
%   Perf.Pos_RMSE
%   Perf.Att_RMSE
%   Perf.Pos_MAE
%   Perf.Att_MAE
%   Perf.Precision_Score
% ============================================================

    % ---------- 总喷气时长 ----------
    if isfield(log, 'Total_Pulse')
        Perf.Total_Pulse = log.Total_Pulse;
    else
        Perf.Total_Pulse = NaN;
    end

    if isfield(log, 'Control_Count') && log.Control_Count > 0
        Perf.Avg_Pulse_Per_Control = log.Total_Pulse / log.Control_Count;
    else
        Perf.Avg_Pulse_Per_Control = NaN;
    end

    % ---------- 位置误差 ----------
    r_real = log.Y(1:3, :);
    r_ref  = log.R;

    pos_err = r_ref - r_real;
    pos_err_norm = vecnorm(pos_err, 2, 1);

    Perf.Pos_RMSE = sqrt(mean(pos_err_norm.^2));
    Perf.Pos_MAE  = mean(pos_err_norm);

    % ---------- 姿态误差 ----------
    euler_real = log.Y_euler;
    euler_ref  = log.E;

    att_err = euler_ref - euler_real;
    att_err = wrapToPi_local(att_err);
    att_err_norm = vecnorm(att_err, 2, 1);

    Perf.Att_RMSE = sqrt(mean(att_err_norm.^2));
    Perf.Att_MAE  = mean(att_err_norm);

    % ---------- 控制精度得分 ----------
    Pos_Score = 1 / (1 + Perf.Pos_RMSE);
    Att_Score = 1 / (1 + Perf.Att_RMSE);

    Perf.Precision_Score = 0.5 * Pos_Score + 0.5 * Att_Score;

    function x = wrapToPi_local(x)
        x = mod(x + pi, 2*pi) - pi;
    end
end

%% --- 辅助封装函数 (为了主程序干净，把遍历13个状态封装成一个函数) ---
% function [Z_Force, Z_Torque, Z_Total, StateNames] = get_Z_matrix(params, B_matrix)
% % ============================================================
% % 输出:
% %   Z_Force  : [Jc_F, Ja_F, Jo, Jf]
% %   Z_Torque : [Jc_T, Ja_T, Jo, Jf]
% %   Z_Total  : [Jc_all, Ja_all, Jo, Jf]
% % ============================================================
% 
%     nState = params.Num + 1;
% 
%     Jc = zeros(nState, 2);
%     Ja = zeros(nState, 2);
%     Jo = zeros(nState, 1);
%     Jf = zeros(nState, 1);   % 现在建议 Jf 是整体标量
% 
%     StateNames = cell(nState, 1);
% 
%     for idx = 1:nState
%         if idx == 1
%             eval_fault = [];
%             StateNames{idx} = '正常标况';
%         else
%             eval_fault = idx - 1;
%             StateNames{idx} = sprintf('推力器%02d故障', idx - 1);
%         end
% 
%         [~, Jc(idx, :), ~, Ja(idx, :), Jo(idx), Jf(idx)] = Reconfig_eval(params, B_matrix, eval_fault);
%     end
% 
%     % % Force 子评价矩阵
%     % Z_Force = [Jc(:,1), Ja(:,1), Jo, Jf];
%     % 
%     % % Torque 子评价矩阵
%     % Z_Torque = [Jc(:,2), Ja(:,2), Jo, Jf];
%     % 
%     % % Total 总评价矩阵（平动/转动等权）
%     % Jc_all = 0.5 * Jc(:,1) + 0.5 * Jc(:,2);
%     % Ja_all = 0.5 * Ja(:,1) + 0.5 * Ja(:,2);
%     % 
%     % Z_Total = [Jc_all, Ja_all, Jo, Jf];
%     % Force 子评价矩阵，指标顺序：[Jc, Ja, Jf, Jo]
% Z_Force = [Jc(:,1), Ja(:,1), Jf, Jo];
% 
% % Torque 子评价矩阵，指标顺序：[Jc, Ja, Jf, Jo]
% Z_Torque = [Jc(:,2), Ja(:,2), Jf, Jo];
% 
% % Total 总评价矩阵，指标顺序：[Jc, Ja, Jf, Jo]
% Jc_all = 0.5 * Jc(:,1) + 0.5 * Jc(:,2);
% Ja_all = 0.5 * Ja(:,1) + 0.5 * Ja(:,2);
% 
% Z_Total = [Jc_all, Ja_all, Jf, Jo];
% end
function [Z_Force, Z_Torque, Z_Total, StateNames] = get_Z_matrix(params, B_matrix)
% ============================================================
% 指标顺序统一为：
%   Z_Force  = [Jc_F,   Ja_F,   Jf, Jo]
%   Z_Torque = [Jc_T,   Ja_T,   Jf, Jo]
%   Z_Total  = [Jc_all, Ja_all, Jf, Jo]
% ============================================================

    nState = params.Num + 1;

    Jc = zeros(nState, 2);
    Ja = zeros(nState, 2);
    Jo = zeros(nState, 1);
    Jf = zeros(nState, 1);

    StateNames = cell(nState, 1);

    for idx = 1:nState
        if idx == 1
            eval_fault = [];
            StateNames{idx} = '正常标况';
        else
            eval_fault = idx - 1;
            StateNames{idx} = sprintf('推力器%02d故障', idx - 1);
        end

        [~, Jc(idx, :), ~, Ja(idx, :), Jo(idx), Jf(idx)] = ...
            Reconfig_eval(params, B_matrix, eval_fault);
    end

    Jc_all = 0.5 * Jc(:,1) + 0.5 * Jc(:,2);
    Ja_all = 0.5 * Ja(:,1) + 0.5 * Ja(:,2);

    Z_Force  = [Jc(:,1), Ja(:,1), Jf, Jo];
    Z_Torque = [Jc(:,2), Ja(:,2), Jf, Jo];
    Z_Total  = [Jc_all,  Ja_all,  Jf, Jo];
end

function [F, W, U_ahp, V_entropy] = Comprehensive_Eval(Z, G_AHP)
% ============================================================
% 输入:
%   Z     : 原始指标矩阵，m×n
%   G_AHP : AHP 判断矩阵
%
% 输出:
%   F         : 综合评价值
%   W         : 最终组合权重
%   U_ahp     : AHP主观权重
%   V_entropy : 熵权法客观权重
% ============================================================

    [m, n] = size(Z);

    %% --------------------------------------------------------
    % 1. 规范化（这里默认所有指标均为效益型：越大越优）
    %% --------------------------------------------------------
    X = zeros(m, n);
    for j = 1:n
        z_max = max(Z(:, j));
        z_min = min(Z(:, j));

        if abs(z_max - z_min) < 1e-12
            X(:, j) = 1;
        else
            X(:, j) = (Z(:, j) - z_min) / (z_max - z_min);
        end
    end

    %% --------------------------------------------------------
    % 2. AHP 主观权重
    %% --------------------------------------------------------
    U_ahp = AHP_Weight(G_AHP);

    %% --------------------------------------------------------
    % 3. 熵权法客观权重
    %% --------------------------------------------------------
    V_entropy = zeros(n, 1);

    for j = 1:n
        col_sum = sum(X(:, j));
        if col_sum < 1e-12
            r = ones(m,1) / m;
        else
            r = X(:, j) / col_sum;
        end

        r_valid = r(r > 1e-12);
        E_j = -(1 / log(m)) * sum(r_valid .* log(r_valid));
        V_entropy(j) = 1 - E_j;
    end

    if sum(V_entropy) < 1e-12
        V_entropy = ones(n,1) / n;
    else
        V_entropy = V_entropy / sum(V_entropy);
    end

    %% --------------------------------------------------------
    % 4. 最小二乘组合赋权
    %% --------------------------------------------------------
    s = sum(X.^2, 1)';      % n×1
    A = diag(s);
    B = 0.5 * (U_ahp + V_entropy) .* s;

    e = ones(n, 1);
    Ainv = inv(A);
    W = Ainv * B + ((1 - e' * Ainv * B) / (e' * Ainv * e)) * (Ainv * e);

    % 数值修正
    W(W < 0) = 0;
    if sum(W) < 1e-12
        W = ones(n,1) / n;
    else
        W = W / sum(W);
    end

    %% --------------------------------------------------------
    % 5. TOPSIS 综合评价
    %% --------------------------------------------------------
    x_plus  = max(X, [], 1);
    x_minus = min(X, [], 1);

    L = zeros(m, 1);
    D = zeros(m, 1);
    F = zeros(m, 1);

    for i = 1:m
        % 按胡宇桑论文写法：权重在括号里
        L(i) = sqrt(sum((W' .* (X(i, :) - x_plus)).^2));
        D(i) = sqrt(sum((W' .* (X(i, :) - x_minus)).^2));
        F(i) = D(i) / (L(i) + D(i) + 1e-12);
    end
end

function U = AHP_Weight(G)
% ============================================================
% AHP 主观权重计算（特征向量法）
% ============================================================

    [V_eig, D_eig] = eig(G);
    [~, idx] = max(real(diag(D_eig)));
    U = real(V_eig(:, idx));
    U = U / sum(U);

    % 保证正值
    U = abs(U);
    U = U / sum(U);
end



%% 辅助函数
function [J,Matrix_conf_opt] = Evaluation(params, B_opt)
    J = zeros((params.Num+1)*6,2);
    Jc_opt = zeros(params.Num+1, 2);
    Ja_opt = zeros(params.Num+1, 2);
    Jo_opt = zeros(params.Num+1, 1);
    Jf_opt = zeros(params.Num+1, 1);
    for idx = 1:params.Num+1
        if idx == 1
            eval_fault = [];% 标况
        else
            eval_fault = idx - 1;% 单个推力器故障
        end
        [Matrix_conf_opt,Jc_opt(idx, :),~,Ja_opt(idx, :),Jo_opt(idx, :),Jf_opt(idx, :)] = Reconfig_eval(params, B_opt, eval_fault);
        J((idx-1)*6+1, :) = [params.Jc(idx,1),Jc_opt(idx,1)];
        J((idx-1)*6+2, :) = [params.Jc(idx,2),Jc_opt(idx,2)];
        J((idx-1)*6+3, :) = [params.Ja(idx,1),Ja_opt(idx,1)];
        J((idx-1)*6+4, :) = [params.Ja(idx,2),Ja_opt(idx,2)];
        J((idx-1)*6+5, :) = [params.Jo(idx),Jo_opt(idx)];
        J((idx-1)*6+6, :) = [params.Jf(idx),Jf_opt(idx)];
    end
end

%% 卫星动力学与运动学方程
function dY = Spacecraft_dynamics(Y, params)    
    r = Y(1:3);
    v = Y(4:6);
    sigma = Y(7:9);
    omega = Y(10:12);
    m = params.m; 
    n = params.n; 
    J = params.J;
    F_body = params.current_F;
    T_body = params.current_T;
    % C-W 轨道动力学
    [sigma_f, sigma_c, R_b2o] = Body_to_Orbit(sigma);
    F_orbit = R_b2o * F_body;
    ax = 3*n^2*r(1) + 2*n*v(2) + F_orbit(1)/m;
    ay = -2*n*v(1) + F_orbit(2)/m;
    az = -n^2*r(3) + F_orbit(3)/m;
    dv = [ax; ay; az];
    % 姿态运动学
    G = 0.25 * ((1 - sigma_f) * eye(3) + 2 * sigma_c + 2 * (sigma * sigma'));
    dsigma = G * omega;
    % 姿态动力学
    domega = J \ (-cross(omega, J * omega) + T_body);
    dY = [v; dv; dsigma; domega];
end

%% 五次多项式路径规划
function [r_d, v_d] = Guidance(t, r_start, r_final, T_total)
    tau = min(t / T_total, 1);
    s = 10*tau^3 - 15*tau^4 + 6*tau^5;
    ds = (30*tau^2 - 60*tau^3 + 30*tau^4) / T_total;
    r_d = r_start + (r_final - r_start) * s;
    v_d = (r_final - r_start) * ds;
end

%% 本体系到轨道系旋转矩阵
function [sigma_f, sigma_c, R_b2o] = Body_to_Orbit(sigma)
    sigma_f = dot(sigma, sigma);
    sigma_c = [0, -sigma(3), sigma(2); sigma(3), 0, -sigma(1); -sigma(2), sigma(1), 0];
    R_b2o = eye(3) + (8 * sigma_c^2 - 4 * (1 - sigma_f) * sigma_c) / (1 + sigma_f)^2;
end

%% 欧拉角转换为修正罗得里格斯参数
function sigma = Euler_to_MRPs(euler)
    c1 = cos(euler(3)/2); s1 = sin(euler(3)/2);
    c2 = cos(euler(2)/2); s2 = sin(euler(2)/2);
    c3 = cos(euler(1)/2); s3 = sin(euler(1)/2);
    q0 = c1*c2*c3 + s1*s2*s3;
    q1 = c1*c2*s3 - s1*s2*c3;
    q2 = c1*s2*c3 + s1*c2*s3;
    q3 = s1*c2*c3 - c1*s2*s3;
    sigma = [q1; q2; q3] / (1 + q0);
    if norm(sigma) > 1
        sigma = -sigma / (sigma' * sigma); 
    end
end

%% 修正罗得里格斯参数转为欧拉角
function euler = MRPs_to_Euler(sigma)
    q0 = (1 - sigma' * sigma) / (1 + sigma' * sigma);
    q1 = 2 * sigma(1) / (1 + sigma' * sigma);
    q2 = 2 * sigma(2) / (1 + sigma' * sigma);
    q3 = 2 * sigma(3) / (1 + sigma' * sigma);
    phi = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2));
    theta = asin(2*(q0*q2 - q3*q1));
    psi = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2));
    euler = [phi; theta; psi];
end