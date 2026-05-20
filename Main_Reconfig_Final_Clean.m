clear; clc; close all;

params = Get_params();
data_files = dir('Optim_config_data_*.mat');
if isempty(data_files)
    error('未找到优化结果文件，请先运行 Optim_Algorithm.m。');
end
[~, latest_idx] = max([data_files.datenum]);
latest_file = data_files(latest_idx).name;
load(latest_file, 'B_opt', 'r_opt', 'fval');
fprintf('已加载最新优化结果：%s\n', latest_file);

disp('正在计算 原布局 底层指标...');
[Z_Force_orig, Z_Torque_orig, Z_Total_orig, StateNames] = get_Z_matrix(params, params.B_all);

disp('正在计算 优化后布局 底层指标...');
[Z_Force_opt, Z_Torque_opt, Z_Total_opt, ~] = get_Z_matrix(params, B_opt);

% 原布局和优化布局必须放进同一个评价池，统一规范化、统一赋权、统一TOPSIS
Z_Force_combined  = [Z_Force_orig;  Z_Force_opt];
Z_Torque_combined = [Z_Torque_orig; Z_Torque_opt];
Z_Total_combined  = [Z_Total_orig;  Z_Total_opt];

%% =========================================================
% 2. 布局层综合评价 F_layout
%    布局层指标顺序：[Jc, Ja, Jf, Jo]
% ==========================================================
G_AHP_layout = [
    1,   2,   3,   4;
   1/2, 1,   2,   3;
   1/3, 1/2, 1,   2;
   1/4, 1/3, 1/2, 1
];

[F_Force_combined,  W_Force,  U_Force,  V_Force]  = Comprehensive_Eval(Z_Force_combined,  G_AHP_layout);
[F_Torque_combined, W_Torque, U_Torque, V_Torque] = Comprehensive_Eval(Z_Torque_combined, G_AHP_layout);
[F_Total_combined,  W_Total,  U_Total,  V_Total]  = Comprehensive_Eval(Z_Total_combined,  G_AHP_layout);

nState = params.Num + 1;

F_Force_orig  = F_Force_combined(1:nState);
F_Force_opt   = F_Force_combined(nState+1:end);

F_Torque_orig = F_Torque_combined(1:nState);
F_Torque_opt  = F_Torque_combined(nState+1:end);

F_Total_orig  = F_Total_combined(1:nState);
F_Total_opt   = F_Total_combined(nState+1:end);

Print_Layout_Evaluation(StateNames, F_Force_orig, F_Force_opt, ...
    F_Torque_orig, F_Torque_opt, F_Total_orig, F_Total_opt, ...
    W_Force, W_Torque, W_Total);

%% =========================================================
% 3. 闭环仿真对比：原布局 vs 优化布局
% ==========================================================
sim_cfg = Default_Sim_Config(params);

% 公平对比：原布局和优化布局使用同一故障时间、同一故障推力器
sim_cfg.num_faults = 1;
sim_cfg.fault_time = 0.5 * sim_cfg.T_sim;
sim_cfg.faulty_thrusters_fixed = 3;

fprintf('\n================ 开始原布局闭环仿真 ================\n');
[log_orig, Perf_orig, fault_orig] = Run_ClosedLoop_Sim(params, params.B_all, sim_cfg);

fprintf('\n================ 开始优化布局闭环仿真 ================\n');
[log_opt, Perf_opt, fault_opt] = Run_ClosedLoop_Sim(params, B_opt, sim_cfg);

Compare_ClosedLoop_Performance(Perf_orig, Perf_opt);

%% =========================================================
% 4. 系统层综合评价 F_system
%    系统层指标顺序：[Je, Jc, Ja, Jf, Jo]
% ==========================================================
G_AHP_system = [
    1,    2,    3,    4,    5;
    1/2,  1,    2,    3,    4;
    1/3,  1/2,  1,    2,    3;
    1/4,  1/3,  1/2,  1,    2;
    1/5,  1/4,  1/3,  1/2,  1
];

if sim_cfg.num_faults == 0
    eval_state_idx = 1;  % 标况
else
    eval_state_idx = sim_cfg.faulty_thrusters_fixed(1) + 1;  % 单故障：推力器k故障对应第k+1行
end

% Z_Total列顺序：[Jc, Ja, Jf, Jo]
Z_System = [
    Perf_orig.Precision_Score, Z_Total_orig(eval_state_idx, 1), Z_Total_orig(eval_state_idx, 2), Z_Total_orig(eval_state_idx, 3), Z_Total_orig(eval_state_idx, 4);
    Perf_opt.Precision_Score,  Z_Total_opt(eval_state_idx, 1),  Z_Total_opt(eval_state_idx, 2),  Z_Total_opt(eval_state_idx, 3),  Z_Total_opt(eval_state_idx, 4)
];

[F_System, W_System, U_System, V_System] = Comprehensive_Eval(Z_System, G_AHP_system);

Print_System_Evaluation(StateNames{eval_state_idx}, Z_System, F_System, W_System);

%% =========================================================
% 5. 结果绘图与表格输出
% ==========================================================
Plot_results(params, B_opt, r_opt, ...
             log_orig, log_opt, Perf_orig, Perf_opt, ...
             fault_orig, ...
             Z_Force_orig, Z_Force_opt, ...
             Z_Torque_orig, Z_Torque_opt, ...
             Z_Total_orig, Z_Total_opt, ...
             F_Force_orig, F_Force_opt, ...
             F_Torque_orig, F_Torque_opt, ...
             F_Total_orig, F_Total_opt, ...
             StateNames, eval_state_idx, ...
             Z_System, F_System, W_System);

%% ========================================================================
% 局部函数区
% ========================================================================

function sim_cfg = Default_Sim_Config(params)
    sim_cfg.r0 = [0;15;55];
    sim_cfg.rt = [5;25;35];
    sim_cfg.v0 = [0;0;0];

    sim_cfg.euler0 = deg2rad([0;15;55]);
    sim_cfg.eulert = deg2rad([5;25;35]);

    sim_cfg.T_sim = 2000;
    sim_cfg.dt = 0.005;

    sim_cfg.Kp_pos = 10;
    sim_cfg.Kd_pos = 300;
    sim_cfg.Kp_att = 400 * eye(3);
    sim_cfg.Kd_att = 3200 * eye(3);
    sim_cfg.Ki_att = 20 * eye(3);

    sim_cfg.num_faults = 1;
    sim_cfg.fault_time = 0.5 * sim_cfg.T_sim;
    sim_cfg.faulty_thrusters_fixed = 3;
end

function [Z_Force, Z_Torque, Z_Total, StateNames] = get_Z_matrix(params, B_matrix)
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
% 规范化 + AHP + 熵权法 + 最小二乘组合赋权 + TOPSIS
% 默认所有指标均为效益型：越大越优

    [m, n] = size(Z);

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

    U_ahp = AHP_Weight(G_AHP);

    V_entropy = zeros(n, 1);
    for j = 1:n
        col_sum = sum(X(:, j));
        if col_sum < 1e-12
            r = ones(m, 1) / m;
        else
            r = X(:, j) / col_sum;
        end
        r_valid = r(r > 1e-12);
        E_j = -(1 / log(m)) * sum(r_valid .* log(r_valid));
        V_entropy(j) = 1 - E_j;
    end

    if sum(V_entropy) < 1e-12
        V_entropy = ones(n, 1) / n;
    else
        V_entropy = V_entropy / sum(V_entropy);
    end

    s = sum(X.^2, 1)';
    A = diag(s);
    B = 0.5 * (U_ahp + V_entropy) .* s;

    e = ones(n, 1);
    Ainv = pinv(A);
    W = Ainv * B + ((1 - e' * Ainv * B) / (e' * Ainv * e + 1e-12)) * (Ainv * e);

    W(W < 0) = 0;
    if sum(W) < 1e-12
        W = ones(n, 1) / n;
    else
        W = W / sum(W);
    end

    x_plus  = max(X, [], 1);
    x_minus = min(X, [], 1);

    L = zeros(m, 1);
    D = zeros(m, 1);
    F = zeros(m, 1);

    for i = 1:m
        L(i) = sqrt(sum((W' .* (X(i, :) - x_plus)).^2));
        D(i) = sqrt(sum((W' .* (X(i, :) - x_minus)).^2));
        F(i) = D(i) / (L(i) + D(i) + 1e-12);
    end
end

function U = AHP_Weight(G)
    [V_eig, D_eig] = eig(G);
    [~, idx] = max(real(diag(D_eig)));
    U = abs(real(V_eig(:, idx)));
    U = U / sum(U);
end

function Print_Layout_Evaluation(StateNames, F_Force_orig, F_Force_opt, ...
    F_Torque_orig, F_Torque_opt, F_Total_orig, F_Total_opt, ...
    W_Force, W_Torque, W_Total)

    nState = length(StateNames);

    fprintf('\n==============================================================\n');
    fprintf('布局层最终组合权重（统一评价池下）\n');
    fprintf('指标顺序：[Jc, Ja, Jf, Jo]\n');
    fprintf('--------------------------------------------------------------\n');
    fprintf('Force  权重 = [%s]\n', num2str(W_Force',  '%.4f '));
    fprintf('Torque 权重 = [%s]\n', num2str(W_Torque', '%.4f '));
    fprintf('Total  权重 = [%s]\n', num2str(W_Total',  '%.4f '));
    fprintf('==============================================================\n');

    fprintf('\n==================== F_layout 状态对比 ====================\n');
    fprintf('%-14s | %10s | %10s | %10s\n', '状态', '原布局', '优化布局', '提升(%)');
    for i = 1:nState
        imp = (F_Total_opt(i) - F_Total_orig(i)) / (F_Total_orig(i) + 1e-12) * 100;
        fprintf('%-14s | %10.4f | %10.4f | %+9.2f\n', StateNames{i}, F_Total_orig(i), F_Total_opt(i), imp);
    end

    fprintf('\n==================== 平均布局综合性能 ====================\n');
    fprintf('Force  平均F: 原布局 %.4f, 优化布局 %.4f, 提升 %+6.2f%%\n', ...
        mean(F_Force_orig), mean(F_Force_opt), PercentImprove(mean(F_Force_orig), mean(F_Force_opt), true));
    fprintf('Torque 平均F: 原布局 %.4f, 优化布局 %.4f, 提升 %+6.2f%%\n', ...
        mean(F_Torque_orig), mean(F_Torque_opt), PercentImprove(mean(F_Torque_orig), mean(F_Torque_opt), true));
    fprintf('Total  平均F: 原布局 %.4f, 优化布局 %.4f, 提升 %+6.2f%%\n', ...
        mean(F_Total_orig), mean(F_Total_opt), PercentImprove(mean(F_Total_orig), mean(F_Total_opt), true));
end

function Print_System_Evaluation(state_name, Z_System, F_System, W_System)
    fprintf('\n======================= 最终系统综合评价 =======================\n');
    fprintf('当前系统评价对应状态：%s\n', state_name);
    fprintf('系统层指标顺序：[Je, Jc, Ja, Jf, Jo]\n');
    fprintf('系统层组合权重 W = [%s]\n', num2str(W_System', '%.4f '));
    fprintf('---------------------------------------------------------------\n');
    fprintf('%-12s | %10s | %10s\n', '指标', '原布局', '优化布局');
    fprintf('%-12s | %10.4f | %10.4f\n', 'Je', Z_System(1,1), Z_System(2,1));
    fprintf('%-12s | %10.4f | %10.4f\n', 'Jc', Z_System(1,2), Z_System(2,2));
    fprintf('%-12s | %10.4f | %10.4f\n', 'Ja', Z_System(1,3), Z_System(2,3));
    fprintf('%-12s | %10.4f | %10.4f\n', 'Jf', Z_System(1,4), Z_System(2,4));
    fprintf('%-12s | %10.4f | %10.4f\n', 'Jo', Z_System(1,5), Z_System(2,5));
    fprintf('---------------------------------------------------------------\n');
    fprintf('原布局 F_System   = %.4f\n', F_System(1));
    fprintf('优化布局 F_System = %.4f\n', F_System(2));
    fprintf('系统综合提升      = %+6.2f%%\n', PercentImprove(F_System(1), F_System(2), true));
    fprintf('===============================================================\n');
end

function [log, Perf, fault_info] = Run_ClosedLoop_Sim(params, B, sim_cfg)
    Matrix_conf = params.F_max * B;

    r0 = sim_cfg.r0;
    rt = sim_cfg.rt;
    v0 = sim_cfg.v0;
    euler0 = sim_cfg.euler0;
    eulert = sim_cfg.eulert;

    sigma0 = Euler_to_MRPs(euler0);
    omega0 = [0;0;0];
    Y = [r0; v0; sigma0; omega0];

    Next_Control_Time = 0;
    T_sim = sim_cfg.T_sim;
    dt = sim_cfg.dt;
    N = floor(T_sim / dt);

    int = [0;0;0];
    Kp_pos = sim_cfg.Kp_pos;
    Kd_pos = sim_cfg.Kd_pos;
    Kp_att = sim_cfg.Kp_att;
    Kd_att = sim_cfg.Kd_att;
    Ki_att = sim_cfg.Ki_att;

    faulty_thrusters = [];
    fault_time = sim_cfg.fault_time;
    fault_trig = false;
    num_faults = sim_cfg.num_faults;

    Prop_Final = zeros(params.Num, 1);

    log.Y_euler = zeros(3, N);
    log.Time = zeros(1, N);
    log.R = zeros(3, N);
    log.V = zeros(3, N);
    log.E = zeros(3, N);
    log.O = zeros(3, N);
    log.Y = zeros(12, N);
    log.Pulse_Widths = zeros(params.Num, N);

    max_ctrl = ceil(T_sim / params.T) + 10;
    log.Pulse_Command_History = zeros(params.Num, max_ctrl);
    log.Control_Time = zeros(1, max_ctrl);
    log.Total_Pulse = 0;
    log.Control_Count = 0;

    tic;
    for k = 1:N
        t = (k-1) * dt;

        if t >= fault_time && ~fault_trig
            fault_trig = true;
            if num_faults == 0
                faulty_thrusters = [];
            else
                faulty_thrusters = sim_cfg.faulty_thrusters_fixed;
            end
        end

        if t >= Next_Control_Time
            r = Y(1:3);
            v = Y(4:6);
            sigma = Y(7:9);
            omega = Y(10:12);

            [r_d, v_d] = Guidance(t, r0, rt, T_sim);
            [euler_d, euler_rate_d] = Guidance(t, euler0, eulert, T_sim);

            R = [1, 0, -sin(euler_d(2));
                 0, cos(euler_d(1)), sin(euler_d(1))*cos(euler_d(2));
                 0, -sin(euler_d(1)), cos(euler_d(1))*cos(euler_d(2))];

            omega_d = R * euler_rate_d;
            sigma_d = Euler_to_MRPs(euler_d);

            F_orbit_req = Kp_pos * (r_d - r) + Kd_pos * (v_d - v);
            [~, ~, R_b2o] = Body_to_Orbit(sigma);
            F_body_req = R_b2o' * F_orbit_req;

            sigma_err = ((1-sigma'*sigma)*sigma_d ...
                        -(1-sigma_d'*sigma_d)*sigma ...
                        -2*cross(sigma_d,-sigma)) / ...
                        (1+(sigma_d'*sigma_d)*(sigma'*sigma)-2*dot(sigma_d,-sigma));

            int = int + sigma_err * params.T;
            T_body_req = Kp_att * sigma_err + Kd_att * (omega_d - omega) + Ki_att * int;

            Prop_Final = Thruster_invocation(F_body_req, T_body_req, Matrix_conf, faulty_thrusters, params);

            log.Total_Pulse = log.Total_Pulse + sum(Prop_Final);
            log.Control_Count = log.Control_Count + 1;
            log.Pulse_Command_History(:, log.Control_Count) = Prop_Final;
            log.Control_Time(log.Control_Count) = t;

            Next_Control_Time = Next_Control_Time + params.T;
        end

        time_in_cycle = t - (Next_Control_Time - params.T);
        u_applied = params.F_max * (time_in_cycle < Prop_Final);

        W_total = B * u_applied;
        params.current_F = W_total(1:3);
        params.current_T = W_total(4:6);

        dY = Spacecraft_dynamics(Y, params);
        Y = Y + dY * dt;

        [r_d_log, v_d_log] = Guidance(t, r0, rt, T_sim);
        [euler_d_log, euler_rate_d_log] = Guidance(t, euler0, eulert, T_sim);
        R_log = [1, 0, -sin(euler_d_log(2));
                 0, cos(euler_d_log(1)), sin(euler_d_log(1))*cos(euler_d_log(2));
                 0, -sin(euler_d_log(1)), cos(euler_d_log(1))*cos(euler_d_log(2))];
        omega_d_log = R_log * euler_rate_d_log;

        log.Y_euler(:, k) = MRPs_to_Euler(Y(7:9));
        log.Time(k) = t;
        log.R(:, k) = r_d_log;
        log.V(:, k) = v_d_log;
        log.E(:, k) = euler_d_log;
        log.O(:, k) = omega_d_log;
        log.Y(:, k) = Y;
        log.Pulse_Widths(:, k) = Prop_Final;
    end
    toc;

    log.Pulse_Command_History = log.Pulse_Command_History(:, 1:log.Control_Count);
    log.Control_Time = log.Control_Time(1:log.Control_Count);

    Perf = Evaluate_ClosedLoop_Performance(log);

    fault_info.fault_time = fault_time;
    fault_info.faulty_thrusters = faulty_thrusters;
end

function Perf = Evaluate_ClosedLoop_Performance(log)
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

    pos_err = log.R - log.Y(1:3, :);
    pos_err_norm = vecnorm(pos_err, 2, 1);
    Perf.Pos_RMSE = sqrt(mean(pos_err_norm.^2));
    Perf.Pos_MAE  = mean(pos_err_norm);

    att_err = wrapToPi_local(log.E - log.Y_euler);
    att_err_norm = vecnorm(att_err, 2, 1);
    Perf.Att_RMSE = sqrt(mean(att_err_norm.^2));
    Perf.Att_MAE  = mean(att_err_norm);

    Pos_Score = 1 / (1 + Perf.Pos_RMSE);
    Att_Score = 1 / (1 + Perf.Att_RMSE);
    Perf.Precision_Score = 0.5 * Pos_Score + 0.5 * Att_Score;

    function x = wrapToPi_local(x)
        x = mod(x + pi, 2*pi) - pi;
    end
end

function Compare_ClosedLoop_Performance(Perf_orig, Perf_opt)
    fprintf('\n==============================================================\n');
    fprintf('                 闭环仿真性能对比\n');
    fprintf('==============================================================\n');
    Print_Perf_Row('总喷气时长 Total_Pulse', Perf_orig.Total_Pulse, Perf_opt.Total_Pulse, false);
    Print_Perf_Row('平均每周期喷气时长', Perf_orig.Avg_Pulse_Per_Control, Perf_opt.Avg_Pulse_Per_Control, false);
    Print_Perf_Row('位置 RMSE', Perf_orig.Pos_RMSE, Perf_opt.Pos_RMSE, false);
    Print_Perf_Row('姿态 RMSE', Perf_orig.Att_RMSE, Perf_opt.Att_RMSE, false);
    Print_Perf_Row('位置 MAE', Perf_orig.Pos_MAE, Perf_opt.Pos_MAE, false);
    Print_Perf_Row('姿态 MAE', Perf_orig.Att_MAE, Perf_opt.Att_MAE, false);
    Print_Perf_Row('综合控制精度得分', Perf_orig.Precision_Score, Perf_opt.Precision_Score, true);
    fprintf('==============================================================\n');
end

function Print_Perf_Row(name_str, val_orig, val_opt, is_benefit)
    improve = PercentImprove(val_orig, val_opt, is_benefit);
    fprintf('%-26s | 原布局 = %12.6f | 优化布局 = %12.6f | 改善 = %+8.2f%%\n', ...
        name_str, val_orig, val_opt, improve);
end

function p = PercentImprove(v_orig, v_opt, is_benefit)
    if is_benefit
        p = (v_opt - v_orig) / (abs(v_orig) + 1e-12) * 100;
    else
        p = (v_orig - v_opt) / (abs(v_orig) + 1e-12) * 100;
    end
end

%% ========================= 绘图函数 =========================
function Plot_results(params, B_opt, r_opt, log_orig, log_opt, Perf_orig, Perf_opt, ...
                      fault_info, Z_Force_orig, Z_Force_opt, Z_Torque_orig, Z_Torque_opt, ...
                      Z_Total_orig, Z_Total_opt, F_Force_orig, F_Force_opt, ...
                      F_Torque_orig, F_Torque_opt, F_Total_orig, F_Total_opt, ...
                      StateNames, eval_state_idx, Z_System, F_System, W_System)

    faulty_thrusters = fault_info.faulty_thrusters;
    fault_time = fault_info.fault_time;

    Plot_Layout_Compare(params, params.r_all, params.B_all, r_opt, B_opt, faulty_thrusters);
    Plot_Envelope_PointCloud(params, params.B_all, B_opt, faulty_thrusters);
    Plot_Jo_FullVector(params, params.B_all, B_opt, faulty_thrusters);
    Plot_Index_Compare_AllStates(Z_Total_orig, Z_Total_opt, F_Total_orig, F_Total_opt);
    Plot_F_Compare(F_Force_orig, F_Force_opt, F_Torque_orig, F_Torque_opt, F_Total_orig, F_Total_opt);
    Plot_ClosedLoop_Response(log_orig, log_opt, fault_time);
    Plot_Pulse_Compare(params, log_orig, log_opt, Perf_orig, Perf_opt, fault_time);
    Print_Result_Tables(Z_Total_orig, Z_Total_opt, F_Total_orig, F_Total_opt, ...
        Perf_orig, Perf_opt, StateNames, eval_state_idx, Z_System, F_System, W_System);
end

function Plot_Layout_Compare(params, r_orig, B_orig, r_opt, B_opt, faulty_thrusters)
    figure('Name','推力器布局优化前后对比','Color','w','Position',[100 100 1300 600]);
    subplot(1,2,1); Plot_Thruster_Layout(params, r_orig, B_orig, faulty_thrusters, '优化前推力器布局');
    subplot(1,2,2); Plot_Thruster_Layout(params, r_opt, B_opt, faulty_thrusters, '优化后推力器布局');
end

function Plot_Thruster_Layout(params, r, B, faulty_thrusters, title_str)
    hold on; grid on;
    Lx = 2; Ly = 0.6; Lz = 0.6;
    vert = [-Lx -Ly -Lz; Lx -Ly -Lz; Lx Ly -Lz; -Lx Ly -Lz;
            -Lx -Ly  Lz; Lx -Ly  Lz; Lx Ly  Lz; -Lx Ly  Lz];
    fac = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
    patch('Vertices',vert,'Faces',fac,'FaceColor',[0.8 0.8 0.8], ...
          'FaceAlpha',0.18,'EdgeColor',[0.6 0.6 0.6]);

    healthy_idx = setdiff(1:params.Num, faulty_thrusters);
    plot3(r(1,healthy_idx), r(2,healthy_idx), r(3,healthy_idx), ...
          'o','MarkerSize',7,'MarkerFaceColor',[0.2 0.7 1],'MarkerEdgeColor','k');
    quiver3(r(1,healthy_idx), r(2,healthy_idx), r(3,healthy_idx), ...
            B(1,healthy_idx), B(2,healthy_idx), B(3,healthy_idx), ...
            0.45,'r','LineWidth',1.4);

    if ~isempty(faulty_thrusters)
        plot3(r(1,faulty_thrusters), r(2,faulty_thrusters), r(3,faulty_thrusters), ...
              's','MarkerSize',9,'MarkerFaceColor',[0.5 0.5 0.5],'MarkerEdgeColor','k');
        quiver3(r(1,faulty_thrusters), r(2,faulty_thrusters), r(3,faulty_thrusters), ...
                B(1,faulty_thrusters), B(2,faulty_thrusters), B(3,faulty_thrusters), ...
                0.45,'Color',[0.5 0.5 0.5],'LineStyle','--','LineWidth',1.4);
    end

    for j = 1:params.Num
        text(r(1,j), r(2,j), r(3,j)+0.12, sprintf('%d',j), ...
             'FontSize',9,'FontWeight','bold','Color','k');
    end
    title(title_str);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    axis equal; xlim([-3 3]); ylim([-1.2 1.2]); zlim([-1.2 1.2]);
    set(gca,'XDir','reverse','ZDir','reverse');
    view(3);
end

function Plot_Envelope_PointCloud(params, B_orig, B_opt, faulty_thrusters)
    figure('Name','控制包络与离散点云对比','Color','w','Position',[100 100 1400 650]);
    Matrix_orig = params.F_max * B_orig;
    Matrix_opt  = params.F_max * B_opt;
    healthy_idx = setdiff(1:params.Num, faulty_thrusters);
    M_orig = Matrix_orig(:, healthy_idx);
    M_opt  = Matrix_opt(:, healthy_idx);

    N_h = length(healthy_idx);
    tau = dec2bin(0:2^N_h-1) - '0';
    Pts_orig = M_orig * tau';
    Pts_opt  = M_opt  * tau';

    c = 0.5 * ones(N_h,1);
    G = 0.5 * eye(N_h);
    Z_orig = zonotope(M_orig*c, M_orig*G);
    Z_opt  = zonotope(M_opt*c,  M_opt*G);

    warning('off','all');
    subplot(1,2,1); hold on; grid on;
    plot(Z_orig,[1 2 3],'FaceColor',[0.6 1 0.6],'FaceAlpha',0.15,'EdgeColor',[0 0.6 0]);
    plot(Z_opt, [1 2 3],'FaceColor',[1 0.6 0.6],'FaceAlpha',0.20,'EdgeColor',[0.8 0 0]);
    plot3(Pts_orig(1,:),Pts_orig(2,:),Pts_orig(3,:),'.','Color',[0 0.55 0],'MarkerSize',4);
    plot3(Pts_opt(1,:), Pts_opt(2,:), Pts_opt(3,:),'.','Color',[0.75 0 0],'MarkerSize',4);
    plot3(0,0,0,'ko','MarkerFaceColor','k');
    title('力空间包络与离散点云'); xlabel('Fx'); ylabel('Fy'); zlabel('Fz');
    legend('原布局包络','优化布局包络','原布局离散点','优化布局离散点','原点');
    axis equal; view(3);

    subplot(1,2,2); hold on; grid on;
    plot(Z_orig,[4 5 6],'FaceColor',[0.6 1 0.6],'FaceAlpha',0.15,'EdgeColor',[0 0.6 0]);
    plot(Z_opt, [4 5 6],'FaceColor',[1 0.6 0.6],'FaceAlpha',0.20,'EdgeColor',[0.8 0 0]);
    plot3(Pts_orig(4,:),Pts_orig(5,:),Pts_orig(6,:),'.','Color',[0 0.55 0],'MarkerSize',4);
    plot3(Pts_opt(4,:), Pts_opt(5,:), Pts_opt(6,:),'.','Color',[0.75 0 0],'MarkerSize',4);
    plot3(0,0,0,'ko','MarkerFaceColor','k');
    title('力矩空间包络与离散点云'); xlabel('Mx'); ylabel('My'); zlabel('Mz');
    legend('原布局包络','优化布局包络','原布局离散点','优化布局离散点','原点');
    axis equal; view(3);
    warning('on','all');
end

function Plot_Jo_FullVector(params, B_orig, B_opt, faulty_thrusters)
    if nargin < 4
        faulty_thrusters = [];
    end

    K_orig = params.F_max * B_orig;
    K_opt  = params.F_max * B_opt;
    idx_orig = setdiff(1:params.Num, faulty_thrusters);
    idx_opt  = setdiff(1:params.Num, faulty_thrusters);

    K_orig = K_orig(:, idx_orig);
    K_opt  = K_opt(:, idx_opt);
    K_orig = K_orig ./ (vecnorm(K_orig,2,1) + 1e-12);
    K_opt  = K_opt  ./ (vecnorm(K_opt,2,1) + 1e-12);

    [U_orig,~,~] = svd(K_orig,'econ');
    [U_opt,~,~]  = svd(K_opt,'econ');
    P_orig = U_orig(:,1:3)' * K_orig;
    P_opt  = U_opt(:,1:3)'  * K_opt;

    figure('Name','Jo可诊断性：6维控制向量方向（PCA投影）','Color','w','Position',[100 100 1300 550]);
    % subplot(1,2,1); 
    % Plot_Vector3D(P_orig, idx_orig, '原布局控制向量方向');
    Plot_Jo_6D_Direct(K_orig, K_opt, idx_orig, idx_opt)
    % subplot(1,2,2); 
    % Plot_Vector3D(P_opt,  idx_opt,  '优化布局控制向量方向');
end

function Plot_Vector3D(V, idx, title_str)
    hold on; grid on; axis equal;
    N = size(V,2);
    colors = lines(N);
    for i = 1:N
        quiver3(0,0,0, V(1,i), V(2,i), V(3,i), 0, 'LineWidth',1.8,'Color',colors(i,:));
        text(1.1*V(1,i),1.1*V(2,i),1.1*V(3,i), sprintf('%d', idx(i)), ...
            'FontSize',10,'FontWeight','bold');
    end
    xlabel('主方向1'); ylabel('主方向2'); zlabel('主方向3');
    title(title_str); view(35,25);
end
function Plot_Jo_6D_Direct(K_orig, K_opt, idx_orig, idx_opt)
    % 确保输入已经是归一化的单位 6 维向量 (你的原代码已经做了这一步)
    
    % ==========================================
    % 1. 平行坐标图 (Parallel Coordinates Plot)
    % ==========================================
    figure('Name','6维控制向量：平行坐标图','Color','w','Position',[150 150 1000 400]);
    
    % 构造维度标签
    dimLabels = {'Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz'}; 
    
    subplot(1,2,1);
    parallelplot(K_orig', 'CoordinateTickLabels', dimLabels);
    title('原布局：6维分量分布');
    
    subplot(1,2,2);
    parallelplot(K_opt', 'CoordinateTickLabels', dimLabels);
    title('优化布局：6维分量分布');
    
    % ==========================================
    % 2. 推进器两两之间的夹角热力图 (可诊断性核心)
    % ==========================================
    % 由于 K 已经是单位向量，矩阵转置相乘 K' * K 就是两两之间的余弦相似度
    cos_orig = K_orig' * K_orig;
    cos_opt  = K_opt'  * K_opt;
    
    % 防止浮点数误差导致越界，将其限制在 [-1, 1] 内，然后算反余弦得到角度
    angle_orig = acos(min(max(cos_orig, -1), 1)) * 180 / pi;
    angle_opt  = acos(min(max(cos_opt,  -1), 1)) * 180 / pi;
    
    figure('Name','可诊断性：推进器方向夹角热力图','Color','w','Position',[200 200 1000 450]);
    
    subplot(1,2,1);
    h1 = heatmap(idx_orig, idx_orig, angle_orig);
    h1.Title = '原布局：向量夹角(度)';
    h1.XLabel = '推进器编号'; h1.YLabel = '推进器编号';
    colormap(h1, jet); % 使用彩虹色带，更容易看出角度大小差异
    
    subplot(1,2,2);
    h2 = heatmap(idx_opt, idx_opt, angle_opt);
    h2.Title = '优化布局：向量夹角(度)';
    h2.XLabel = '推进器编号'; h2.YLabel = '推进器编号';
    colormap(h2, jet);
    
    % 对于可诊断性，我们要尽量避免出现夹角接近 0 或 180 的情况
    % 在热力图上，如果优化后的对角线以外区域，深蓝色(0度)和深红色(180度)变少了，
    % 就直接证明了优化布局的可诊断性更好！
end

function Plot_Index_Compare_AllStates(Z_orig, Z_opt, F_orig, F_opt)
    idx = 0:length(F_orig)-1;
    figure('Name','所有故障状态下综合指标对比','Color','w','Position',[80 80 1400 800]);
    names = {'Jc_{all}','Ja_{all}','Jf','Jo','F_{layout}'};
    data_orig = [Z_orig, F_orig];
    data_opt  = [Z_opt,  F_opt];

    for j = 1:5
        subplot(3,2,j); hold on; grid on;
        bar(idx, [data_orig(:,j), data_opt(:,j)], 'grouped');
        title([names{j} ' 对比']);
        xlabel('故障状态编号'); ylabel(names{j});
        legend('原布局','优化布局','Location','best');
        xticks(idx);
    end

    subplot(3,2,6); hold on; grid on;
    improvement = (F_opt - F_orig) ./ (abs(F_orig) + 1e-12) * 100;
    bar(idx, improvement); yline(0,'--k');
    title('F_{layout} 提升率'); xlabel('故障状态编号'); ylabel('提升率 (%)');
    xticks(idx);
end

function Plot_F_Compare(F_Force_orig, F_Force_opt, F_Torque_orig, F_Torque_opt, F_Total_orig, F_Total_opt)
    idx = 0:length(F_Total_orig)-1;
    figure('Name','Force / Torque / Total 综合评价值对比','Color','w','Position',[100 100 1400 700]);

    subplot(3,1,1); hold on; grid on;
    bar(idx,[F_Force_orig,F_Force_opt],'grouped');
    title('Force 综合评价值 F_{Force}'); ylabel('F'); legend('原布局','优化布局'); xticks(idx);

    subplot(3,1,2); hold on; grid on;
    bar(idx,[F_Torque_orig,F_Torque_opt],'grouped');
    title('Torque 综合评价值 F_{Torque}'); ylabel('F'); legend('原布局','优化布局'); xticks(idx);

    subplot(3,1,3); hold on; grid on;
    bar(idx,[F_Total_orig,F_Total_opt],'grouped');
    title('Total 综合评价值 F_{layout}'); xlabel('故障状态编号'); ylabel('F'); legend('原布局','优化布局'); xticks(idx);
end

function Plot_ClosedLoop_Response(log_orig, log_opt, fault_time)
    t = log_orig.Time;
    figure('Name','闭环位置与姿态响应对比','Color','w','Position',[80 80 1450 850]);

    subplot(2,2,1); Plot_3Axis(t, log_orig.R, log_orig.Y(1:3,:), log_opt.Y(1:3,:), '位置响应', 'm', fault_time);
    subplot(2,2,2); Plot_3Axis(t, log_orig.E, log_orig.Y_euler, log_opt.Y_euler, '姿态欧拉角响应', 'rad', fault_time);

    pos_err_orig = log_orig.Y(1:3,:) - log_orig.R;
    pos_err_opt  = log_opt.Y(1:3,:)  - log_opt.R;
    att_err_orig = wrapToPi_local(log_orig.Y_euler - log_orig.E);
    att_err_opt  = wrapToPi_local(log_opt.Y_euler  - log_opt.E);

    subplot(2,2,3); hold on; grid on;
    plot(t, vecnorm(pos_err_orig,2,1), 'LineWidth',1.2);
    plot(t, vecnorm(pos_err_opt,2,1),  'LineWidth',1.2);
    xline(fault_time,'--r');
    title('位置误差范数'); xlabel('时间 (s)'); ylabel('||e_r|| (m)'); legend('原布局','优化布局');

    subplot(2,2,4); hold on; grid on;
    plot(t, vecnorm(att_err_orig,2,1), 'LineWidth',1.2);
    plot(t, vecnorm(att_err_opt,2,1),  'LineWidth',1.2);
    xline(fault_time,'--r');
    title('姿态误差范数'); xlabel('时间 (s)'); ylabel('||e_\theta|| (rad)'); legend('原布局','优化布局');

    function Plot_3Axis(t, ref, y_orig, y_opt, title_str, unit_str, fault_time)
        hold on; grid on;
        colors = lines(3);
        for ii = 1:3
            plot(t, ref(ii,:), '--', 'Color', colors(ii,:), 'LineWidth',0.9);
            plot(t, y_orig(ii,:), '-',  'Color', colors(ii,:), 'LineWidth',1.1);
            plot(t, y_opt(ii,:),  ':',  'Color', colors(ii,:), 'LineWidth',1.6);
        end
        xline(fault_time,'--r');
        title(title_str); xlabel('时间 (s)'); ylabel(unit_str);
        legend('ref-X','orig-X','opt-X','ref-Y','orig-Y','opt-Y','ref-Z','orig-Z','opt-Z','Location','bestoutside');
    end

    function x = wrapToPi_local(x)
        x = mod(x + pi, 2*pi) - pi;
    end
end

function Plot_Pulse_Compare(params, log_orig, log_opt, Perf_orig, Perf_opt, fault_time)
    figure('Name','总喷气时长与推力器脉宽对比','Color','w','Position',[100 100 1400 800]);

    subplot(2,2,1); hold on; grid on;
    data = [Perf_orig.Total_Pulse, Perf_opt.Total_Pulse];
    bar(1:2, data); xticks(1:2); xticklabels({'原布局','优化布局'});
    ylabel('总喷气时长 (s)'); title('全任务总喷气时长对比'); ylim([0, max(data)*1.2 + eps]);

    subplot(2,2,2); hold on; grid on;
    data = [Perf_orig.Avg_Pulse_Per_Control, Perf_opt.Avg_Pulse_Per_Control];
    bar(1:2, data); xticks(1:2); xticklabels({'原布局','优化布局'});
    ylabel('平均每周期喷气时长 (s)'); title('平均喷气时长对比'); ylim([0, max(data)*1.2 + eps]);

    subplot(2,2,3); hold on; grid on;
    total_per_thr_orig = sum(log_orig.Pulse_Command_History, 2);
    total_per_thr_opt  = sum(log_opt.Pulse_Command_History, 2);
    bar(1:params.Num, [total_per_thr_orig, total_per_thr_opt], 'grouped');
    xlabel('推力器编号'); ylabel('累计脉宽 (s)'); title('各推力器累计喷气时长对比');
    legend('原布局','优化布局');

    subplot(2,2,4); hold on; grid on;
    plot(log_orig.Control_Time, sum(log_orig.Pulse_Command_History,1), 'LineWidth',1.0);
    plot(log_opt.Control_Time,  sum(log_opt.Pulse_Command_History,1),  'LineWidth',1.0);
    xline(fault_time,'--r');
    xlabel('时间 (s)'); ylabel('当前控制周期总脉宽 (s)'); title('控制周期总脉宽变化');
    legend('原布局','优化布局');
end

function Print_Result_Tables(Z_orig, Z_opt, F_orig, F_opt, Perf_orig, Perf_opt, ...
    StateNames, eval_state_idx, Z_System, F_System, W_System)

    fprintf('\n==================== 不同状态布局指标对比表 ====================\n');
    fprintf('%-12s | %8s %8s %8s %8s %8s || %8s %8s %8s %8s %8s\n', ...
        '状态','Jc_o','Ja_o','Jf_o','Jo_o','F_o','Jc_p','Ja_p','Jf_p','Jo_p','F_p');

    for i = 1:length(F_orig)
        fprintf('%-12s | %8.4f %8.4f %8.4f %8.4f %8.4f || %8.4f %8.4f %8.4f %8.4f %8.4f\n', ...
            StateNames{i}, ...
            Z_orig(i,1), Z_orig(i,2), Z_orig(i,3), Z_orig(i,4), F_orig(i), ...
            Z_opt(i,1),  Z_opt(i,2),  Z_opt(i,3),  Z_opt(i,4),  F_opt(i));
    end
    fprintf('================================================================\n');

    fprintf('\n==================== 闭环仿真性能对比表 ====================\n');
    fprintf('%-28s | %12s | %12s | %12s\n', '指标','原布局','优化布局','改善率(%)');
    Print_Perf_Row('总喷气时长', Perf_orig.Total_Pulse, Perf_opt.Total_Pulse, false);
    Print_Perf_Row('平均每周期喷气时长', Perf_orig.Avg_Pulse_Per_Control, Perf_opt.Avg_Pulse_Per_Control, false);
    Print_Perf_Row('位置 RMSE', Perf_orig.Pos_RMSE, Perf_opt.Pos_RMSE, false);
    Print_Perf_Row('姿态 RMSE', Perf_orig.Att_RMSE, Perf_opt.Att_RMSE, false);
    Print_Perf_Row('综合控制精度得分 Je', Perf_orig.Precision_Score, Perf_opt.Precision_Score, true);
    fprintf('============================================================\n');

    fprintf('\n==================== 系统层综合评价表 ====================\n');
    fprintf('当前评价对应状态：%s\n', StateNames{eval_state_idx});
    fprintf('系统层指标顺序：[Je, Jc, Ja, Jf, Jo]\n');
    fprintf('系统层权重：[%s]\n', num2str(W_System', '%.4f '));
    fprintf('----------------------------------------------------------\n');
    fprintf('%-12s | %10s | %10s\n', '指标', '原布局', '优化布局');
    fprintf('%-12s | %10.4f | %10.4f\n', 'Je', Z_System(1,1), Z_System(2,1));
    fprintf('%-12s | %10.4f | %10.4f\n', 'Jc', Z_System(1,2), Z_System(2,2));
    fprintf('%-12s | %10.4f | %10.4f\n', 'Ja', Z_System(1,3), Z_System(2,3));
    fprintf('%-12s | %10.4f | %10.4f\n', 'Jf', Z_System(1,4), Z_System(2,4));
    fprintf('%-12s | %10.4f | %10.4f\n', 'Jo', Z_System(1,5), Z_System(2,5));
    fprintf('----------------------------------------------------------\n');
    fprintf('%-12s | %10.4f | %10.4f\n', 'F_System', F_System(1), F_System(2));
    fprintf('==========================================================\n');
end

%% ========================= 动力学与姿态辅助函数 =========================
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

    [sigma_f, sigma_c, R_b2o] = Body_to_Orbit(sigma);
    F_orbit = R_b2o * F_body;

    ax = 3*n^2*r(1) + 2*n*v(2) + F_orbit(1)/m;
    ay = -2*n*v(1) + F_orbit(2)/m;
    az = -n^2*r(3) + F_orbit(3)/m;
    dv = [ax; ay; az];

    G = 0.25 * ((1 - sigma_f) * eye(3) + 2 * sigma_c + 2 * (sigma * sigma'));
    dsigma = G * omega;
    domega = J \ (-cross(omega, J * omega) + T_body);

    dY = [v; dv; dsigma; domega];
end

function [r_d, v_d] = Guidance(t, r_start, r_final, T_total)
    tau = min(t / T_total, 1);
    s = 10*tau^3 - 15*tau^4 + 6*tau^5;
    ds = (30*tau^2 - 60*tau^3 + 30*tau^4) / T_total;
    r_d = r_start + (r_final - r_start) * s;
    v_d = (r_final - r_start) * ds;
end

function [sigma_f, sigma_c, R_b2o] = Body_to_Orbit(sigma)
    sigma_f = dot(sigma, sigma);
    sigma_c = [0, -sigma(3), sigma(2); sigma(3), 0, -sigma(1); -sigma(2), sigma(1), 0];
    R_b2o = eye(3) + (8 * sigma_c^2 - 4 * (1 - sigma_f) * sigma_c) / (1 + sigma_f)^2;
end

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
