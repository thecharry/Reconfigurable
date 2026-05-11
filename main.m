%% 主程序
clear; clc; close all;

params = Get_params();
load('Optim_config_data1.mat', 'B_opt', 'r_opt', 'fval');

log_orig = closedloop(params, params.B_all);
log_opt = closedloop(params, B_opt);

Plot_results(log_orig, log_opt, params, B_opt, r_opt);

%% 闭环仿真函数
function log = closedloop(params, B)
    % 初始化 
    r0 = [0;15;55];
    rt = [5;25;35];
    v0 = [0; 0; 0];
    euler0 = deg2rad([0;15;55]);
    eulert = deg2rad([5;25;35]);
    sigma0 = Euler_to_MRPs(euler0);
    omega0 = [0;0;0];
    Y = [r0; v0; sigma0; omega0];

    next_ctrl = 0;
    T_sim = 2000;% 仿真时间
    dt = 0.005;
    N = floor(T_sim / dt);
    max_ctrl = ceil(T_sim / params.T) + 10;
    Prop_Final = zeros(params.Num, 1);
    Matrix_conf = params.F_max * B;
    fault_trig = false;% 故障标志位
    num_faults = 1;% 允许的最大同时故障台数

    int = [0;0;0];
    Kp_pos = 10;
    Kd_pos = 300;
    Kp_att = 400 * eye(3);
    Kd_att = 3200 * eye(3);
    Ki_att = 20 * eye(3);

    log.Y_euler = zeros(3, N);
    log.Time = zeros(1, N);
    log.R = zeros(3, N);
    log.V = zeros(3, N);
    log.E = zeros(3, N);
    log.O = zeros(3, N);
    log.Y = zeros(12, N);
    log.Pulse_Widths = zeros(params.Num, N);% 
    log.Pulse_History = zeros(params.Num, max_ctrl);
    log.Control_Time = zeros(1, max_ctrl);
    log.Total_Pulse = 0;% 整个任务累计总喷气时长
    log.Control_Count = 0;% 控制更新次数
    log.faulty_thrusters = [];% 空数组表示推力器无故障
    % log.falut_time = rand() * T_sim;
    log.falut_time = 0.5*T_sim;

    tic;
    for k = 1:N
        t = (k-1) * dt;
        if t >= log.falut_time && ~fault_trig
            fault_trig = true;
            if num_faults == 0
                log.faulty_thrusters = [];% 标况
            else
                % faulty_thrusters = randperm(params.Num, num_faults);
                log.faulty_thrusters = 3;
            end
        end
        if t >= next_ctrl
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
            Prop_Final = Thruster_invocation(F_body_req,T_body_req,Matrix_conf,log.faulty_thrusters,params);
            % 累计真实总喷气时长
            log.Total_Pulse = log.Total_Pulse + sum(Prop_Final);
            log.Control_Count = log.Control_Count + 1;
            log.Pulse_History(:, log.Control_Count) = Prop_Final;
            log.Control_Time(log.Control_Count) = t;
            % 更新时间
            next_ctrl = next_ctrl + params.T;
        end
        time_in_cycle = t - (next_ctrl - params.T);
        u_applied = params.F_max * (time_in_cycle < Prop_Final);
        % 实际力和力矩
        W_total = B * u_applied;
        params.current_F = W_total(1:3);
        params.current_T = W_total(4:6);
        % 动力学积分
        dY = Spacecraft_dynamics(Y, params);
        Y = Y + dY * dt;
        % 数据记录
        log.Y_euler(:,k) = MRPs_to_Euler(Y(7:9,:));
        log.Time(k) = t;
        log.R(:,k) = r_d;
        log.V(:,k) = v_d;
        log.E(:,k) = euler_d;
        log.O(:,k) = omega_d;
        log.Y(:,k) = Y;
        log.Pulse_Widths(1:12,k) = Prop_Final;
    end
    toc;
    log.Pulse_History = log.Pulse_History(:, 1:log.Control_Count);
    log.Control_Time = log.Control_Time(1:log.Control_Count);
end



% function Perf = Evaluate_ClosedLoop_Performance(log)
%     % 总喷气时长
%     if isfield(log, 'Total_Pulse')
%         Perf.Total_Pulse = log.Total_Pulse;
%     else
%         Perf.Total_Pulse = NaN;
%     end
    
%     if isfield(log, 'Control_Count') && log.Control_Count > 0
%         Perf.Avg_Pulse_Per_Control = log.Total_Pulse / log.Control_Count;
%     else
%         Perf.Avg_Pulse_Per_Control = NaN;
%     end
    
%     % 位置误差
%     r_real = log.Y(1:3, :);
%     r_ref  = log.R;
    
%     pos_err = r_ref - r_real;
%     pos_err_norm = vecnorm(pos_err, 2, 1);
    
%     Perf.Pos_RMSE = sqrt(mean(pos_err_norm.^2));
%     Perf.Pos_MAE  = mean(pos_err_norm);
    
%     % 姿态误差
%     euler_real = log.Y_euler;
%     euler_ref  = log.E;
    
%     att_err = euler_ref - euler_real;
%     att_err = wrapToPi_local(att_err);
%     att_err_norm = vecnorm(att_err, 2, 1);
    
%     Perf.Att_RMSE = sqrt(mean(att_err_norm.^2));
%     Perf.Att_MAE  = mean(att_err_norm);
    
%     % 控制精度得分
%     Pos_Score = 1 / (1 + Perf.Pos_RMSE);
%     Att_Score = 1 / (1 + Perf.Att_RMSE);
    
%     Perf.Precision_Score = 0.5 * Pos_Score + 0.5 * Att_Score;

%     function x = wrapToPi_local(x)
%         x = mod(x + pi, 2*pi) - pi;
%     end
% end

%% 原始指标矩阵
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
%% 综合评价指标处理
function [F, W, U_ahp, V_entropy] = Comprehensive_Eval(Z, G_AHP)
    [m, n] = size(Z);
    % 规范化处理
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
    % AHP主观权重
    U_ahp = AHP_Weight(G_AHP);
    % 熵权法客观权重
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

    % 最小二乘组合赋权
    s = sum(X.^2, 1)';
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

    % TOPSIS 综合评价
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
    function U = AHP_Weight(G)
        % AHP 主观权重计算（特征向量法）

        [V_eig, D_eig] = eig(G);
        [~, idx] = max(real(diag(D_eig)));
        U = real(V_eig(:, idx));
        U = U / sum(U);

        % 保证正值
        U = abs(U);
        U = U / sum(U);
    end
end
%% 输出综合评价指标数据
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