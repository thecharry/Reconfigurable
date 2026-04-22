%% 主程序：姿轨控推力器系统可重构性仿真与综合评价
clear; clc; close all;

% 获取参数 (假设你已有 Get_params 函数)
params = Get_params();
load('Optim_config_data1.mat', 'B_opt', 'r_opt', 'fval');

% 初始化指标矩阵
Jc_opt = zeros(params.Num + 1, 2);
Ja_opt = zeros(params.Num + 1, 2);
Jo_opt = zeros(params.Num + 1, 1);
Jf_opt = zeros(params.Num + 1, 2);

disp('正在进行 标况 及 12种单点故障 的能力与效能遍历评估...');
for idx = 1:params.Num+1
    if idx == 1
        eval_fault = []; % 标况
    else
        eval_fault = idx - 1; % 单个推力器故障
    end
    % 调用更新后的 Reconfig_eval，新增了 Jf_opt 输出
    [Matrix_conf_opt, Jc_opt(idx, :), ~, Ja_opt(idx, :), Jo_opt(idx, :), Jf_opt(idx, :)] = Reconfig_eval(params, B_opt, eval_fault);
end
disp('底层指标计算完成！');

%% ==========================================================
%% 🌟 核心融合：多属性决策综合评价 (AHP + 熵权法 + TOPSIS)
%% ==========================================================

% 1. 构造 AHP 专家判断矩阵 G (按照 Jc, Ja, Jo, Jf 的顺序)
% Jc(能力) 最重要，Ja(精度) 和 Jf(能耗) 其次且同等重要，Jo(诊断) 最次
G_AHP = [
    1,    3,    5,    3;   % Jc
   1/3,   1,    3,    1;   % Ja
   1/5,  1/3,   1,   1/3;  % Jo
   1/3,   1,    3,    1    % Jf
];

% 2. 提取评价矩阵 Z (13个状态 x 4个指标)
% 由于姿态(Torque)和轨道(Force)的重要程度不同，我们分开评价
Z_Force  = [Jc_opt(:,1), Ja_opt(:,1), Jo_opt, Jf_opt(:,1)]; % 轨道控制重构健康度
Z_Torque = [Jc_opt(:,2), Ja_opt(:,2), Jo_opt, Jf_opt(:,2)]; % 姿态控制重构健康度

Z_Force_orig  = [params.Jc(:,1), params.Ja(:,1), params.Jo, params.Jf(:,1)];
Z_Torque_orig = [params.Jc(:,2), params.Ja(:,2), params.Jo, params.Jf(:,2)];

Z_Force_combined  = [Z_Force_orig;  Z_Force];
Z_Torque_combined = [Z_Torque_orig; Z_Torque];

% 3. 调用综合评价算法
% [F_score_Force, W_Force]   = Comprehensive_Eval(Z_Force_combined, G_AHP);
% [F_score_Torque, W_Torque] = Comprehensive_Eval(Z_Torque_combined, G_AHP);
[F_Force_combined, ~]   = Evaluation(Z_Force_combined, G_AHP);
[F_Torque_combined, ~] = Evaluation(Z_Torque_combined, G_AHP);

F_Force_orig = F_Force_combined(1:13);
F_Force_opt  = F_Force_combined(14:26);

F_Torque_orig = F_Torque_combined(1:13);
F_Torque_opt  = F_Torque_combined(14:26);

% --- 打印直观的对比结果 ---
disp('================================================================');
disp('   姿态控制(力矩) 综合健康度 Fi 对比 (统一基准下)');
disp('================================================================');
disp('状态编号     | 原布局 Fi   | 优化布局 Fi | 性能提升(%)');
for i = 1:params.Num+1
    if i == 1
        state_str = '正常标况';
    else
        state_str = sprintf('推力器%02d故障', i-1);
    end
    
    % 计算提升百分比
    improvement = (F_Torque_opt(i) - F_Torque_orig(i)) / (F_Torque_orig(i) + 1e-6) * 100;
    
    fprintf('%-10s | %10.4f | %10.4f | %+8.2f%%\n', ...
        state_str, F_Torque_orig(i), F_Torque_opt(i), improvement);
end
disp('================================================================');

% (选做) 你可以直接画个对比柱状图放在你的论文里
figure('Name', '可重构健康度对比');
bar(1:13, [F_Torque_orig, F_Torque_opt]);
legend('原布局', '优化后布局');
title('各单点故障下姿态控制综合健康度(Fi)对比');
xlabel('故障状态 (1为标况, 2-13为对应推力器故障)');
ylabel('综合重构健康度 Fi');

% % 打印结果
% disp('----------------------------------------------------');
% disp('主客观综合权重 W (依次为 Jc, Ja, Jo, Jf):');
% disp(['轨道(力) 权重: ', num2str(W_Force')]);
% disp(['姿态(力矩)权重: ', num2str(W_Torque')]);
% disp('----------------------------------------------------');
% disp('各状态综合健康度得分 F (0~1之间，越大越好):');
% disp('状态编号 | 轨道控制健康度 | 姿态控制健康度');
% for i = 1:params.Num+1
%     if i == 1
%         state_str = '正常标况';
%     else
%         state_str = sprintf('推力器%02d故障', i-1);
%     end
%     fprintf('%-8s | %10.4f | %10.4f\n', state_str, F_score_Force(i), F_score_Torque(i));
% end
% disp('----------------------------------------------------');

%% ==========================================================
%% 以下为原有的动态仿真代码 (主循环部分保持不变)
%% ==========================================================

B = B_opt;
Matrix_conf = Matrix_conf_opt;
% 初始化状态
r0 = [0;15;55]; rt = [5;25;35];
v0 = [0; 0; 0];
euler0 = deg2rad([0;15;55]); eulert = deg2rad([5;25;35]);
sigma0 = Euler_to_MRPs(euler0); omega0 = [0; 0; 0];
Y = [r0; v0; sigma0; omega0];

Next_Control_Time = 0;
T_sim = 2000; dt = 0.005; N = floor(T_sim / dt);
int = [0;0;0];
Kp_pos = 10; Kd_pos = 300;
Kp_att = 400 * eye(3); Kd_att = 3200 * eye(3); Ki_att = 20 * eye(3);

faulty_thrusters = [];
falut_time = rand() * T_sim;
fault_trig = false; num_faults = 1;

log.Y_euler = zeros(3, N); log.Time = zeros(1, N);
log.R = zeros(3, N); log.V = zeros(3, N);
log.E = zeros(3, N); log.O = zeros(3, N);
log.Y = zeros(12, N); log.Pulse_Widths = zeros(12, N);

tic;
disp('开始动力学仿真...');
for k = 1:N
    t = (k-1) * dt;
    if t >= falut_time && ~fault_trig
        fault_trig = true;
        if num_faults ~= 0
            faulty_thrusters = randperm(params.Num, num_faults);
            disp(['时间 ', num2str(t), 's 注入故障: 推力器 ', num2str(faulty_thrusters)]);
        end
    end
    if t >= Next_Control_Time
        r = Y(1:3); v = Y(4:6); sigma = Y(7:9); omega = Y(10:12);
        [r_d, v_d] = Guidance(t, r0, rt, T_sim);
        [euler_d, euler_rate_d] = Guidance(t, euler0, eulert, T_sim);
        R = [1, 0, -sin(euler_d(2));
             0, cos(euler_d(1)), sin(euler_d(1))*cos(euler_d(2));
             0, -sin(euler_d(1)), cos(euler_d(1))*cos(euler_d(2))];
        omega_d = R * euler_rate_d; 
        sigma_d = Euler_to_MRPs(euler_d);
        
        F_orbit_req =  Kp_pos * (r_d - r) + Kd_pos * (v_d - v);
        [~, ~, R_b2o] = Body_to_Orbit(sigma);
        F_body_req = R_b2o' * F_orbit_req;
        
        sigma_err = ((1-sigma'*sigma)*sigma_d-(1-sigma_d'*sigma_d)*sigma-2*cross(sigma_d,-sigma))/...
                    (1+(sigma_d'*sigma_d)*(sigma'*sigma)-2*dot(sigma_d,-sigma));
        int = int + sigma_err * params.T; 
        T_body_req = Kp_att * sigma_err + Kd_att * (omega_d - omega)+ Ki_att * int;
        
        u_F_opt = Thruster_invocation(F_body_req, Matrix_conf(1:3, :), faulty_thrusters);
        u_T_opt = Thruster_invocation(T_body_req, Matrix_conf(4:6, :), faulty_thrusters);
        
        Prop_F = u_F_opt * params.T;
        Prop_T = u_T_opt * params.T;
        Prop_Final = Thruster_reuse(Prop_T, Prop_F, params);
        
        Next_Control_Time = Next_Control_Time + params.T;
    end
    time_in_cycle = t - (Next_Control_Time - params.T);
    u_applied = params.F_max * (time_in_cycle < Prop_Final);
    
    W_total = B * u_applied;
    params.current_F = W_total(1:3);
    params.current_T = W_total(4:6);
    
    dY = Spacecraft_dynamics(Y, params);
    Y = Y + dY * dt;  
    
    log.Y_euler(1:3,k) = MRPs_to_Euler(Y(7:9,:));
    log.Time(k) = t; log.R(1:3,k) = r_d; log.V(1:3,k) = v_d;
    log.E(1:3,k) = euler_d; log.O(1:3,k) = omega_d;
    log.Y(1:12,k) = Y; log.Pulse_Widths(1:12,k) = Prop_Final;
end
toc;
% Plot_results(faulty_thrusters, J, log, params, B_opt, r_opt, B, falut_time);


%% ==========================================================
%% 底层指标计算函数 (包含 Jc, Ja, Jo, Jf)
%% ==========================================================
function [Matrix_conf, Jc1, Jc2, Ja, Jo, Jf_out] = Reconfig_eval(params, Ball, faulty_thrusters)
    Matrix_conf = params.F_max * Ball;
    healthy_idx = setdiff(1:params.Num, faulty_thrusters);
    
    Matrix_conf_F = Matrix_conf(1:3, healthy_idx);
    Matrix_conf_T = Matrix_conf(4:6, healthy_idx);
    Matrix_conf_faulty = Matrix_conf(:, healthy_idx); 
    
    % 1. 控制能力 Jc
    [Jc_Force, Jc_Force1] = Capability(Matrix_conf_F);
    [Jc_Torque, Jc_Torque1] = Capability(Matrix_conf_T);
    
    % 2. 精度降级 Ja
    Ja_Force = Precision(Matrix_conf(1:3, :), Matrix_conf_F, params.t_min);
    Ja_Torque = Precision(Matrix_conf(4:6, :), Matrix_conf_T, params.t_min);
    
    % 3. 可诊断性 Jo
    Jo = Diagnosability(Matrix_conf_faulty);
    
    % 4. 燃料效能 Jf (简化版 1-范数评估)
    % 设定标准测试载荷 (根据你的卫星质量和转动惯量调整量级)
    test_Force = 1.0;  % 1 N
    test_Torque = 0.1; % 0.1 Nm
    Jf_Force = Fuel_Efficiency_Simple(Matrix_conf_F, test_Force);
    Jf_Torque = Fuel_Efficiency_Simple(Matrix_conf_T, test_Torque);
    
    Jc1 = [Jc_Force', Jc_Torque'];
    Jc2 = [Jc_Force1', Jc_Torque1'];
    Ja = [Ja_Force', Ja_Torque'];
    Jo = Jo';
    Jf_out = [Jf_Force, Jf_Torque];
    
    % --- 内部指标计算函数 ---
    function Jo = Diagnosability(Matrix_sub)
        N_sub = size(Matrix_sub, 2);
        if N_sub < 2
            Jo = 0; return;
        end
        K_nor = Matrix_sub ./ vecnorm(Matrix_sub, 2, 1);
        CosSim_Matrix = K_nor' * K_nor; 
        CosSim_Matrix(logical(eye(N_sub))) = -2; 
        max_cos = max(CosSim_Matrix, [], 'all'); 
        Jo = 1 - max_cos;
    end

    function [umin, umax] = Capability(Matrix_sub)
        N_sub = size(Matrix_sub, 2);
        if N_sub == 0
            umin = 0; umax = 0; return;
        end
        c = 0.5 * ones(N_sub, 1); 
        G = 0.5 * diag(ones(N_sub, 1));
        Z = zonotope(Matrix_sub * c, Matrix_sub * G);
        P = polytope(Z); 
        H = P.A; w = P.b;
        if any(w < -1e-6)
            umin = 0; umax = 0;
        else
            d = w ./ vecnorm(H, 2, 2); 
            umin = min(d); umax = max(d);
        end
    end

    function Ja = Precision(Matrix_healthy, Matrix_faulty, t_min)
        D_ideal = Min_steps(Matrix_healthy, t_min);
        D_actual = Min_steps(Matrix_faulty, t_min);
        J_axes = zeros(1, 6);
        for k = 1:6
            if isinf(D_actual(k))
                J_axes(k) = 0; 
            else
                J_axes(k) = D_ideal(k) / D_actual(k);
            end
        end
        Ja = min(J_axes);
        
        function D_min = Min_steps(Matrix_sub, t)
            N = size(Matrix_sub, 2);
            if N == 0
                D_min = inf(1, 6); return;
            end
            states = 2^N;
            tau = dec2bin(0:states-1) - '0';
            I0 = Matrix_sub * tau' * t; 
            axes = [1 0 0; -1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1]';
            D_min = inf(1, 6);
            for idx = 1:6
                v = axes(:, idx); p = v' * I0; 
                valid_idx = p > 1e-6; 
                if any(valid_idx)
                    D_min(idx) = min(p(valid_idx));
                end
            end
        end
    end

    function Jf = Fuel_Efficiency_Simple(Matrix_sub, test_cmd)
        N_sub = size(Matrix_sub, 2);
        if N_sub == 0
            Jf = 0; return;
        end
        Test_Vectors = [1, -1,  0,  0,  0,  0;
                        0,  0,  1, -1,  0,  0;
                        0,  0,  0,  0,  1, -1] * test_cmd;
        c = ones(N_sub, 1); lb = zeros(N_sub, 1);
        options = optimoptions('linprog', 'Display', 'off');
        total_cost = 0; valid_dirs = 0;
        
        for idx_k = 1:6
            beq = Test_Vectors(:, idx_k);
            [~, fval, exitflag] = linprog(c, [], [], Matrix_sub, beq, lb, [], options);
            if exitflag == 1
                total_cost = total_cost + fval;
                valid_dirs = valid_dirs + 1;
            end
        end
        
        if valid_dirs < 6
            Jf = 0; % 无法包络完整三轴
        else
            avg_cost = total_cost / 6;
            Jf = 1 / avg_cost; 
        end
    end
end

%% 综合评价函数
function [F, W, U_ahp, V_entropy] = Evaluation(Z, G_AHP)
    [m, n] = size(Z);
    % 规范化
    X = zeros(m, n);
    for j = 1:n
        z_max = max(Z(:, j)); z_min = min(Z(:, j));
        if z_max == z_min
            X(:, j) = 1; 
        else
            X(:, j) = (Z(:, j) - z_min) / (z_max - z_min);
        end
    end
    % AHP主观权重
    [V_eig, D_eig] = eig(G_AHP);
    [~, idx] = max(diag(D_eig));
    U_ahp = V_eig(:, idx);
    U_ahp = U_ahp / sum(U_ahp); 
    % 熵权法客观权重
    V_entropy = zeros(n, 1);
    for j = 1:n
        r = X(:, j) / sum(X(:, j));
        r_valid = r(r > 0); 
        E_j = -(1 / log(m)) * sum(r_valid .* log(r_valid));
        V_entropy(j) = 1 - E_j; 
    end
    V_entropy = V_entropy / sum(V_entropy);
    % 最小二乘综合权重
    A = diag(sum(X.^2, 1)); 
    B = zeros(n, 1);
    for j = 1:n
        B(j) = sum(0.5 * (U_ahp(j) + V_entropy(j)) * X(:, j).^2);
    end
    e = ones(n, 1); invA = inv(A);
    W = invA * (B + ((1 - e' * invA * B) / (e' * invA * e)) * e);
    % TOPSIS计算综合得分 F
    x_plus = max(X, [], 1);
    x_minus = min(X, [], 1);
    L = zeros(m, 1); D = zeros(m, 1); F = zeros(m, 1);
    for i = 1:m
        L(i) = sqrt(sum(W' .* (X(i, :) - x_plus).^2));
        D(i) = sqrt(sum(W' .* (X(i, :) - x_minus).^2));
        F(i) = D(i) / (L(i) + D(i));
    end
end

%% ==========================================================
%% 姿轨控底层辅助函数 (保持不变)
%% ==========================================================
function u_opt = Thruster_invocation(v_cmd, Matrix_conf, faulty_thrusters)
    N = size(Matrix_conf, 2); u_opt = zeros(N, 1);
    for axis = 1:3
        cmd = v_cmd(axis);
        if abs(cmd) < 1e-6, continue; end
        if cmd > 0
            valid_idx = find(Matrix_conf(axis, :) > 1e-3);
        else
            valid_idx = find(Matrix_conf(axis, :) < -1e-3);
        end
        if ~isempty(faulty_thrusters)
            valid_idx = setdiff(valid_idx, faulty_thrusters); 
        end
        if isempty(valid_idx), continue; end
        eff = abs(Matrix_conf(axis, valid_idx)');
        u_axis = abs(cmd) .* eff ./ sum(eff.^2);
        for j = 1:length(valid_idx)
            u_opt(valid_idx(j)) = u_opt(valid_idx(j)) + u_axis(j);
        end
    end
    u_opt(u_opt > 1) = 1; 
    u_opt(u_opt > 0 & u_opt < 0.05) = 0;
end

function Prop_Final = Thruster_reuse(Prop_T, Prop_F, params)
    Prop_Final = zeros(params.Num, 1);
    t_att = min(Prop_T, params.T); 
    t_rem = max(0, params.T - t_att); 
    normal = 1.0; 
    for i = 1:params.Num
        if Prop_F(i) > 1e-6 && Prop_F(i) > t_rem(i)
            current = t_rem(i) / Prop_F(i);
            if current < normal, normal = current; end
        end
    end
    for i = 1:params.Num
        t_orb = Prop_F(i) * normal;
        Prop_Final(i) = t_att(i) + t_orb;
    end
end

function dY = Spacecraft_dynamics(Y, params)    
    r = Y(1:3); v = Y(4:6); sigma = Y(7:9); omega = Y(10:12);
    m = params.m; n = params.n; J = params.J;
    F_body = params.current_F; T_body = params.current_T;
    
    [~, sigma_c, R_b2o] = Body_to_Orbit(sigma);
    F_orbit = R_b2o * F_body;
    ax = 3*n^2*r(1) + 2*n*v(2) + F_orbit(1)/m;
    ay = -2*n*v(1) + F_orbit(2)/m;
    az = -n^2*r(3) + F_orbit(3)/m;
    dv = [ax; ay; az];
    
    G = 0.25 * ((1 - dot(sigma, sigma)) * eye(3) + 2 * sigma_c + 2 * (sigma * sigma'));
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
    q0 = c1*c2*c3 + s1*s2*s3; q1 = c1*c2*s3 - s1*s2*c3;
    q2 = c1*s2*c3 + s1*c2*s3; q3 = s1*c2*c3 - c1*s2*s3;
    sigma = [q1; q2; q3] / (1 + q0);
    if norm(sigma) > 1, sigma = -sigma / (sigma' * sigma); end
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