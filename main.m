%% 主程序
clear; clc; close all;
params = Get_params();
load('Optim_config_data1.mat', 'B_opt', 'r_opt', 'fval');
% load('Optim_config_data_GlobalFree.mat', 'B_opt', 'r_opt', 'fval');


disp('正在计算 原布局 底层指标...');
[Z_Force_orig, Z_Torque_orig] = get_Z_matrix(params, params.B_all);
disp('正在计算 优化后布局 底层指标...');
[Z_Force_opt, Z_Torque_opt] = get_Z_matrix(params, B_opt);
%% 3. 🌟 关键：拼装全局评价矩阵 (共 26 个方案)
% 前13行是原布局，后13行是优化布局。放在同一个池子里评价尺度才统一！
Z_Force_combined  = [Z_Force_orig;  Z_Force_opt];
Z_Torque_combined = [Z_Torque_orig; Z_Torque_opt];

%% 4. AHP 主观打分与联合综合评价
G_AHP = [
    1,    3,    5,    3;   % Jc
   1/3,   1,    3,    1;   % Ja
   1/5,  1/3,   1,   1/3;  % Jo
   1/3,   1,    3,    1    % Jf
];

[F_Force_combined, ~]   = Comprehensive_Eval(Z_Force_combined, G_AHP);
[F_Torque_combined, ~] = Comprehensive_Eval(Z_Torque_combined, G_AHP);

%% 5. 拆分得分并进行对比分析
% 拆分出原布局和优化布局的得分
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

tic;
%% 主循环
for k = 1:N
    t = (k-1) * dt;
    if t >= falut_time && ~fault_trig
        fault_trig = true;
        if num_faults == 0
            faulty_thrusters = [];% 标况
        else
            faulty_thrusters = randperm(params.Num, num_faults);
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

% 绘图
Plot_results(faulty_thrusters, J, log, params, B_opt, r_opt, B,falut_time);

% --- 辅助封装函数 (为了主程序干净，把遍历13个状态封装成一个函数) ---
function [Z_Force, Z_Torque] = get_Z_matrix(params, B_matrix)
    Jc = zeros(params.Num + 1, 2);
    Ja = zeros(params.Num + 1, 2);
    Jo = zeros(params.Num + 1, 1);
    Jf = zeros(params.Num + 1, 2);
    
    for idx = 1:params.Num+1
        if idx == 1, eval_fault = []; else, eval_fault = idx - 1; end
        [~, Jc(idx, :), ~, Ja(idx, :), Jo(idx, :), Jf(idx, :)] = Reconfig_eval(params, B_matrix, eval_fault);
    end
    Z_Force  = [Jc(:,1), Ja(:,1), Jo, Jf(:,1)];
    Z_Torque = [Jc(:,2), Ja(:,2), Jo, Jf(:,2)];
end
function [F, W, U_ahp, V_entropy] = Comprehensive_Eval(Z, G_AHP)
    [m, n] = size(Z);
    
    % 1. 规范化
    X = zeros(m, n);
    for j = 1:n
        z_max = max(Z(:, j)); z_min = min(Z(:, j));
        if z_max == z_min
            X(:, j) = 1; 
        else
            X(:, j) = (Z(:, j) - z_min) / (z_max - z_min);
        end
    end
    
    % 2. AHP 主观权重
    [V_eig, D_eig] = eig(G_AHP);
    [~, idx] = max(diag(D_eig));
    U_ahp = V_eig(:, idx);
    U_ahp = U_ahp / sum(U_ahp); 
    
    % 3. 熵权法 客观权重
    V_entropy = zeros(n, 1);
    for j = 1:n
        r = X(:, j) / sum(X(:, j));
        r_valid = r(r > 0); 
        E_j = -(1 / log(m)) * sum(r_valid .* log(r_valid));
        V_entropy(j) = 1 - E_j; 
    end
    V_entropy = V_entropy / sum(V_entropy);
    
    % 4. 最小二乘综合权重
    A = diag(sum(X.^2, 1)); 
    B = zeros(n, 1);
    for j = 1:n
        B(j) = sum(0.5 * (U_ahp(j) + V_entropy(j)) * X(:, j).^2);
    end
    e = ones(n, 1); invA = inv(A);
    W = invA * (B + ((1 - e' * invA * B) / (e' * invA * e)) * e);
    
    % 5. TOPSIS 计算综合得分 F
    x_plus = max(X, [], 1);
    x_minus = min(X, [], 1);
    L = zeros(m, 1); D = zeros(m, 1); F = zeros(m, 1);
    
    for i = 1:m
        L(i) = sqrt(sum(W' .* (X(i, :) - x_plus).^2));
        D(i) = sqrt(sum(W' .* (X(i, :) - x_minus).^2));
        F(i) = D(i) / (L(i) + D(i));
    end
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