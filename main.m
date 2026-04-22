%% 主程序
clear; clc; close all;
params = Get_params();
load('Optim_config_data1.mat', 'B_opt', 'r_opt', 'fval');
% load('Optim_config_data_GlobalFree.mat', 'B_opt', 'r_opt', 'fval');

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
        Prop_Final = Thruster_invocation(F_body_req,T_body_req,Matrix_conf,faulty_thrusters);
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

%% 辅助函数
function [J,Matrix_conf_opt] = Evaluation(params, B_opt)
    J = zeros((params.Num+1)*7,2);
    Jc_opt = zeros(params.Num+1, 2);
    Ja_opt = zeros(params.Num+1, 2);
    Jo_opt = zeros(params.Num+1, 1);
    Jf_opt = zeros(params.Num+1, 2);
    for idx = 1:params.Num+1
        if idx == 1
            eval_fault = []; % 标况
        else
            eval_fault = idx - 1; % 单个推力器故障
        end
        [Matrix_conf_opt,Jc_opt(idx, :),~,Ja_opt(idx, :),Jo_opt(idx, :),Jf_opt(idx, :)] = Reconfig_eval(params, B_opt, eval_fault);
        J((idx-1)*7+1, :) = [params.Jc(idx,1),Jc_opt(idx,1)];
        J((idx-1)*7+2, :) = [params.Jc(idx,2),Jc_opt(idx,2)];
        J((idx-1)*7+3, :) = [params.Ja(idx,1),Ja_opt(idx,1)];
        J((idx-1)*7+4, :) = [params.Ja(idx,2),Ja_opt(idx,2)];
        J((idx-1)*7+5, :) = [params.Jo(idx),Jo_opt(idx)];
        J((idx-1)*7+6, :) = [params.Jf(idx,1),Jf_opt(idx,1)];
        J((idx-1)*7+7, :) = [params.Jf(idx,2),Jf_opt(idx,2)];
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