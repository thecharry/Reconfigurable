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
    log.faluty_thrusters = [];% 空数组表示推力器无故障
    % log.falut_time = rand() * T_sim;
    log.faluty_time = 0.5*T_sim;

    tic;
    for k = 1:N
        t = (k-1) * dt;
        if t >= log.faluty_time && ~fault_trig
            fault_trig = true;
            if num_faults == 0
                log.faluty_thrusters = [];% 标况
            else
                % faulty_thrusters = randperm(params.Num, num_faults);
                log.faluty_thrusters = 3;
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
            Prop_Final = Thruster_invocation(F_body_req,T_body_req,Matrix_conf,log.faluty_thrusters,params);
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
end