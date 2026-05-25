%% 可重构评价指标函数
function [Z, Jc1, Jc2, Ja, Jo, Jf, Jc] = Reconfig_eval(params, Ball, max_faultys)
    Matrix_conf = params.F_max * Ball;
    if nargin < 3 || isempty(max_faultys)
        max_faultys = 1;
    end
    % 生成所有故障组合
    faultysets = {};
    faultysets{end+1, 1} = [];
    combs = nchoosek(1:params.Num, max_faultys);
    for i = 1:size(combs, 1)
        faultysets{end+1, 1} = combs(i, :);
    end
    Jc1 = zeros(length(faultysets), 2);
    Jc2 = zeros(length(faultysets), 2);
    Ja = zeros(length(faultysets), 2);
    Jo = zeros(length(faultysets), 1);
    Jf = zeros(length(faultysets), 1);
    Jc = zeros(length(faultysets), 1);
    scale = max(abs(Matrix_conf), [], 2);% 每行的最大绝对值
    scale(scale < 1e-12) = 1;
    for i = 1:length(faultysets)
        faulty_idx = faultysets{i};
        healthy_idx = setdiff(1:params.Num, faulty_idx);% 剔除故障推力器
        % Matrix_conf_F = Matrix_conf(1:3, healthy_idx);
        % Matrix_conf_T = Matrix_conf(4:6, healthy_idx);
        Matrix_conf_H = Matrix_conf(:, healthy_idx);
        % 控制能力指标
        [Jc(i), ~] = Capability(Matrix_conf_H ./ scale);
        % [Jc_Force, Jc_Force1]  = Capability(Matrix_conf_F);
        % [Jc_Torque, Jc_Torque1] = Capability(Matrix_conf_T);
        % Jc1(i,:) = [Jc_Force, Jc_Torque];
        % Jc2(i,:) = [Jc_Force1, Jc_Torque1];
        
        % % 控制分辨率指标
        % Ja_Force = Precision(Matrix_conf(1:3, :), Matrix_conf_F, params.t_min);
        % Ja_Torque = Precision(Matrix_conf(4:6, :), Matrix_conf_T, params.t_min);
        % Ja(i,:) = [Ja_Force, Ja_Torque];
        % % 可诊断性指标
        % Jo(i) = Diagnosability(Matrix_conf_H);
        % % 燃料效能指标
        % Jf(i) = Efficiency(Matrix_conf, faulty_idx, params);
    end
    % 评价指标综合
    Jc_all = 0.5 * Jc1(:,1) + 0.5 * Jc1(:,2);
    Ja_all = 0.5 * Ja(:,1) + 0.5 * Ja(:,2);
    Z = [Jc_all,Ja_all,Jo,Jf];
    
    %% 基于推力器方向分布的可诊断性指标Jo
    function Jo = Diagnosability(Matrix_sub)
        [~, N_sub] = size(Matrix_sub);
        if N_sub < 2
            Jo = 0;
            return;
        end
        col_norm = vecnorm(Matrix_sub, 2, 1);
        if any(col_norm < 1e-12)
            Jo = 0;
            return;
        end
        K_nor = Matrix_sub ./ col_norm;
        CosSim_Matrix = K_nor' * K_nor; % Kp/||Kp||_2 · (Kq/||Kq||_2)
        CosSim_Matrix(logical(eye(N_sub))) = -2; % 排除自身列向量相乘情况
        max_cos = max(CosSim_Matrix, [], 'all'); % 最大余弦值（最小夹角）
        % Jo = max(1 - max_cos, 0);
        Jo = 1 - max_cos;
    end

    %% 基于可达域几何特性的控制能力指标Jc
    function [umin, umax] = Capability(Matrix_sub)
        N_sub = size(Matrix_sub, 2);
        if N_sub == 0
            umin = 0;
            umax = 0;
            return;
        end

        try
            c = 0.5 * ones(N_sub, 1);
            G = 0.5 * diag(ones(N_sub, 1));
            Z = zonotope(Matrix_sub * c, Matrix_sub * G);
            P = polytope(Z); % 获取半空间表达式H-rep(Hx <= w)
            H = P.A;
            w = P.b;
            if isempty(H) || isempty(w) || any(w < -1e-6)
                umin = 0; % 原点在外部
                umax = 0;
            else
                d = w ./ (vecnorm(H, 2, 2) + 1e-12); % 原点到所有边界面的最短距离:d = w/||H||_2
                umin = min(d);
                umax = max(d);
            end
        catch
            umin = 0;
            umax = 0;
        end
    end

    %% 基于离散脉冲组合的控制分辨率指标Ja
    function Ja = Precision(Matrix_healthy, Matrix_faulty, t_min)
        D_ideal = Min_steps(Matrix_healthy, t_min);
        D_actual = Min_steps(Matrix_faulty, t_min);
        J_axes = zeros(1, 6);
        for k = 1:6
            if isinf(D_actual(k))||D_actual(k) < 1e-12
                J_axes(k) = 0; % 彻底失效
            else
                J_axes(k) = D_ideal(k) / D_actual(k);
            end
        end
        Ja = min(J_axes);
        
        % 寻找轴向上最小顶点
        function D_min = Min_steps(Matrix_sub, t)
            N = size(Matrix_sub, 2);
            if N == 0
                D_min = inf(1, 6);
                return;
            end
            states = 2^N;
            tau = dec2bin(0:states-1) - '0';
            I0 = Matrix_sub * tau' * t; 
            axes = [1 0 0; -1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1]';
            D_min = inf(1, 6);
            for idx = 1:6
                v = axes(:, idx);   
                p = v' * I0; 
                valid_idx = p > 1e-6; 
                if any(valid_idx)
                    p_valid = p(valid_idx);
                    D_min(idx) = min(p_valid);
                end
            end
        end
    end

    %% 平均燃料效能指标Jf
    function Jf = Efficiency(Matrix_conf, faulty_thrusters, params)
        N = 50;
        % 轨道方向采样
        U_F = FibonacciSphere(N, 0.0);
        % 姿态方向采样
        U_T = FibonacciSphere(N, 0.5);
        total = 0;
        for k = 1:N
            v_cmd1 = U_F(:, k);
            v_cmd2 = U_T(:, k);
            Prop_Final = Thruster_invocation(v_cmd1, v_cmd2, Matrix_conf, faulty_thrusters, params);
            cost_k = sum(Prop_Final);
            total = total + cost_k;
        end
        avg = total / N;
        Jf = 1 / (1 + avg);

        % 斐波那契球面均匀采样
        function U = FibonacciSphere(N, phase_shift)
            phi_golden = (1 + sqrt(5)) / 2;
            idx = 0:(N - 1);
            theta = 2 * pi * (idx + phase_shift) / phi_golden;
            z = 1 - (2 * idx + 1) / N;
            r = sqrt(max(0, 1 - z.^2));
            x = r .* cos(theta);
            y = r .* sin(theta);
            U = [x; y; z];
        end
    end
end
