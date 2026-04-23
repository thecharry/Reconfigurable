%% 可重构评价指标函数
function [Matrix_conf, Jc1, Jc2, Ja, Jo, Jf,F,W] = Reconfig_eval(params, Ball, faulty_thrusters)
    Matrix_conf = params.F_max * Ball;
    healthy_idx = setdiff(1:params.Num, faulty_thrusters);% 剔除所有故障推力器
    Matrix_conf_F = Matrix_conf(1:3, healthy_idx);
    Matrix_conf_T = Matrix_conf(4:6, healthy_idx);
    Matrix_conf_H = Matrix_conf(:, healthy_idx);
    % 控制能力指标
    [Jc_Force,  Jc_Force1]  = Capability(Matrix_conf_F);
    [Jc_Torque, Jc_Torque1] = Capability(Matrix_conf_T);
    Jc1 = [Jc_Force,  Jc_Torque];
    Jc2 = [Jc_Force1,  Jc_Torque1];
    % 控制分辨率指标
    Ja_Force = Precision(Matrix_conf(1:3, :), Matrix_conf_F, params.t_min);
    Ja_Torque = Precision(Matrix_conf(4:6, :), Matrix_conf_T, params.t_min);
    Ja = [Ja_Force, Ja_Torque];
    % 可诊断性指标
    Jo = Diagnosability(Matrix_conf_H);
    % 燃料效能指标
    Jf = Efficiency(Matrix_conf, faulty_thrusters, params);

    % % AHP判断矩阵G(Jc最重要，Ja和Jf其次且同等重要，Jo最次)
    % G_AHP = [1,   3,   5,   3;% Jc
    %          1/3, 1,   3,   1;% Ja
    %          1/5, 1/3, 1, 1/3;% Jo
    %          1/3, 1,   3,  1];% Jf
    % Z_Force  = [params.Jc(:,1), params.Ja(:,1), params.Jo, params.Jf;Jc_Force(:,1), Ja_Force(:,1), Jo, Jf];% 轨道控制评价矩阵
    % Z_Torque = [params.Jc(:,2), params.Ja(:,2), params.Jo, params.Jf;Jc_Torque(:,2), Ja_Torque(:,2), Jo, Jf];% 姿态控制评价矩阵
    % [F_Force, W_Force] = Evaluation(Z_Force, G_AHP);
    % [F_Torque, W_Torque] = Evaluation(Z_Torque, G_AHP);
    % F = [F_Force, F_Torque];
    % W = [W_Force, W_Torque];
    % 
    % %% 综合评价函数
    % function [F, W, U_ahp, V_entropy] = Evaluation(Z, G_AHP)
    %     [m, n] = size(Z);
    %     % 规范化
    %     X = zeros(m, n);
    %     for j = 1:n
    %         z_max = max(Z(:, j)); z_min = min(Z(:, j));
    %         if z_max == z_min
    %             X(:, j) = 1;
    %         else
    %             X(:, j) = (Z(:, j) - z_min) / (z_max - z_min);
    %         end
    %     end
    %     % AHP主观权重
    %     [V_eig, D_eig] = eig(G_AHP);
    %     [~, idx] = max(diag(D_eig));
    %     U_ahp = V_eig(:, idx);
    %     U_ahp = U_ahp / sum(U_ahp);
    %     % 熵权法客观权重
    %     V_entropy = zeros(n, 1);
    %     for j = 1:n
    %         r = X(:, j) / sum(X(:, j));
    %         r_valid = r(r > 0);
    %         E_j = -(1 / log(m)) * sum(r_valid .* log(r_valid));
    %         V_entropy(j) = 1 - E_j;
    %     end
    %     V_entropy = V_entropy / sum(V_entropy);
    %     % 最小二乘综合权重
    %     A = diag(sum(X.^2, 1));
    %     B = zeros(n, 1);
    %     for j = 1:n
    %         B(j) = sum(0.5 * (U_ahp(j) + V_entropy(j)) * X(:, j).^2);
    %     end
    %     e = ones(n, 1);
    %     W = A \ (B + ((1 - e' * inv(A) * B) / (e' * inv(A) * e)) * e);
    %     % TOPSIS计算综合得分 F
    %     x_plus = max(X, [], 1);
    %     x_minus = min(X, [], 1);
    %     L = zeros(m, 1); D = zeros(m, 1); F = zeros(m, 1);
    %     for i = 1:m
    %         L(i) = sqrt(sum(W' .* (X(i, :) - x_plus).^2));
    %         D(i) = sqrt(sum(W' .* (X(i, :) - x_minus).^2));
    %         F(i) = D(i) / (L(i) + D(i));
    %     end
    % end
    
    %% 基于推力器方向分布的可诊断性指标Jo
    function Jo = Diagnosability(Matrix_sub)
        N_sub = size(Matrix_sub, 2);
        col_norm = vecnorm(Matrix_sub, 2, 1);
        if any(col_norm < 1e-12)
            Jo = 0;
            return;
        end
        K_nor = Matrix_sub ./ col_norm;
        CosSim_Matrix = K_nor' * K_nor; % Kp/||Kp||_2 · (Kq/||Kq||_2)
        CosSim_Matrix(logical(eye(N_sub))) = -2; % 排除自身列向量相乘情况
        max_cos = max(CosSim_Matrix, [], 'all'); % 最大余弦值（最小夹角）
        Jo = max(1 - max_cos, 0);
    end

    %% 基于可达域几何特性的控制能力指标Jc
    function [umin, umax] = Capability(Matrix_sub)
        N_sub = size(Matrix_sub, 2);
        c = 0.5 * ones(N_sub, 1); 
        G = 0.5 * diag(ones(N_sub, 1));
        Z = zonotope(Matrix_sub * c, Matrix_sub * G);
        P = polytope(Z); % 获取半空间表达式H-rep(Hx <= w)
        H = P.A;
        w = P.b;
        if any(w < -1e-6)
            umin = 0; % 原点在外部
        else
            d = w ./ vecnorm(H, 2, 2); % 原点到所有边界面的最短距离:d = w/||H||_2
            umin = min(d);
            umax = max(d);
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
        
        %% 寻找轴向上最小顶点
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

        %% 斐波那契球面均匀采样
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