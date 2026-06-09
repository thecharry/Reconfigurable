%% 可重构基础评价指标函数
function [Z, Jc1, Jc2, Jc, Jo, Jt, Jf] = Reconfig_eval(params, Ball, max_faultys)
    if nargin < 3 || isempty(max_faultys)
        max_faultys = 1;
    end
    Matrix_conf = params.F_max * Ball;% 推力器矩阵
    % 故障组合列表
    faultysets = {[]};
    combs = nchoosek(1:params.Num, max_faultys);
    for ii = 1:size(combs, 1)
        faultysets{end + 1, 1} = combs(ii, :);
    end

    n_case = numel(faultysets);
    Jc1 = zeros(n_case, 2);% 控制能力最小值
    Jc2 = zeros(n_case, 2);% 控制能力最大值
    Jc = zeros(n_case, 1);% 6维控制能力
    Jo = zeros(n_case, 1);% 可诊断性
    Jt = zeros(n_case, 1);% 跟踪性能
    Jf = zeros(n_case, 1);% 能耗效率

    % 评价指标计算
    for i = 1:n_case
        faulty_idx = faultysets{i};
        healthy_idx = setdiff(1:params.Num, faulty_idx);
        Matrix_conf_F = Matrix_conf(1:3, healthy_idx);
        Matrix_conf_T = Matrix_conf(4:6, healthy_idx);
        Matrix_conf_H = Matrix_conf(:, healthy_idx);
        scale = max(abs(Matrix_conf), [], 2);
        scale(scale < 1e-12) = 1;
        
        [Jc1(i, 1), Jc2(i, 1)] = Capability(Matrix_conf_F);
        [Jc1(i, 2), Jc2(i, 2)] = Capability(Matrix_conf_T);
        [Jc(i), ~] = Capability(Matrix_conf_H ./ scale);
        Jo(i) = Diagnosability(Matrix_conf_H);
        [Jt(i), Jf(i)] = Tracking_Energy(Matrix_conf, faulty_idx, scale, params);
    end

    % 四个基础评价指标，均为越大越好。
    Z = struct();
    Z.FaultSets = faultysets;
    Z.Jc = Clip01(min(Jc1(:, 1) ./ max(Jc1(1, 1), eps), Jc1(:, 2) ./ max(Jc1(1, 2), eps)));
    Z.Jo = Clip01(Jo ./ pi);
    Z.Jt = Clip01(Jt);
    Z.Jf = Clip01(1 ./ (1 + Jf));

    %% 控制能力评价指标
    function [Jc_min, Jc_max] = Capability(Matrix_sub)
        n_sub = size(Matrix_sub, 2);
        if n_sub == 0 || isempty(Matrix_sub)
            Jc_min = 0;
            Jc_max = 0;
            return;
        end

        try
            c = 0.5 * ones(n_sub, 1);
            G = 0.5 * eye(n_sub);
            Z = zonotope(Matrix_sub * c, Matrix_sub * G);
            P = polytope(Z);
            H = P.A;
            w = P.b;
            if isempty(H) || isempty(w) || any(w < -1e-6)
                Jc_min = 0;
                Jc_max = 0;
            else
                d = w ./ (vecnorm(H, 2, 2) + 1e-12);
                Jc_min = min(d);
                Jc_max = max(d);
            end
        catch
            Jc_min = 0;
            Jc_max = 0;
        end
    end

    %% 可诊断性评价指标
    function Jo = Diagnosability(Matrix_sub)
        n_sub = size(Matrix_sub, 2);
        if n_sub < 2 || isempty(Matrix_sub)
            Jo = 0;
            return;
        end

        col_norm = vecnorm(Matrix_sub, 2, 1);
        valid_idx = col_norm > 1e-12;
        if sum(valid_idx) < 2
            Jo = 0;
            return;
        end

        K_nor = Matrix_sub(:, valid_idx) ./ col_norm(valid_idx);
        n_sub = size(K_nor, 2);
        cos_matrix = min(max(K_nor' * K_nor, -1), 1);
        angle_matrix = acos(cos_matrix);
        angle_matrix(logical(eye(n_sub))) = inf;
        min_angle = min(angle_matrix(:));
        if isfinite(min_angle)
            Jo = min_angle;
        else
            Jo = 0;
        end
    end

    %% 跟踪性能和能耗效率评价指标
    function [Jt, Jf] = Tracking_Energy(Matrix_conf, faulty_idx, scale, params)
        % 构建典型指令集
        dirs = [eye(3), -eye(3)];
        F_cmds = zeros(3, 0);
        T_cmds = zeros(3, 0);
        cmd_idx = 0;
        weight = 0.2;
        force_level = max(max(abs(Matrix_conf(1:3, :))));
        torque_level = max(max(abs(Matrix_conf(4:6, :))));
        for f = 1:size(dirs, 2)
            cmd_idx = cmd_idx + 1;
            F_cmds(:, cmd_idx) = weight * force_level * dirs(:, f);
            T_cmds(:, cmd_idx) = zeros(3, 1);
        end
        for t = 1:size(dirs, 2)
            cmd_idx = cmd_idx + 1;
            F_cmds(:, cmd_idx) = zeros(3, 1);
            T_cmds(:, cmd_idx) = weight * torque_level * dirs(:, t);
        end
        for f_idx = 1:size(dirs, 2)
            for t_idx = 1:size(dirs, 2)
                cmd_idx = cmd_idx + 1;
                F_cmds(:, cmd_idx) = weight * force_level * dirs(:, f_idx);
                T_cmds(:, cmd_idx) = weight * torque_level * dirs(:, t_idx);
            end
        end

        n_cmd = size(F_cmds, 2);
        cmd_dev = zeros(n_cmd, 1);
        pulse_sum = zeros(n_cmd, 1);
        for k = 1:n_cmd
            F_cmd = F_cmds(:, k);
            T_cmd = T_cmds(:, k);
            cmd = [F_cmd; T_cmd];
            try
                [Prop_Final, info] = Thruster_invocation_decoupled(F_cmd, T_cmd, Matrix_conf, faulty_idx, params);
                actual = info.actual;
            catch
                Prop_Final = Thruster_invocation(F_cmd, T_cmd, Matrix_conf, faulty_idx, params);
                actual = Matrix_conf * (Prop_Final / params.T);
            end
            cmd_dev(k) = (1 - norm((cmd - actual) ./ scale) / (norm(cmd ./ scale) + 1e-12));
            pulse_sum(k) = sum(Prop_Final);
        end
        Jt = mean(cmd_dev);
        Jf = mean(pulse_sum);
    end

    function value = Clip01(value)
        value(~isfinite(value)) = 0;
        value = max(0, min(1, value));
    end
end
