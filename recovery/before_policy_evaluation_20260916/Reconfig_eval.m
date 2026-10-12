%% 可重构基础评价指标函数
function [Z, Jc1, Jc2, Jc, Jo, Jt, Jf] = Reconfig_eval(params, Ball, max_faultys, include_combined_commands)
    if nargin < 3 || isempty(max_faultys)
        max_faultys = 1;
    end
    if nargin < 4 || isempty(include_combined_commands)
        include_combined_commands = true;
    end
    Matrix_conf = params.F_max * Ball;% 推力器矩阵
    % 故障组合列表
    faultysets = {[]};
    if max_faultys > 0
        combs = nchoosek(1:params.Num, max_faultys);
        faultysets = cell(size(combs, 1) + 1, 1);
        faultysets{1} = [];
        for ii = 1:size(combs, 1)
            faultysets{ii + 1, 1} = combs(ii, :);
        end
    end

    n_case = numel(faultysets);
    Jc1 = zeros(n_case, 2);% 控制能力最小值
    Jc2 = zeros(n_case, 2);% 控制能力最大值
    Jc = zeros(n_case, 1);% 6维控制能力
    Jo = zeros(n_case, 1);% 可诊断性
    Jt = zeros(n_case, 1);% 跟踪性能
    Jt_err = zeros(n_case, 1);% 跟踪误差
    Jf = zeros(n_case, 1);% 能耗效率
    invocation_template = struct('axis_callable', false(6, 2), ...
        'strict_axis', false(6, 2), 'axis_capacity', zeros(6, 2), ...
        'force_capacity', 0, 'torque_capacity', 0, 'worst_capacity', 0, ...
        'policy_reconfigurable', false, 'strictly_reconfigurable', false, ...
        'uses_impulse_margin_fallback', false, ...
        'worst_reference_quality', 0, 'reference_commands_callable', false);
    Invocation = repmat(invocation_template, n_case, 1);

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
        [Jt(i), Jf(i), Jt_err(i), Invocation(i)] = Tracking_Energy(Matrix_conf, faulty_idx, scale, params, include_combined_commands);
    end
    % 仅输出原始评价量；跨布局归一化在Plot_results.m中统一完成。
    Z = struct();
    Z.FaultSets = faultysets;
    Z.Jc6 = Jc;
    Z.Jc = Jc1;
    Z.Jo = Jo;
    Z.Jt = Jt;
    Z.Jf = Jf;
    % Jc 表示任意健康推力器占空比组合可达到的硬件上限；Invocation
    % 则只统计现有“最简主备调用 + 分时复用”实际能够执行的能力。
    Z.Invocation = Invocation;
    % Z.Raw = struct('JcForce', Jc1(:, 1), 'JcTorque', Jc1(:, 2), ...
    %                'Jc6', Jc, 'JoAngle', Jo, ...
    %                'JtQuality', Jt, 'JtError', Jt_err, 'JfPulse', Jf);
    Z.Raw = struct('JcForce', Jc1(:, 1), 'JcTorque', Jc1(:, 2), ...
                   'Jc6', Jc, 'JoAngle', Jo, ...
                   'JtQuality', Jt, 'JtError', Jt_err, 'JfPulse', Jf);

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
            % Jo = sin(min_angle / 2);
        else
            Jo = 0;
        end
    end

    %% 跟踪性能和能耗效率评价指标
    function [Jt, Jf, mean_err, invocation] = Tracking_Energy(Matrix_conf, faulty_idx, scale, params, include_combined_commands)
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
        if include_combined_commands
            for f_idx = 1:size(dirs, 2)
                for t_idx = 1:size(dirs, 2)
                    cmd_idx = cmd_idx + 1;
                    F_cmds(:, cmd_idx) = weight * force_level * dirs(:, f_idx);
                    T_cmds(:, cmd_idx) = weight * torque_level * dirs(:, t_idx);
                end
            end
        end

        n_cmd = size(F_cmds, 2);
        cmd_quality = zeros(n_cmd, 1);
        cmd_err = zeros(n_cmd, 1);
        pulse_sum = zeros(n_cmd, 1);
        invocation = invocation_template;
        for k = 1:n_cmd
            F_cmd = F_cmds(:, k);
            T_cmd = T_cmds(:, k);
            cmd = [F_cmd; T_cmd];
            [Prop_Final, info] = Thruster_invocation( ...
                F_cmd, T_cmd, Matrix_conf, faulty_idx, params);
            actual = info.actual;
            rel_err = norm((cmd - actual) ./ scale) / (norm(cmd ./ scale) + 1e-12);
            cmd_err(k) = rel_err;
            cmd_quality(k) = 1 / (1 + rel_err);
            pulse_sum(k) = sum(Prop_Final);

            % 前12条指令分别是六个正负轴向的独立中等幅值指令。它们
            % 用于检查当前调用策略是否真的能给每个带符号轴安排完整组，
            % 并由组内最大占空比推得该组在一个控制周期内的有效上限。
            if k <= 12
                if k <= 6
                    row_idx = mod(k - 1, 3) + 1;
                    control_type = 1;
                else
                    row_idx = mod(k - 7, 3) + 4;
                    control_type = 2;
                end
                direction_col = 1 + (cmd(row_idx) > 0);
                axis_idx = row_idx - 3 * (control_type - 1);
                strategy = info.axis_strategy{control_type, axis_idx};
                is_group_strategy = isstruct(strategy) && isfield(strategy, 'duty') && ...
                    ~isempty(strategy.duty) && any(strategy.duty > 1e-12);
                is_joint_mode = isfield(info, 'mode') && ...
                    strcmp(info.mode, 'joint_optimization') && any(info.duty > 1e-12);
                axis_ok = rel_err <= 1e-5 && (is_group_strategy || is_joint_mode);
                invocation.axis_callable(row_idx, direction_col) = axis_ok;
                if axis_ok
                    if is_group_strategy
                        max_duty = max(strategy.duty);
                    else
                        max_duty = max(info.duty);
                    end
                    if max_duty > 1e-12
                        capacity = abs(cmd(row_idx)) / max_duty;
                        % 冲量裕度备用只在当前测试幅值下验证过安全性，
                        % 不把它外推到满周期，以免高估故障后的实际能力。
                        if is_group_strategy && strategy.is_impulse_margin_fallback
                            capacity = min(capacity, abs(cmd(row_idx)));
                            invocation.uses_impulse_margin_fallback = true;
                        end
                        invocation.axis_capacity(row_idx, direction_col) = capacity;
                    end
                    if is_group_strategy && isfield(strategy, 'instantaneous_pure')
                        invocation.strict_axis(row_idx, direction_col) = strategy.instantaneous_pure;
                    end
                end
            end
        end
        Jt = mean(cmd_quality);
        mean_err = mean(cmd_err);
        Jf = mean(pulse_sum);
        invocation.force_capacity = min(invocation.axis_capacity(1:3, :), [], 'all');
        invocation.torque_capacity = min(invocation.axis_capacity(4:6, :), [], 'all');
        invocation.worst_capacity = min(invocation.force_capacity, invocation.torque_capacity);
        invocation.policy_reconfigurable = all(invocation.axis_callable(:));
        invocation.strictly_reconfigurable = all(invocation.strict_axis(:));
        invocation.worst_reference_quality = min(cmd_quality);
        invocation.reference_commands_callable = all(cmd_err <= 1e-5);
    end

end
