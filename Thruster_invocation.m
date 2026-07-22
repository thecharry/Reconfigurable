%% 推力器控制效率调用策略
function [Prop_Final, info] = Thruster_invocation(v_cmd1, v_cmd2, Matrix_sub, faulty_thrusters, params)
    % Matrix_sub(:, i) 是第 i 台推力器单位占空比产生的 6 维控制效率：
    %   Matrix_sub = [F_x; F_y; F_z; M_x; M_y; M_z]
    % 对每个单轴指令 c，本函数先在健康推力器中选择最简低耦合组合，
    % 再将轨控脉宽 Prop_F 与姿控脉宽 Prop_T 按任务书复用策略合成。
    N = size(Matrix_sub, 2);
    faulty_thrusters = Sanitize_Faults(faulty_thrusters);
    min_duty = Get_Param(params, 'alloc_min_duty', Get_Param(params, 't_min', 0.02) / params.T);
    row_scale = Prepare_Row_Scale(Get_Param(params, 'alloc_row_scale', max(abs(Matrix_sub), [], 2)));

    reuse_mode = char(string(Get_Param(params, 'alloc_mode', ...
        Get_Param(params, 'reuse_mode', 'task_book'))));
    if any(strcmpi(reuse_mode, {'joint_6d', 'six_d_qp', 'full_6d', 'full6d'}))
        [Prop_Final, info] = Full_6D_Allocation( ...
            v_cmd1, v_cmd2, Matrix_sub, faulty_thrusters, min_duty);
        return;
    end

    [u, Prop_F_axis, axis_strategy] = Axis_Split_Allocation(v_cmd1, v_cmd2, Matrix_sub, faulty_thrusters, min_duty);
    Prop_F = u(:, 1) * params.T;
    Prop_T = u(:, 2) * params.T;

    [Prop_Final, Prop_F_used, Prop_T_used, reuse_scale] = Task_Book_Reuse(Prop_F, Prop_T, Prop_F_axis);

    if nargout > 1
        duty = Prop_Final / params.T;
        command = [v_cmd1(:); v_cmd2(:)];
        info.mode = 'task_book';
        info.solver = 'axis_split';
        info.Prop_F = Prop_F;
        info.Prop_T = Prop_T;
        info.Prop_F_axis = Prop_F_axis;
        info.axis_strategy = axis_strategy;
        info.Prop_F_used = Prop_F_used;
        info.Prop_T_used = Prop_T_used;
        info.Prop_6D = zeros(N, 1);
        info.reuse_scale = reuse_scale;
        info.duty = duty;
        info.command = command;
        info.actual = Matrix_sub * duty;
        info.residual = command - info.actual;
        info.orbit_keep_ratio = sum(Prop_F_used) / (sum(Prop_F) + 1e-12);
        info.att_keep_ratio = sum(Prop_T_used) / (sum(Prop_T) + 1e-12);
        info.max_raw_pulse = max(Prop_F + Prop_T);
        info.attitude_window = NaN;
        info.position_window = NaN;
    end

    function [u, Prop_F_axis, axis_strategy] = Axis_Split_Allocation(v_cmd1, v_cmd2, Matrix_sub, faulty_thrusters, min_duty)
        % 两类指令分开生成：
        %   u(:, 1) -> 轨控占空比，对应 Matrix_sub 的第 1:3 行；
        %   u(:, 2) -> 姿控占空比，对应 Matrix_sub 的第 4:6 行。
        % 每个轴独立求最简组合，故障时只需从候选集合中剔除故障推力器。
        u = zeros(N, 2);
        orbit_axis_duty_raw = zeros(N, 3);
        axis_strategy = cell(2, 3);

        for v_cmd = 1:2
            u_opt = zeros(N, 1);
            for axis = 1:3
                if v_cmd == 1
                    cmd = v_cmd1(axis);
                    row_idx = axis;
                else
                    cmd = v_cmd2(axis);
                    row_idx = axis + 3;
                end

                if abs(cmd) < 1e-6
                    axis_strategy{v_cmd, axis} = Empty_Strategy(row_idx, cmd);
                    continue;
                end

                [u_axis, strategy] = Minimal_Axis_Allocation(row_idx, cmd, Matrix_sub, faulty_thrusters, min_duty);
                axis_strategy{v_cmd, axis} = strategy;
                if ~any(u_axis > 0)
                    continue;
                end

                u_opt = u_opt + u_axis;
                if v_cmd == 1
                    orbit_axis_duty_raw(:, axis) = orbit_axis_duty_raw(:, axis) + u_axis;
                end
            end

            u_opt(u_opt > 1) = 1;
            u_opt(u_opt > 0 & u_opt < min_duty) = 0;
            u(:, v_cmd) = u_opt;
        end

        Prop_F_axis = Orbit_Axis_Pulse(orbit_axis_duty_raw, u(:, 1), params.T);
    end

    function [u_axis, strategy] = Minimal_Axis_Allocation(row_idx, cmd, Matrix_sub, faulty_thrusters, min_duty)
        % 单轴最简调用策略：
        % 1) 候选集 C = {i | sign(cmd) * B(row_idx, i) > eps, i 未故障}
        % 2) 枚举 C 中 1 台、2 台、...、m_max 台的所有组合 S
        % 3) 对每个组合求解“满足目标轴、最小化其余 5 维耦合”的等式约束问题
        % 4) 在接近最小耦合的组合中，优先选实际点火推力器数量最少者
        u_axis = zeros(N, 1);
        strategy = Empty_Strategy(row_idx, cmd);

        direction = sign(cmd);
        eff_tol = Get_Param(params, 'alloc_axis_eff_tol', 1e-3);
        candidate_idx = find(direction * Matrix_sub(row_idx, :) > eff_tol);
        candidate_idx = setdiff(candidate_idx, faulty_thrusters);
        if isempty(candidate_idx)
            strategy.status = 'no_candidate';
            return;
        end

        allocation_strategy = char(string(Get_Param(params, ...
            'allocation_strategy', 'primary_backup')));
        if any(strcmpi(allocation_strategy, ...
                {'axis', 'axis_split', 'axis_pseudoinverse'}))
            [u_axis, strategy] = Axis_Candidate_Allocation( ...
                row_idx, cmd, Matrix_sub, candidate_idx, min_duty);
            return;
        end

        max_thrusters = Get_Param(params, 'alloc_max_thrusters_per_axis', min(6, numel(candidate_idx)));
        max_thrusters = max(1, min(numel(candidate_idx), max_thrusters));
        results = struct('idx', {}, 'u', {}, 'active_count', {}, 'coupling_norm', {}, ...
                         'pulse_sum', {}, 'max_duty', {}, 'desired_residual', {}, ...
                         'valid', {}, 'constraint_violation', {});

        for subset_size = 1:max_thrusters
            combs = nchoosek(candidate_idx, subset_size);
            for row = 1:size(combs, 1)
                subset_idx = combs(row, :);
                result = Solve_Subset(row_idx, cmd, Matrix_sub, subset_idx, min_duty);
                results(end + 1) = result;
            end
        end

        if isempty(results)
            strategy.status = 'no_result';
            return;
        end

        valid_mask = [results.valid];
        if any(valid_mask)
            % 先找到耦合最小的可行组合；耦合在容差内几乎等价时，再按
            % 点火台数、总占空比、最大占空比排序，从而得到“最简”策略。
            valid_results = results(valid_mask);
            coupling_values = [valid_results.coupling_norm];
            min_coupling = min(coupling_values);
            rel_tol = Get_Param(params, 'alloc_coupling_rel_tol', 0.05);
            abs_tol = Get_Param(params, 'alloc_coupling_abs_tol', 1e-8);
            close_mask = coupling_values <= min_coupling * (1 + rel_tol) + abs_tol;
            pool = valid_results(close_mask);
            rank = [[pool.active_count]', [pool.coupling_norm]', [pool.pulse_sum]', [pool.max_duty]'];
            [~, order] = sortrows(rank, [1 2 3 4]);
            best = pool(order(1));
            strategy.status = 'minimal_coupling';
        else
            % 如果所有组合都因占空比约束或最小脉宽约束不可行，则保留
            % 约束违背量最小的 best-effort 方案，避免分配器直接失效。
            rank = [[results.constraint_violation]', [results.desired_residual]', ...
                    [results.active_count]', [results.coupling_norm]', [results.pulse_sum]'];
            [~, order] = sortrows(rank, [1 2 3 4 5]);
            best = results(order(1));
            strategy.status = 'best_effort';
        end

        active = best.u > 1e-12;
        selected_idx = best.idx(active);
        selected_u = best.u(active);
        u_axis(selected_idx) = selected_u;

        strategy.thrusters = selected_idx;
        strategy.duty = selected_u;
        strategy.coupling_norm = best.coupling_norm;
        strategy.desired_residual = best.desired_residual;
        strategy.max_duty = best.max_duty;
        strategy.pulse_sum = best.pulse_sum;
    end

    function [u_axis, strategy] = Axis_Candidate_Allocation( ...
            row_idx, cmd, Matrix_sub, candidate_idx, min_duty)
        % 报告4.3.3中的轴向候选集局部伪逆分配。对目标轴同向的
        % 所有健康推力器求一行矩阵的最小范数伪逆解。
        u_axis = zeros(N, 1);
        strategy = Empty_Strategy(row_idx, cmd);
        efficiency = Matrix_sub(row_idx, candidate_idx)';
        denominator = efficiency' * efficiency;
        if denominator <= 1e-12
            strategy.status = 'no_candidate';
            return;
        end
        duty = cmd * efficiency / denominator;
        duty(~isfinite(duty)) = 0;
        duty = max(0, min(1, duty));
        duty(duty > 0 & duty < min_duty) = 0;
        active = duty > 1e-12;
        selected = candidate_idx(active);
        u_axis(selected) = duty(active);

        actual = Matrix_sub * u_axis;
        other_rows = setdiff(1:6, row_idx);
        strategy.thrusters = selected;
        strategy.duty = duty(active);
        strategy.coupling_norm = norm(actual(other_rows) ./ row_scale(other_rows));
        strategy.desired_residual = abs(actual(row_idx) - cmd) / (abs(cmd) + 1e-12);
        strategy.max_duty = max([duty(:); 0]);
        strategy.pulse_sum = sum(duty);
        if isempty(selected)
            strategy.status = 'below_minimum_pulse';
        else
            strategy.status = 'axis_pseudoinverse';
        end
    end

    function result = Solve_Subset(row_idx, cmd, Matrix_sub, subset_idx, min_duty)
        % 对给定组合 S，记 a = B(row_idx, S)，C = D^{-1}B(other_rows, S)，
        % 其中 D 为各通道控制能力尺度。求解：
        %   min_u  ||C u||_2^2 + lambda ||u||_2^2
        %   s.t.   a u = cmd
        % KKT 方程为：
        %   [H a'] [u ] = [0  ],  H = C'C + lambda I
        %   [a 0 ] [mu]   [cmd]
        % 随后检查 0 <= u <= 1 以及 u_i >= t_min / T 的最小脉宽约束。
        subset_idx = subset_idx(:)';
        subset_num = numel(subset_idx);
        target_row = Matrix_sub(row_idx, subset_idx);
        other_rows = setdiff(1:6, row_idx);
        coupling_matrix = Matrix_sub(other_rows, subset_idx);
        coupling_matrix = bsxfun(@rdivide, coupling_matrix, row_scale(other_rows));
        energy_weight = Get_Param(params, 'alloc_energy_weight', 1e-6);
        H = coupling_matrix' * coupling_matrix + energy_weight * eye(subset_num);
        H = 0.5 * (H + H');
        KKT = [H, target_row(:); target_row(:)', 0];
        rhs = [zeros(subset_num, 1); cmd];

        if rcond(KKT) > 1e-12
            sol = KKT \ rhs;
        else
            sol = pinv(KKT) * rhs;
        end
        u_sub = sol(1:subset_num);
        u_sub(abs(u_sub) < 1e-12) = 0;

        actual = Matrix_sub(:, subset_idx) * u_sub;
        desired_residual = abs(actual(row_idx) - cmd) / (abs(cmd) + 1e-12);
        coupling_norm = norm(actual(other_rows) ./ row_scale(other_rows));
        active_u = u_sub(u_sub > 1e-12);
        if isempty(active_u)
            min_active = inf;
            active_count = 0;
        else
            min_active = min(active_u);
            active_count = numel(active_u);
        end

        neg_violation = sum(max(0, -u_sub));
        max_duty = max([u_sub(:); 0]);
        high_violation = max(0, max_duty - 1);
        low_violation = sum(max(0, min_duty - active_u));
        fit_tol = Get_Param(params, 'alloc_fit_tol', 1e-6);
        duty_tol = Get_Param(params, 'alloc_duty_tol', 1e-9);
        valid = desired_residual <= fit_tol && neg_violation <= duty_tol && ...
                high_violation <= duty_tol && (isempty(active_u) || min_active >= min_duty - duty_tol);

        result.idx = subset_idx;
        result.u = max(0, min(1, u_sub));
        result.active_count = active_count;
        result.coupling_norm = coupling_norm;
        result.pulse_sum = sum(max(0, u_sub));
        result.max_duty = max_duty;
        result.desired_residual = desired_residual;
        result.valid = valid;
        result.constraint_violation = desired_residual + neg_violation + high_violation + low_violation;
    end

    function strategy = Empty_Strategy(row_idx, cmd)
        strategy = struct('row_idx', row_idx, 'command', cmd, 'thrusters', [], ...
                          'duty', [], 'coupling_norm', NaN, 'desired_residual', NaN, ...
                          'max_duty', 0, 'pulse_sum', 0, 'status', 'empty');
    end

    function Prop_F_axis = Orbit_Axis_Pulse(orbit_axis_duty_raw, orbit_duty, T_ctrl)
        % 轨控复用裁剪需要知道某台推力器的轨控脉宽分别来自哪个轴。
        % 若多个轨控轴复用同一推力器，则按原始轴向占空比比例拆分。
        Prop_F_axis = zeros(size(orbit_axis_duty_raw));
        axis_sum = sum(orbit_axis_duty_raw, 2);
        for ii = 1:size(orbit_axis_duty_raw, 1)
            if axis_sum(ii) > 1e-12 && orbit_duty(ii) > 1e-12
                Prop_F_axis(ii, :) = orbit_axis_duty_raw(ii, :) * orbit_duty(ii) / axis_sum(ii) * T_ctrl;
            end
        end
    end

    %% 任务书推力器复用策略
    function [Prop_Final, Prop_F_used, Prop_T_used, reuse_scale] = Task_Book_Reuse(Prop_F, Prop_T, Prop_F_axis)
        % 任务书策略：
        %   Prop_Zong = Prop_T + Prop_F
        % 若 Prop_Zong(i) <= T，直接输出；若 Prop_Zong(i) > T，
        % 超出量 delta_T 优先从第 i 台轨控脉宽中扣除，并按该推力器
        % 参与的轨控轴比例，把对应 delta_T 同步扣到同轴配对推力器。
        Prop_T_used = min(Prop_T, params.T);
        Prop_F_used = Prop_F;

        for ii = 1:N
            Prop_Zong = Prop_T_used + Prop_F_used;
            delta_T = Prop_Zong(ii) - params.T;
            if delta_T <= 1e-12
                continue;
            end

            Prop_F_used(ii) = max(0, Prop_F_used(ii) - delta_T);

            axis_weight = Prop_F_axis(ii, :);
            active_axes = find(axis_weight > 1e-12);
            if isempty(active_axes)
                continue;
            end

            weight_sum = sum(axis_weight(active_axes));
            for axis = active_axes
                % delta_axis 对应任务书中的
                % delta_T * |T_ocp_axis| / sum(|T_ocp_active_axes|)
                delta_axis = delta_T * axis_weight(axis) / (weight_sum + 1e-12);
                partner_idx = find(Prop_F_axis(:, axis) > 1e-12);
                partner_idx = setdiff(partner_idx, ii);
                for jj = 1:length(partner_idx)
                    partner = partner_idx(jj);
                    Prop_F_used(partner) = max(0, Prop_F_used(partner) - delta_axis);
                end
            end
        end

        Prop_F_used = max(0, min(Prop_F, Prop_F_used));
        Prop_Final = Prop_T_used + Prop_F_used;
        Prop_Final = max(0, min(params.T, Prop_Final));

        reuse_scale = ones(N, 1);
        active_orbit = Prop_F > 1e-12;
        reuse_scale(active_orbit) = Prop_F_used(active_orbit) ./ Prop_F(active_orbit);
    end

    function [Prop_Final, allocation_info] = Full_6D_Allocation( ...
            force_cmd, torque_cmd, matrix, faults, minimum_duty)
        % 对完整六维指令进行有界最小二乘分配。优先使用 lsqlin，
        % 工具箱不可用时退化为正则化伪逆并投影到占空比边界。
        command = [force_cmd(:); torque_cmd(:)];
        lower_bound = zeros(N, 1);
        upper_bound = ones(N, 1);
        upper_bound(faults) = 0;
        scaled_matrix = bsxfun(@rdivide, matrix, row_scale);
        scaled_command = command ./ row_scale;
        energy_weight = Get_Param(params, 'qp_energy_weight', 1e-4);
        duty = [];
        solver = 'regularized_pinv';
        exitflag = 0;

        if exist('lsqlin', 'file') > 0
            try
                objective_matrix = [scaled_matrix; ...
                    sqrt(energy_weight) * eye(N)];
                objective_command = [scaled_command; zeros(N, 1)];
                options = optimoptions('lsqlin', 'Display', 'off');
                [duty, ~, ~, exitflag] = lsqlin(objective_matrix, ...
                    objective_command, [], [], [], [], lower_bound, ...
                    upper_bound, [], options);
                solver = 'lsqlin';
            catch
                duty = [];
            end
        end
        if isempty(duty)
            normal_matrix = scaled_matrix' * scaled_matrix + ...
                energy_weight * eye(N);
            duty = normal_matrix \ (scaled_matrix' * scaled_command);
            duty = max(lower_bound, min(upper_bound, duty));
        end
        duty(~isfinite(duty)) = 0;
        duty(duty > 0 & duty < minimum_duty) = 0;
        duty(faults) = 0;
        Prop_Final = duty * params.T;

        allocation_info.mode = 'joint_6d';
        allocation_info.solver = solver;
        allocation_info.solver_exitflag = exitflag;
        allocation_info.Prop_F = zeros(N, 1);
        allocation_info.Prop_T = zeros(N, 1);
        allocation_info.Prop_F_axis = zeros(N, 3);
        allocation_info.Prop_F_used = zeros(N, 1);
        allocation_info.Prop_T_used = zeros(N, 1);
        allocation_info.Prop_6D = Prop_Final;
        allocation_info.Prop_Final = Prop_Final;
        allocation_info.reuse_scale = ones(N, 1);
        allocation_info.duty = duty;
        allocation_info.command = command;
        allocation_info.actual = matrix * duty;
        allocation_info.residual = command - allocation_info.actual;
        allocation_info.orbit_keep_ratio = 1;
        allocation_info.att_keep_ratio = 1;
        allocation_info.max_raw_pulse = max([Prop_Final; 0]);
        allocation_info.attitude_window = NaN;
        allocation_info.position_window = NaN;
    end

    function value = Get_Param(params, field_name, default_value)
        if isfield(params, field_name) && ~isempty(params.(field_name))
            value = params.(field_name);
        else
            value = default_value;
        end
    end

    function row_scale = Prepare_Row_Scale(row_scale)
        row_scale = row_scale(:);
        if isscalar(row_scale)
            row_scale = row_scale * ones(6, 1);
        end
        if numel(row_scale) ~= 6
            row_scale = max(abs(Matrix_sub), [], 2);
        end
        row_scale(row_scale < 1e-12) = 1;
    end

    function faulty_thrusters = Sanitize_Faults(faulty_thrusters)
        faulty_thrusters = faulty_thrusters(:)';
        faulty_thrusters = faulty_thrusters(faulty_thrusters >= 1 & faulty_thrusters <= N);
        faulty_thrusters = unique(faulty_thrusters);
    end
end
