%% 推力器调用策略对比
function [Prop_Final, info] = Thruster_invocation_decoupled(v_cmd1, v_cmd2, Matrix_sub, faulty_thrusters, params)
    % 仅保留三种策略，便于后续对比：
    %   six_d_qp    : lsqlin/quadprog 一次性求解完整 6D 分配
    %   task_book   : 任务书异步分时复用策略
    %   strict_sync : 严格同步分时复用策略，轨控裁剪沿用任务书策略
    N = size(Matrix_sub, 2);
    mode = Select_Mode(params);
    min_duty = Get_Param(params, 'alloc_min_duty', params.t_min / params.T);

    if strcmpi(mode, 'six_d_qp')
        [Prop_Final, info] = Full_6D_Allocation(v_cmd1, v_cmd2, Matrix_sub, faulty_thrusters, min_duty);
        return;
    end

    [u, Prop_F_axis] = Axis_Split_Allocation(v_cmd1, v_cmd2, Matrix_sub, faulty_thrusters, min_duty);
    Prop_F = u(:, 1) * params.T;
    Prop_T = u(:, 2) * params.T;

    if strcmpi(mode, 'task_book')
        attitude_window = NaN;
        position_window = NaN;
        position_limit = max(0, params.T - min(Prop_T, params.T));
        raw_max_pulse = max(Prop_F + Prop_T);
    elseif strcmpi(mode, 'strict_sync')
        attitude_window = min(max(Prop_T), params.T);
        position_window = max(0, params.T - attitude_window);
        position_limit = position_window * ones(N, 1);
        raw_max_pulse = attitude_window + max(Prop_F);
    else
        error('未知的 Thruster_invocation_decoupled 复用策略: %s', mode);
    end

    [Prop_Final, Prop_F_used, Prop_T_used, reuse_scale] = ...
        Task_Book_Orbit_Clip(Prop_F, Prop_T, Prop_F_axis, position_limit);

    if nargout > 1
        duty = Prop_Final / params.T;
        info.mode = mode;
        info.solver = 'axis_split';
        info.Prop_F = Prop_F;
        info.Prop_T = Prop_T;
        info.Prop_F_axis = Prop_F_axis;
        info.Prop_F_used = Prop_F_used;
        info.Prop_T_used = Prop_T_used;
        info.reuse_scale = reuse_scale;
        info.duty = duty;
        info.command = [v_cmd1(:); v_cmd2(:)];
        info.actual = Matrix_sub * duty;
        info.residual = info.command - info.actual;
        info.orbit_keep_ratio = sum(Prop_F_used) / (sum(Prop_F) + 1e-12);
        info.att_keep_ratio = sum(Prop_T_used) / (sum(Prop_T) + 1e-12);
        info.max_raw_pulse = raw_max_pulse;
        info.attitude_window = attitude_window;
        info.position_window = position_window;
    end

    function [u, Prop_F_axis] = Axis_Split_Allocation(v_cmd1, v_cmd2, Matrix_sub, faulty_thrusters, min_duty)
        u = zeros(N, 2);
        orbit_axis_duty_raw = zeros(N, 3);

        for v_cmd = 1:2
            u_opt = zeros(N, 1);
            for axis = 1:3
                if v_cmd == 1
                    cmd = v_cmd1(axis);
                    Matrix_conf = Matrix_sub(1:3, :);
                else
                    cmd = v_cmd2(axis);
                    Matrix_conf = Matrix_sub(4:6, :);
                end

                if abs(cmd) < 1e-6
                    continue;
                end

                if cmd > 0
                    valid_idx = find(Matrix_conf(axis, :) > 1e-3);
                else
                    valid_idx = find(Matrix_conf(axis, :) < -1e-3);
                end

                if ~isempty(faulty_thrusters)
                    valid_idx = setdiff(valid_idx, faulty_thrusters);
                end

                if isempty(valid_idx)
                    continue;
                end

                eff = abs(Matrix_conf(axis, valid_idx)');
                u_axis = abs(cmd) .* eff ./ sum(eff.^2);
                for jj = 1:length(valid_idx)
                    thruster_idx = valid_idx(jj);
                    u_opt(thruster_idx) = u_opt(thruster_idx) + u_axis(jj);
                    if v_cmd == 1
                        orbit_axis_duty_raw(thruster_idx, axis) = ...
                            orbit_axis_duty_raw(thruster_idx, axis) + u_axis(jj);
                    end
                end
            end

            u_opt(u_opt > 1) = 1;
            u_opt(u_opt > 0 & u_opt < min_duty) = 0;
            u(:, v_cmd) = u_opt;
        end

        Prop_F_axis = Orbit_Axis_Pulse(orbit_axis_duty_raw, u(:, 1), params.T);
    end

    function Prop_F_axis = Orbit_Axis_Pulse(orbit_axis_duty_raw, orbit_duty, T_ctrl)
        Prop_F_axis = zeros(size(orbit_axis_duty_raw));
        axis_sum = sum(orbit_axis_duty_raw, 2);
        for ii = 1:size(orbit_axis_duty_raw, 1)
            if axis_sum(ii) > 1e-12 && orbit_duty(ii) > 1e-12
                Prop_F_axis(ii, :) = orbit_axis_duty_raw(ii, :) * orbit_duty(ii) / axis_sum(ii) * T_ctrl;
            end
        end
    end

    function [Prop_Final, Prop_F_used, Prop_T_used, reuse_scale] = ...
            Task_Book_Orbit_Clip(Prop_F, Prop_T, Prop_F_axis, position_limit)
        Prop_T_used = min(Prop_T, params.T);
        Prop_F_used = Prop_F;
        position_limit = max(0, min(params.T, position_limit(:)));

        for ii = 1:N
            overflow = Prop_F_used(ii) - position_limit(ii);
            if overflow <= 1e-12
                continue;
            end

            Prop_F_used(ii) = max(0, Prop_F_used(ii) - overflow);

            axis_weight = Prop_F_axis(ii, :);
            active_axes = find(axis_weight > 1e-12);
            if isempty(active_axes)
                continue;
            end

            axis_weight = axis_weight / (sum(axis_weight(active_axes)) + 1e-12);
            for axis = active_axes
                partner_idx = find(Prop_F_axis(:, axis) > 1e-12);
                partner_idx = setdiff(partner_idx, ii);
                if isempty(partner_idx)
                    continue;
                end

                delta_axis = overflow * axis_weight(axis);
                for jj = 1:length(partner_idx)
                    p = partner_idx(jj);
                    reduction = min(delta_axis, Prop_F_used(p));
                    Prop_F_used(p) = Prop_F_used(p) - reduction;
                end
            end
        end

        Prop_F_used = max(0, min(Prop_F, min(Prop_F_used, position_limit)));
        reuse_scale = ones(N, 1);
        for ii = 1:N
            if Prop_F(ii) > 1e-12
                reuse_scale(ii) = Prop_F_used(ii) / Prop_F(ii);
            end
        end

        Prop_Final = Prop_T_used + Prop_F_used;
        Prop_Final = max(0, min(params.T, Prop_Final));
    end

    function [Prop_Final, info] = Full_6D_Allocation(v_cmd1, v_cmd2, Matrix_sub, faulty_thrusters, min_duty)
        cmd = [v_cmd1(:); v_cmd2(:)];
        lb = zeros(N, 1);
        ub = ones(N, 1);
        faulty_thrusters = faulty_thrusters(:);
        faulty_thrusters = faulty_thrusters(faulty_thrusters >= 1 & faulty_thrusters <= N);
        if ~isempty(faulty_thrusters)
            ub(faulty_thrusters) = 0;
        end

        row_scale = Get_Param(params, 'alloc_row_scale', max(abs(Matrix_sub), [], 2));
        row_scale = row_scale(:);
        if isscalar(row_scale)
            row_scale = row_scale * ones(6, 1);
        end
        row_scale(row_scale < 1e-12) = 1;

        A = Matrix_sub ./ row_scale;
        b = cmd ./ row_scale;
        energy_weight = Get_Param(params, 'qp_energy_weight', 1e-4);

        u_opt = [];
        exitflag = NaN;
        solver = 'none';

        if exist('lsqlin', 'file') > 0
            try
                C = [A; sqrt(energy_weight) * eye(N)];
                d = [b; zeros(N, 1)];
                options = Make_Optim_Options('lsqlin');
                if isempty(options)
                    [u_opt, ~, ~, exitflag] = lsqlin(C, d, [], [], [], [], lb, ub);
                else
                    [u_opt, ~, ~, exitflag] = lsqlin(C, d, [], [], [], [], lb, ub, [], options);
                end
                solver = 'lsqlin';
            catch
                u_opt = [];
            end
        end

        if isempty(u_opt) && exist('quadprog', 'file') > 0
            try
                H = 2 * (A' * A + energy_weight * eye(N));
                H = 0.5 * (H + H');
                f = -2 * (A' * b);
                options = Make_Optim_Options('quadprog');
                if isempty(options)
                    [u_opt, ~, exitflag] = quadprog(H, f, [], [], [], [], lb, ub);
                else
                    [u_opt, ~, exitflag] = quadprog(H, f, [], [], [], [], lb, ub, [], options);
                end
                solver = 'quadprog';
            catch
                u_opt = [];
            end
        end

        if isempty(u_opt)
            u_opt = A \ b;
            u_opt = min(max(u_opt, lb), ub);
            exitflag = 0;
            solver = 'pinv_clamped';
        end

        u_opt(~isfinite(u_opt)) = 0;
        u_opt = min(max(u_opt, lb), ub);
        u_opt(u_opt > 0 & u_opt < min_duty) = 0;
        if ~isempty(faulty_thrusters)
            u_opt(faulty_thrusters) = 0;
        end

        Prop_Final = u_opt * params.T;
        duty = Prop_Final / params.T;

        info.mode = mode;
        info.solver = solver;
        info.solver_exitflag = exitflag;
        info.Prop_F = zeros(N, 1);
        info.Prop_T = zeros(N, 1);
        info.Prop_6D = Prop_Final;
        info.Prop_F_axis = zeros(N, 3);
        info.Prop_F_used = zeros(N, 1);
        info.Prop_T_used = zeros(N, 1);
        info.reuse_scale = ones(N, 1);
        info.duty = duty;
        info.command = cmd;
        info.actual = Matrix_sub * duty;
        info.residual = cmd - info.actual;
        info.orbit_keep_ratio = 1;
        info.att_keep_ratio = 1;
        info.max_raw_pulse = max(Prop_Final);
        info.attitude_window = NaN;
        info.position_window = NaN;
    end

    function options = Make_Optim_Options(solver_name)
        options = [];
        if exist('optimoptions', 'file') <= 0
            return;
        end
        try
            options = optimoptions(solver_name, 'Display', 'off');
        catch
            options = [];
        end
    end

    function mode = Select_Mode(params)
        if isfield(params, 'reuse_mode') && ~isempty(params.reuse_mode)
            raw_mode = char(params.reuse_mode);
        elseif isfield(params, 'alloc_mode') && ~isempty(params.alloc_mode)
            raw_mode = char(params.alloc_mode);
        else
            raw_mode = 'task_book';
        end

        if any(strcmpi(raw_mode, {'six_d_qp', 'full_6d', 'full6d', 'lsqlin', 'quadprog', 'qp'}))
            mode = 'six_d_qp';
        elseif any(strcmpi(raw_mode, {'task_book', 'mission_book', 'async', 'asynchronous'}))
            mode = 'task_book';
        elseif any(strcmpi(raw_mode, {'strict_sync', 'sync_task_book', 'synchronous', 'strict_time_split'}))
            mode = 'strict_sync';
        else
            error(['未知的 Thruster_invocation_decoupled 策略: %s。', ...
                   '可选: six_d_qp, task_book, strict_sync'], raw_mode);
        end
    end

    function value = Get_Param(params, field_name, default_value)
        if isfield(params, field_name) && ~isempty(params.(field_name))
            value = params.(field_name);
        else
            value = default_value;
        end
    end
end
