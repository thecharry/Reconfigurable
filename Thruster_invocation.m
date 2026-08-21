%% 最简主份/单故障备份推力器调用策略
function [Prop_Final, info] = Thruster_invocation( ...
        v_cmd1, v_cmd2, Matrix_sub, faulty_thrusters, params)
    % Matrix_sub(:,i) 为第 i 台推力器满占空比产生的六维控制效率：
    %   Matrix_sub = [F_x; F_y; F_z; M_x; M_y; M_z].
    %
    % 策略只有两层：
    % 1) 标况：为每个正负轴向选取能够瞬时保持纯力/纯力矩的最少
    %    完整偶数推力器组；
    % 2) 单故障：故障命中主份时，当前完整主份组整体退出，改用健康
    %    的完整偶数备用组，绝不由主份残余成员继续喷气。
    %
    % 健康推力器允许在新的完整备用组中复用；永久关停某一固定搭档
    % 会破坏当前布局部分轴向的单故障重构能力。候选组必须满足
    %   B_S u = d*s_k*e_k, u>0,
    % 因而目标轴之外的五维控制量接近零，从调用层抑制姿轨耦合。

    N = size(Matrix_sub, 2);
    if size(Matrix_sub, 1) ~= 6
        error('Thruster_invocation:InvalidMatrix', ...
            'Matrix_sub 必须是 6xN 控制效率矩阵。');
    end
    command = [v_cmd1(:); v_cmd2(:)];
    if numel(command) ~= 6
        error('Thruster_invocation:InvalidCommand', ...
            '轨控和姿控指令必须分别包含三个分量。');
    end

    faulty_thrusters = Sanitize_Faults(faulty_thrusters);
    min_duty = Get_Param(params, 'alloc_min_duty', ...
        Get_Param(params, 't_min', 0.02) / params.T);
    row_scale = Prepare_Row_Scale(Get_Param(params, ...
        'alloc_row_scale', max(abs(Matrix_sub), [], 2)));
    config = Allocation_Config();

    % 布局不变时复用主份和故障备份表。故障表按故障集合按需建立，
    % 在线控制阶段只需查表和按指令幅值缩放占空比。
    persistent strategy_cache
    cache_idx = Find_Cache_Entry(strategy_cache, Matrix_sub, config);
    if cache_idx == 0
        [nominal_bank, nominal_usage] = Build_Nominal_Bank();
        cache_entry = struct('matrix', Matrix_sub, 'config', config, ...
            'nominal_bank', {nominal_bank}, ...
            'nominal_usage', nominal_usage, ...
            'fault_keys', {{}}, 'fault_banks', {{}});
        if isempty(strategy_cache)
            strategy_cache = cache_entry;
        else
            strategy_cache(end + 1) = cache_entry;
        end
        cache_limit = max(1, round(Get_Param(params, ...
            'alloc_cache_size', 6)));
        if numel(strategy_cache) > cache_limit
            strategy_cache(1:(numel(strategy_cache) - cache_limit)) = [];
        end
        cache_idx = numel(strategy_cache);
    end

    nominal_bank = strategy_cache(cache_idx).nominal_bank;
    nominal_usage = strategy_cache(cache_idx).nominal_usage;
    if isempty(faulty_thrusters)
        active_bank = nominal_bank;
    else
        fault_key = sprintf('%d,', faulty_thrusters);
        fault_idx = find(strcmp( ...
            strategy_cache(cache_idx).fault_keys, fault_key), 1);
        if isempty(fault_idx)
            fault_bank = Build_Fault_Bank( ...
                faulty_thrusters, nominal_bank, nominal_usage);
            strategy_cache(cache_idx).fault_keys{end + 1} = fault_key;
            strategy_cache(cache_idx).fault_banks{end + 1} = fault_bank;
            fault_idx = numel(strategy_cache(cache_idx).fault_banks);
        end
        active_bank = strategy_cache(cache_idx).fault_banks{fault_idx};
    end

    % 先为每一个有指令的轴保存一整组“纯轴向”占空比模板。这里不做
    % 单台饱和或单台最小脉宽截断；否则会改变组内各台的比例，原本的
    % 力/力矩配平将不再成立。
    [orbit_axis_duty, attitude_axis_duty, axis_strategy] = ...
        Axis_Split_Allocation(command, active_bank, nominal_bank);

    % 任务书的姿控优先在这里以“整组同倍率缩放”实现：
    %
    %   u_T = lambda_T * sum_j(u_T,j),
    %   u_F = lambda_F * sum_j(u_F,j),
    %   0 <= lambda_T, lambda_F <= 1.
    %
    % 每个 u_T,j、u_F,j 都是纯轴向模板。最小脉宽筛选后保留下来的
    % 指令分别记为 F_sel、T_sel，故缩放后仍满足
    %   B*u_F = [lambda_F*F_sel; 0],
    %   B*u_T = [0; lambda_T*T_sel].
    % 先为姿控留足硬件脉宽；仅当保留姿控组无需容量缩放时，才将剩余
    % 脉宽分配给轨控。
    [u_F_raw, u_T_raw, ~, ~, ...
            orbit_axis_duty, attitude_axis_duty, axis_strategy, scaling] = ...
        Decoupled_Group_Scaling(orbit_axis_duty, attitude_axis_duty, ...
        axis_strategy, min_duty);
    Prop_F = u_F_raw * params.T;
    Prop_T = u_T_raw * params.T;

    % 分配层只确定“本周期每台喷多久”。时序层进一步决定“何时喷”：
    % 同一个纯轴向组中的推力器被安排到共同起点，且同一台推力器的
    % 不同组不允许重叠。这样可消除标况等脉宽组因全部 t=0 起喷造成的
    % 瞬时残余；故障备用组若无法满足峰值力矩约束，则整条轨控轴停用。
    Prop_F_axis_raw = scaling.force_scale * orbit_axis_duty * params.T;
    Prop_T_axis_raw = scaling.attitude_scale * attitude_axis_duty * params.T;
    has_impulse_margin_fallback = Has_Impulse_Margin_Fallback(axis_strategy);
    scheduler_params = params;
    scheduler_params.scheduler_allow_odd_coupled_groups = ...
        has_impulse_margin_fallback;
    scheduler_params.scheduler_use_impulse_margin_fallback = ...
        has_impulse_margin_fallback;
    [pulse_schedule, Prop_F_axis, Prop_T_axis, scheduler_info] = ...
        Thruster_pulse_scheduler(Matrix_sub, Prop_F_axis_raw, ...
        Prop_T_axis_raw, scheduler_params);
    Prop_F_used = sum(Prop_F_axis, 2);
    Prop_T_used = sum(Prop_T_axis, 2);
    Prop_Final = pulse_schedule.total_pulse;
    u_F_used = Prop_F_used / params.T;
    u_T_used = Prop_T_used / params.T;

    for axis_idx = scheduler_info.suppressed_force_axes(:)'
        strategy = axis_strategy{1, axis_idx};
        strategy.applied_duty = zeros(size(strategy.duty));
        strategy.applied_scale = 0;
        strategy.status = 'peak_constraint_suppressed';
        axis_strategy{1, axis_idx} = strategy;
    end

    reuse_scale = ones(N, 1);
    force_active = Prop_F > config.duty_tol * params.T;
    reuse_scale(force_active) = Prop_F_used(force_active) ./ ...
        Prop_F(force_active);

    if nargout > 1
        duty = Prop_Final / params.T;
        info.mode = 'strict_paired_with_impulse_margin_backup';
        info.solver = 'paired_primary_strict_backup_impulse_margin_fallback';
        info.fault_policy = 'retire_affected_complete_group';
        info.strict_paired_mode = config.require_instantaneous_group;
        info.uses_impulse_margin_fallback = has_impulse_margin_fallback;
        info.nominal_reuse_count = nominal_usage;
        info.structurally_reconfigurable = Bank_Reconfigurable(active_bank);
        info.strictly_reconfigurable = Bank_Strict_Reconfigurable(active_bank);
        info.Prop_F = Prop_F;
        info.Prop_T = Prop_T;
        info.Prop_F_axis = Prop_F_axis;
        info.Prop_F_axis_raw = Prop_F_axis_raw;
        info.Prop_T_axis = Prop_T_axis;
        info.Prop_T_axis_raw = Prop_T_axis_raw;
        info.axis_strategy = axis_strategy;
        info.impulse_margin_unavailable_axes = ...
            Impulse_Margin_Unavailable_Axes(axis_strategy);
        info.Prop_F_used = Prop_F_used;
        info.Prop_T_used = Prop_T_used;
        info.Prop_6D = zeros(N, 1);
        info.reuse_scale = reuse_scale;
        info.schedule = pulse_schedule;
        info.scheduler = scheduler_info;
        info.scheduler_force_axis_suppressed = ...
            scheduler_info.suppressed_force_axes;
        info.force_scale = scaling.force_scale;
        info.attitude_scale = scaling.attitude_scale;
        info.force_scale_reason = scaling.force_reason;
        info.attitude_scale_reason = scaling.attitude_reason;
        info.orbit_wrench_raw = Matrix_sub * u_F_raw;
        info.attitude_wrench_raw = Matrix_sub * u_T_raw;
        info.orbit_wrench = Matrix_sub * u_F_used;
        info.attitude_wrench = Matrix_sub * u_T_used;
        info.orbit_cross_torque = norm(info.orbit_wrench(4:6));
        info.attitude_cross_force = norm(info.attitude_wrench(1:3));
        info.period_average_decoupled = ...
            info.orbit_cross_torque <= config.fit_tol && ...
            info.attitude_cross_force <= config.fit_tol;
        info.duty = duty;
        info.command = command;
        info.actual = Matrix_sub * duty;
        info.residual = command - info.actual;
        info.orbit_keep_ratio = ...
            sum(Prop_F_used) / (sum(Prop_F) + 1e-12);
        info.att_keep_ratio = scaling.attitude_scale;
        info.max_raw_pulse = max(Prop_F + Prop_T);
        info.attitude_window = NaN;
        info.position_window = NaN;
    end

    function config = Allocation_Config()
        config.max_group_size = min(N, max(1, round(Get_Param( ...
            params, 'alloc_max_group_size', 6))));
        config.candidate_limit = max(8, round(Get_Param( ...
            params, 'alloc_candidate_limit', 128)));
        config.fit_tol = Get_Param(params, 'alloc_fit_tol', 1e-8);
        config.unit_duty_tol = Get_Param( ...
            params, 'alloc_unit_duty_tol', 1e-10);
        config.command_tol = Get_Param(params, 'alloc_command_tol', 1e-6);
        config.duty_tol = Get_Param(params, 'alloc_duty_tol', 1e-9);
        config.even_group_only = logical(Get_Param(params, ...
            'alloc_even_group_only', true));
        config.prefer_instantaneous_group = logical(Get_Param(params, ...
            'alloc_prefer_instantaneous_group', true));
        config.require_instantaneous_group = logical(Get_Param(params, ...
            'alloc_require_instantaneous_group', true));
        config.instantaneous_tol = Get_Param(params, ...
            'alloc_instantaneous_tol', config.fit_tol);
        config.impulse_margin_fallback = logical(Get_Param(params, ...
            'alloc_impulse_margin_fallback', true));
        config.impulse_margin_limit = Get_Param(params, ...
            'alloc_impulse_margin_limit', 0.80);
        config.impulse_margin_force_weight = Get_Param(params, ...
            'alloc_impulse_margin_force_weight', 0.35);
        config.impulse_candidate_limit = max(config.candidate_limit, ...
            round(Get_Param(params, 'alloc_impulse_candidate_limit', 512)));
        config.row_scale = row_scale;
    end

    function [bank, usage_count] = Build_Nominal_Bank()
        bank = Build_Raw_Bank([], false);
        usage_count = zeros(N, 1);

        % 候选数少的轴向优先选主份。每个轴向只在“最少台数候选”
        % 之间比较，先减少与已选主份的重叠，再比较总脉宽。
        channel_rows = repelem((1:6)', 2);
        channel_cols = repmat((1:2)', 6, 1);
        channel_num = numel(channel_rows);
        difficulty = inf(channel_num, 1);
        for channel_idx = 1:channel_num
            entry = bank{channel_rows(channel_idx), ...
                channel_cols(channel_idx)};
            difficulty(channel_idx) = numel(entry.candidates);
        end
        [~, channel_order] = sort(difficulty, 'ascend');

        for order_idx = 1:channel_num
            channel_idx = channel_order(order_idx);
            row_idx = channel_rows(channel_idx);
            direction_col = channel_cols(channel_idx);
            entry = bank{row_idx, direction_col};
            if isempty(entry.candidates)
                continue;
            end

            rank = zeros(numel(entry.candidates), 4);
            for candidate_idx = 1:numel(entry.candidates)
                group = entry.candidates(candidate_idx).group;
                trial_usage = usage_count;
                trial_usage(group) = trial_usage(group) + 1;
                rank(candidate_idx, :) = [ ...
                    sum(usage_count(group)), ...
                    max(trial_usage), ...
                    entry.candidates(candidate_idx).pulse_sum_unit, ...
                    candidate_idx];
            end
            [~, order] = sortrows(rank, [1 2 3 4]);
            entry.preferred_idx = order(1);
            entry.nominal_group = ...
                entry.candidates(entry.preferred_idx).group;
            usage_count(entry.nominal_group) = ...
                usage_count(entry.nominal_group) + 1;
            bank{row_idx, direction_col} = entry;
        end
    end

    function bank = Build_Fault_Bank(faults, nominal, nominal_usage)
        bank = Build_Raw_Bank(faults, config.impulse_margin_fallback);
        for row_idx = 1:6
            for direction_col = 1:2
                entry = bank{row_idx, direction_col};
                nominal_entry = nominal{row_idx, direction_col};
                if nominal_entry.preferred_idx > 0
                    entry.nominal_group = nominal_entry.candidates( ...
                        nominal_entry.preferred_idx).group;
                end
                if isempty(entry.candidates)
                    bank{row_idx, direction_col} = entry;
                    continue;
                end

                % 冲量裕度候选需要结合实时指令幅值、反向储备和最小
                % 脉宽逐一评分，不能在离线建表阶段固定某一个主份。
                if entry.uses_impulse_margin_fallback
                    entry.preferred_idx = 0;
                    bank{row_idx, direction_col} = entry;
                    continue;
                end

                % 故障未命中主份时保持原主份，避免无谓切换。
                if ~any(ismember(entry.nominal_group, faults))
                    same_idx = Find_Group( ...
                        entry.candidates, entry.nominal_group);
                    if same_idx > 0
                        entry.preferred_idx = same_idx;
                        bank{row_idx, direction_col} = entry;
                        continue;
                    end
                end

                % 故障命中主份时，从健康最小组中优先选择较少占用其他
                % 标况轴向主份的组合，随后比较所需脉宽。
                rank = zeros(numel(entry.candidates), 4);
                for candidate_idx = 1:numel(entry.candidates)
                    group = entry.candidates(candidate_idx).group;
                    rank(candidate_idx, :) = [ ...
                        sum(nominal_usage(group)), ...
                        max(nominal_usage(group)), ...
                        entry.candidates(candidate_idx).pulse_sum_unit, ...
                        candidate_idx];
                end
                [~, order] = sortrows(rank, [1 2 3 4]);
                entry.preferred_idx = order(1);
                bank{row_idx, direction_col} = entry;
            end
        end
    end

    function bank = Build_Raw_Bank(blocked_thrusters, allow_impulse_fallback)
        healthy = setdiff(1:N, blocked_thrusters);
        bank = cell(6, 2);
        for row_idx = 1:6
            for direction_col = 1:2
                direction = 2 * direction_col - 3;
                [candidates, uses_impulse_margin_fallback] = ...
                    Build_Minimal_Candidates(row_idx, direction, healthy, ...
                    allow_impulse_fallback);
                bank{row_idx, direction_col} = struct( ...
                    'direction', direction, ...
                    'candidates', candidates, ...
                    'preferred_idx', double(~isempty(candidates)), ...
                    'nominal_group', [], ...
                    'uses_impulse_margin_fallback', ...
                    uses_impulse_margin_fallback);
            end
        end
    end

    function [candidates, uses_impulse_margin_fallback] = ...
            Build_Minimal_Candidates(row_idx, direction, healthy, ...
            allow_impulse_fallback)
        % 候选搜索分为两个互不混淆的层级：
        %
        % 严格层：仅偶数台，并要求每个开关阶段均为纯目标轴向；
        %         这是标况主份和优先故障备份。
        % 裕度层：仅在严格层不存在且故障允许时启用。它保留周期平均
        %         纯控制约束 B_S*u=d*e_k，但不限制台数奇偶；在线阶段
        %         再依据实际脉冲引起的累计干扰冲量进行筛选。
        candidates = Build_Candidate_Pool(row_idx, direction, healthy, ...
            true, true);
        uses_impulse_margin_fallback = false;
        if ~isempty(candidates) || ~allow_impulse_fallback
            return;
        end

        candidates = Build_Candidate_Pool(row_idx, direction, healthy, ...
            false, false);
        uses_impulse_margin_fallback = ~isempty(candidates);
    end

    function candidates = Build_Candidate_Pool(row_idx, direction, healthy, ...
            require_even_group, require_instantaneous)
        normalized_matrix = bsxfun(@rdivide, Matrix_sub, row_scale);
        target = zeros(6, 1);
        target(row_idx) = direction;
        candidates = repmat(Empty_Candidate(), 0, 1);

        if require_even_group
            group_sizes = 2:2:min(config.max_group_size, numel(healthy));
        else
            group_sizes = 1:min(config.max_group_size, numel(healthy));
        end

        for group_size = group_sizes
            combinations = nchoosek(healthy, group_size);
            size_candidates = repmat(Empty_Candidate(), 0, 1);
            for combination_idx = 1:size(combinations, 1)
                group = combinations(combination_idx, :);
                A = normalized_matrix(:, group);
                unit_duty = pinv(A) * target;
                actual = A * unit_duty;
                residual = actual - target;
                if any(~isfinite(unit_duty)) || ...
                        any(unit_duty <= config.unit_duty_tol) || ...
                        norm(residual) > config.fit_tol
                    continue;
                end

                other_rows = setdiff(1:6, row_idx);
                candidate = Empty_Candidate();
                candidate.group = group;
                candidate.unit_duty = unit_duty;
                candidate.group_size = group_size;
                candidate.coupling_norm = norm(actual(other_rows));
                candidate.target_residual = abs(residual(row_idx));
                candidate.pulse_sum_unit = sum(unit_duty);
                candidate.max_duty_unit = max(unit_duty);
                [candidate.instantaneous_pure, ...
                    candidate.instantaneous_mode_count] = ...
                    Is_Instantaneously_Pure_Group( ...
                    group, unit_duty, row_idx, direction, normalized_matrix);
                candidate.pair_count = floor(group_size / 2);
                candidate.is_impulse_margin_fallback = ...
                    ~require_instantaneous;
                candidate.valid = true;
                size_candidates(end + 1, 1) = candidate; %#ok<AGROW>
            end

            if require_instantaneous
                instantaneous_mask = [size_candidates.instantaneous_pure];
                if any(instantaneous_mask)
                    candidates = size_candidates(instantaneous_mask);
                    break;
                end
            else
                candidates = [candidates; size_candidates]; %#ok<AGROW>
            end
        end

        if isempty(candidates)
            return;
        end
        if require_instantaneous
            rank = [[candidates.coupling_norm]', ...
                [candidates.pulse_sum_unit]', ...
                [candidates.max_duty_unit]'];
            [~, order] = sortrows(rank, [1 2 3]);
        else
            % 在线的冲量裕度评分会决定最终备份；此处仅保留规模较小、
            % 单位脉宽较短的一批候选，控制离线组合搜索开销。
            rank = [[candidates.group_size]', ...
                [candidates.pulse_sum_unit]', ...
                [candidates.max_duty_unit]'];
            [~, order] = sortrows(rank, [1 2 3]);
        end
        candidates = candidates(order);
        if require_instantaneous
            candidate_limit = config.candidate_limit;
        else
            candidate_limit = config.impulse_candidate_limit;
        end
        candidates = candidates(1:min(numel(candidates), candidate_limit));
    end

    function [is_pure, mode_count] = Is_Instantaneously_Pure_Group( ...
            group, unit_duty, row_idx, direction, normalized_matrix)
        % 同一起点喷气时，脉宽较短的成员会先关断。逐一检查每个
        % 关断阶段仍处于开启状态的集合，只有每个集合均为偶数台且
        % 仅产生目标轴向时，才称该调用组为瞬时纯控制组。
        is_pure = true;
        mode_count = 0;
        positive = unit_duty > config.unit_duty_tol;
        levels = unique(unit_duty(positive));
        other_rows = setdiff(1:6, row_idx);
        for level_idx = 1:numel(levels)
            active = unit_duty >= levels(level_idx) - config.unit_duty_tol;
            active_group = group(active);
            if config.even_group_only && mod(numel(active_group), 2) ~= 0
                is_pure = false;
                return;
            end
            instantaneous_wrench = sum( ...
                normalized_matrix(:, active_group), 2);
            if direction * instantaneous_wrench(row_idx) <= ...
                    config.instantaneous_tol || ...
                    norm(instantaneous_wrench(other_rows)) > ...
                    config.instantaneous_tol
                is_pure = false;
                return;
            end
            mode_count = mode_count + 1;
        end
    end

    function [orbit_axis_duty, attitude_axis_duty, axis_strategy] = ...
            Axis_Split_Allocation(cmd, bank, nominal)
        % 列 axis 保存一个完整的纯轴向调用组，不同列之间允许复用同一
        % 推力器。后续只能对整类（全部轨控组或全部姿控组）乘同一倍率，
        % 绝不能对列中的某一台单独截断。
        orbit_axis_duty = zeros(N, 3);
        attitude_axis_duty = zeros(N, 3);
        axis_strategy = cell(2, 3);

        for control_type = 1:2
            for axis = 1:3
                row_idx = axis + 3 * (control_type - 1);
                axis_cmd = cmd(row_idx);
                if abs(axis_cmd) < config.command_tol
                    axis_strategy{control_type, axis} = ...
                        Empty_Strategy(row_idx, axis_cmd);
                    continue;
                end

                [u_axis, strategy] = Select_Axis_Strategy( ...
                    row_idx, axis_cmd, bank, nominal);
                axis_strategy{control_type, axis} = strategy;
                if control_type == 1
                    orbit_axis_duty(:, axis) = u_axis;
                else
                    attitude_axis_duty(:, axis) = u_axis;
                end
            end
        end
    end

    function [u_axis, strategy] = Select_Axis_Strategy( ...
            row_idx, cmd, bank, nominal)
        u_axis = zeros(N, 1);
        strategy = Empty_Strategy(row_idx, cmd);
        direction_col = Direction_Column(sign(cmd));
        entry = bank{row_idx, direction_col};
        nominal_entry = nominal{row_idx, direction_col};

        if nominal_entry.preferred_idx > 0
            nominal_group = nominal_entry.candidates( ...
                nominal_entry.preferred_idx).group;
        else
            nominal_group = [];
        end
        strategy.nominal_thrusters = nominal_group;
        strategy.blocked_thrusters = faulty_thrusters;
        strategy.structurally_reconfigurable = ...
            ~isempty(entry.candidates);
        if isempty(entry.candidates)
            strategy.status = 'unreconfigurable_fault';
            return;
        end

        order = 1:numel(entry.candidates);
        if entry.preferred_idx > 0
            order = [entry.preferred_idx, ...
                setdiff(order, entry.preferred_idx, 'stable')];
        end

        command_scale = abs(cmd) / row_scale(row_idx);

        % 候选已经按“标况主份/故障备份”的优先级排好。此处故意不做
        % 0~1 限幅：限幅应在六轴叠加后统一进行，才能保留组内比例。
        impulse_margin = Empty_Impulse_Margin();
        if entry.uses_impulse_margin_fallback
            [selected_idx, impulse_margin] = ...
                Select_Impulse_Margin_Candidate( ...
                entry.candidates, row_idx, command_scale, bank);
            if selected_idx == 0
                strategy.status = 'impulse_margin_unavailable';
                strategy.structurally_reconfigurable = false;
                strategy.impulse_margin = impulse_margin;
                return;
            end
        else
            selected_idx = order(1);
        end
        selected = entry.candidates(selected_idx);
        selected_duty = command_scale * selected.unit_duty;
        u_axis(selected.group) = selected_duty;
        actual = Matrix_sub * u_axis;
        other_rows = setdiff(1:6, row_idx);
        fault_hits_primary = any(ismember(nominal_group, faulty_thrusters));
        if entry.uses_impulse_margin_fallback
            selected_status = 'fault_impulse_margin_backup';
        elseif fault_hits_primary
            if selected.instantaneous_pure
                selected_status = 'fault_paired_backup_group';
            else
                selected_status = 'fault_even_average_backup';
            end
        elseif isequal(sort(selected.group), sort(nominal_group))
            if selected.instantaneous_pure
                selected_status = 'nominal_paired_instantaneous_group';
            else
                selected_status = 'nominal_even_average_group';
            end
        else
            if selected.instantaneous_pure
                selected_status = 'command_paired_backup_group';
            else
                selected_status = 'command_even_average_backup';
            end
        end
        strategy.thrusters = selected.group;
        strategy.duty = selected_duty;
        strategy.applied_duty = selected_duty;
        strategy.active_count = numel(selected.group);
        strategy.coupling_norm = ...
            norm(actual(other_rows) ./ row_scale(other_rows));
        strategy.desired_residual = ...
            abs(actual(row_idx) - cmd) / (abs(cmd) + 1e-12);
        strategy.max_duty = max(selected_duty);
        strategy.pulse_sum = sum(selected_duty);
        strategy.pair_count = selected.pair_count;
        strategy.instantaneous_pure = selected.instantaneous_pure;
        strategy.instantaneous_mode_count = ...
            selected.instantaneous_mode_count;
        strategy.is_impulse_margin_fallback = ...
            entry.uses_impulse_margin_fallback;
        strategy.impulse_margin = impulse_margin;
        strategy.retired_primary_thrusters = [];
        if fault_hits_primary
            % 故障时不允许主份内未故障的成员以残缺组方式续喷。当前
            % 主份整组退役；若健康成员在备用组中再次出现，它属于新组，
            % 而非故障组的残余部分。
            strategy.retired_primary_thrusters = nominal_group;
        end
        strategy.reconfigured = ...
            ~isequal(sort(selected.group), sort(nominal_group));
        strategy.status = selected_status;
    end

    function [selected_idx, selected_margin] = ...
            Select_Impulse_Margin_Candidate(candidates, row_idx, ...
            command_scale, bank)
        % 对周期平均纯控制候选，按实际共同起喷后的累计干扰冲量评分：
        %
        %   score = max_j,t |J_dist,j(t)| / J_opp,reserve,j.
        %
        % J_opp,reserve,j 是故障后尚可用于反向抵消第 j 轴扰动的严格
        % 纯控制冲量。只有 score 不超过阈值的候选才能成为备用组。
        selected_idx = 0;
        selected_margin = Empty_Impulse_Margin();
        best_rank = inf(1, 4);
        for candidate_idx = 1:numel(candidates)
            candidate = candidates(candidate_idx);
            duty = command_scale * candidate.unit_duty;
            pulse = duty * params.T;
            if any(pulse < min_duty - config.duty_tol)
                continue;
            end

            margin = Evaluate_Impulse_Margin( ...
                candidate, duty, row_idx, bank);
            if ~margin.feasible
                continue;
            end
            rank = [margin.score, sum(duty), ...
                candidate.group_size, max(duty)];
            if Is_Better_Impulse_Rank(rank, best_rank)
                selected_idx = candidate_idx;
                selected_margin = margin;
                best_rank = rank;
            end
        end
    end

    function tf = Is_Better_Impulse_Rank(rank, best_rank)
        % 先最小化裕度消耗，再依次最小化总脉宽、台数和最大占空比。
        tf = false;
        for item_idx = 1:numel(rank)
            if rank(item_idx) < best_rank(item_idx) - config.duty_tol
                tf = true;
                return;
            end
            if rank(item_idx) > best_rank(item_idx) + config.duty_tol
                return;
            end
        end
    end

    function margin = Evaluate_Impulse_Margin(candidate, duty, row_idx, bank)
        [positive_peak, negative_peak] = Cross_Impulse_Peaks( ...
            candidate.group, duty, row_idx);
        occupied_duty = zeros(N, 1);
        occupied_duty(candidate.group) = duty;
        cross_rows = setdiff(1:6, row_idx);
        ratio = zeros(6, 1);
        reserve_positive = zeros(6, 1);
        reserve_negative = zeros(6, 1);

        for cross_row = cross_rows
            % 正干扰需要负向严格纯控制抵消；负干扰则相反。
            if positive_peak(cross_row) > config.duty_tol
                reserve_negative(cross_row) = ...
                    Directional_Impulse_Reserve( ...
                    cross_row, -1, bank, occupied_duty);
                ratio(cross_row) = max(ratio(cross_row), ...
                    positive_peak(cross_row) / ...
                    (reserve_negative(cross_row) + eps));
            end
            if negative_peak(cross_row) > config.duty_tol
                reserve_positive(cross_row) = ...
                    Directional_Impulse_Reserve( ...
                    cross_row, 1, bank, occupied_duty);
                ratio(cross_row) = max(ratio(cross_row), ...
                    negative_peak(cross_row) / ...
                    (reserve_positive(cross_row) + eps));
            end
        end

        weights = ones(6, 1);
        % 非目标的平动力一般允许比姿态扰动占用更少的资源；权重仍可
        % 由参数调整，且每个方向最终都必须具备反向纯控制储备。
        weights(1:3) = config.impulse_margin_force_weight;
        weighted_ratio = ratio .* weights;
        margin = Empty_Impulse_Margin();
        margin.positive_peak = positive_peak;
        margin.negative_peak = negative_peak;
        margin.reserve_positive = reserve_positive;
        margin.reserve_negative = reserve_negative;
        margin.axis_ratio = ratio;
        margin.score = max(weighted_ratio(cross_rows));
        margin.feasible = isfinite(margin.score) && ...
            margin.score <= config.impulse_margin_limit + config.duty_tol;
    end

    function [positive_peak, negative_peak] = Cross_Impulse_Peaks( ...
            group, duty, row_idx)
        % 组内推力器共同起喷、按各自脉宽依次关断。对每个时段积分
        % 瞬时六维控制量，记录非目标轴向累计冲量的正负峰值。
        pulse = duty(:) * params.T;
        event_time = unique([0; pulse]);
        cumulative_impulse = zeros(6, 1);
        positive_peak = zeros(6, 1);
        negative_peak = zeros(6, 1);
        for interval_idx = 1:(numel(event_time) - 1)
            start_time = event_time(interval_idx);
            end_time = event_time(interval_idx + 1);
            active = pulse > start_time + config.duty_tol;
            if ~any(active)
                continue;
            end
            instantaneous_wrench = sum( ...
                Matrix_sub(:, group(active)), 2);
            cumulative_impulse = cumulative_impulse + ...
                instantaneous_wrench * (end_time - start_time);
            positive_peak = max(positive_peak, cumulative_impulse);
            negative_peak = max(negative_peak, -cumulative_impulse);
        end
        positive_peak(row_idx) = 0;
        negative_peak(row_idx) = 0;
    end

    function reserve = Directional_Impulse_Reserve( ...
            row_idx, direction, bank, occupied_duty)
        % 从故障后的严格纯控制候选中，寻找扣除当前备用组已占用脉宽
        % 后仍可给出最大反向补偿冲量的一组。耦合候选不能作为这里的
        % 储备，避免用新的干扰去抵消已有干扰。
        entry = bank{row_idx, Direction_Column(direction)};
        reserve = 0;
        for candidate_idx = 1:numel(entry.candidates)
            candidate = entry.candidates(candidate_idx);
            if ~candidate.instantaneous_pure
                continue;
            end
            available = max(0, 1 - occupied_duty(candidate.group));
            scale = min(available ./ candidate.unit_duty);
            if ~isfinite(scale) || scale <= config.duty_tol
                continue;
            end
            compensation = Matrix_sub(:, candidate.group) * ...
                (scale * candidate.unit_duty);
            reserve = max(reserve, abs(compensation(row_idx)) * params.T);
        end
    end

    function [u_F_raw, u_T_raw, u_F_used, u_T_used, ...
            orbit_axis, attitude_axis, strategies, scaling] = ...
            Decoupled_Group_Scaling(orbit_axis, attitude_axis, strategies, duty_min)
        % 本函数是调用策略的执行层。它不修改任意一个组内部的相对
        % 脉宽，只会删除整组，或将同一控制类别的全部组乘以同一个
        % 标量。因此纯轨控组仍只产生力，纯姿控组仍只产生力矩。
        [attitude_axis, strategies, lambda_T, attitude_reason] = ...
            Fit_Group_Family(attitude_axis, strategies, 2, ...
            ones(N, 1), duty_min);
        u_T_raw = sum(attitude_axis, 2);
        u_T_used = lambda_T * u_T_raw;

        if lambda_T < 1 - config.duty_tol
            % 姿控需求本身已超出一个周期的可执行范围。按照姿控硬优先，
            % 本周期停掉全部轨控组；姿控仍保持原三轴方向的同比例缩放。
            [orbit_axis, strategies] = Suppress_Orbit_Groups( ...
                orbit_axis, strategies);
            lambda_F = 0;
            force_reason = 'attitude_capacity_priority';
        else
            remaining_duty = max(0, 1 - u_T_used);
            [orbit_axis, strategies, lambda_F, force_reason] = ...
                Fit_Group_Family(orbit_axis, strategies, 1, ...
                remaining_duty, duty_min);
        end
        u_F_raw = sum(orbit_axis, 2);
        u_F_used = lambda_F * u_F_raw;

        scaling = struct('attitude_scale', lambda_T, ...
            'force_scale', lambda_F, ...
            'attitude_reason', attitude_reason, ...
            'force_reason', force_reason);
    end

    function [axis_duty, strategies, scale, reason] = Fit_Group_Family( ...
            axis_duty, strategies, control_type, available_duty, duty_min)
        % 求所有该类控制组共享的最大倍率
        %
        %   scale = min(1, min_i available_duty(i)/sum_j(u_j(i))).
        %
        % 因为每个组与每个轴都使用同一 scale，六维配平方程在缩放前后
        % 同时成立。若缩放后某组任一成员低于最小脉宽，则丢弃整组，再
        % 重新求倍率，直至稳定。
        reason = 'full_command';
        for iteration = 1:(size(axis_duty, 2) + 1)
            raw_duty = sum(axis_duty, 2);
            scale = Capacity_Scale(raw_duty, available_duty);
            [axis_duty, strategies, group_removed] = ...
                Drop_Undersized_Groups(axis_duty, strategies, ...
                control_type, scale, duty_min);
            if ~group_removed
                break;
            end
            reason = 'minimum_pulse_group_removed';
        end
        raw_duty = sum(axis_duty, 2);
        scale = Capacity_Scale(raw_duty, available_duty);
        if scale < 1 - config.duty_tol
            reason = 'hardware_capacity_scaled';
        end
        strategies = Mark_Family_Scale(strategies, control_type, scale);
    end

    function scale = Capacity_Scale(raw_duty, available_duty)
        active = raw_duty > config.duty_tol;
        if ~any(active)
            scale = 1;
            return;
        end
        scale = min(1, min(available_duty(active) ./ raw_duty(active)));
        scale = max(0, scale);
    end

    function [axis_duty, strategies, removed] = ...
            Drop_Undersized_Groups(axis_duty, strategies, ...
            control_type, scale, duty_min)
        removed = false;
        if duty_min <= config.duty_tol
            return;
        end
        for axis = 1:size(axis_duty, 2)
            group_duty = axis_duty(:, axis);
            active = group_duty > config.duty_tol;
            if ~any(active) || ...
                    all(scale * group_duty(active) >= duty_min - config.duty_tol)
                continue;
            end

            % 最小脉宽不能通过提高组内某台的脉宽来满足；那会在这一
            % 周期额外引入力/力矩。整组跳过仍保持已执行控制的纯轴向性。
            axis_duty(:, axis) = 0;
            strategy = strategies{control_type, axis};
            strategy.applied_duty = zeros(size(strategy.duty));
            strategy.applied_scale = 0;
            strategy.status = 'below_minimum_pulse';
            strategies{control_type, axis} = strategy;
            removed = true;
        end
    end

    function strategies = Mark_Family_Scale(strategies, control_type, scale)
        for axis = 1:3
            strategy = strategies{control_type, axis};
            if isempty(strategy) || isempty(strategy.duty) || ...
                    strcmp(strategy.status, 'below_minimum_pulse') || ...
                    strcmp(strategy.status, 'attitude_priority_suppressed')
                continue;
            end
            strategy.applied_scale = scale;
            strategy.applied_duty = scale * strategy.duty;
            strategies{control_type, axis} = strategy;
        end
    end

    function [orbit_axis, strategies] = Suppress_Orbit_Groups( ...
            orbit_axis, strategies)
        orbit_axis(:) = 0;
        for axis = 1:3
            strategy = strategies{1, axis};
            if isempty(strategy) || isempty(strategy.duty)
                continue;
            end
            strategy.applied_duty = zeros(size(strategy.duty));
            strategy.applied_scale = 0;
            strategy.status = 'attitude_priority_suppressed';
            strategies{1, axis} = strategy;
        end
    end

    function tf = Bank_Reconfigurable(bank)
        tf = true;
        for row_idx = 1:6
            for direction_col = 1:2
                if isempty(bank{row_idx, direction_col}.candidates)
                    tf = false;
                    return;
                end
            end
        end
    end

    function tf = Bank_Strict_Reconfigurable(bank)
        tf = true;
        for row_idx = 1:6
            for direction_col = 1:2
                entry = bank{row_idx, direction_col};
                if isempty(entry.candidates) || ...
                        ~any([entry.candidates.instantaneous_pure])
                    tf = false;
                    return;
                end
            end
        end
    end

    function tf = Has_Impulse_Margin_Fallback(strategies)
        tf = false;
        for control_type = 1:2
            for axis = 1:3
                strategy = strategies{control_type, axis};
                if isempty(strategy) || ~isstruct(strategy) || ...
                        ~isfield(strategy, 'is_impulse_margin_fallback')
                    continue;
                end
                if strategy.is_impulse_margin_fallback && ...
                        ~isempty(strategy.applied_duty) && ...
                        any(strategy.applied_duty > config.duty_tol)
                    tf = true;
                    return;
                end
            end
        end
    end

    function axes = Impulse_Margin_Unavailable_Axes(strategies)
        axes = [];
        for control_type = 1:2
            for axis = 1:3
                strategy = strategies{control_type, axis};
                if isempty(strategy) || ~isstruct(strategy) || ...
                        ~strcmp(strategy.status, 'impulse_margin_unavailable')
                    continue;
                end
                axes(end + 1) = axis + 3 * (control_type - 1); %#ok<AGROW>
            end
        end
    end

    function idx = Find_Group(candidates, group)
        idx = 0;
        for candidate_idx = 1:numel(candidates)
            if isequal(sort(candidates(candidate_idx).group), sort(group))
                idx = candidate_idx;
                return;
            end
        end
    end

    function candidate = Empty_Candidate()
        candidate = struct('group', [], 'unit_duty', [], ...
            'group_size', inf, 'coupling_norm', inf, ...
            'target_residual', inf, 'pulse_sum_unit', inf, ...
            'max_duty_unit', inf, 'pair_count', inf, ...
            'instantaneous_pure', false, ...
            'instantaneous_mode_count', 0, ...
            'is_impulse_margin_fallback', false, 'valid', false);
    end

    function margin = Empty_Impulse_Margin()
        margin = struct('positive_peak', zeros(6, 1), ...
            'negative_peak', zeros(6, 1), ...
            'reserve_positive', zeros(6, 1), ...
            'reserve_negative', zeros(6, 1), ...
            'axis_ratio', zeros(6, 1), ...
            'score', NaN, 'feasible', false);
    end

    function strategy = Empty_Strategy(row_idx, cmd)
        strategy = struct('row_idx', row_idx, 'command', cmd, ...
            'thrusters', [], 'nominal_thrusters', [], ...
            'blocked_thrusters', [], ...
            'duty', [], 'applied_duty', [], 'applied_scale', 1, ...
            'coupling_norm', NaN, ...
            'desired_residual', NaN, 'max_duty', 0, ...
            'pulse_sum', 0, 'active_count', 0, ...
            'pair_count', 0, 'instantaneous_pure', false, ...
            'instantaneous_mode_count', 0, ...
            'is_impulse_margin_fallback', false, ...
            'impulse_margin', Empty_Impulse_Margin(), ...
            'retired_primary_thrusters', [], ...
            'reconfigured', false, ...
            'structurally_reconfigurable', true, ...
            'status', 'empty');
    end

    function cache_idx = Find_Cache_Entry(cache, matrix, cfg)
        cache_idx = 0;
        if isempty(cache)
            return;
        end
        for idx = 1:numel(cache)
            if isequaln(cache(idx).matrix, matrix) && ...
                    isequaln(cache(idx).config, cfg)
                cache_idx = idx;
                return;
            end
        end
    end

    function direction_col = Direction_Column(direction)
        if direction < 0
            direction_col = 1;
        else
            direction_col = 2;
        end
    end

    function value = Get_Param(input_params, field_name, default_value)
        if isfield(input_params, field_name) && ...
                ~isempty(input_params.(field_name))
            value = input_params.(field_name);
        else
            value = default_value;
        end
    end

    function scale = Prepare_Row_Scale(scale)
        scale = scale(:);
        if isscalar(scale)
            scale = scale * ones(6, 1);
        end
        if numel(scale) ~= 6
            scale = max(abs(Matrix_sub), [], 2);
        end
        scale(scale < 1e-12) = 1;
    end

    function faults = Sanitize_Faults(faults)
        faults = faults(:)';
        faults = faults(faults >= 1 & faults <= N);
        faults = unique(faults);
    end
end
