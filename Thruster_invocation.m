%% 最简主份/单故障备份推力器调用策略
% 为六维控制指令生成最简主备调用组、同步时序及完整诊断信息。
% 旧口径保留：连续分配视图不再使用时序样本凸包。
%     if strcmpi(reuse_mode, 'asynchronous_time_division') || strcmpi(reuse_mode, 'task_book')
% 旧口径保留：连续分配视图不再使用时序样本凸包。
%     min_duty = params.t_min / params.T;
function [Prop_Final, info] = Thruster_invocation(v_cmd1, v_cmd2, Matrix_sub, faulty_thrusters, params)
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
        error('Thruster_invocation:InvalidMatrix', 'Matrix_sub 必须是 6xN 控制效率矩阵。');
    end
    command = [v_cmd1(:); v_cmd2(:)];
    if numel(command) ~= 6
        error('Thruster_invocation:InvalidCommand', '轨控和姿控指令必须分别包含三个分量。');
    end
    faulty_thrusters = faulty_thrusters(:)';
    faulty_thrusters = unique(faulty_thrusters(faulty_thrusters >= 1 & faulty_thrusters <= N));

    % 同一入口保留软件中的三种调用方式，避免再由多个 .m 文件转发。
    reuse_mode = 'synchronous_time_division';
    allocation_mode = 'primary_backup';
    if isfield(params, 'alloc_mode') && ~isempty(params.alloc_mode)
        reuse_mode = char(string(params.alloc_mode));
    end
    if isfield(params, 'allocation_strategy') && ~isempty(params.allocation_strategy)
        allocation_mode = char(string(params.allocation_strategy));
    end
    % 连续能力只读取分配规则，不进入脉宽、周期或调度执行层。
    capability_only = isfield(params,'continuous_capability_only') && params.continuous_capability_only;
    if capability_only && (strcmpi(reuse_mode,'joint_optimization') || strcmpi(allocation_mode,'joint_optimization'))
        Prop_Final = [];
        info = struct('mode',reuse_mode,'allocation_domain','independent_continuous', ...
            'healthy',setdiff(1:N,faulty_thrusters),'groups',[]);
        return;
    end
    if strcmpi(reuse_mode, 'joint_optimization') || strcmpi(allocation_mode, 'joint_optimization')
        [Prop_Final, info] = Joint_Optimization();
        return;
    end
    % 原分支保留：if strcmpi(reuse_mode, 'asynchronous_time_division') || strcmpi(reuse_mode, 'task_book')
    if ~capability_only && (strcmpi(reuse_mode, 'asynchronous_time_division') || strcmpi(reuse_mode, 'task_book'))
        [Prop_Final, info] = Asynchronous_Reuse();
        return;
    end

    % 原执行口径：min_duty = params.t_min / params.T;
    if capability_only
        min_duty = 0;
    else
        min_duty = params.t_min / params.T;
    end
    row_scale = max(abs(Matrix_sub), [], 2);
    row_scale(row_scale < 1e-12) = 1;
    % 严格纯控制组允许使用全部健康推力器中的任意偶数台；冲量裕度
    % 备用仍限制到六台，避免故障后在线枚举不必要地变慢。
    config.max_strict_group_size = min(N, 6); % 每个轴向最多使用6台推力器。
    config.max_impulse_group_size = min(N, 6);
    config.candidate_limit = 128;
    config.fit_tol = 1e-8;
    config.unit_duty_tol = 1e-10;
    config.command_tol = 1e-6;
    config.duty_tol = 1e-9;
    config.even_group_only = true;
    config.require_instantaneous_group = true;
    config.instantaneous_tol = config.fit_tol;
    config.impulse_margin_fallback = true;
    config.impulse_margin_limit = 0.80;
    config.impulse_margin_force_weight = 0.35;
    config.impulse_candidate_limit = 512;
    config.row_scale = row_scale;
    % 优化布局的严格验收契约：统一物理归一化、纯组容差及能力门槛。
    config.strict_group_contract = [];
    if isfield(params,'strict_group_contract') && ~isempty(params.strict_group_contract)
        contract = params.strict_group_contract;
        if ~all(isfield(contract,{'row_reference','pure_tolerance','min_axis_capacity','max_group_size'})) || ...
                numel(contract.row_reference) ~= 6 || any(~isfinite(contract.row_reference(:)) | contract.row_reference(:)<=0) || ...
                ~isscalar(contract.pure_tolerance) || ~isfinite(contract.pure_tolerance) || contract.pure_tolerance<=0 || ...
                ~isscalar(contract.min_axis_capacity) || ~isfinite(contract.min_axis_capacity) || contract.min_axis_capacity<=contract.pure_tolerance || ...
                ~isscalar(contract.max_group_size) || ~ismember(contract.max_group_size,[2,4,6])
            error('Thruster_invocation:InvalidStrictContract','严格组验收契约无效。');
        end
        config.strict_group_contract = contract;
        config.max_strict_group_size = min(N,contract.max_group_size);
    end
    % 冲量裕度评价：记录耦合备用组在一个周期内造成的累计干扰及补偿储备。
    margin_template = struct( ...
        'positive_peak', zeros(6, 1), ... % 六维累计干扰冲量的正向最大值。
        'negative_peak', zeros(6, 1), ... % 六维累计干扰冲量的负向绝对最大值。
        'reserve_positive', zeros(6, 1), ... % 可用于产生正向补偿冲量的严格纯控制储备。
        'reserve_negative', zeros(6, 1), ... % 可用于产生负向补偿冲量的严格纯控制储备。
        'axis_ratio', zeros(6, 1), ... % 每个非目标轴的干扰冲量与反向储备之比。
        'score', NaN, ... % 加权后的最大冲量占用比例，越小表示备用组越好。
        'feasible', false); % 该备用组是否满足允许的冲量裕度上限。

    % 候选调用组：记录能够完成某一带符号轴向控制的推力器组合及其性质。
    candidate_template = struct( ...
        'group', [], ... % 组内推力器编号。
        'unit_duty', [], ... % 使归一化目标轴控制量等于 1 时，各成员的占空比。
        'group_size', inf, ... % 该候选组包含的推力器台数。
        'coupling_norm', inf, ... % 非目标五个轴上的归一化残余耦合量。
        'pulse_sum_unit', inf, ... % 单位目标控制量所需的组内占空比总和。
        'max_duty_unit', inf, ... % 单位目标控制量时组内最大的单台占空比。
        'pair_count', inf, ... % 组内可构成的推力器对数量，仅作诊断记录。
        'instantaneous_pure', false, ... % 同步喷气的每个阶段是否都只产生目标轴控制量。
        'instantaneous_mode_count', 0, ... % 同步喷气过程中保持纯控制的阶段数量。
        'is_impulse_margin_fallback', false); % 是否为严格纯控制组不存在时的冲量裕度备用。

    % 轴向调用策略：记录某一个力轴或力矩轴在当前周期的实际调用结果。
    strategy_template = struct( ...
        'row_idx', 0, ... % 六维控制矩阵的行号：1~3 为力，4~6 为力矩。
        'command', 0, ... % 该轴的期望控制量。
        'thrusters', [], ... % 当前实际选中的推力器编号。
        'nominal_thrusters', [], ... % 无故障主份调用组的推力器编号。
        'blocked_thrusters', [], ... % 当前已故障、不可调用的推力器编号。
        'duty', [], ... % 缩放前该组各成员的期望占空比。
        'applied_duty', [], ... % 经最小脉宽和周期容量处理后的实际占空比。
        'applied_scale', 1, ... % 实际占空比相对期望占空比的组统一缩放比例。
        'coupling_norm', NaN, ... % 当前调用组在非目标轴上的归一化耦合量。
        'desired_residual', NaN, ... % 当前轴实际控制量相对期望控制量的误差。
        'max_duty', 0, ... % 当前组内最大的单台占空比。
        'pulse_sum', 0, ... % 当前组内各推力器占空比之和。
        'active_count', 0, ... % 当前实际喷气的推力器台数。
        'pair_count', 0, ... % 当前调用组的推力器对数量。
        'instantaneous_pure', false, ... % 当前组同步喷气时是否始终保持纯控制。
        'instantaneous_mode_count', 0, ... % 当前组同步喷气的纯控制阶段数量。
        'is_impulse_margin_fallback', false, ... % 当前组是否采用冲量裕度备用规则。
        'impulse_margin', margin_template, ... % 当前组的累计干扰冲量评价结果。
        'retired_primary_thrusters', [], ... % 故障后整组退役、不再按主份喷气的推力器编号。
        'reconfigured', false, ... % 当前调用组是否相对主份发生切换。
        'structurally_reconfigurable', true, ... % 当前故障下该轴是否还能找到候选调用组。
        'status', 'empty'); % 当前轴调用状态，例如主份、备用、最小脉宽剔除等。

    % 布局不变时复用主份和故障备份表。故障表按故障集合按需建立，
    % 在线控制阶段只需查表和按指令幅值缩放占空比。
    persistent strategy_cache
    cache_idx = 0;
    for index = 1:numel(strategy_cache)
        if isequaln(strategy_cache(index).matrix, Matrix_sub) && isequaln(strategy_cache(index).config, config)
            cache_idx = index;
            break;
        end
    end
    if cache_idx == 0
        [nominal_bank, nominal_usage] = Build_Nominal_Bank();
        cache_entry = struct('matrix', Matrix_sub, 'config', config, 'nominal_bank', {nominal_bank}, 'nominal_usage', nominal_usage, 'fault_keys', {{}}, 'fault_banks', {{}});
        if isempty(strategy_cache)
            strategy_cache = cache_entry;
        else
            strategy_cache(end + 1) = cache_entry;
        end
        cache_limit = 6;
        if numel(strategy_cache) > cache_limit
            strategy_cache(1:end - cache_limit) = [];
        end
        cache_idx = numel(strategy_cache);
    end

    nominal_bank = strategy_cache(cache_idx).nominal_bank;
    nominal_usage = strategy_cache(cache_idx).nominal_usage;
    if isempty(faulty_thrusters)
        active_bank = nominal_bank;
    else
        fault_key = sprintf('%d,', faulty_thrusters);
        fault_idx = find(strcmp(strategy_cache(cache_idx).fault_keys, fault_key), 1);
        if isempty(fault_idx)
            fault_bank = Build_Fault_Bank(faulty_thrusters, nominal_bank, nominal_usage);
            strategy_cache(cache_idx).fault_keys{end + 1} = fault_key;
            strategy_cache(cache_idx).fault_banks{end + 1} = fault_bank;
            fault_idx = numel(strategy_cache(cache_idx).fault_banks);
        end
        active_bank = strategy_cache(cache_idx).fault_banks{fault_idx};
    end

    if capability_only
        Prop_Final=[];
        info=Continuous_Allocation_Domain();
        return;
    end

    % 先为每一个有指令的轴保存一整组“纯轴向”占空比模板。这里不做
    % 单台饱和或单台最小脉宽截断；否则会改变组内各台的比例，原本的
    % 力/力矩配平将不再成立。
    orbit_axis_duty = zeros(N, 3);
    attitude_axis_duty = zeros(N, 3);
    axis_strategy = cell(2, 3);
    for allocation_type = 1:2
        for axis_idx = 1:3
            row_idx = axis_idx + 3 * (allocation_type - 1);
            if abs(command(row_idx)) < config.command_tol
                axis_strategy{allocation_type, axis_idx} = strategy_template;
                axis_strategy{allocation_type, axis_idx}.row_idx = row_idx;
                axis_strategy{allocation_type, axis_idx}.command = command(row_idx);
            else
                [u_axis, axis_strategy{allocation_type, axis_idx}] = Select_Axis_Strategy(row_idx, command(row_idx), active_bank, nominal_bank);
                if allocation_type == 1
                    orbit_axis_duty(:, axis_idx) = u_axis;
                else
                    attitude_axis_duty(:, axis_idx) = u_axis;
                end
            end
        end
    end

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
    [u_F_raw, u_T_raw, ~, ~, orbit_axis_duty, attitude_axis_duty, axis_strategy, scaling] = Decoupled_Group_Scaling(orbit_axis_duty, attitude_axis_duty, axis_strategy, min_duty);
    Prop_F = u_F_raw * params.T;
    Prop_T = u_T_raw * params.T;

    % 同一轴调用组共同起喷；同一台推力器的不同组按最早空闲时刻排列。
    Prop_F_axis_raw = scaling.force_scale * orbit_axis_duty * params.T;
    Prop_T_axis_raw = scaling.attitude_scale * attitude_axis_duty * params.T;
    has_impulse_margin_fallback = false;
    for fallback_type = 1:2
        for axis_idx = 1:3
            strategy = axis_strategy{fallback_type, axis_idx};
            if isstruct(strategy) && isfield(strategy, 'is_impulse_margin_fallback') && strategy.is_impulse_margin_fallback && any(strategy.applied_duty > config.duty_tol)
                has_impulse_margin_fallback = true;
                break;
            end
        end
        if has_impulse_margin_fallback
            break;
        end
    end
    [pulse_schedule, Prop_F_axis, Prop_T_axis, scheduler_info] = Build_Synchronous_Schedule(Prop_F_axis_raw, Prop_T_axis_raw);
    if scheduler_info.time_scale < 1 - config.duty_tol
        scaling.force_scale = scaling.force_scale * scheduler_info.time_scale;
        scaling.attitude_scale = scaling.attitude_scale * scheduler_info.time_scale;
        for scale_type = 1:2
            for axis_idx = 1:3
                strategy = axis_strategy{scale_type, axis_idx};
                strategy.applied_duty = strategy.applied_duty * scheduler_info.time_scale;
                strategy.applied_scale = strategy.applied_scale * scheduler_info.time_scale;
                axis_strategy{scale_type, axis_idx} = strategy;
            end
        end
    end
    Prop_F_used = sum(Prop_F_axis, 2);
    Prop_T_used = sum(Prop_T_axis, 2);
    Prop_Final = pulse_schedule.total_pulse;
    u_F_used = Prop_F_used / params.T;
    u_T_used = Prop_T_used / params.T;

    reuse_scale = ones(N, 1);
    force_active = Prop_F > config.duty_tol * params.T;
    reuse_scale(force_active) = Prop_F_used(force_active) ./ Prop_F(force_active);

    if nargout > 1
        duty = Prop_Final / params.T;
        info.mode = 'synchronous_time_division';
        info.reuse_strategy = 'synchronous_time_division';
        info.allocation_strategy = 'primary_backup';
        info.solver = 'paired_primary_strict_backup_impulse_margin_fallback';
        info.fault_policy = 'retire_affected_complete_group';
        info.strict_paired_mode = config.require_instantaneous_group;
        info.uses_impulse_margin_fallback = has_impulse_margin_fallback;
        info.nominal_reuse_count = nominal_usage;
        info.structurally_reconfigurable = true;
        info.strictly_reconfigurable = true;
        for diagnostic_row = 1:6
            for diagnostic_direction = 1:2
                candidates = active_bank{diagnostic_row, diagnostic_direction}.candidates;
                info.structurally_reconfigurable = info.structurally_reconfigurable && ~isempty(candidates);
                info.strictly_reconfigurable = info.strictly_reconfigurable && ~isempty(candidates) && any([candidates.instantaneous_pure]);
            end
        end
        info.Prop_F = Prop_F;
        info.Prop_T = Prop_T;
        info.Prop_F_axis = Prop_F_axis;
        info.Prop_F_axis_raw = Prop_F_axis_raw;
        info.Prop_T_axis = Prop_T_axis;
        info.Prop_T_axis_raw = Prop_T_axis_raw;
        info.axis_strategy = axis_strategy;
        info.impulse_margin_unavailable_axes = [];
        for info_type = 1:2
            for axis_idx = 1:3
                strategy = axis_strategy{info_type, axis_idx};
                if strcmp(strategy.status, 'impulse_margin_unavailable')
                    info.impulse_margin_unavailable_axes(end + 1) = axis_idx + 3 * (info_type - 1);
                end
            end
        end
        info.Prop_F_used = Prop_F_used;
        info.Prop_T_used = Prop_T_used;
        info.Prop_6D = zeros(N, 1);
        info.Prop_Final = Prop_Final;
        info.reuse_scale = reuse_scale;
        info.schedule = pulse_schedule;
        info.scheduler = scheduler_info;
        info.scheduler_force_axis_suppressed = scheduler_info.suppressed_force_axes;
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
        info.period_average_decoupled = info.orbit_cross_torque <= config.fit_tol && info.attitude_cross_force <= config.fit_tol;
        info.duty = duty;
        info.command = command;
        info.actual = Matrix_sub * duty;
        info.residual = command - info.actual;
        info.orbit_keep_ratio = sum(Prop_F_used) / (sum(Prop_F) + 1e-12);
        info.att_keep_ratio = scaling.attitude_scale;
        info.max_raw_pulse = max(Prop_F + Prop_T);
        info.attitude_window = NaN;
        info.position_window = NaN;
    end

    function ccInfo=Continuous_Allocation_Domain()
        % 与执行路径隔离局部变量，避免 MATLAB 嵌套工作区共享 entry/candidate。
        % 固定主/备组选 preferred_idx；需在线评分的备用组保留其候选库并集。
        % 不执行冲量裕度评分：它引入本视图排除的时序限制。
        ccGroups=repmat(struct('axis',0,'direction',0,'thrusters',[], ...
            'ratio',[],'endpoint',zeros(6,1),'backup_candidates',false),0,1);
        for ccAxis=1:6
            for ccDirection=1:2
                ccEntry=active_bank{ccAxis,ccDirection};
                if isempty(ccEntry.candidates), continue; end
                if ccEntry.preferred_idx>0
                    ccIndices=ccEntry.preferred_idx;
                else
                    ccIndices=1:numel(ccEntry.candidates);
                end
                for ccIndex=ccIndices
                    ccCandidate=ccEntry.candidates(ccIndex);
                    ccRatio=ccCandidate.unit_duty(:)/max(ccCandidate.unit_duty);
                    ccEndpoint=Matrix_sub(:,ccCandidate.group)*ccRatio;
                    ccGroups(end+1,1)=struct('axis',ccAxis,'direction',2*ccDirection-3, ...
                        'thrusters',ccCandidate.group,'ratio',ccRatio,'endpoint',ccEndpoint, ...
                        'backup_candidates',ccEntry.uses_impulse_margin_fallback); %#ok<AGROW>
                end
            end
        end
        ccInfo=struct('mode',reuse_mode,'allocation_domain','fixed_ratio_union', ...
            'healthy',setdiff(1:N,faulty_thrusters),'groups',ccGroups);
    end

    % 为无故障工况建立最简主份调用库并记录推力器复用次数。
    function [bank, usage_count] = Build_Nominal_Bank()
        bank = Build_Raw_Bank([], false);
        usage_count = zeros(N, 1);

        % 候选数少的轴向优先选主份。每个轴向只在“最少台数候选”
        % 之间比较，先减少与已选主份的重叠，再比较总脉宽。
        channel_rows = repelem((1:6)', 2);
        channel_cols = repmat((1:2)', 6, 1);
        difficulty = inf(numel(channel_rows), 1);
        for channel_idx = 1:numel(channel_rows)
            entry = bank{channel_rows(channel_idx), channel_cols(channel_idx)};
            difficulty(channel_idx) = numel(entry.candidates);
        end
        [~, channel_order] = sort(difficulty, 'ascend');

        for order_idx = 1:numel(channel_rows)
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
                rank(candidate_idx, :) = [sum(usage_count(group)), max(trial_usage), entry.candidates(candidate_idx).pulse_sum_unit, candidate_idx];
            end
            [~, order] = sortrows(rank, [1 2 3 4]);
            entry.preferred_idx = order(1);
            entry.nominal_group = entry.candidates(entry.preferred_idx).group;
            usage_count(entry.nominal_group) = usage_count(entry.nominal_group) + 1;
            bank{row_idx, direction_col} = entry;
        end
    end

    % 为给定故障集合建立健康备用调用库。
    function bank = Build_Fault_Bank(faults, nominal, nominal_usage)
        bank = Build_Raw_Bank(faults, config.impulse_margin_fallback);
        for bank_row = 1:6
            for direction_col = 1:2
                entry = bank{bank_row, direction_col};
                nominal_entry = nominal{bank_row, direction_col};
                if nominal_entry.preferred_idx > 0
                    entry.nominal_group = nominal_entry.candidates(nominal_entry.preferred_idx).group;
                end
                if isempty(entry.candidates)
                    bank{bank_row, direction_col} = entry;
                    continue;
                end

                % 冲量裕度候选需要结合实时指令幅值、反向储备和最小
                % 脉宽逐一评分，不能在离线建表阶段固定某一个主份。
                if entry.uses_impulse_margin_fallback
                    entry.preferred_idx = 0;
                    bank{bank_row, direction_col} = entry;
                    continue;
                end

                % 故障未命中主份时保持原主份，避免无谓切换。
                if ~any(ismember(entry.nominal_group, faults))
                    same_idx = 0;
                    for candidate_idx = 1:numel(entry.candidates)
                        if isequal(sort(entry.candidates(candidate_idx).group), sort(entry.nominal_group))
                            same_idx = candidate_idx;
                            break;
                        end
                    end
                    if same_idx > 0
                        entry.preferred_idx = same_idx;
                        bank{bank_row, direction_col} = entry;
                        continue;
                    end
                end

                % 故障命中主份时，从健康最小组中优先选择较少占用其他
                % 标况轴向主份的组合，随后比较所需脉宽。
                rank = zeros(numel(entry.candidates), 4);
                for candidate_idx = 1:numel(entry.candidates)
                    group = entry.candidates(candidate_idx).group;
                    rank(candidate_idx, :) = [sum(nominal_usage(group)), max(nominal_usage(group)), entry.candidates(candidate_idx).pulse_sum_unit, candidate_idx];
                end
                [~, order] = sortrows(rank, [1 2 3 4]);
                entry.preferred_idx = order(1);
                bank{bank_row, direction_col} = entry;
            end
        end
    end

    % 搜索所有带符号轴向的严格候选组及必要的冲量裕度备用组。
    function bank = Build_Raw_Bank(blocked_thrusters, allow_impulse_fallback)
        healthy = setdiff(1:N, blocked_thrusters);
        bank = cell(6, 2);
        for bank_row = 1:6
            for direction_col = 1:2
                direction = 2 * direction_col - 3;
                candidates = Build_Candidate_Pool(bank_row, direction, healthy, true, true);
                uses_impulse_margin_fallback = false;
                if isempty(candidates) && allow_impulse_fallback
                    candidates = Build_Candidate_Pool(bank_row, direction, healthy, false, false);
                    uses_impulse_margin_fallback = ~isempty(candidates);
                end
                bank{bank_row, direction_col} = struct('direction', direction, 'candidates', candidates, 'preferred_idx', double(~isempty(candidates)), 'nominal_group', [], 'uses_impulse_margin_fallback', uses_impulse_margin_fallback);
            end
        end
    end

    % 枚举给定轴向的最简纯控制候选组。
    function candidates = Build_Candidate_Pool(row_idx, direction, healthy, require_even_group, require_instantaneous)
        normalized_matrix = bsxfun(@rdivide, Matrix_sub, row_scale);
        target = zeros(6, 1);
        target(row_idx) = direction;
        candidates = repmat(candidate_template, 0, 1);

        if require_even_group
            group_sizes = 2:2:min(config.max_strict_group_size, numel(healthy));
        else
            group_sizes = 1:min(config.max_impulse_group_size, numel(healthy));
        end

        for group_size = group_sizes
            % healthy 包含全部健康推力器；此处不按模块限制组合，因此会
            % 同时搜索同模块、对角线、跨模块及故障后的备用调用组。
            combinations = nchoosek(healthy, group_size);
            size_candidates = repmat(candidate_template, 0, 1);
            for combination_idx = 1:size(combinations, 1)
                group = combinations(combination_idx, :);
                A = normalized_matrix(:, group);
                other_rows = setdiff(1:6, row_idx);
                equal_wrench = sum(A, 2);
                equal_scale = direction * equal_wrench(row_idx);
                if require_instantaneous && ~isempty(config.strict_group_contract)
                    contract = config.strict_group_contract;
                    physical = sum(Matrix_sub(:,group),2)./contract.row_reference(:);
                    if direction*physical(row_idx) >= contract.min_axis_capacity-contract.pure_tolerance && ...
                            max(abs(physical(other_rows))) <= contract.pure_tolerance && equal_scale > 0
                        candidate = candidate_template;
                        candidate.group = group;
                        candidate.unit_duty = ones(group_size,1)/equal_scale;
                        candidate.group_size = group_size;
                        candidate.coupling_norm = norm(physical(other_rows));
                        candidate.pulse_sum_unit = sum(candidate.unit_duty);
                        candidate.max_duty_unit = max(candidate.unit_duty);
                        candidate.instantaneous_pure = true;
                        candidate.instantaneous_mode_count = 1;
                        candidate.pair_count = group_size/2;
                        candidate.is_impulse_margin_fallback = false;
                        size_candidates(end+1,1) = candidate; %#ok<AGROW>
                    end
                    continue;
                end
                duty_options = zeros(group_size, 0);
                if equal_scale > config.fit_tol && norm(equal_wrench(other_rows)) <= config.fit_tol
                    % 对所有成员施加相同脉宽时，整个喷气阶段始终为纯控制。
                    duty_options(:, end + 1) = ones(group_size, 1) / equal_scale; %#ok<AGROW>
                end
                pinv_duty = pinv(A) * target;
                pinv_actual = A * pinv_duty;
                if all(isfinite(pinv_duty)) && all(pinv_duty > config.unit_duty_tol) && norm(pinv_actual - target) <= config.fit_tol
                    if isempty(duty_options) || norm(pinv_duty - duty_options(:, 1)) > config.unit_duty_tol
                        duty_options(:, end + 1) = pinv_duty; %#ok<AGROW>
                    end
                end
                for duty_index = 1:size(duty_options, 2)
                    unit_duty = duty_options(:, duty_index);
                    actual = A * unit_duty;
                    candidate = candidate_template;
                    candidate.group = group;
                    candidate.unit_duty = unit_duty;
                    candidate.group_size = group_size;
                    candidate.coupling_norm = norm(actual(other_rows));
                    candidate.pulse_sum_unit = sum(unit_duty);
                    candidate.max_duty_unit = max(unit_duty);
                    [candidate.instantaneous_pure, candidate.instantaneous_mode_count] = Is_Instantaneously_Pure_Group(group, unit_duty, row_idx, direction, normalized_matrix);
                    candidate.pair_count = floor(group_size / 2);
                    candidate.is_impulse_margin_fallback = ~require_instantaneous;
                    size_candidates(end + 1, 1) = candidate; %#ok<AGROW>
                end
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
            rank = [[candidates.coupling_norm]', [candidates.pulse_sum_unit]', [candidates.max_duty_unit]'];
            [~, order] = sortrows(rank, [1 2 3]);
        else
            % 在线的冲量裕度评分会决定最终备份；此处仅保留规模较小、
            % 单位脉宽较短的一批候选，控制离线组合搜索开销。
            rank = [[candidates.group_size]', [candidates.pulse_sum_unit]', [candidates.max_duty_unit]'];
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

    % 检查共同起喷、逐台关断的每个阶段是否仍保持纯目标轴向。
    function [is_pure, mode_count] = Is_Instantaneously_Pure_Group(group, unit_duty, row_idx, direction, normalized_matrix)
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
            instantaneous_wrench = sum(normalized_matrix(:, active_group), 2);
            if direction * instantaneous_wrench(row_idx) <= config.instantaneous_tol || norm(instantaneous_wrench(other_rows)) > config.instantaneous_tol
                is_pure = false;
                return;
            end
            mode_count = mode_count + 1;
        end
    end

    % 按主份、严格备用或冲量裕度备用的优先级选择一个轴向调用组。
    function [u_axis, strategy] = Select_Axis_Strategy(row_idx, cmd, bank, nominal)
        u_axis = zeros(N, 1);
        strategy = strategy_template;
        strategy.row_idx = row_idx;
        strategy.command = cmd;
        direction_col = Direction_Column(sign(cmd));
        entry = bank{row_idx, direction_col};
        nominal_entry = nominal{row_idx, direction_col};

        if nominal_entry.preferred_idx > 0
            nominal_group = nominal_entry.candidates(nominal_entry.preferred_idx).group;
        else
            nominal_group = [];
        end
        strategy.nominal_thrusters = nominal_group;
        strategy.blocked_thrusters = faulty_thrusters;
        strategy.structurally_reconfigurable = ~isempty(entry.candidates);
        if isempty(entry.candidates)
            strategy.status = 'unreconfigurable_fault';
            return;
        end

        order = 1:numel(entry.candidates);
        if entry.preferred_idx > 0
            order = [entry.preferred_idx, setdiff(order, entry.preferred_idx, 'stable')];
        end

        command_scale = abs(cmd) / row_scale(row_idx);

        % 候选已经按“标况主份/故障备份”的优先级排好。此处故意不做
        % 0~1 限幅：限幅应在六轴叠加后统一进行，才能保留组内比例。
        impulse_margin = margin_template;
        if entry.uses_impulse_margin_fallback
            [selected_idx, impulse_margin] = Select_Impulse_Margin_Candidate(entry.candidates, row_idx, command_scale, bank);
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
        strategy.coupling_norm = norm(actual(other_rows) ./ row_scale(other_rows));
        strategy.desired_residual = abs(actual(row_idx) - cmd) / (abs(cmd) + 1e-12);
        strategy.max_duty = max(selected_duty);
        strategy.pulse_sum = sum(selected_duty);
        strategy.pair_count = selected.pair_count;
        strategy.instantaneous_pure = selected.instantaneous_pure;
        strategy.instantaneous_mode_count = selected.instantaneous_mode_count;
        strategy.is_impulse_margin_fallback = entry.uses_impulse_margin_fallback;
        strategy.impulse_margin = impulse_margin;
        strategy.retired_primary_thrusters = [];
        if fault_hits_primary
            % 故障时不允许主份内未故障的成员以残缺组方式续喷。当前
            % 主份整组退役；若健康成员在备用组中再次出现，它属于新组，
            % 而非故障组的残余部分。
            strategy.retired_primary_thrusters = nominal_group;
        end
        strategy.reconfigured = ~isequal(sort(selected.group), sort(nominal_group));
        strategy.status = selected_status;
    end

    % 在周期平均备用候选中选取冲量裕度消耗最小的一组。
% 原实现保留（口径或界面已调整；冲突部分停用）：
%             if any(pulse < min_duty - config.duty_tol)
    function [selected_idx, selected_margin] = Select_Impulse_Margin_Candidate(candidates, row_idx, command_scale, bank)
        % 对周期平均纯控制候选，按实际共同起喷后的累计干扰冲量评分：
        %
        %   score = max_j,t |J_dist,j(t)| / J_opp,reserve,j.
        %
        % J_opp,reserve,j 是故障后尚可用于反向抵消第 j 轴扰动的严格
        % 纯控制冲量。只有 score 不超过阈值的候选才能成为备用组。
        selected_idx = 0;
        selected_margin = margin_template;
        best_rank = inf(1, 4);
        for candidate_idx = 1:numel(candidates)
            candidate = candidates(candidate_idx);
            duty = command_scale * candidate.unit_duty;
            pulse = duty * params.T;
            if any(pulse < params.t_min - config.duty_tol * params.T)
                continue;
            end

            margin = Evaluate_Impulse_Margin(candidate, duty, row_idx, bank);
            if ~margin.feasible
                continue;
            end
            rank = [margin.score, sum(duty), candidate.group_size, max(duty)];
            better = false;
            for rank_index = 1:numel(rank)
                if rank(rank_index) < best_rank(rank_index) - config.duty_tol
                    better = true;
                    break;
                end
                if rank(rank_index) > best_rank(rank_index) + config.duty_tol
                    break;
                end
            end
            if better
                selected_idx = candidate_idx;
                selected_margin = margin;
                best_rank = rank;
            end
        end
    end

    % 计算候选备用组的最大累计干扰冲量与反向补偿储备之比。
    function margin = Evaluate_Impulse_Margin(candidate, duty, row_idx, bank)
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
            instantaneous_wrench = sum(Matrix_sub(:, candidate.group(active)), 2);
            cumulative_impulse = cumulative_impulse + instantaneous_wrench * (end_time - start_time);
            positive_peak = max(positive_peak, cumulative_impulse);
            negative_peak = max(negative_peak, -cumulative_impulse);
        end
        positive_peak(row_idx) = 0;
        negative_peak(row_idx) = 0;
        occupied_duty = zeros(N, 1);
        occupied_duty(candidate.group) = duty;
        cross_rows = setdiff(1:6, row_idx);
        ratio = zeros(6, 1);
        reserve_positive = zeros(6, 1);
        reserve_negative = zeros(6, 1);

        for cross_row = cross_rows
            % 正干扰需要负向严格纯控制抵消；负干扰则相反。
            if positive_peak(cross_row) > config.duty_tol
                reserve_negative(cross_row) = Directional_Impulse_Reserve(cross_row, -1, bank, occupied_duty);
                ratio(cross_row) = max(ratio(cross_row), positive_peak(cross_row) / (reserve_negative(cross_row) + eps));
            end
            if negative_peak(cross_row) > config.duty_tol
                reserve_positive(cross_row) = Directional_Impulse_Reserve(cross_row, 1, bank, occupied_duty);
                ratio(cross_row) = max(ratio(cross_row), negative_peak(cross_row) / (reserve_positive(cross_row) + eps));
            end
        end

        weights = ones(6, 1);
        % 非目标的平动力一般允许比姿态扰动占用更少的资源；权重仍可
        % 由参数调整，且每个方向最终都必须具备反向纯控制储备。
        weights(1:3) = config.impulse_margin_force_weight;
        weighted_ratio = ratio .* weights;
        margin = margin_template;
        margin.positive_peak = positive_peak;
        margin.negative_peak = negative_peak;
        margin.reserve_positive = reserve_positive;
        margin.reserve_negative = reserve_negative;
        margin.axis_ratio = ratio;
        margin.score = max(weighted_ratio(cross_rows));
        margin.feasible = isfinite(margin.score) && margin.score <= config.impulse_margin_limit + config.duty_tol;
    end

    % 计算给定干扰方向可调用的反向严格纯控制冲量储备。
    function reserve = Directional_Impulse_Reserve(row_idx, direction, bank, occupied_duty)
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
            compensation = Matrix_sub(:, candidate.group) * (scale * candidate.unit_duty);
            reserve = max(reserve, abs(compensation(row_idx)) * params.T);
        end
    end

    % 按姿控优先原则对轨控组和姿控组实施整组同比例容量缩放。
    function [u_F_raw, u_T_raw, u_F_used, u_T_used, orbit_axis, attitude_axis, strategies, scaling] = Decoupled_Group_Scaling(orbit_axis, attitude_axis, strategies, duty_min)
        % 本函数是调用策略的执行层。它不修改任意一个组内部的相对
        % 脉宽，只会删除整组，或将同一控制类别的全部组乘以同一个
        % 标量。因此纯轨控组仍只产生力，纯姿控组仍只产生力矩。
        [attitude_axis, strategies, lambda_T, attitude_reason] = Fit_Group_Family(attitude_axis, strategies, 2, ones(N, 1), duty_min);
        u_T_raw = sum(attitude_axis, 2);
        u_T_used = lambda_T * u_T_raw;

        if lambda_T < 1 - config.duty_tol
            % 姿控需求本身已超出一个周期的可执行范围。按照姿控硬优先，
            % 本周期停掉全部轨控组；姿控仍保持原三轴方向的同比例缩放。
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
            lambda_F = 0;
            force_reason = 'attitude_capacity_priority';
        else
            remaining_duty = max(0, 1 - u_T_used);
            [orbit_axis, strategies, lambda_F, force_reason] = Fit_Group_Family(orbit_axis, strategies, 1, remaining_duty, duty_min);
        end
        u_F_raw = sum(orbit_axis, 2);
        u_F_used = lambda_F * u_F_raw;

        scaling = struct('attitude_scale', lambda_T, 'force_scale', lambda_F, 'attitude_reason', attitude_reason, 'force_reason', force_reason);
    end

    % 在不破坏各调用组内部比例的条件下拟合一类控制组的可用脉宽。
    function [axis_duty, strategies, scale, reason] = Fit_Group_Family(axis_duty, strategies, control_type, available_duty, duty_min)
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
            [axis_duty, strategies, group_removed] = Drop_Undersized_Groups(axis_duty, strategies, control_type, scale, duty_min);
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
        for axis = 1:3
            strategy = strategies{control_type, axis};
            if isempty(strategy) || isempty(strategy.duty) || strcmp(strategy.status, 'below_minimum_pulse') || strcmp(strategy.status, 'attitude_priority_suppressed')
                continue;
            end
            strategy.applied_scale = scale;
            strategy.applied_duty = scale * strategy.duty;
            strategies{control_type, axis} = strategy;
        end
    end

    % 计算一类调用组可共同使用的最大占空比倍率。
    function scale = Capacity_Scale(raw_duty, available_duty)
        active = raw_duty > config.duty_tol;
        if ~any(active)
            scale = 1;
            return;
        end
        scale = min(1, min(available_duty(active) ./ raw_duty(active)));
        scale = max(0, scale);
    end

    % 删除缩放后任一成员低于最小脉宽的完整调用组。
    function [axis_duty, strategies, removed] = Drop_Undersized_Groups(axis_duty, strategies, control_type, scale, duty_min)
        removed = false;
        if duty_min <= config.duty_tol
            return;
        end
        for axis = 1:size(axis_duty, 2)
            group_duty = axis_duty(:, axis);
            active = group_duty > config.duty_tol;
            if ~any(active) || all(scale * group_duty(active) >= duty_min - config.duty_tol)
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

    % 将同步调用组安排为同组同步、同台不重叠的实际喷气时段。
    function [schedule_out, force_axis, attitude_axis, scheduler_out] = Build_Synchronous_Schedule(force_axis, attitude_axis)
        intervals = cell(N, 1);
        force_intervals = cell(N, 1);
        attitude_intervals = cell(N, 1);
        axis_intervals = cell(N, 1);
        available_time = zeros(N, 1);
        for schedule_type = 2:-1:1
            for control_axis = 1:3
                if schedule_type == 1
                    axis_pulse = force_axis(:, control_axis);
                    axis_code = control_axis;
                else
                    axis_pulse = attitude_axis(:, control_axis);
                    axis_code = control_axis + 3;
                end
                members = find(axis_pulse > config.duty_tol * params.T);
                if isempty(members)
                    continue;
                end
                start_time = max(available_time(members));
                for member_idx = members(:)'
                    interval = [start_time, start_time + axis_pulse(member_idx)];
                    if schedule_type == 1
                        force_intervals{member_idx}(end + 1, :) = interval;
                    else
                        attitude_intervals{member_idx}(end + 1, :) = interval;
                    end
                    axis_intervals{member_idx}(end + 1, :) = [interval, axis_code];
                    available_time(member_idx) = interval(2);
                end
            end
        end
        time_scale = min(1, params.T / max(params.T, max(available_time)));
        if time_scale < 1
            force_axis = time_scale * force_axis;
            attitude_axis = time_scale * attitude_axis;
            for member_idx = 1:N
                force_intervals{member_idx} = time_scale * force_intervals{member_idx};
                attitude_intervals{member_idx} = time_scale * attitude_intervals{member_idx};
                if ~isempty(axis_intervals{member_idx})
                    axis_intervals{member_idx}(:, 1:2) = time_scale * axis_intervals{member_idx}(:, 1:2);
                end
            end
        end
        for member_idx = 1:N
            intervals{member_idx} = [attitude_intervals{member_idx}; force_intervals{member_idx}];
        end
        schedule_out = struct('enabled', true, 'intervals', {intervals}, 'force_intervals', {force_intervals}, 'attitude_intervals', {attitude_intervals}, 'total_pulse', sum(force_axis + attitude_axis, 2), 'force_pulse', sum(force_axis, 2), 'attitude_pulse', sum(attitude_axis, 2), 'job_starts', [], 'axis_intervals', {axis_intervals});
        scheduler_out = struct('peak_orbit_torque', NaN, 'peak_attitude_force', NaN, 'peak_torque_limit', NaN, 'peak_force_limit', NaN, 'peak_constraint_met', true, 'suppressed_force_axes', [], 'force_axis_pulse', force_axis, 'attitude_axis_pulse', attitude_axis, 'time_scale', time_scale, 'status', 'simple_group_schedule');
    end

    % 执行任务书定义的逐台姿控优先异步分时复用。
    function [output_pulse, result_info] = Asynchronous_Reuse()
        synchronous_params = params;
        synchronous_params.alloc_mode = 'synchronous_time_division';
        synchronous_params.allocation_strategy = 'primary_backup';
        [~, force_info] = Thruster_invocation(v_cmd1, zeros(3, 1), Matrix_sub, faulty_thrusters, synchronous_params);
        [~, attitude_info] = Thruster_invocation(zeros(3, 1), v_cmd2, Matrix_sub, faulty_thrusters, synchronous_params);
        requested_force_axis = max(0, force_info.Prop_F_axis);
        requested_attitude_axis = max(0, attitude_info.Prop_T_axis);
        requested_force = sum(requested_force_axis, 2);
        requested_attitude = sum(requested_attitude_axis, 2);
        attitude_pulse = min(requested_attitude, params.T);
        force_pulse = min(requested_force, max(0, params.T - attitude_pulse));
        attitude_ratio = zeros(N, 1);
        force_ratio = zeros(N, 1);
        active_attitude = requested_attitude > 1e-12;
        active_force = requested_force > 1e-12;
        attitude_ratio(active_attitude) = attitude_pulse(active_attitude) ./ requested_attitude(active_attitude);
        force_ratio(active_force) = force_pulse(active_force) ./ requested_force(active_force);
        attitude_axis = requested_attitude_axis .* attitude_ratio;
        force_axis = requested_force_axis .* force_ratio;
        [schedule_out, scheduler_out] = Build_Asynchronous_Schedule(force_axis, attitude_axis);
        output_pulse = attitude_pulse + force_pulse;
        result_info = force_info;
        result_info.mode = 'asynchronous_time_division';
        result_info.reuse_strategy = 'asynchronous_time_division';
        result_info.allocation_strategy = 'primary_backup';
        result_info.solver = 'primary_backup_then_taskbook_per_thruster_clipping';
        result_info.strict_paired_mode = force_info.strict_paired_mode && attitude_info.strict_paired_mode;
        result_info.uses_impulse_margin_fallback = force_info.uses_impulse_margin_fallback || attitude_info.uses_impulse_margin_fallback;
        result_info.nominal_reuse_count = max(force_info.nominal_reuse_count, attitude_info.nominal_reuse_count);
        result_info.structurally_reconfigurable = force_info.structurally_reconfigurable && attitude_info.structurally_reconfigurable;
        result_info.strictly_reconfigurable = force_info.strictly_reconfigurable && attitude_info.strictly_reconfigurable;
        result_info.Prop_F = requested_force;
        result_info.Prop_T = requested_attitude;
        result_info.Prop_F_used = force_pulse;
        result_info.Prop_T_used = attitude_pulse;
        result_info.Prop_F_axis = force_axis;
        result_info.Prop_T_axis = attitude_axis;
        result_info.Prop_F_axis_raw = requested_force_axis;
        result_info.Prop_T_axis_raw = requested_attitude_axis;
        result_info.Prop_6D = zeros(N, 1);
        result_info.Prop_Final = output_pulse;
        result_info.schedule = schedule_out;
        result_info.scheduler = scheduler_out;
        result_info.scheduler_force_axis_suppressed = [];
        result_info.axis_strategy(2, :) = attitude_info.axis_strategy(2, :);
        result_info.impulse_margin_unavailable_axes = unique([force_info.impulse_margin_unavailable_axes(:); attitude_info.impulse_margin_unavailable_axes(:)]);
        result_info.force_scale = sum(force_pulse) / (sum(requested_force) + eps);
        result_info.attitude_scale = sum(attitude_pulse) / (sum(requested_attitude) + eps);
        result_info.force_scale_reason = 'taskbook_per_thruster_remaining_time';
        result_info.attitude_scale_reason = 'attitude_priority';
        result_info.orbit_wrench_raw = Matrix_sub * (requested_force / params.T);
        result_info.attitude_wrench_raw = Matrix_sub * (requested_attitude / params.T);
        result_info.orbit_wrench = Matrix_sub * (force_pulse / params.T);
        result_info.attitude_wrench = Matrix_sub * (attitude_pulse / params.T);
        result_info.orbit_cross_torque = norm(result_info.orbit_wrench(4:6));
        result_info.attitude_cross_force = norm(result_info.attitude_wrench(1:3));
        result_info.period_average_decoupled = result_info.orbit_cross_torque <= 1e-8 && result_info.attitude_cross_force <= 1e-8;
        result_info.duty = output_pulse / params.T;
        result_info.command = command;
        result_info.actual = Matrix_sub * result_info.duty;
        result_info.residual = command - result_info.actual;
        result_info.reuse_scale = ones(N, 1);
        result_info.reuse_scale(active_force) = force_ratio(active_force);
        result_info.orbit_keep_ratio = result_info.force_scale;
        result_info.att_keep_ratio = result_info.attitude_scale;
        result_info.max_raw_pulse = max(requested_force + requested_attitude);
    end

    % 按每台推力器的“姿控后轨控”顺序生成异步脉冲时序。
    function [schedule_out, scheduler_out] = Build_Asynchronous_Schedule(force_axis, attitude_axis)
        intervals = cell(N, 1);
        force_intervals = cell(N, 1);
        attitude_intervals = cell(N, 1);
        axis_intervals = cell(N, 1);
        for member_idx = 1:N
            start_time = 0;
            for async_type = 2:-1:1
                for control_axis = 1:3
                    if async_type == 1
                        pulse = force_axis(member_idx, control_axis);
                        axis_code = control_axis;
                    else
                        pulse = attitude_axis(member_idx, control_axis);
                        axis_code = control_axis + 3;
                    end
                    if pulse <= 1e-12
                        continue;
                    end
                    interval = [start_time, start_time + pulse];
                    if async_type == 1
                        force_intervals{member_idx}(end + 1, :) = interval;
                    else
                        attitude_intervals{member_idx}(end + 1, :) = interval;
                    end
                    axis_intervals{member_idx}(end + 1, :) = [interval, axis_code];
                    start_time = interval(2);
                end
            end
            intervals{member_idx} = [attitude_intervals{member_idx}; force_intervals{member_idx}];
        end
        schedule_out = struct('enabled', true, 'intervals', {intervals}, 'force_intervals', {force_intervals}, 'attitude_intervals', {attitude_intervals}, 'total_pulse', sum(force_axis + attitude_axis, 2), 'force_pulse', sum(force_axis, 2), 'attitude_pulse', sum(attitude_axis, 2), 'job_starts', [], 'axis_intervals', {axis_intervals});
        scheduler_out = struct('peak_orbit_torque', NaN, 'peak_attitude_force', NaN, 'peak_torque_limit', NaN, 'peak_force_limit', NaN, 'peak_constraint_met', true, 'suppressed_force_axes', [], 'force_axis_pulse', force_axis, 'attitude_axis_pulse', attitude_axis, 'time_scale', 1, 'status', 'taskbook_asynchronous');
    end

    % 直接求健康推力器的有界六维最小二乘占空比。
% 原实现保留（口径或界面已调整；冲突部分停用）：
%         result_info = struct('mode', 'joint_optimization', 'reuse_strategy', 'joint_optimization', 'allocation_strategy', 'joint_optimization', 'solver', 'bounded_projected_gradient_weighted_least_squares', 'fault_policy', 'remove_faulty_columns_before_optimization', 'strict_paired_mode', false, 'uses_impulse_margin_fallback', false, 'nominal_reuse_count', zeros(N, 1), 'structurally_reconfigurable', relative_residual <= 5e-3, 'strictly_reconfigurable', relative_residual <= 5e-3, 'Prop_F', zeros(N, 1), 'Prop_T', zeros(N, 1), 'Prop_F_used', zeros(N, 1), 'Prop_T_used', zeros(N, 1), 'Prop_F_axis', zeros(N, 3), 'Prop_T_axis', zeros(N, 3), 'Prop_F_axis_raw', zeros(N, 3), 'Prop_T_axis_raw', zeros(N, 3), 'Prop_6D', output_pulse, 'Prop_Final', output_pulse, 'schedule', schedule_out, 'scheduler', struct('peak_orbit_torque', NaN, 'peak_attitude_force', NaN, 'peak_torque_limit', NaN, 'peak_force_limit', NaN, 'peak_constraint_met', true, 'suppressed_force_axes', [], 'force_axis_pulse', zeros(N, 3), 'attitude_axis_pulse', zeros(N, 3), 'time_scale', 1, 'status', 'joint_optimization'), 'scheduler_force_axis_suppressed', [], 'axis_strategy', {joint_strategy}, 'impulse_margin_unavailable_axes', [], 'force_scale', NaN, 'attitude_scale', NaN, 'force_scale_reason', 'not_separated_in_joint_optimization', 'attitude_scale_reason', 'not_separated_in_joint_optimization', 'orbit_wrench_raw', NaN(6, 1), 'attitude_wrench_raw', NaN(6, 1), 'orbit_wrench', NaN(6, 1), 'attitude_wrench', NaN(6, 1), 'orbit_cross_torque', NaN, 'attitude_cross_force', NaN, 'period_average_decoupled', false, 'duty', duty, 'command', command, 'actual', actual, 'residual', command - actual, 'relative_residual', relative_residual, 'iterations', iterations, 'healthy_thrusters', healthy, 'faulty_thrusters', faulty_thrusters, 'reuse_scale', ones(N, 1), 'orbit_keep_ratio', NaN, 'att_keep_ratio', NaN, 'max_raw_pulse', max(output_pulse), 'attitude_window', NaN, 'position_window', NaN);
    function [output_pulse, result_info] = Joint_Optimization()
        healthy = setdiff(1:N, faulty_thrusters);
        joint_scale = max(abs(Matrix_sub), [], 2);
        joint_scale(joint_scale < 1e-12) = 1;
        A = Matrix_sub(:, healthy) ./ joint_scale;
        b = command ./ joint_scale;
        duty_healthy = zeros(numel(healthy), 1);
        iterations = 0;
        if ~isempty(healthy)
            [duty_healthy, ~] = Projected_Gradient(A, b, 1e-6, zeros(numel(healthy), 1), ones(numel(healthy), 1), 800, 1e-9);
            active = duty_healthy >= params.t_min / params.T - 1e-9;
            lower = zeros(numel(healthy), 1);
            upper = zeros(numel(healthy), 1);
            lower(active) = params.t_min / params.T;
            upper(active) = 1;
            [duty_healthy, iterations] = Projected_Gradient(A, b, 1e-6, lower, upper, 800, 1e-9);
        end
        duty = zeros(N, 1);
        duty(healthy) = duty_healthy;
        output_pulse = duty * params.T;
        intervals = cell(N, 1);
        axis_intervals = cell(N, 1);
        for member_idx = find(output_pulse > 1e-12)'
            intervals{member_idx} = [0, output_pulse(member_idx)];
            axis_intervals{member_idx} = [0, output_pulse(member_idx), 0];
        end
        schedule_out = struct('enabled', true, 'intervals', {intervals}, 'force_intervals', {cell(N, 1)}, 'attitude_intervals', {cell(N, 1)}, 'total_pulse', output_pulse, 'force_pulse', zeros(N, 1), 'attitude_pulse', zeros(N, 1), 'job_starts', [], 'axis_intervals', {axis_intervals});
        joint_strategy = cell(2, 3);
        for joint_type = 1:2
            for control_axis = 1:3
                joint_strategy{joint_type, control_axis} = struct('status', 'joint_optimization', 'reconfigured', ~isempty(faulty_thrusters));
            end
        end
        actual = Matrix_sub * duty;
        relative_residual = norm((command - actual) ./ joint_scale) / (norm(command ./ joint_scale) + eps);
        % 单条指令的拟合误差不能证明12向可重构，更不能证明瞬时纯控制。
        % 全工况结论由Reconfig_eval复核，false在这里表示尚未取得该证明。
        result_info = struct('mode', 'joint_optimization', 'reuse_strategy', 'joint_optimization', 'allocation_strategy', 'joint_optimization', 'solver', 'bounded_projected_gradient_weighted_least_squares', 'fault_policy', 'remove_faulty_columns_before_optimization', 'strict_paired_mode', false, 'uses_impulse_margin_fallback', false, 'nominal_reuse_count', zeros(N, 1), 'structurally_reconfigurable', false, 'strictly_reconfigurable', false, 'reconfigurability_status', 'not_evaluated', 'command_tracking_met', relative_residual <= 5e-3, 'Prop_F', zeros(N, 1), 'Prop_T', zeros(N, 1), 'Prop_F_used', zeros(N, 1), 'Prop_T_used', zeros(N, 1), 'Prop_F_axis', zeros(N, 3), 'Prop_T_axis', zeros(N, 3), 'Prop_F_axis_raw', zeros(N, 3), 'Prop_T_axis_raw', zeros(N, 3), 'Prop_6D', output_pulse, 'Prop_Final', output_pulse, 'schedule', schedule_out, 'scheduler', struct('peak_orbit_torque', NaN, 'peak_attitude_force', NaN, 'peak_torque_limit', NaN, 'peak_force_limit', NaN, 'peak_constraint_met', true, 'suppressed_force_axes', [], 'force_axis_pulse', zeros(N, 3), 'attitude_axis_pulse', zeros(N, 3), 'time_scale', 1, 'status', 'joint_optimization'), 'scheduler_force_axis_suppressed', [], 'axis_strategy', {joint_strategy}, 'impulse_margin_unavailable_axes', [], 'force_scale', NaN, 'attitude_scale', NaN, 'force_scale_reason', 'not_separated_in_joint_optimization', 'attitude_scale_reason', 'not_separated_in_joint_optimization', 'orbit_wrench_raw', NaN(6, 1), 'attitude_wrench_raw', NaN(6, 1), 'orbit_wrench', NaN(6, 1), 'attitude_wrench', NaN(6, 1), 'orbit_cross_torque', NaN, 'attitude_cross_force', NaN, 'period_average_decoupled', false, 'duty', duty, 'command', command, 'actual', actual, 'residual', command - actual, 'relative_residual', relative_residual, 'iterations', iterations, 'healthy_thrusters', healthy, 'faulty_thrusters', faulty_thrusters, 'reuse_scale', ones(N, 1), 'orbit_keep_ratio', NaN, 'att_keep_ratio', NaN, 'max_raw_pulse', max(output_pulse), 'attitude_window', NaN, 'position_window', NaN);
    end

    % 用投影梯度法求解不依赖工具箱的有界最小二乘问题。
    function [duty, iterations] = Projected_Gradient(A, b, regularization, lower, upper, max_iterations, tolerance)
        duty = min(upper, max(lower, A \ b));
        step = 1 / max(norm(A, 2)^2 + regularization, 1e-12);
        for iterations = 1:max_iterations
            next_duty = min(upper, max(lower, duty - step * (A' * (A * duty - b) + regularization * duty)));
            if norm(next_duty - duty) <= tolerance * max(1, norm(duty))
                duty = next_duty;
                return;
            end
            duty = next_duty;
        end
    end

    % 将正负方向映射到调用库的两列。
    function direction_col = Direction_Column(direction)
        if direction < 0
            direction_col = 1;
        else
            direction_col = 2;
        end
    end

end
