%% 带峰值耦合约束的推力器脉冲时序调度器
function [schedule, force_axis_pulse, attitude_axis_pulse, result] = ...
        Thruster_pulse_scheduler(Matrix_sub, force_axis_pulse, ...
        attitude_axis_pulse, params)
%THRUSTER_PULSE_SCHEDULER 将分配脉宽安排为实际喷气时间段。
%
% 输入的 force_axis_pulse、attitude_axis_pulse 均为 N×3 矩阵：
% 第 j 列表示第 j 个力轴或力矩轴对应纯控制组中每台推力器的总脉宽。
% 分配层已保证它们在一个控制周期内满足
%
%   (1/T) * B * P_F = [F_sel; 0],
%   (1/T) * B * P_T = [0; T_sel].
%
% 但若所有推力器都在 t=0 开始喷气，脉宽不相等时会出现瞬时残余：
%
%   d_M^F(t) = [B*b_F(t)]_(4:6),
%   d_F^T(t) = [B*b_T(t)]_(1:3).
%
% 本函数以“一个轴向调用组”为最小调度单元，将同组推力器安排在
% 同一开始时刻。对标况等脉宽纯组，这使每个喷气时段仍为纯轴向；
% 对故障后不等脉宽的组，调度器搜索开始时刻以降低残余峰值。
% 所有同一台推力器的组时间段均不允许重叠，因此真实喷气总时长
% 与分配层给出的脉宽严格一致。

    N = size(Matrix_sub, 2);
    if size(Matrix_sub, 1) ~= 6
        error('Thruster_pulse_scheduler:InvalidMatrix', ...
            'Matrix_sub 必须为 6xN 控制效率矩阵。');
    end
    if ~isequal(size(force_axis_pulse), [N, 3]) || ...
            ~isequal(size(attitude_axis_pulse), [N, 3])
        error('Thruster_pulse_scheduler:InvalidPulse', ...
            '力和力矩分轴脉宽必须为 N×3 矩阵。');
    end

    T_ctrl = Get_Param(params, 'T', 0.4);
    config = Scheduler_Config();
    force_axis_pulse = max(0, force_axis_pulse);
    attitude_axis_pulse = max(0, attitude_axis_pulse);
    allow_odd_coupled_groups = logical(Get_Param(params, ...
        'scheduler_allow_odd_coupled_groups', false));
    if logical(Get_Param(params, 'alloc_even_group_only', true)) && ...
            ~allow_odd_coupled_groups
        Validate_Even_Groups(force_axis_pulse, '轨控');
        Validate_Even_Groups(attitude_axis_pulse, '姿控');
    end

    if ~config.enable
        % 关闭调度器时保留旧执行方式：每台推力器从周期起点连续喷气。
        schedule = Start_At_Zero_Schedule( ...
            force_axis_pulse, attitude_axis_pulse);
        result = Evaluate_Result(schedule, force_axis_pulse, ...
            attitude_axis_pulse, []);
        result.status = 'scheduler_disabled';
        return;
    end

    jobs = Build_Jobs(force_axis_pulse, attitude_axis_pulse);
    suppressed_force_axes = [];
    [schedule, scheduled_mask, peaks] = Build_Schedule(jobs);

    % 调度无法在一个周期内安排某个轨控组时，直接停用该完整轨控组。
    % 绝不删减组内单台脉宽，以免重新引入周期平均耦合。
    while any(~scheduled_mask & [jobs.active] & ...
            strcmp({jobs.type}, 'force'))
        failed_idx = find(~scheduled_mask & [jobs.active] & ...
            strcmp({jobs.type}, 'force'), 1);
        jobs(failed_idx).active = false;
        force_axis_index = jobs(failed_idx).axis;
        force_axis_pulse(:, force_axis_index) = 0;
        suppressed_force_axes(end + 1) = force_axis_index; %#ok<AGROW>
        [schedule, scheduled_mask, peaks] = Build_Schedule(jobs);
    end

    % 姿控作业优先：若姿控作业本身无法排入周期，不删减其脉宽，而是
    % 返回不可行状态，由上层记录风险。这样不会以牺牲姿控换取表面可行。
    if any(~scheduled_mask & [jobs.active] & ...
            strcmp({jobs.type}, 'attitude'))
        schedule = Start_At_Zero_Schedule( ...
            force_axis_pulse, attitude_axis_pulse);
        result = Evaluate_Result(schedule, force_axis_pulse, ...
            attitude_axis_pulse, suppressed_force_axes);
        result.status = 'attitude_schedule_infeasible';
        result.peak_constraint_met = false;
        return;
    end

    % 硬约束只作用于“轨控产生的瞬时残余力矩”。若当前作业集合超过
    % 上限，则逐次移除能最大幅度降低峰值的整条轨控轴，直至满足约束。
    % 这对应 d_M^F(t) 必须保留在姿控预留裕度以内的工程要求。
    while peaks.orbit_torque > config.peak_torque_limit + config.time_tol
        active_force_idx = find([jobs.active] & ...
            strcmp({jobs.type}, 'force'));
        if isempty(active_force_idx)
            break;
        end

        best_idx = 0;
        best_score = inf;
        best_schedule = schedule;
        best_mask = scheduled_mask;
        best_peaks = peaks;
        for candidate_idx = active_force_idx
            trial_jobs = jobs;
            trial_jobs(candidate_idx).active = false;
            [trial_schedule, trial_mask, trial_peaks] = ...
                Build_Schedule(trial_jobs);
            if any(~trial_mask & [trial_jobs.active] & ...
                    strcmp({trial_jobs.type}, 'attitude'))
                continue;
            end
            trial_score = Peak_Score(trial_peaks);
            if trial_score < best_score - config.time_tol || ...
                    (abs(trial_score - best_score) <= config.time_tol && ...
                     jobs(candidate_idx).axis < jobs(max(best_idx, 1)).axis)
                best_idx = candidate_idx;
                best_score = trial_score;
                best_schedule = trial_schedule;
                best_mask = trial_mask;
                best_peaks = trial_peaks;
            end
        end

        if best_idx == 0
            break;
        end
        jobs(best_idx).active = false;
        force_axis_index = jobs(best_idx).axis;
        force_axis_pulse(:, force_axis_index) = 0;
        suppressed_force_axes(end + 1) = force_axis_index; %#ok<AGROW>
        schedule = best_schedule;
        scheduled_mask = best_mask;
        peaks = best_peaks;
    end

    if peaks.orbit_torque > config.peak_torque_limit + config.time_tol
        % 若有限次重排和逐轴删减仍无法满足硬约束，执行保守失效保护：
        % 停用剩余轨控组，只保留姿控。宁可暂时损失轨控量，也不能带着
        % 已知超出姿控预留裕度的瞬时扰动继续喷气。
        remaining_force_idx = find([jobs.active] & ...
            strcmp({jobs.type}, 'force'));
        for remaining_job_idx = remaining_force_idx
            jobs(remaining_job_idx).active = false;
            force_axis_index = jobs(remaining_job_idx).axis;
            force_axis_pulse(:, force_axis_index) = 0;
            suppressed_force_axes(end + 1) = force_axis_index; %#ok<AGROW>
        end
        [schedule, ~, peaks] = Build_Schedule(jobs);
    end

    result = Evaluate_Result(schedule, force_axis_pulse, ...
        attitude_axis_pulse, suppressed_force_axes);
    if result.peak_constraint_met
        result.status = 'peak_constrained_schedule';
    elseif ~isempty(suppressed_force_axes)
        result.status = 'peak_constraint_unmet_after_force_suppression';
    else
        result.status = 'peak_constraint_unmet';
    end

    function config = Scheduler_Config()
        config.enable = logical(Get_Param(params, ...
            'scheduler_enable', true));
        config.time_tol = Get_Param(params, 'scheduler_time_tol', 1e-10);
        config.peak_torque_ratio = Get_Param(params, ...
            'scheduler_peak_torque_ratio', 0.25);
        config.peak_force_ratio = Get_Param(params, ...
            'scheduler_peak_force_ratio', inf);
        config.use_impulse_margin_fallback = logical(Get_Param(params, ...
            'scheduler_use_impulse_margin_fallback', false));

        % 无显式上限时，以全部推力器同向可提供的最小轴向力矩能力
        % 的一定比例作为保守的姿控预留裕度。用户可在 params 中直接
        % 给 scheduler_peak_torque_limit 覆盖该默认值。
        torque_capacity = sum(abs(Matrix_sub(4:6, :)), 2);
        force_capacity = sum(abs(Matrix_sub(1:3, :)), 2);
        default_torque_limit = config.peak_torque_ratio * ...
            min(torque_capacity(torque_capacity > config.time_tol));
        if isempty(default_torque_limit) || ~isfinite(default_torque_limit)
            default_torque_limit = inf;
        end
        default_force_limit = config.peak_force_ratio * ...
            min(force_capacity(force_capacity > config.time_tol));
        if isempty(default_force_limit) || ~isfinite(default_force_limit)
            default_force_limit = inf;
        end
        config.peak_torque_limit = Get_Param(params, ...
            'scheduler_peak_torque_limit', default_torque_limit);
        config.peak_force_limit = Get_Param(params, ...
            'scheduler_peak_force_limit', default_force_limit);
        if config.use_impulse_margin_fallback
            % 非严格故障备用已经在分配层用“累计干扰冲量/反向储备”
            % 判据筛选。这里不再用与持续时间无关的固定峰值阈值二次
            % 否决，避免短时、可恢复的扰动被过度保守地停用。
            config.peak_torque_limit = inf;
            config.peak_force_limit = inf;
        end
        if config.peak_torque_limit <= 0
            config.peak_torque_limit = inf;
        end
        if config.peak_force_limit <= 0
            config.peak_force_limit = inf;
        end
    end

    function Validate_Even_Groups(axis_pulse, control_name)
        % 调度器只接受完整偶数调用组。这样即使上层代码后续修改，
        % 也不会把某台故障后的残余成员当成一个新的“纯控制组”执行。
        for axis_idx = 1:size(axis_pulse, 2)
            active_count = sum(axis_pulse(:, axis_idx) > config.time_tol);
            if mod(active_count, 2) ~= 0
                error('Thruster_pulse_scheduler:OddGroup', ...
                    '%s第 %d 轴调用组含 %d 台推力器；成对调用策略不允许奇数台。', ...
                    control_name, axis_idx, active_count);
            end
        end
    end

    function jobs = Build_Jobs(force_pulse, attitude_pulse)
        jobs = repmat(struct('type', '', 'axis', 0, 'pulse', [], ...
            'active', false), 6, 1);
        for channel_idx = 1:3
            jobs(channel_idx).type = 'attitude';
            jobs(channel_idx).axis = channel_idx;
            jobs(channel_idx).pulse = attitude_pulse(:, channel_idx);
            jobs(channel_idx).active = any( ...
                jobs(channel_idx).pulse > config.time_tol);

            jobs(channel_idx + 3).type = 'force';
            jobs(channel_idx + 3).axis = channel_idx;
            jobs(channel_idx + 3).pulse = force_pulse(:, channel_idx);
            jobs(channel_idx + 3).active = any( ...
                jobs(channel_idx + 3).pulse > config.time_tol);
        end
    end

    function [schedule_out, scheduled, peak_value] = Build_Schedule(job_list)
        total_intervals = cell(N, 1);
        force_intervals = cell(N, 1);
        attitude_intervals = cell(N, 1);
        scheduled = false(numel(job_list), 1);
        starts = nan(numel(job_list), 1);

        % 姿控组总是先排；同类中先排持续时间较长的组，降低后续冲突。
        active_idx = find([job_list.active]);
        attitude_idx = active_idx(strcmp({job_list(active_idx).type}, 'attitude'));
        force_idx = active_idx(strcmp({job_list(active_idx).type}, 'force'));
        [~, attitude_order] = sort(arrayfun(@(x)max(job_list(x).pulse), ...
            attitude_idx), 'descend');
        [~, force_order] = sort(arrayfun(@(x)max(job_list(x).pulse), ...
            force_idx), 'descend');
        order = [attitude_idx(attitude_order), force_idx(force_order)];

        for scheduled_job_idx = order
            job = job_list(scheduled_job_idx);
            [start_time, is_feasible] = Find_Start( ...
                job, total_intervals, force_intervals, attitude_intervals);
            if ~is_feasible
                continue;
            end
            [total_intervals, force_intervals, attitude_intervals] = ...
                Insert_Job(job, start_time, total_intervals, ...
                force_intervals, attitude_intervals);
            scheduled(scheduled_job_idx) = true;
            starts(scheduled_job_idx) = start_time;
        end

        schedule_out = Make_Schedule(total_intervals, force_intervals, ...
            attitude_intervals, starts, true, job_list, scheduled);
        peak_value = Evaluate_Peaks(schedule_out);
    end

    function [start_time, feasible] = Find_Start( ...
            job, total_intervals, force_intervals, attitude_intervals)
        active_thrusters = find(job.pulse > config.time_tol);
        max_pulse = max([job.pulse(active_thrusters); 0]);
        if isempty(active_thrusters) || max_pulse <= config.time_tol
            start_time = 0;
            feasible = true;
            return;
        end
        if max_pulse > T_ctrl + config.time_tol
            start_time = NaN;
            feasible = false;
            return;
        end

        candidates = [0; T_ctrl - max_pulse];
        for thruster_idx = active_thrusters(:)'
            intervals = total_intervals{thruster_idx};
            if ~isempty(intervals)
                candidates = [candidates; intervals(:, 1); intervals(:, 2)]; %#ok<AGROW>
            end
        end
        candidates = unique(candidates);
        candidates = candidates(candidates >= -config.time_tol & ...
            candidates + max_pulse <= T_ctrl + config.time_tol);

        best_score = inf;
        start_time = NaN;
        for candidate = candidates(:)'
            if ~Is_Start_Feasible(job, candidate, total_intervals)
                continue;
            end
            [trial_total, trial_force, trial_attitude] = Insert_Job( ...
                job, candidate, total_intervals, force_intervals, ...
                attitude_intervals);
            trial_schedule = Make_Schedule(trial_total, trial_force, ...
                trial_attitude, [], true);
            score = Peak_Score(Evaluate_Peaks(trial_schedule));
            if score < best_score - config.time_tol || ...
                    (abs(score - best_score) <= config.time_tol && ...
                     (isnan(start_time) || candidate < start_time))
                best_score = score;
                start_time = candidate;
            end
        end
        feasible = ~isnan(start_time);
    end

    function tf = Is_Start_Feasible(job, start_time, total_intervals)
        tf = true;
        for thruster_idx = find(job.pulse > config.time_tol(:))'
            end_time = start_time + job.pulse(thruster_idx);
            intervals = total_intervals{thruster_idx};
            if end_time > T_ctrl + config.time_tol
                tf = false;
                return;
            end
            if isempty(intervals)
                continue;
            end
            overlap = start_time < intervals(:, 2) - config.time_tol & ...
                end_time > intervals(:, 1) + config.time_tol;
            if any(overlap)
                tf = false;
                return;
            end
        end
    end

    function [total_out, force_out, attitude_out] = Insert_Job( ...
            job, start_time, total_in, force_in, attitude_in)
        total_out = total_in;
        force_out = force_in;
        attitude_out = attitude_in;
        for thruster_idx = find(job.pulse > config.time_tol(:))'
            interval = [start_time, start_time + job.pulse(thruster_idx)];
            total_out{thruster_idx} = Sort_Intervals( ...
                [total_out{thruster_idx}; interval]);
            if strcmp(job.type, 'force')
                force_out{thruster_idx} = Sort_Intervals( ...
                    [force_out{thruster_idx}; interval]);
            else
                attitude_out{thruster_idx} = Sort_Intervals( ...
                    [attitude_out{thruster_idx}; interval]);
            end
        end
    end

    function schedule_out = Make_Schedule(total, force, attitude, starts, ...
            enabled, job_list, scheduled)
        if nargin < 6
            job_list = [];
        end
        if nargin < 7
            scheduled = [];
        end
        schedule_out = struct();
        schedule_out.enabled = enabled;
        schedule_out.intervals = total;
        schedule_out.force_intervals = force;
        schedule_out.attitude_intervals = attitude;
        schedule_out.total_pulse = cellfun(@Interval_Length, total);
        schedule_out.force_pulse = cellfun(@Interval_Length, force);
        schedule_out.attitude_pulse = cellfun(@Interval_Length, attitude);
        schedule_out.job_starts = starts;
        schedule_out.axis_intervals = Build_Axis_Intervals( ...
            job_list, starts, scheduled);
    end

    function schedule_out = Start_At_Zero_Schedule(force_pulse, attitude_pulse)
        force = cell(N, 1);
        attitude = cell(N, 1);
        total = cell(N, 1);
        force_total = sum(force_pulse, 2);
        attitude_total = sum(attitude_pulse, 2);
        for thruster_idx = 1:N
            if force_total(thruster_idx) > config.time_tol
                force{thruster_idx} = [0, force_total(thruster_idx)];
            end
            if attitude_total(thruster_idx) > config.time_tol
                attitude{thruster_idx} = [0, attitude_total(thruster_idx)];
            end
            total_pulse = force_total(thruster_idx) + ...
                attitude_total(thruster_idx);
            if total_pulse > config.time_tol
                total{thruster_idx} = [0, min(T_ctrl, total_pulse)];
            end
        end
        schedule_out = Make_Schedule(total, force, attitude, [], false);
        schedule_out.axis_intervals = Start_At_Zero_Axis_Intervals( ...
            force_pulse, attitude_pulse);
    end

    function axis_intervals = Build_Axis_Intervals( ...
            job_list, starts, scheduled)
        % Each row is [start_time, end_time, axis_id].  axis_id 1..3 is
        % Fx/Fy/Fz, and 4..6 is Mx/My/Mz.  The plotting layer uses this
        % record directly, so a coloured line always represents one real
        % scheduled control group rather than a reconstructed pulse total.
        axis_intervals = cell(N, 1);
        if isempty(job_list) || isempty(starts) || isempty(scheduled)
            return;
        end
        for job_idx = 1:numel(job_list)
            if ~scheduled(job_idx) || ~job_list(job_idx).active || ...
                    ~isfinite(starts(job_idx))
                continue;
            end
            if strcmp(job_list(job_idx).type, 'force')
                axis_id = job_list(job_idx).axis;
            else
                axis_id = job_list(job_idx).axis + 3;
            end
            for thruster_idx = find( ...
                    job_list(job_idx).pulse > config.time_tol(:))'
                axis_intervals{thruster_idx}(end + 1, :) = [ ...
                    starts(job_idx), starts(job_idx) + ...
                    job_list(job_idx).pulse(thruster_idx), axis_id]; %#ok<AGROW>
            end
        end
        for thruster_idx = 1:N
            if ~isempty(axis_intervals{thruster_idx})
                axis_intervals{thruster_idx} = sortrows( ...
                    axis_intervals{thruster_idx}, [1, 2]);
            end
        end
    end

    function axis_intervals = Start_At_Zero_Axis_Intervals( ...
            force_pulse, attitude_pulse)
        axis_intervals = cell(N, 1);
        for thruster_idx = 1:N
            for axis_idx = 1:3
                if attitude_pulse(thruster_idx, axis_idx) > config.time_tol
                    axis_intervals{thruster_idx}(end + 1, :) = [0, ...
                        attitude_pulse(thruster_idx, axis_idx), axis_idx + 3]; %#ok<AGROW>
                end
                if force_pulse(thruster_idx, axis_idx) > config.time_tol
                    axis_intervals{thruster_idx}(end + 1, :) = [0, ...
                        force_pulse(thruster_idx, axis_idx), axis_idx]; %#ok<AGROW>
                end
            end
        end
    end

    function peaks = Evaluate_Peaks(schedule_in)
        edges = [0; T_ctrl];
        for thruster_idx = 1:N
            edges = [edges; schedule_in.force_intervals{thruster_idx}(:); ...
                schedule_in.attitude_intervals{thruster_idx}(:)]; %#ok<AGROW>
        end
        edges = unique(edges);
        peaks = struct('orbit_torque', 0, 'attitude_force', 0);
        for edge_idx = 1:(numel(edges) - 1)
            if edges(edge_idx + 1) - edges(edge_idx) <= config.time_tol
                continue;
            end
            sample_time = 0.5 * (edges(edge_idx) + edges(edge_idx + 1));
            force_on = Interval_On(schedule_in.force_intervals, sample_time);
            attitude_on = Interval_On( ...
                schedule_in.attitude_intervals, sample_time);
            force_wrench = Matrix_sub * double(force_on);
            attitude_wrench = Matrix_sub * double(attitude_on);
            peaks.orbit_torque = max(peaks.orbit_torque, ...
                norm(force_wrench(4:6)));
            peaks.attitude_force = max(peaks.attitude_force, ...
                norm(attitude_wrench(1:3)));
        end
    end

    function on = Interval_On(intervals, sample_time)
        on = false(N, 1);
        for thruster_idx = 1:N
            interval = intervals{thruster_idx};
            if isempty(interval)
                continue;
            end
            on(thruster_idx) = any(sample_time >= interval(:, 1) - ...
                config.time_tol & sample_time < interval(:, 2) - config.time_tol);
        end
    end

    function result_out = Evaluate_Result( ...
            schedule_in, force_pulse, attitude_pulse, suppressed_axes)
        peaks = Evaluate_Peaks(schedule_in);
        result_out = struct();
        result_out.peak_orbit_torque = peaks.orbit_torque;
        result_out.peak_attitude_force = peaks.attitude_force;
        result_out.peak_torque_limit = config.peak_torque_limit;
        result_out.peak_force_limit = config.peak_force_limit;
        result_out.peak_constraint_met = ...
            peaks.orbit_torque <= config.peak_torque_limit + config.time_tol && ...
            peaks.attitude_force <= config.peak_force_limit + config.time_tol;
        result_out.suppressed_force_axes = unique(suppressed_axes);
        result_out.force_axis_pulse = force_pulse;
        result_out.attitude_axis_pulse = attitude_pulse;
        result_out.status = 'unknown';
    end

    function score = Peak_Score(peaks)
        torque_score = peaks.orbit_torque / config.peak_torque_limit;
        force_score = peaks.attitude_force / config.peak_force_limit;
        score = max(torque_score, force_score);
    end

    function output = Sort_Intervals(input)
        if isempty(input)
            output = zeros(0, 2);
        else
            output = sortrows(input, [1, 2]);
        end
    end

    function length = Interval_Length(intervals)
        if isempty(intervals)
            length = 0;
        else
            length = sum(intervals(:, 2) - intervals(:, 1));
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
end
