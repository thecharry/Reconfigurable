%% 闭环仿真函数
function log = Closedloop_sim(params, B, sim_cfg_override)
    if nargin < 3 || isempty(sim_cfg_override)
        sim_cfg_override = struct();
    end

    % 初始化
    sim_cfg = Sim_config(params, sim_cfg_override);
    r0 = sim_cfg.r0;
    rt = sim_cfg.rt;
    v0 = sim_cfg.v0;
    euler0 = sim_cfg.euler0;
    eulert = sim_cfg.eulert;
    sigma0 = sim_cfg.sigma0;
    omega0 = sim_cfg.omega0;
    T_sim = sim_cfg.T_sim;
    dt = sim_cfg.dt;

    next_ctrl = 0;
    Matrix_conf = params.F_max * B;
    Y = [r0; v0; sigma0; omega0];
    N = floor(T_sim / dt);
    max_ctrl = ceil(T_sim / params.T) + 10;
    faulty_trig = false;% 故障触发标志位
    true_faults = sim_cfg.true_faults;
    estimated_faults = [];
    active_diag_next_idx = 1;
    diagnosis_trig = false;% 诊断触发标志位
    diagnosis_success = false;
    diagnosis_time = NaN;
    diagnosis_enable = true;% 启用诊断功能

    int = [0;0;0];
    Kp_pos = sim_cfg.Kp_pos;
    Kd_pos = sim_cfg.Kd_pos;
    Kp_att = sim_cfg.Kp_att;
    Kd_att = sim_cfg.Kd_att;
    Ki_att = sim_cfg.Ki_att;

    log.Y_euler = zeros(3, N);
    log.Time = zeros(1, N);
    log.R = zeros(3, N);
    log.V = zeros(3, N);
    log.E = zeros(3, N);
    log.O = zeros(3, N);
    log.Y = zeros(12, N);
    log.Pulse_Widths = zeros(params.Num, N);
    log.Pulse_History = zeros(params.Num, max_ctrl);
    log.Pulse_Command_History = zeros(params.Num, max_ctrl);
    log.Pulse_Actual_History = zeros(params.Num, max_ctrl);
    log.Pulse_Att_History = zeros(params.Num, max_ctrl);
    log.Pulse_Orbit_History = zeros(params.Num, max_ctrl);
    log.Pulse_6D_History = zeros(params.Num, max_ctrl);
    log.Pulse_Schedule_History = cell(1, max_ctrl);
    log.Scheduler_Peak_Torque_History = NaN(1, max_ctrl);
    log.Scheduler_Peak_Force_History = NaN(1, max_ctrl);
    log.Scheduler_Torque_Limit_History = NaN(1, max_ctrl);
    log.Scheduler_Force_Suppressed_History = cell(1, max_ctrl);
    log.Requested_Wrench_History = zeros(6, max_ctrl);
    log.Commanded_Wrench_History = zeros(6, max_ctrl);
    log.Actual_Wrench_History = zeros(6, max_ctrl);
    log.Attitude_Window_History = NaN(1, max_ctrl);
    log.Position_Window_History = NaN(1, max_ctrl);
    log.Alloc_Mode_History = strings(1, max_ctrl);
    log.Active_Diagnosis_History = zeros(1, max_ctrl);
    log.Control_Time = zeros(1, max_ctrl);
    log.Control_Period = params.T;
    log.Total_Pulse = 0;% 整个任务累计总喷气时长
    log.Control_Count = 0;% 控制更新次数
    log.faulty_thrusters = [];
    log.estimated_faults = [];
    log.faulty_time = sim_cfg.faulty_time;
    log.diagnosis_time = NaN;
    log.diagnosis_success = false;

    % tic;
    for k = 1:N
        t = (k-1) * dt;
        if t >= log.faulty_time && ~faulty_trig
            faulty_trig = true;
            log.faulty_thrusters = true_faults;
            if ~diagnosis_enable
                estimated_faults = true_faults;
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
            [~, ~, R_b2o, R] = Matrix_conver(sigma, euler_d);
            sigma_d = Euler_to_MRPs(euler_d);
            omega_d = R * euler_rate_d;
            % 轨道控制期望控制力
            F_orbit_req =  Kp_pos * (r_d - r) + Kd_pos * (v_d - v);
            F_body_req = R_b2o' * F_orbit_req;
            % 姿态控制期望控制力矩
            sigma_err = ((1-sigma'*sigma)*sigma_d-(1-sigma_d'*sigma_d)*sigma-2*cross(sigma_d,-sigma))/...
                        (1+(sigma_d'*sigma_d)*(sigma'*sigma)-2*dot(sigma_d,-sigma));% MRP乘法运算法则求误差姿态
            int = int + sigma_err * params.T;
            T_body_req = Kp_att * sigma_err + Kd_att * (omega_d - omega)+ Ki_att * int;
            % 推力器调用策略
            [Prop_Command, alloc_info] = Invoke_Thruster_Allocator(F_body_req,T_body_req,Matrix_conf,estimated_faults,params);
            [Prop_Command, active_diag_next_idx, active_diag_thruster] = ...
                Apply_Active_Diagnosis(Prop_Command, active_diag_next_idx, faulty_trig, diagnosis_trig, diagnosis_enable, sim_cfg);
            Prop_Actual = Actual_invocation(Prop_Command, true_faults, faulty_trig);
            % 故障诊断
            if faulty_trig && ~diagnosis_trig && diagnosis_enable
                [faults] = Diagnose_faults(Matrix_conf, Prop_Command, Prop_Actual, sim_cfg.residual_threshold);
                if ~isempty(faults)
                    diagnosis_trig = true;
                    estimated_faults = faults;
                    diagnosis_success = isequal(estimated_faults, true_faults);
                    diagnosis_time = t;
                    [Prop_Command, alloc_info] = Invoke_Thruster_Allocator(F_body_req,T_body_req,Matrix_conf,estimated_faults,params);
                    Prop_Actual = Actual_invocation(Prop_Command,true_faults,faulty_trig);
                end
            end
            [Prop_Att, Prop_Orbit, Prop_6D] = Allocation_Ownership(alloc_info, Prop_Command, Prop_Actual);
            % 累计真实总喷气时长
            log.Total_Pulse = log.Total_Pulse + sum(Prop_Actual);
            log.Control_Count = log.Control_Count + 1;
            log.Pulse_Command_History(:, log.Control_Count) = Prop_Command;
            log.Pulse_Actual_History(:, log.Control_Count) = Prop_Actual;
            log.Pulse_History(:, log.Control_Count) = Prop_Actual;
            log.Pulse_Att_History(:, log.Control_Count) = Prop_Att;
            log.Pulse_Orbit_History(:, log.Control_Count) = Prop_Orbit;
            log.Pulse_6D_History(:, log.Control_Count) = Prop_6D;
            if isfield(alloc_info, 'schedule')
                schedule_for_log = alloc_info.schedule;
                if faulty_trig && ~isempty(true_faults)
                    % The timing view represents physical firing, not the
                    % pre-diagnosis command.  Failed thrusters therefore
                    % disappear immediately at the fault instant.
                    schedule_for_log = Remove_Faulty_Schedule( ...
                        schedule_for_log, true_faults);
                end
                log.Pulse_Schedule_History{log.Control_Count} = ...
                    schedule_for_log;
            end
            if isfield(alloc_info, 'scheduler')
                scheduler_info = alloc_info.scheduler;
                log.Scheduler_Peak_Torque_History(log.Control_Count) = ...
                    scheduler_info.peak_orbit_torque;
                log.Scheduler_Peak_Force_History(log.Control_Count) = ...
                    scheduler_info.peak_attitude_force;
                log.Scheduler_Torque_Limit_History(log.Control_Count) = ...
                    scheduler_info.peak_torque_limit;
                log.Scheduler_Force_Suppressed_History{ ...
                    log.Control_Count} = scheduler_info.suppressed_force_axes;
            end
            log.Requested_Wrench_History(:, log.Control_Count) = ...
                [F_body_req; T_body_req];
            log.Commanded_Wrench_History(:, log.Control_Count) = ...
                Matrix_conf * (Prop_Command / max(params.T, eps));
            log.Actual_Wrench_History(:, log.Control_Count) = ...
                Matrix_conf * (Prop_Actual / max(params.T, eps));
            log.Attitude_Window_History(log.Control_Count) = Get_Info_Field(alloc_info, 'attitude_window', NaN);
            log.Position_Window_History(log.Control_Count) = Get_Info_Field(alloc_info, 'position_window', NaN);
            log.Alloc_Mode_History(log.Control_Count) = string(Get_Info_Field(alloc_info, 'mode', 'unknown'));
            log.Active_Diagnosis_History(log.Control_Count) = active_diag_thruster;
            log.Control_Time(log.Control_Count) = t;
            % 更新时间
            next_ctrl = next_ctrl + params.T;
        end
        time_in_cycle = t - (next_ctrl - params.T);
        use_scheduled_pulse = active_diag_thruster == 0 && ...
            isfield(alloc_info, 'schedule') && ...
            isfield(alloc_info.schedule, 'enabled') && ...
            alloc_info.schedule.enabled;
        if use_scheduled_pulse
            thruster_on = Scheduled_Thruster_On( ...
                alloc_info.schedule, time_in_cycle, params.Num);
            if faulty_trig && ~isempty(true_faults)
                thruster_on(true_faults) = false;
            end
            u_applied = params.F_max * double(thruster_on);
        else
            % 主动诊断会临时加入单台试探脉冲，未写入纯组时序表；该
            % 周期退回连续起喷模型，避免遗漏诊断激励。
            u_applied = params.F_max * (time_in_cycle < Prop_Actual);
        end
        % 实际力和力矩
        W_total = B * u_applied;
        params.current_F = W_total(1:3);
        params.current_T = W_total(4:6);
        % 动力学积分
        dY = Spacecraft_dynamics(Y, params);
        Y = Y + dY * dt;

        log.Y_euler(:,k) = MRPs_to_Euler(Y(7:9,:));
        log.Time(k) = t;
        log.R(:,k) = r_d;
        log.V(:,k) = v_d;
        log.E(:,k) = euler_d;
        log.O(:,k) = omega_d;
        log.Y(:,k) = Y;
        log.Pulse_Widths(1:params.Num,k) = Prop_Actual;
    end
    % toc;
    log.Pulse_History = log.Pulse_History(:, 1:log.Control_Count);
    log.Pulse_Command_History = log.Pulse_Command_History(:, 1:log.Control_Count);
    log.Pulse_Actual_History = log.Pulse_Actual_History(:, 1:log.Control_Count);
    log.Pulse_Att_History = log.Pulse_Att_History(:, 1:log.Control_Count);
    log.Pulse_Orbit_History = log.Pulse_Orbit_History(:, 1:log.Control_Count);
    log.Pulse_6D_History = log.Pulse_6D_History(:, 1:log.Control_Count);
    log.Pulse_Schedule_History = log.Pulse_Schedule_History(1:log.Control_Count);
    log.Scheduler_Peak_Torque_History = ...
        log.Scheduler_Peak_Torque_History(1:log.Control_Count);
    log.Scheduler_Peak_Force_History = ...
        log.Scheduler_Peak_Force_History(1:log.Control_Count);
    log.Scheduler_Torque_Limit_History = ...
        log.Scheduler_Torque_Limit_History(1:log.Control_Count);
    log.Scheduler_Force_Suppressed_History = ...
        log.Scheduler_Force_Suppressed_History(1:log.Control_Count);
    log.Requested_Wrench_History = ...
        log.Requested_Wrench_History(:, 1:log.Control_Count);
    log.Commanded_Wrench_History = ...
        log.Commanded_Wrench_History(:, 1:log.Control_Count);
    log.Actual_Wrench_History = ...
        log.Actual_Wrench_History(:, 1:log.Control_Count);
    log.Attitude_Window_History = log.Attitude_Window_History(1:log.Control_Count);
    log.Position_Window_History = log.Position_Window_History(1:log.Control_Count);
    log.Alloc_Mode_History = log.Alloc_Mode_History(1:log.Control_Count);
    log.Active_Diagnosis_History = log.Active_Diagnosis_History(1:log.Control_Count);
    log.Control_Time = log.Control_Time(1:log.Control_Count);
    log.estimated_faults = estimated_faults;
    log.diagnosis_time = diagnosis_time;
    log.diagnosis_success = diagnosis_success;

    %% 仿真参数设置
    function sim_cfg = Sim_config(params, sim_cfg_override)
        sim_cfg.r0 = [0;15;55];
        sim_cfg.rt = [5;25;35];
        sim_cfg.v0 = [0;0;0];
        sim_cfg.euler0 = deg2rad([0;15;55]);
        sim_cfg.eulert = deg2rad([5;25;35]);
        sim_cfg.omega0 = [0;0;0];
        sim_cfg.T_sim = 2000;
        sim_cfg.dt = 0.005;
        % sim_cfg.faulty_time = 0.5 * sim_cfg.T_sim;% 固定故障时间
        sim_cfg.faulty_time = (0.2 + 0.6 * rand) * sim_cfg.T_sim;% 随机故障时间
        if isfield(params, 'true_faults') && ~isempty(params.true_faults)
            sim_cfg.true_faults = params.true_faults;
        else
            sim_cfg.true_faults = [];
        end
        sim_cfg.residual_threshold = 1e-9;% 残差阈值
        % 单台主动诊断喷气不经过纯轴向调用组，会刻意产生已知六维扰动。
        % 因此默认关闭；需要诊断试验时可在 sim_cfg_override 中显式打开。
        sim_cfg.active_diagnosis_enable = false;% 主动诊断激励开关
        sim_cfg.active_diagnosis_pulse = params.t_min;% 主动诊断最小脉宽
        sim_cfg.active_diagnosis_command_tol = 1e-12;% 已有指令判定阈值
        if isfield(params, 'active_diagnosis_enable') && ~isempty(params.active_diagnosis_enable)
            sim_cfg.active_diagnosis_enable = params.active_diagnosis_enable;
        end
        if isfield(params, 'active_diagnosis_pulse') && ~isempty(params.active_diagnosis_pulse)
            sim_cfg.active_diagnosis_pulse = params.active_diagnosis_pulse;
        end
        if isfield(params, 'active_diagnosis_command_tol') && ~isempty(params.active_diagnosis_command_tol)
            sim_cfg.active_diagnosis_command_tol = params.active_diagnosis_command_tol;
        end
        sim_cfg.Kp_pos = 10;
        sim_cfg.Kd_pos = 300;
        sim_cfg.Kp_att = 400 * eye(3);
        sim_cfg.Kd_att = 3200 * eye(3);
        sim_cfg.Ki_att = 20 * eye(3);

        has_sigma_override = isfield(sim_cfg_override, 'sigma0') && ~isempty(sim_cfg_override.sigma0);
        sim_cfg = Merge_Sim_Config(sim_cfg, sim_cfg_override);
        if ~has_sigma_override
            sim_cfg.sigma0 = Euler_to_MRPs(sim_cfg.euler0);
        end
    end

    function sim_cfg = Merge_Sim_Config(sim_cfg, sim_cfg_override)
        if isempty(sim_cfg_override)
            return;
        end

        if isfield(sim_cfg_override, 'fault_time') && ~isempty(sim_cfg_override.fault_time)
            sim_cfg_override.faulty_time = sim_cfg_override.fault_time;
        end
        if isfield(sim_cfg_override, 'faulty_thrusters_fixed') && ~isempty(sim_cfg_override.faulty_thrusters_fixed)
            sim_cfg_override.true_faults = sim_cfg_override.faulty_thrusters_fixed;
        end

        names = fieldnames(sim_cfg_override);
        for field_idx = 1:numel(names)
            field_name = names{field_idx};
            if ~isempty(sim_cfg_override.(field_name))
                sim_cfg.(field_name) = sim_cfg_override.(field_name);
            end
        end
    end

    %% 实际推力器输出脉宽
    function [Prop_Command, alloc_info] = Invoke_Thruster_Allocator(F_cmd, T_cmd, Matrix_conf, faulty_thrusters, params)
        [Prop_Command, alloc_info] = Thruster_allocator( ...
            F_cmd, T_cmd, Matrix_conf, faulty_thrusters, params);
    end

    function [Prop_Att, Prop_Orbit, Prop_6D] = Allocation_Ownership(alloc_info, Prop_Command, Prop_Actual)
        Prop_Att = zeros(params.Num, 1);
        Prop_Orbit = zeros(params.Num, 1);
        Prop_6D = zeros(params.Num, 1);

        actual_ratio = zeros(params.Num, 1);
        valid_idx = Prop_Command > 1e-12;
        actual_ratio(valid_idx) = Prop_Actual(valid_idx) ./ Prop_Command(valid_idx);
        actual_ratio = max(0, min(1, actual_ratio));

        if isfield(alloc_info, 'Prop_6D') && any(alloc_info.Prop_6D > 1e-12)
            Prop_6D = alloc_info.Prop_6D(:) .* actual_ratio;
            return;
        end

        if isfield(alloc_info, 'Prop_T_used')
            Prop_Att = alloc_info.Prop_T_used(:) .* actual_ratio;
        end
        if isfield(alloc_info, 'Prop_F_used')
            Prop_Orbit = alloc_info.Prop_F_used(:) .* actual_ratio;
        end
    end

    function [Prop_Command, next_idx, active_thruster] = Apply_Active_Diagnosis( ...
            Prop_Command, next_idx, faulty_trig, diagnosis_trig, diagnosis_enable, sim_cfg)
        active_thruster = 0;
        if ~faulty_trig || diagnosis_trig || ~diagnosis_enable || ...
                ~sim_cfg.active_diagnosis_enable
            return;
        end

        pulse = min(max(sim_cfg.active_diagnosis_pulse, 0), params.T);
        command_tol = sim_cfg.active_diagnosis_command_tol;
        if pulse <= command_tol
            return;
        end

        N_thr = numel(Prop_Command);
        for offset = 0:(N_thr - 1)
            thruster_idx = mod(next_idx - 1 + offset, N_thr) + 1;
            if Prop_Command(thruster_idx) > command_tol
                continue;
            end

            available_pulse = max(0, params.T - Prop_Command(thruster_idx));
            diag_pulse = min(pulse, available_pulse);
            if diag_pulse <= command_tol
                continue;
            end

            Prop_Command(thruster_idx) = Prop_Command(thruster_idx) + diag_pulse;
            active_thruster = thruster_idx;
            next_idx = mod(thruster_idx, N_thr) + 1;
            return;
        end
    end

    function value = Get_Info_Field(info, field_name, default_value)
        if isfield(info, field_name) && ~isempty(info.(field_name))
            value = info.(field_name);
        else
            value = default_value;
        end
    end

    function thruster_on = Scheduled_Thruster_On( ...
            schedule, time_in_cycle, thruster_count)
        % 同一台推力器的所有调用组已由时序调度器排成不重叠区间。
        % 在当前控制周期内，只要时刻落入任一时间段便输出满推力。
        thruster_on = false(thruster_count, 1);
        if ~isfield(schedule, 'intervals') || ...
                numel(schedule.intervals) ~= thruster_count
            return;
        end
        for thruster_idx = 1:thruster_count
            intervals = schedule.intervals{thruster_idx};
            if isempty(intervals)
                continue;
            end
            thruster_on(thruster_idx) = any( ...
                time_in_cycle >= intervals(:, 1) - 1e-12 & ...
                time_in_cycle < intervals(:, 2) - 1e-12);
        end
    end

    function schedule_out = Remove_Faulty_Schedule(schedule_in, faults)
        schedule_out = schedule_in;
        faults = unique(round(double(faults(:)')));
        faults = faults(faults >= 1 & faults <= params.Num);
        interval_fields = {'intervals', 'force_intervals', ...
            'attitude_intervals', 'axis_intervals'};
        for field_idx = 1:numel(interval_fields)
            field_name = interval_fields{field_idx};
            if ~isfield(schedule_out, field_name) || ...
                    ~iscell(schedule_out.(field_name))
                continue;
            end
            for thruster_idx = faults
                schedule_out.(field_name){thruster_idx} = [];
            end
        end
        pulse_fields = {'total_pulse', 'force_pulse', 'attitude_pulse'};
        for field_idx = 1:numel(pulse_fields)
            field_name = pulse_fields{field_idx};
            if isfield(schedule_out, field_name) && ...
                    numel(schedule_out.(field_name)) >= params.Num
                schedule_out.(field_name)(faults) = 0;
            end
        end
    end

    function Prop_Actual = Actual_invocation(Prop_Command, true_faults, fault_trig)
        Prop_Actual = Prop_Command;
        if fault_trig && ~isempty(true_faults)
            Prop_Actual(true_faults) = 0;
        end
    end

    %% 故障诊断函数
    function [faults] = Diagnose_faults(Matrix_conf, Prop_Command, Prop_Actual, residual_threshold)
        residual = Matrix_conf * (Prop_Command - Prop_Actual);% 实际残差
        faults = [];
        if norm(residual) <= residual_threshold
            return;
        end

        errors = zeros(1, numel(Prop_Command));
        for ii = 1:numel(Prop_Command)
            expected_loss = Matrix_conf(:, ii) * Prop_Command(ii);% 预期残差
            errors(ii) = norm(residual - expected_loss);
        end
        [~, faults] = min(errors);
    end

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
        [sigma_f, sigma_c, R_b2o] = Matrix_conver(sigma);
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

    %% 矩阵转换
    function [sigma_f, sigma_c, R_b2o, R] = Matrix_conver(sigma, euler_d)
        sigma_f = dot(sigma, sigma);
        sigma_c = [0, -sigma(3), sigma(2); sigma(3), 0, -sigma(1); -sigma(2), sigma(1), 0];
        R_b2o = eye(3) + (8 * sigma_c^2 - 4 * (1 - sigma_f) * sigma_c) / (1 + sigma_f)^2;% 本体系到轨道系旋转矩阵
        if nargin > 1
            R = [1, 0, -sin(euler_d(2));
                0, cos(euler_d(1)), sin(euler_d(1))*cos(euler_d(2));
                0, -sin(euler_d(1)), cos(euler_d(1))*cos(euler_d(2))];% 欧拉角速率与角速度转换矩阵
        end
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
