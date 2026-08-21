%% 联合优化六维推力器调用策略
function [Prop_Final, info] = Thruster_joint_optimization( ...
        v_cmd1, v_cmd2, Matrix_sub, faulty_thrusters, params)
%THRUSTER_JOINT_OPTIMIZATION Directly allocate one six-dimensional duty.
%
% For healthy thrusters H, solve the bounded weighted least-squares
% problem
%
%   min_u  1/2 ||D^(-1)(Matrix_sub(:,H)*u - w_cmd)||_2^2
%          + rho/2 ||u||_2^2,
%   s.t.   l <= u <= 1.
%
% w_cmd = [F_x,F_y,F_z,M_x,M_y,M_z]' is the desired six-dimensional
% wrench. D normalises rows with different force/torque units.  rho makes
% the solution unique and mildly favours shorter total firing time.  The
% lower bound l is either zero or t_min/T after the physical minimum-pulse
% active set has been selected.  A projected-gradient solver is used so no
% optimisation toolbox is required.

    N = size(Matrix_sub, 2);
    if size(Matrix_sub, 1) ~= 6
        error('Thruster_joint_optimization:InvalidMatrix', ...
            'Matrix_sub 必须是6xN控制效率矩阵。');
    end
    command = [v_cmd1(:); v_cmd2(:)];
    if numel(command) ~= 6
        error('Thruster_joint_optimization:InvalidCommand', ...
            '轨控和姿控指令必须分别包含三个分量。');
    end
    T_ctrl = Get_Param(params, 'T', 0.4);
    if ~isfinite(T_ctrl) || T_ctrl <= 0
        error('Thruster_joint_optimization:InvalidPeriod', ...
            '控制周期T必须为正数。');
    end

    faulty = Sanitize_Faults(faulty_thrusters);
    healthy = setdiff(1:N, faulty);
    row_scale = Get_Param(params, 'alloc_row_scale', ...
        max(abs(Matrix_sub), [], 2));
    row_scale = row_scale(:);
    if isscalar(row_scale)
        row_scale = repmat(row_scale, 6, 1);
    end
    if numel(row_scale) ~= 6
        row_scale = max(abs(Matrix_sub), [], 2);
    end
    row_scale(row_scale < 1e-12) = 1;
    A = Matrix_sub(:, healthy) ./ row_scale;
    b = command ./ row_scale;
    rho = max(0, Get_Param(params, 'joint_opt_regularization', 1e-6));
    max_iter = max(1, round(Get_Param(params, 'joint_opt_max_iter', 800)));
    tolerance = max(eps, Get_Param(params, 'joint_opt_tolerance', 1e-9));
    minimum_duty = min(1, max(0, Get_Param(params, 't_min', 0.02) / T_ctrl));
    enforce_minimum = logical(Get_Param(params, ...
        'joint_opt_enforce_min_pulse', true));

    healthy_duty = zeros(numel(healthy), 1);
    if ~isempty(healthy)
        % First solve the continuous relaxation.  Entries below t_min/T
        % cannot be implemented by a real thruster and are removed before
        % the final bounded solve.
        lower = zeros(numel(healthy), 1);
        upper = ones(numel(healthy), 1);
        [healthy_duty, iterations] = Projected_Gradient(A, b, rho, ...
            lower, upper, max_iter, tolerance);
        if enforce_minimum && minimum_duty > 0
            active = healthy_duty >= minimum_duty - tolerance;
            lower = zeros(numel(healthy), 1);
            upper = zeros(numel(healthy), 1);
            lower(active) = minimum_duty;
            upper(active) = 1;
            [healthy_duty, iterations] = Projected_Gradient(A, b, rho, ...
                lower, upper, max_iter, tolerance);
        end
    else
        iterations = 0;
    end

    duty = zeros(N, 1);
    duty(healthy) = healthy_duty;
    Prop_Final = duty * T_ctrl;
    schedule = Joint_Schedule(Prop_Final);
    actual = Matrix_sub * duty;
    residual = command - actual;
    relative_residual = norm(residual ./ row_scale) / ...
        (norm(command ./ row_scale) + eps);
    feasible = relative_residual <= Get_Param(params, ...
        'joint_opt_reconfig_tolerance', 5e-3);

    info = struct();
    info.mode = 'joint_optimization';
    info.reuse_strategy = 'joint_optimization';
    info.allocation_strategy = 'joint_optimization';
    info.solver = 'bounded_projected_gradient_weighted_least_squares';
    info.optimization_problem = 'min ||D^{-1}(B*u-w)||^2 + rho||u||^2';
    info.fault_policy = 'remove_faulty_columns_before_optimization';
    info.strict_paired_mode = false;
    info.uses_impulse_margin_fallback = false;
    info.nominal_reuse_count = zeros(N, 1);
    info.structurally_reconfigurable = feasible;
    info.strictly_reconfigurable = feasible;
    info.Prop_F = zeros(N, 1);
    info.Prop_T = zeros(N, 1);
    info.Prop_F_used = zeros(N, 1);
    info.Prop_T_used = zeros(N, 1);
    info.Prop_F_axis = zeros(N, 3);
    info.Prop_T_axis = zeros(N, 3);
    info.Prop_F_axis_raw = zeros(N, 3);
    info.Prop_T_axis_raw = zeros(N, 3);
    info.Prop_6D = Prop_Final;
    info.Prop_Final = Prop_Final;
    info.schedule = schedule;
    info.scheduler = struct('peak_orbit_torque', NaN, ...
        'peak_attitude_force', NaN, 'peak_torque_limit', NaN, ...
        'peak_force_limit', NaN, 'peak_constraint_met', NaN, ...
        'suppressed_force_axes', [], 'force_axis_pulse', zeros(N, 3), ...
        'attitude_axis_pulse', zeros(N, 3), 'status', 'joint_optimization');
    info.scheduler_force_axis_suppressed = [];
    info.axis_strategy = Joint_Axis_Strategy();
    info.impulse_margin_unavailable_axes = [];
    info.force_scale = NaN;
    info.attitude_scale = NaN;
    info.force_scale_reason = 'not_separated_in_joint_optimization';
    info.attitude_scale_reason = 'not_separated_in_joint_optimization';
    info.orbit_wrench_raw = NaN(6, 1);
    info.attitude_wrench_raw = NaN(6, 1);
    info.orbit_wrench = NaN(6, 1);
    info.attitude_wrench = NaN(6, 1);
    info.orbit_cross_torque = NaN;
    info.attitude_cross_force = NaN;
    info.period_average_decoupled = false;
    info.duty = duty;
    info.command = command;
    info.actual = actual;
    info.residual = residual;
    info.relative_residual = relative_residual;
    info.iterations = iterations;
    info.healthy_thrusters = healthy;
    info.faulty_thrusters = faulty;
    info.reuse_scale = ones(N, 1);
    info.orbit_keep_ratio = NaN;
    info.att_keep_ratio = NaN;
    info.max_raw_pulse = max(Prop_Final);
    info.attitude_window = NaN;
    info.position_window = NaN;

    function [u, iter] = Projected_Gradient(A_in, b_in, rho_in, ...
            lower, upper, iter_max, tol)
        column_count = size(A_in, 2);
        if column_count == 0
            u = zeros(0, 1);
            iter = 0;
            return;
        end
        % L bounds the gradient Lipschitz constant.  The fixed step 1/L
        % makes every iteration a descent step before box projection.
        L = norm(A_in, 2)^2 + rho_in;
        step = 1 / max(L, 1e-12);
        u = min(upper, max(lower, A_in \ b_in));
        for iter = 1:iter_max
            gradient = A_in' * (A_in * u - b_in) + rho_in * u;
            next = min(upper, max(lower, u - step * gradient));
            if norm(next - u) <= tol * max(1, norm(u))
                u = next;
                return;
            end
            u = next;
        end
    end

    function schedule_out = Joint_Schedule(pulse)
        intervals = cell(N, 1);
        axis_intervals = cell(N, 1);
        for index = 1:N
            if pulse(index) > 1e-12
                intervals{index} = [0, pulse(index)];
                % Axis 0 denotes one direct six-dimensional task.  Unlike
                % the two time-division strategies it has no unique pure
                % Fx/Fy/Fz/Mx/My/Mz ownership and is drawn in neutral gray.
                axis_intervals{index} = [0, pulse(index), 0];
            end
        end
        schedule_out = struct('enabled', true, 'intervals', {intervals}, ...
            'force_intervals', {cell(N, 1)}, ...
            'attitude_intervals', {cell(N, 1)}, ...
            'total_pulse', pulse, 'force_pulse', zeros(N, 1), ...
            'attitude_pulse', zeros(N, 1), 'job_starts', [], ...
            'axis_intervals', {axis_intervals});
    end

    function strategies = Joint_Axis_Strategy()
        strategies = cell(2, 3);
        for row = 1:2
            for column = 1:3
                strategies{row, column} = struct('status', ...
                    'joint_optimization', 'reconfigured', ~isempty(faulty));
            end
        end
    end

    function faults = Sanitize_Faults(faults)
        faults = unique(round(double(faults(:)')));
        faults = faults(faults >= 1 & faults <= N);
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
