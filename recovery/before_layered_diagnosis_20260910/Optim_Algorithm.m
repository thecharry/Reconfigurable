%{
%% 旧版推力器布局优化设计（保留供对照，不参与运行）
function result = Optim_Algorithm(params, opt_cfg)
%OPTIM_ALGORITHM App 可调用的推力器布局优化接口。
%
% result = Optim_Algorithm(params, opt_cfg)
%
% 输入：
%   params  - Get_params() 返回的参数结构体
%   opt_cfg - 优化设置结构体，可包含：
%             x_face, y_max, z_max, position_a_range,
%             position_b_range, alpha_range, beta_range, min_install_distance,
%             min_install_angle_deg, population_size,
%             max_generations, function_tolerance, use_parallel,
%             display, save_result, output_dir, progress_callback,
%             optimization_target, enforce_invocation_balance
%
% 输出：
%   result.B_opt / result.r_opt / result.x_opt / result.fval
%
% 说明：
%   如果当前 MATLAB 环境没有 ga/optimoptions，或优化过程失败，
%   函数会自动读取工程目录下最新的 Optim_data_*.mat 作为可用结果，
%   保证 App 第一版可以直接演示和使用。

    if nargin < 1 || isempty(params)
        params = Get_params();
    end
    if nargin < 2 || isempty(opt_cfg)
        opt_cfg = struct();
    end

    opt_cfg = Fill_Opt_Config(opt_cfg);
    if ~strcmp(char(string(opt_cfg.optimization_target)), '调用可重构优先')
        error('RCDesigner:UnsupportedOptimizationTarget', ...
            '当前优化器仅支持“调用可重构优先”。');
    end
    if ~isfield(params, 'Num') || params.Num < 4 || ...
            params.Num ~= round(params.Num) || mod(params.Num, 4) ~= 0
        error('RCDesigner:InvalidThrusterCount', ...
            '推力器数量必须为4的整数倍（4n），例如4、8、12或16。');
    end
    if opt_cfg.x_face <= 0 || opt_cfg.y_max < 0 || opt_cfg.z_max < 0 || ...
            opt_cfg.min_install_distance < 0 || ...
            opt_cfg.min_install_angle_deg < 0 || ...
            opt_cfg.min_install_angle_deg > 180
        error('RCDesigner:InvalidLayoutConstraint', ...
            'X安装面应大于0，安装边界和最小间距应不小于0，最小夹角应在0～180度之间。');
    end
    rangeNames = {'position_a_range', 'position_b_range', 'alpha_range', 'beta_range'};
    for rangeIndex = 1:numel(rangeNames)
        rangeValue = opt_cfg.(rangeNames{rangeIndex});
        if ~isnumeric(rangeValue) || numel(rangeValue) ~= 2 || ...
                any(~isfinite(rangeValue)) || rangeValue(1) > rangeValue(2)
            error('RCDesigner:InvalidOptimizationRange', ...
                '优化变量范围 %s 必须是由小到大的两个数值。', rangeNames{rangeIndex});
        end
    end
    root_dir = fileparts(mfilename('fullpath'));
    if ~isfield(opt_cfg, 'output_dir') || isempty(opt_cfg.output_dir)
        opt_cfg.output_dir = root_dir;
    end
    params.optim_x_face = opt_cfg.x_face;

    Y_max = opt_cfg.y_max;       % 安装面尺寸约束
    Z_max = opt_cfg.z_max;
    positionARange = [max(opt_cfg.position_a_range(1), -Y_max), ...
        min(opt_cfg.position_a_range(2), Y_max)];
    positionBRange = [max(opt_cfg.position_b_range(1), -Z_max), ...
        min(opt_cfg.position_b_range(2), Z_max)];
    if positionARange(1) > positionARange(2) || ...
            positionBRange(1) > positionBRange(2)
        error('RCDesigner:EmptyOptimizationRange', ...
            '位置参数范围与安装边界 L/W 没有交集。');
    end
    module_count = params.Num / 4;
    pair_count = params.Num / 2;
    lb = repmat([positionARange(1), positionBRange(1), ...
        opt_cfg.alpha_range(1), opt_cfg.beta_range(1)], 1, module_count);
    ub = repmat([positionARange(2), positionBRange(2), ...
        opt_cfg.alpha_range(2), opt_cfg.beta_range(2)], 1, module_count);
    nvars = 4 * module_count;
    initial_population = Build_Invocation_Seed(params, lb, ub);
    evaluationCount = 0;
    progressClock = tic;
    lastProgressNotification = -inf;

    can_run_ga = exist('ga', 'file') == 2 && exist('optimoptions', 'file') == 2;
    if can_run_ga
        try
            options = optimoptions('ga', ...
                'Display', opt_cfg.display, ...
                'PopulationSize', opt_cfg.population_size, ...
                'MaxGenerations', opt_cfg.max_generations, ...
                'FunctionTolerance', opt_cfg.function_tolerance, ...
                'UseParallel', opt_cfg.use_parallel, ...
                'InitialPopulationMatrix', initial_population, ...
                'OutputFcn', @(options, state, flag) ...
                    GA_Progress_Output(options, state, flag, opt_cfg));

            if opt_cfg.use_parallel
                objectiveFunction = @(x) Optimal_config(x, params);
            else
                objectiveFunction = @Objective_With_Progress;
            end

            tic;
            [x_opt, fval] = ga(objectiveFunction, nvars, ...
                [], [], [], [], lb, ub, ...
                @(x) Physical_constraints(x, params, opt_cfg), options);
            elapsed_time = toc;

            [B_opt, r_opt] = Thruster_reconfig(x_opt, params);
            result = struct();
            result.B_opt = B_opt;
            result.r_opt = r_opt;
            result.fval = fval;
            result.x_opt = x_opt;
            result.elapsed_time = elapsed_time;
            result.source = "ga";
            result.message = "遗传算法优化完成";
            result.thruster_count = params.Num;
            result.base_thruster_count = module_count;
            result.pair_count = pair_count;
            result.parameterization = "strict_centrosymmetric_four_thruster_module";
            result.constraints = opt_cfg;

            if opt_cfg.save_result
                timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
                output_file = fullfile(opt_cfg.output_dir, sprintf('Optim_data_%s_app.mat', timestamp));
                save(output_file, 'B_opt', 'r_opt', 'fval', 'x_opt');
                result.output_file = output_file;
            else
                result.output_file = "";
            end
            return;
        catch ME
            warning('RCDesigner:OptimizationFallback', ...
                '优化计算失败，将载入已有优化结果。原因：%s', ME.message);
        end
    end

    result = Load_Fallback_Optim_Result(opt_cfg.output_dir, params);

    function objectiveValue = Objective_With_Progress(x)
        evaluationCount = evaluationCount + 1;
        currentTime = toc(progressClock);
        if evaluationCount == 1 || currentTime - lastProgressNotification >= 0.25
            info = struct('Stage', 'evaluation', ...
                'Generation', NaN, 'Progress', NaN, ...
                'EvaluationCount', evaluationCount);
            Notify_Progress(opt_cfg, info);
            lastProgressNotification = currentTime;
        end
        objectiveValue = Optimal_config(x, params);
    end
end

function opt_cfg = Fill_Opt_Config(opt_cfg)
    defaults = struct();
    defaults.y_max = 0.6;
    defaults.z_max = 0.6;
    defaults.x_face = 2;
    defaults.position_a_range = [0, 0.6];
    defaults.position_b_range = [-0.6, 0.6];
    defaults.alpha_range = [0, 2*pi];
    defaults.beta_range = [0, pi/2];
    defaults.population_size = 10;
    defaults.max_generations = 1;
    defaults.function_tolerance = 1e-4;
    defaults.use_parallel = false;
    defaults.display = 'off';
    defaults.save_result = true;
    defaults.min_install_distance = 0;
    defaults.min_install_angle_deg = 0;
    defaults.progress_callback = [];
    defaults.optimization_target = '调用可重构优先';
    defaults.enforce_invocation_balance = true;

    names = fieldnames(defaults);
    for index = 1:numel(names)
        name = names{index};
        if ~isfield(opt_cfg, name) || isempty(opt_cfg.(name))
            opt_cfg.(name) = defaults.(name);
        end
    end
end

function [state, options, optchanged] = ...
        GA_Progress_Output(options, state, flag, opt_cfg)
    optchanged = false;
    generation = 0;
    if isfield(state, 'Generation') && ~isempty(state.Generation)
        generation = double(state.Generation);
    end
    if strcmpi(flag, 'done')
        progress = 1;
    else
        progress = generation / max(1, opt_cfg.max_generations);
    end
    info = struct('Stage', char(flag), ...
        'Generation', generation, ...
        'Progress', max(0, min(1, progress)), ...
        'EvaluationCount', 0);
    Notify_Progress(opt_cfg, info);
end

function Notify_Progress(opt_cfg, info)
    if ~isfield(opt_cfg, 'progress_callback') || ...
            ~isa(opt_cfg.progress_callback, 'function_handle')
        return;
    end
    try
        opt_cfg.progress_callback(info);
    catch
        % 进度显示不应中断优化计算。
    end
end

function result = Load_Fallback_Optim_Result(output_dir, params)
    files = dir(fullfile(output_dir, 'Optim_data_*.mat'));
    compatible = false(size(files));
    for index = 1:numel(files)
        try
            fileInfo = whos('-file', fullfile(files(index).folder, files(index).name), 'B_opt', 'r_opt');
            bInfo = fileInfo(strcmp({fileInfo.name}, 'B_opt'));
            rInfo = fileInfo(strcmp({fileInfo.name}, 'r_opt'));
            compatible(index) = ~isempty(bInfo) && ~isempty(rInfo) && ...
                bInfo.size(2) == params.Num && rInfo.size(2) == params.Num;
        catch
            compatible(index) = false;
        end
    end
    files = files(compatible);
    if isempty(files)
        result = struct();
        if isfield(params, 'B_all') && isfield(params, 'r_all') && ...
                size(params.B_all, 2) == params.Num && size(params.r_all, 2) == params.Num
            result.B_opt = params.B_all;
            result.r_opt = params.r_all;
        else
            error('RCDesigner:NoCompatibleOptimizationResult', ...
                '遗传算法未能完成，且未找到包含%d台推力器的已有优化结果。', params.Num);
        end
        result.fval = NaN;
        result.x_opt = [];
        result.elapsed_time = 0;
        result.thruster_count = params.Num;
        result.base_thruster_count = params.Num / 4;
        result.pair_count = params.Num / 2;
        result.parameterization = "strict_centrosymmetric_four_thruster_module";
        result.source = "default";
        result.message = "未找到优化结果文件，暂使用默认原布局";
        result.output_file = "";
        return;
    end

    [~, order] = sort([files.datenum], 'descend');
    file_path = fullfile(files(order(1)).folder, files(order(1)).name);
    data = load(file_path);

    result = struct();
    if isfield(data, 'B_opt')
        result.B_opt = data.B_opt;
    else
        result.B_opt = params.B_all;
    end
    if isfield(data, 'r_opt')
        result.r_opt = data.r_opt;
    else
        result.r_opt = params.r_all;
    end
    result.fval = Get_Field(data, 'fval', NaN);
    result.x_opt = Get_Field(data, 'x_opt', []);
    result.elapsed_time = 0;
    result.thruster_count = params.Num;
    result.base_thruster_count = params.Num / 4;
    result.pair_count = params.Num / 2;
    result.parameterization = "strict_centrosymmetric_four_thruster_module";
    result.source = "fallback_file";
    result.message = "已载入已有优化结果";
    result.output_file = file_path;
end

function value = Get_Field(data, field_name, default_value)
    if isfield(data, field_name)
        value = data.(field_name);
    else
        value = default_value;
    end
end

%% 根据严格中心对称四推力器模块生成完整推力器布局
function [B_all, r] = Thruster_reconfig(x, params)
    if params.Num < 4 || params.Num ~= round(params.Num) || mod(params.Num, 4) ~= 0
        error('RCDesigner:InvalidThrusterCount', ...
            '推力器数量必须为4的整数倍（4n）。');
    end
    module_count = params.Num / 4;
    if numel(x) ~= 4 * module_count
        error('RCDesigner:InvalidOptimizationVector', ...
            '优化变量数量应为每个四推力器模块 4 个变量。');
    end

    r = zeros(3, params.Num);
    d = zeros(3, params.Num);
    x_face = 2;
    if isfield(params, 'optim_x_face') && isnumeric(params.optim_x_face) && ...
            isscalar(params.optim_x_face) && isfinite(params.optim_x_face) && ...
            params.optim_x_face > 0
        x_face = params.optim_x_face;
    end

    % 每个模块由 [y, z, alpha, beta] 唯一确定：+X 面两台推力器
    % 以安装面中心对称，-X 面再对这两台镜像。该硬约束使四台推力器
    % 具有明确的成对抵消关系，方便实际安装，也利于构造纯控制组。
    for module = 1:module_count
        variableIndex = 4 * (module - 1);
        thrusterIndex = 4 * (module - 1);
        y0 = x(variableIndex + 1);
        z0 = x(variableIndex + 2);
        alpha0 = x(variableIndex + 3);
        beta0 = x(variableIndex + 4);
        d1 = [-cos(beta0); sin(beta0) * cos(alpha0); sin(beta0) * sin(alpha0)];
        module_idx = thrusterIndex + (1:4);
        r(:, module_idx) = [x_face, x_face, -x_face, -x_face; ...
            y0, -y0, y0, -y0; z0, -z0, z0, -z0];
        d(:, module_idx) = [d1, [d1(1); -d1(2); -d1(3)], ...
            [-d1(1); d1(2); d1(3)], -d1];
    end

    B_all = zeros(6, params.Num);
    for index = 1:params.Num
        B_all(1:3, index) = d(:, index);
        B_all(4:6, index) = cross(r(:, index), d(:, index));
    end
end

% 从当前布局提取严格对称模块种子；它不参与非线性约束。
function seed = Build_Invocation_Seed(params, lb, ub)
    module_count = params.Num / 4;
    seed = zeros(1, 4 * module_count);
    can_use_layout = isfield(params, 'B_all') && isfield(params, 'r_all') && ...
        size(params.B_all, 2) == params.Num && size(params.r_all, 2) == params.Num;
    template = [0.6, 0, pi, pi / 3; 0.6, 0.6, 3 * pi / 2, pi / 4; 0.6, -0.6, pi / 2, pi / 4];
    for module = 1:module_count
        variableIndex = 4 * (module - 1);
        value = template(mod(module - 1, size(template, 1)) + 1, :);
        if can_use_layout
            positiveFace = find(params.r_all(1, :) > 0 & params.r_all(2, :) >= 0);
            if ~isempty(positiveFace)
                sourceIndex = positiveFace(mod(module - 1, numel(positiveFace)) + 1);
                direction = params.B_all(1:3, sourceIndex);
                directionNorm = norm(direction);
                if directionNorm > 1e-12
                    direction = direction / directionNorm;
                    value = [abs(params.r_all(2, sourceIndex)), params.r_all(3, sourceIndex), ...
                        mod(atan2(direction(3), direction(2)), 2 * pi), ...
                        acos(min(1, max(-1, -direction(1))))];
                end
            end
        end
        seed(variableIndex + (1:4)) = min(ub(variableIndex + (1:4)), ...
            max(lb(variableIndex + (1:4)), value));
    end
end

%% 最优布局参数目标函数
function J = Optimal_config(x, params)
    [B_all, ~] = Thruster_reconfig(x, params);
    % 优化只需12个带符号单轴样本来判定调用结构和有效轴向能力，
    % 不重复计算用于离线展示的36个力/力矩联合跟踪样本。
    [Z_nominal, Jc1_nominal] = Reconfig_eval(params, B_all, 0, false);
    F_ref = params.F_max;
    if isfield(params, 'optim_x_face')
        L_ref = params.optim_x_face;
    else
        L_ref = 2;
    end
    T_ref = params.F_max * L_ref;

    % 此检查与非线性约束中的六维配平方程一致；在目标函数中保留它，
    % 可尽早淘汰不可行候选，避免再为其建立12套故障备用调用库。
    nominal_missing = sum(~Z_nominal.Invocation.strict_axis(:));
    if nominal_missing > 0
        nominal_force = Jc1_nominal(1, 1) / max(F_ref, 1e-12);
        nominal_torque = Jc1_nominal(1, 2) / max(T_ref, 1e-12);
        J = 1e6 * nominal_missing - 0.05 * min(nominal_force, nominal_torque);
        if ~isfinite(J)
            J = 1e6;
        end
        return;
    end

    [Z, Jc1, ~, Jc6] = Reconfig_eval(params, B_all, 1, false);

    force_cap  = Jc1(:, 1) / max(F_ref, 1e-12);
    torque_cap = Jc1(:, 2) / max(T_ref, 1e-12);

    % Jc 是“所有健康推力器可任意组合”时的硬件上限，继续保留它
    % 作为次级性能指标，不能再单独用它判断调用策略能否实际执行。
    balanced_cap = min(force_cap, torque_cap);
    worst_cap = min(balanced_cap);

    % 先检查当前调用策略的结构覆盖。标况下12个带符号轴均须有严格
    % 瞬时纯控制组；单故障下允许使用冲量裕度备用，但每个轴都必须能
    % 在标准测试指令下完成调用。只有前两层都满足，才比较控制能力。
    invocation = Z.Invocation;
    nominal_missing = sum(~invocation(1).strict_axis(:));
    if numel(invocation) > 1
        fault_callable = cat(3, invocation(2:end).axis_callable);
        fault_missing = numel(fault_callable) - nnz(fault_callable);
        fault_strict_ratio = mean([invocation(2:end).strictly_reconfigurable]);
    else
        fault_missing = 0;
        fault_strict_ratio = 1;
    end
    policy_force = [invocation.force_capacity] / max(F_ref, 1e-12);
    policy_torque = [invocation.torque_capacity] / max(T_ref, 1e-12);
    effective_cap = min([policy_force, policy_torque]);

    % 遗传算法执行最小化。大权重保证优化顺序为：标况结构可行、
    % 单故障可调用、有效能力、硬件理论能力和严格备用比例。
    J = 1e6 * nominal_missing + 1e3 * fault_missing - ...
        effective_cap - 0.05 * worst_cap - 0.01 * fault_strict_ratio;

    % 评价计算失败时，不能让 GA 把零值当成正常候选
    if any(~isfinite([J; Jc1(:); Jc6(:); policy_force(:); policy_torque(:)]))
        J = 1e6;
    end
end

%% 检查安装边界和标况六维纯控制组约束
function [c, ceq] = Physical_constraints(x, params, opt_cfg)
    [B_all, r] = Thruster_reconfig(x, params);
    minDistance = opt_cfg.min_install_distance;
    minAngleDeg = opt_cfg.min_install_angle_deg;
    ceq = [];
    pairCount = params.Num * (params.Num - 1) / 2;
    if minDistance > 0
        distanceConstraint = zeros(pairCount, 1);
    else
        distanceConstraint = zeros(0, 1);
    end
    if minAngleDeg > 0
        directions = B_all(1:3, :);
        directions = directions ./ max(vecnorm(directions, 2, 1), 1e-12);
        angleConstraint = zeros(pairCount, 1);
        maximumCosine = cosd(minAngleDeg);
    else
        angleConstraint = zeros(0, 1);
    end

    pairIndex = 0;
    for i = 1:params.Num
        for j = (i + 1):params.Num
            pairIndex = pairIndex + 1;
            if minDistance > 0
                distanceConstraint(pairIndex) = minDistance - norm(r(:, i) - r(:, j));
            end
            if minAngleDeg > 0
                directionCosine = dot(directions(:, i), directions(:, j));
                angleConstraint(pairIndex) = directionCosine - maximumCosine;
            end
        end
    end

    balanceConstraint = zeros(0, 1);
    if opt_cfg.enforce_invocation_balance
        try
            % 对每个正负轴，调用策略都必须在全部健康推力器中找到 S 和 u：
            %     A_S * u = s * e_k,  u_i > 0.
            % A_S 是组内推力器的六维效率列，e_k 是目标力/力矩轴；
            % strict_axis 为真表示方程成立且同步喷气的每一阶段仍为纯控制。
            Z = Reconfig_eval(params, B_all, 0, false);
            balanceConstraint = double(~Z.Invocation(1).strict_axis(:)) - 0.5;
        catch
            % 求解失败的候选不应被遗传算法误判为可行。
            balanceConstraint = ones(12, 1);
        end
    end
    c = [distanceConstraint; angleConstraint; balanceConstraint];
end 
%}

%% 组选相容性与安装几何分解的推力器布局优化设计
% 搜索第一套通过全部单故障纯组和现有调用器验证的严格对称布局。
function result = Optim_Algorithm(params, opt_cfg)
% x=[y1,z1,alpha1,beta1,...,y3,z3,alpha3,beta3]，位置单位m，角度单位rad。
% B=[d;r×d]，实际控制矩阵为F_max*B；A=[d;(r×d)/x_face]为归一化矩阵。
% 只要求健康偶数组满足(A*b)(j)=0(j~=k)、s*(A*b)(k)>=min_axis_capacity。
% 本版不最大化能力；第一个完整通过独立复核的候选即作为结果返回。
    if nargin < 1 || isempty(params)
        params = Get_params();
    end
    if nargin < 2
        opt_cfg = struct();
    end
    [cfg, lower, upper] = Search_Config(params, opt_cfg);
    masks = false(sum(arrayfun(@(n)nchoosek(12,n),2:2:cfg.max_group_size)),12);
    index = 0;
    for count = 2:2:cfg.max_group_size
        combinations = nchoosek(1:12,count);
        for row = 1:size(combinations,1)
            index = index+1;
            masks(index,combinations(row,:)) = true;
        end
    end
    masks = double(masks');
    usable = [true(1,size(masks,2));~logical(masks)];
    timer = tic;
    result = struct('schema_version',7,'status',"not_found",'feasible',false,'global_certified',false, ...
        'search_goal',"first_feasible",'source',"full12_group_feasibility",'B_opt',[],'r_opt',[],'x_opt',[], ...
        'fval',NaN,'output_file',"",'diagnostic_file',"",'solver_log_file',"",'elapsed_time',0, ...
        'starts_attempted',0,'starts_completed',0,'group_count',size(masks,2),'best_coverage',0, ...
        'best_balance_residual',inf,'diagnostic_x',[],'best_coverage_x',[],'resume_candidates',[], ...
        'initial_points',[],'fixed_group_calls',0,'function_evaluations',0,'allocation_verified',false, ...
        'parameterization',"three_independent_centrosymmetric_modules",'thruster_count',12,'base_thruster_count',3,'pair_count',6, ...
        'constraints',cfg,'stage_times',struct('local',0,'joint',0,'polish',0,'verification',0), ...
        'error',struct('identifier',"",'message',"",'report',""),'solver_evidence',struct('status',"not_run",'has_solution',false,'raw_log',""), ...
        'environment',struct('matlab',version,'architecture',computer));
    result.trace = struct('stage',{},'start',{},'round',{},'elapsed',{},'coverage',{},'residual_norm',{},'exitflag',{});
    result.validation_rejections = struct('stage',{},'x',{},'detail',{});
    result.missing_frequency = zeros(13,12);
    best = struct();
    coverage_candidate = struct();
    residual_candidate = struct();
    start_index = 0;
    stage = "initialization";
    local_limit = min(cfg.local_max_time,cfg.max_time);
    if cfg.solver == "native"
        local_limit = cfg.max_time;
    elseif cfg.solver == "scip"
        local_limit = 0;
    end
    try
        [result.initial_points,result.history_sources] = Search_Seeds(params,cfg,lower,upper,masks,usable);
        stage = "local";
        local_timer = tic;
        for start_index = 1:size(result.initial_points,1)
            if toc(timer) >= local_limit
                break;
            end
            result.starts_attempted = result.starts_attempted+1;
            if ismember(cfg.solver,["fair","patterns","decomposed"])
                Observe(result.initial_points(start_index,:),"seed_screen",0,NaN);
            else
                Search_From(result.initial_points(start_index,:),stage,local_limit);
            end
            if toc(timer) < local_limit || ~isempty(fieldnames(best))
                result.starts_completed = result.starts_completed+1;
            end
            if ~isempty(fieldnames(best))
                break;
            end
        end
        result.stage_times.local = max(0,toc(local_timer)-result.stage_times.verification);
        if isempty(fieldnames(best)) && cfg.solver ~= "native" && toc(timer) < cfg.max_time
            stage = "joint_minlp";
            joint_timer = tic;
            seeds = result.initial_points;
            if ~isempty(fieldnames(coverage_candidate))
                seeds = [coverage_candidate.x;residual_candidate.x;seeds];
            end
            result.solver_evidence = Run_Joint_Solver(cfg,lower,upper,cfg.max_time-toc(timer),seeds);
            result.stage_times.joint = toc(joint_timer);
            if isfield(result.solver_evidence,'candidates')
                for candidate_index = 1:size(result.solver_evidence.candidates,1)
                    Observe(result.solver_evidence.candidates(candidate_index,:),"fixed_direction",0,NaN);
                    if ~isempty(fieldnames(best))
                        break;
                    end
                end
            end
            if result.solver_evidence.status == "error"
                result.status = "solver_error";
                result.error = result.solver_evidence.error;
            elseif result.solver_evidence.has_solution
                % 模式候选未通过时继续队列；旧方法仍使用固定组精修。
                stage = "joint_polish";
                polish_timer = tic;
                verification_before = result.stage_times.verification;
                retry_time = 0;
                if isfield(result.solver_evidence,'free_mask') || ...
                        (cfg.solver == "decomposed" && isfield(result.solver_evidence,'search_state'))
                    % 不用全域精修改变该类中固定为0的变量。
                    while result.solver_evidence.has_solution
                        Observe(result.solver_evidence.x(:)',stage,0,NaN);
                        if ~isempty(fieldnames(best)) || toc(timer) >= cfg.max_time
                            break;
                        end
                        retry_cfg = cfg;
                        retry_cfg.search_state = result.solver_evidence.search_state;
                        retry_timer = tic;
                        result.solver_evidence = Run_Joint_Solver(retry_cfg,lower,upper,cfg.max_time-toc(timer),seeds);
                        retry_time = retry_time+toc(retry_timer);
                    end
                else
                    Search_From(result.solver_evidence.x(:)',stage,cfg.max_time);
                end
                result.stage_times.joint = result.stage_times.joint+retry_time;
                result.stage_times.polish = max(0,toc(polish_timer)-retry_time-(result.stage_times.verification-verification_before));
                if isempty(fieldnames(best))
                    result.status = "candidate_rejected";
                    if result.solver_evidence.status == "timelimit"
                        result.status = "time_limit";
                    elseif result.solver_evidence.status == "infeasible"
                        result.status = "infeasible_for_capacity";
                    elseif result.solver_evidence.status == "numerically_infeasible"
                        result.status = "numerically_infeasible";
                    elseif result.solver_evidence.status == "error"
                        result.status = "solver_error";
                        result.error = result.solver_evidence.error;
                    end
                end
            elseif result.solver_evidence.status == "infeasible"
                result.status = "infeasible_for_capacity";
            elseif result.solver_evidence.status == "numerically_infeasible"
                result.status = "numerically_infeasible";
            elseif result.solver_evidence.status == "timelimit"
                result.status = "time_limit";
            else
                result.status = "solver_stopped";
            end
        elseif isempty(fieldnames(best)) && toc(timer) >= local_limit
            result.status = "time_limit";
        end
    catch ME
        result.status = "error";
        result.error = struct('identifier',string(ME.identifier),'message',string(ME.message), ...
            'report',string(getReport(ME,'extended','hyperlinks','off')));
    end
    result.last_stage = stage;
    result.elapsed_time = toc(timer);
    result.best_coverage_candidate = coverage_candidate;
    result.best_residual_candidate = residual_candidate;
    result.diagnostic_missing = struct([]);
    if ~isempty(fieldnames(coverage_candidate))
        result.diagnostic_missing = Missing_Requirements(coverage_candidate,cfg,masks);
        result.best_coverage_x = coverage_candidate.x;
        result.resume_candidates = unique([coverage_candidate.x;residual_candidate.x],'rows','stable');
    end
    if ~isempty(fieldnames(best))
        result.status = "feasible";
        result.feasible = true;
        result.allocation_verified = true;
        result.B_opt = best.B;
        result.r_opt = best.r;
        result.x_opt = best.x;
        result.topology_map = best.groups;
        result.axis_capacity_normalized = best.capacity;
        result.invocation_axis_capacity_normalized = best.invocation_capacity;
        reference = [params.F_max*ones(3,1);params.F_max*cfg.x_face*ones(3,1)];
        result.nominal_axis_capacity = reshape(best.capacity(1,:),2,6)'.*reference;
        result.capacity_definition = "max_single_even_pure_group";
        result.fault_min_normalized_capacity = best.worst_capacity;
        result.strict_fault_complete = true;
        result.fault_strict_complete = true;
        result.fault_strict_coverage = 1;
        result.best_coverage = 156;
        result.certificate = best;
        result.message = "已找到首个通过156项严格组及现有调用器复核的可行布局；未进行控制能力最大化。";
    else
        if result.status == "not_found" && ~isempty(result.validation_rejections)
            result.status = "candidate_rejected";
        end
        if result.status == "infeasible_for_capacity" && isfield(result.solver_evidence,'infeasibility_proof')
            result.status = "proven_infeasible_for_bounds";
        end
        switch result.status
            case "proven_infeasible_for_bounds"
                result.message = "已由恒等式证明当前输入边界内Fz恒为零，无法满足正负Fz能力门槛；证明范围仅限这些输入边界。";
            case "infeasible_for_capacity"
                result.message = sprintf("联合求解器在数值容差下判定当前边界及最低能力%.4g无可行解；不代表其他门槛或构型无解。",cfg.min_axis_capacity);
            case "numerically_infeasible"
                result.message = "组选线性松弛在数值容差下未找到可行拓扑；没有取得整个原问题的精确无解证明。";
            case {"error","solver_error"}
                result.message = "搜索异常 ["+result.error.identifier+"]："+result.error.message;
            case "candidate_rejected"
                result.message = "已得到候选，但安装边界、严格纯组或实际调用器复核未全部通过，未生成新布局。";
            case "time_limit"
                result.message = sprintf("搜索达到时限，最高覆盖%d/156项，未找到全部约束可行布局；不代表无解。",result.best_coverage);
            otherwise
                result.message = sprintf("本轮搜索结束，尝试%d个初值，最高覆盖%d/156项；未找到布局，不代表无解。",result.starts_attempted,result.best_coverage);
        end
    end
    result = Save_Search_Report(result,params);
    fprintf('[%s] %s\n',result.status,result.message);
    if strlength(result.diagnostic_file) > 0
        fprintf('诊断报告：%s\n搜索记录：%s\n',result.diagnostic_file,result.output_file);
    end

    % 从一个初值交替执行“选组”和短程固定组配平，避免在数值求导中反复换组。
    function Search_From(x,phase,deadline)
        diagnostic = Observe(x,phase,0,NaN);
        for round_index = 1:ceil(cfg.max_iterations/cfg.inner_iterations)
            if ~isempty(fieldnames(best)) || toc(timer) >= deadline
                return;
            end
            topology = [diagnostic.selected(:),repelem((1:12)',13)];
            previous_x = x;
            previous_groups = diagnostic.selected;
            iterations = min(cfg.inner_iterations,cfg.max_iterations-(round_index-1)*cfg.inner_iterations);
            options = optimoptions('lsqnonlin','Display','off','MaxIterations',iterations, ...
                'MaxFunctionEvaluations',20*(iterations+1),'FunctionTolerance',1e-13, ...
                'StepTolerance',1e-12,'OptimalityTolerance',1e-11,'OutputFcn',@(a,b,c)Progress(a,b,c,deadline,phase));
            [x,~,~,exitflag,output] = lsqnonlin(@(v)Fixed_Group_Residual(v,topology,masks,cfg),x,lower,upper,options);
            result.fixed_group_calls = result.fixed_group_calls+1;
            result.function_evaluations = result.function_evaluations+output.funcCount;
            diagnostic = Observe(x,phase,round_index,exitflag);
            if norm(x-previous_x,inf) < 1e-10 && isequal(previous_groups,diagnostic.selected)
                return;
            end
        end
    end

    % 记录每次外层候选的覆盖率与残差；只有完整复核通过才能中止并返回布局。
    function diagnostic = Observe(x,phase,round_index,exitflag)
        diagnostic = Evaluate_Layout(x,cfg,masks,usable,lower,upper);
        result.trace(end+1) = struct('stage',phase,'start',start_index,'round',round_index,'elapsed',toc(timer), ...
            'coverage',diagnostic.coverage,'residual_norm',diagnostic.residual_norm,'exitflag',exitflag);
        result.missing_frequency = result.missing_frequency+~diagnostic.requirement_feasible;
        if isempty(fieldnames(coverage_candidate)) || diagnostic.coverage > coverage_candidate.coverage || ...
                (diagnostic.coverage == coverage_candidate.coverage && diagnostic.residual_norm < coverage_candidate.residual_norm)
            coverage_candidate = diagnostic;
            result.best_coverage = diagnostic.coverage;
        end
        if diagnostic.residual_norm < result.best_balance_residual
            residual_candidate = diagnostic;
            result.best_balance_residual = diagnostic.residual_norm;
            result.diagnostic_x = diagnostic.x;
        end
        if diagnostic.coverage == 156
            verification_timer = tic;
            candidate = Certify_Layout(x,cfg,masks,usable,lower,upper);
            if candidate.feasible
                [valid,capacity,detail] = Verify_Invocation(candidate,params,cfg);
                if valid
                    candidate.invocation_capacity = capacity;
                    best = candidate;
                else
                    result.validation_rejections(end+1) = struct('stage',phase,'x',x,'detail',detail);
                end
            else
                result.validation_rejections(end+1) = struct('stage',phase,'x',x,'detail',struct('reason',"安装约束或独立纯组复核失败"));
            end
            result.stage_times.verification = result.stage_times.verification+toc(verification_timer);
        end
        Progress([],[],[],inf,phase);
    end

    % 在求解器迭代中检查阶段时限，并将覆盖率与预算使用进度传给软件。
    function stop = Progress(~,~,~,deadline,phase)
        stop = toc(timer) >= deadline;
        if isa(cfg.progress_callback,'function_handle')
            cfg.progress_callback(struct('Stage',char(phase),'Generation',start_index,'Coverage',result.best_coverage, ...
                'EvaluationCount',numel(result.trace),'Progress',min(0.99,toc(timer)/cfg.max_time)));
        end
    end
end

% 检查物理边界和求解配置，保留每个模块四个独立变量的原始范围。
function [cfg, lower, upper] = Search_Config(params, cfg)
    defaults = struct('optimization_target', '首个可行布局', 'x_face', 2, ...
        'y_max', 0.6, 'z_max', 0.6, 'position_a_range', [0, 0.6], ...
        'position_b_range', [-0.6, 0.6], 'alpha_range', [0, 2*pi], 'beta_range', [0, pi/2], ...
        'min_install_distance', 0, 'min_install_angle_deg', 0, 'start_count', 16, ...
        'max_iterations', 120, 'inner_iterations', 8, 'max_time', 600, 'local_max_time', 60, 'min_axis_capacity', 1e-5, ...
        'pure_tolerance', 1e-9, 'initial_x', [], 'resume_file', "", 'seed', 18, 'save_result', true, ...
        'fixed_direction_time', 8, 'fixed_direction_fraction', 0.85, 'max_group_size', 6, 'node_time', 5, ...
        'master_time',30,'geometry_time',15,'solver', "decomposed", 'python_executable', ...
        fullfile(char(java.lang.System.getProperty('user.home')), 'Library','Caches','RCDesigner','strict-solver-venv','bin','python'), ...
        'output_dir', fileparts(mfilename('fullpath')), 'progress_callback', []);
    for name = fieldnames(defaults)'
        if ~isfield(cfg, name{1}) || isempty(cfg.(name{1}))
            cfg.(name{1}) = defaults.(name{1});
        end
    end
    if params.Num ~= 12 || ~isscalar(params.F_max) || ~isfinite(params.F_max) || params.F_max <= 0
        error('RCDesigner:InvalidThrusters', '当前布局优化要求12台等推力推力器，F_max必须为正。');
    end
    if ~ismember(string(cfg.optimization_target), ["首个可行布局","调用可重构优先"])
        error('RCDesigner:UnsupportedOptimizationTarget', '当前阶段只搜索首个满足全部约束的可行布局。');
    end
    for name = {'position_a_range', 'position_b_range', 'alpha_range', 'beta_range'}
        value = cfg.(name{1});
        if ~isnumeric(value) || numel(value) ~= 2 || any(~isfinite(value)) || value(1) > value(2)
            error('RCDesigner:InvalidOptimizationRange', '%s 必须是有限且递增的二元素范围。', name{1});
        end
    end
    for name = {'x_face','y_max','z_max','start_count','max_iterations','inner_iterations','max_time','local_max_time','min_axis_capacity','pure_tolerance','fixed_direction_time','master_time','geometry_time'}
        value = cfg.(name{1});
        if ~isnumeric(value) || ~isscalar(value) || ~isfinite(value) || value <= 0
            error('RCDesigner:InvalidSearchConfig', '%s 必须为正有限标量。', name{1});
        end
    end
    if cfg.min_axis_capacity <= 100*cfg.pure_tolerance
        error('RCDesigner:CapacityBelowTolerance', '最低控制能力须大于纯控制容差的100倍，避免把近零控制误判为有效控制。');
    end
    if ~isscalar(cfg.max_group_size) || ~ismember(cfg.max_group_size,[2,4,6]) || ~isscalar(cfg.node_time) || ~isfinite(cfg.node_time) || cfg.node_time <= 0
        error('RCDesigner:InvalidSearchConfig', '单轴组上限只能为2、4、6台，分区求解时限须为正数。');
    end
    if ~isscalar(cfg.fixed_direction_fraction) || ~isfinite(cfg.fixed_direction_fraction) || cfg.fixed_direction_fraction < 0 || cfg.fixed_direction_fraction > 1
        error('RCDesigner:InvalidSearchConfig', '固定方向搜索预算比例须在0至1之间。');
    end
    if ~isscalar(cfg.min_install_distance) || ~isfinite(cfg.min_install_distance) || ...
            ~isscalar(cfg.min_install_angle_deg) || ~isfinite(cfg.min_install_angle_deg) || ...
            cfg.min_install_distance < 0 || cfg.min_install_angle_deg < 0 || cfg.min_install_angle_deg > 180
        error('RCDesigner:InvalidInstallation', '最小间距不得为负，最小夹角应在0至180度内。');
    end
    lower = repmat([max(-cfg.y_max,cfg.position_a_range(1)), max(-cfg.z_max,cfg.position_b_range(1)), ...
        cfg.alpha_range(1), cfg.beta_range(1)], 1, 3);
    upper = repmat([min(cfg.y_max,cfg.position_a_range(2)), min(cfg.z_max,cfg.position_b_range(2)), ...
        cfg.alpha_range(2), cfg.beta_range(2)], 1, 3);
    if any(lower > upper)
        error('RCDesigner:EmptyOptimizationRange', '位置范围与安装面边界没有交集。');
    end
    cfg.start_count = max(1,round(cfg.start_count));
    cfg.max_iterations = max(1,round(cfg.max_iterations));
    cfg.inner_iterations = max(1,round(cfg.inner_iterations));
    if ~isscalar(cfg.seed) || ~isfinite(cfg.seed) || cfg.seed < 0 || cfg.seed > 2^32-1 || cfg.seed ~= floor(cfg.seed)
        error('RCDesigner:InvalidSeed', '随机种子须为0至2^32-1之间的整数。');
    end
    cfg.solver = string(cfg.solver);
    if ~ismember(cfg.solver,["decomposed","patterns","fair","native","hybrid","scip"])
        error('RCDesigner:UnknownLayoutSolver', 'solver须为decomposed、patterns、fair、native、hybrid或scip。');
    end
    if exist('lsqnonlin','file') ~= 2
        error('RCDesigner:MissingOptimizationToolbox', '固定组配平需要 MATLAB Optimization Toolbox 的 lsqnonlin。');
    end
    if ~isempty(cfg.initial_x) && (size(cfg.initial_x,2) ~= 12 || any(~isfinite(cfg.initial_x),'all') || ...
            any(cfg.initial_x < lower | cfg.initial_x > upper,'all'))
        error('RCDesigner:InvalidInitialLayout', 'initial_x每行须为安装范围内的12个有限参数。');
    end
end

% 生成独立十二变量初值；仅模块排列去重，不进行独立符号翻转去重。
function [starts,history_sources] = Search_Seeds(params,cfg,lower,upper,masks,usable)
    starts = cfg.initial_x;
    history_sources = strings(0,1);
    resumed = zeros(0,12);
    if strlength(string(cfg.resume_file)) > 0
        if endsWith(string(cfg.resume_file),'.json','IgnoreCase',true)
            checkpoint = jsondecode(fileread(cfg.resume_file));
            record.saved = struct('resume_candidates',checkpoint.search_seeds);
        else
            record = load(cfg.resume_file,'saved');
        end
        if ~isfield(record,'saved')
            error('RCDesigner:InvalidResumeFile','恢复文件不包含saved搜索记录。');
        end
        if isfield(record.saved,'resume_candidates')
            resumed = record.saved.resume_candidates;
        elseif isfield(record.saved,'diagnostic_x')
            resumed = record.saved.diagnostic_x;
        end
        if ~isnumeric(resumed) || (~isempty(resumed) && size(resumed,2) ~= 12)
            error('RCDesigner:InvalidResumeFile','恢复候选必须为每行12个安装参数。');
        end
        if ~isempty(resumed)
            resumed = resumed(all(isfinite(resumed) & resumed >= lower & resumed <= upper,2),:);
            resumed = unique(resumed,'rows','stable');
        end
        starts = [starts;resumed];
    end
    % 历史高覆盖布局不能因新版参数化而丢失；从实际位置和方向重建十二变量。
    root = fileparts(mfilename('fullpath'));
    recent = dir(fullfile(root,'Optim_search_*.mat'));
    [~,order] = sort([recent.datenum],'descend');
    recent = recent(order(1:min(12,numel(order))));
    records = [dir(fullfile(root,'Optim_data_*.mat'));recent];
    [~,order] = sort([records.datenum],'descend');
    history = zeros(0,12);
    history_rank = zeros(0,2);
    for file_index = order
        record = load(fullfile(records(file_index).folder,records(file_index).name));
        seed = zeros(1,12);
        if isfield(record,'saved') && isfield(record.saved,'best_coverage_x') && numel(record.saved.best_coverage_x)==12
            seed = record.saved.best_coverage_x;
        elseif isfield(record,'B_opt') && isfield(record,'r_opt') && isequal(size(record.B_opt),[6,12]) && isequal(size(record.r_opt),[3,12])
            for module = 1:3
                column = 4*(module-1)+1;
                position = record.r_opt(2:3,column);
                direction = record.B_opt(1:3,column);
                if position(1) < 0
                    position = -position;
                    direction(2:3) = -direction(2:3);
                end
                seed(4*(module-1)+(1:4)) = [position',mod(atan2(direction(3),direction(2)),2*pi),atan2(norm(direction(2:3)),-direction(1))];
            end
        else
            continue;
        end
        if all(isfinite(seed) & seed>=lower & seed<=upper)
            diagnostic = Evaluate_Layout(seed,cfg,masks,usable,lower,upper);
            history(end+1,:) = seed; %#ok<AGROW>
            history_rank(end+1,:) = [-diagnostic.coverage,diagnostic.residual_norm]; %#ok<AGROW>
            history_sources(end+1,1) = string(fullfile(records(file_index).folder,records(file_index).name)); %#ok<AGROW>
        end
    end
    [~,order] = sortrows(history_rank,[1,2]);
    starts = [starts;history(order,:)];
    history_sources = history_sources(order);
    stream = RandStream('mt19937ar','Seed',cfg.seed);
    % 失败候选只提供附近起点，不作为满足约束的证明；保留全范围随机探索。
    for index = 1:min(4,size(resumed,1)*2)
        center = resumed(mod(index-1,size(resumed,1))+1,:);
        starts = [starts;min(upper,max(lower,center+0.05*(upper-lower).*randn(stream,1,12)))]; %#ok<AGROW>
    end
    starts = [starts;min(upper,max(lower,[0.45,0,pi,pi/3,0.45,0.45,3*pi/2,pi/4,0.45,-0.45,pi/2,pi/4]))];
    for phase = [0,pi/2]
        for height = [-0.4,0.4]
            seed = [0.4,0,phase,pi/3,0.4,height,phase+2*pi/3,pi/3,0.4,-height,mod(phase+4*pi/3,2*pi),pi/3];
            starts = [starts;min(upper,max(lower,seed))]; %#ok<AGROW>
        end
    end
    if isfield(params,'r_all') && isfield(params,'B_all') && size(params.r_all,2)==12
        seed = zeros(1,12);
        for module = 1:3
            column = 4*(module-1)+1;
            d = params.B_all(1:3,column)/norm(params.B_all(1:3,column));
            seed(4*(module-1)+(1:4)) = [params.r_all(2:3,column)',mod(atan2(d(3),d(2)),2*pi),acos(max(-1,min(1,-d(1))))];
        end
        starts = [starts;min(upper,max(lower,seed))];
    end
    for row = 1:size(starts,1)
        starts(row,:) = reshape(sortrows(reshape(starts(row,:),4,3)')',1,12);
    end
    starts = unique(starts,'rows','stable');
    priority_count = cfg.start_count;
    if cfg.start_count > 2
        priority_count = ceil(2*cfg.start_count/3);
    end
    starts = starts(1:min(priority_count,size(starts,1)),:);
    starts = [starts;lower+rand(stream,cfg.start_count,12).*(upper-lower)];
    for row = 1:size(starts,1)
        starts(row,:) = reshape(sortrows(reshape(starts(row,:),4,3)')',1,12);
    end
    starts = unique(starts,'rows','stable');
    starts = starts(1:min(cfg.start_count,size(starts,1)),:);
end

% 用三个独立的 [y,z,alpha,beta] 种子建立严格对称布局及物理控制矩阵。
function [B,r] = Full_Layout(x,a)
    r = zeros(3,12);
    d = zeros(3,12);
    for module = 1:3
        v = x(4*(module-1)+(1:4));
        c = cos(v(4)); p = sin(v(4))*cos(v(3)); q = sin(v(4))*sin(v(3));
        columns = 4*(module-1)+(1:4);
        r(:,columns) = [a,a,-a,-a; v(1),-v(1),v(1),-v(1); v(2),-v(2),v(2),-v(2)];
        d(:,columns) = [-c,-c,c,c; p,-p,p,-p; q,-q,q,-q];
    end
    B = [d;cross(r,d,1)];
end

% 将安装间距和喷射方向夹角的不等式违反量加入可行性残差。
function residual = Installation_Residual(B,r,cfg)
    residual = zeros(0,1);
    for first = 1:11
        for second = first+1:12
            if cfg.min_install_distance > 0
                residual(end+1,1) = (cfg.min_install_distance-norm(r(:,first)-r(:,second)))/cfg.x_face; %#ok<AGROW>
            end
            if cfg.min_install_angle_deg > 0
                residual(end+1,1) = dot(B(1:3,first),B(1:3,second))-cosd(cfg.min_install_angle_deg); %#ok<AGROW>
            end
        end
    end
    residual = max(0,residual);
end

% 固定组编号后的连续配平残差，内层求导期间不重新选择推力器。
function residual = Fixed_Group_Residual(x,topology,masks,cfg)
    [B,r] = Full_Layout(x,cfg.x_face);
    W = (B./[ones(3,1);cfg.x_face*ones(3,1)])*masks(:,topology(:,1));
    for index = 1:size(topology,1)
        axis = ceil(topology(index,2)/2);
        direction = 2*mod(topology(index,2)+1,2)-1;
        W(axis,index) = max(0,cfg.min_axis_capacity-direction*W(axis,index));
    end
    residual = [W(:);Installation_Residual(B,r,cfg)];
end

% 枚举当前布局的全部健康偶数组，分别记录最近配平组与真正满足门槛的组。
function diagnostic = Evaluate_Layout(x,cfg,masks,usable,lower,upper)
    [B,r] = Full_Layout(x,cfg.x_face);
    W = (B./[ones(3,1);cfg.x_face*ones(3,1)])*masks;
    % 若每个模块的目标行符号系数均为0，该组对该轴恒无能力，不能作为配平候选。
    target_signs = [-1,-1,1,1;1,-1,1,-1;1,-1,1,-1;1,1,1,1;-1,1,1,-1;1,-1,-1,1];
    diagnostic = struct('x',x,'selected',zeros(13,12),'capacity',zeros(13,12), ...
        'cross_residual',zeros(13,12),'closest_wrench',zeros(6,13,12), ...
        'strict_capacity',zeros(13,12),'requirement_feasible',false(13,12), ...
        'coverage',0,'residual_norm',0,'installation_violation',Installation_Residual(B,r,cfg), ...
        'bound_violation',max([0,lower-x,x-upper]));
    for label = 1:12
        axis = ceil(label/2);
        direction = 2*mod(label+1,2)-1;
        cross_residual = max(abs(W(setdiff(1:6,axis),:)),[],1);
        R = W;
        R(axis,:) = max(0,cfg.min_axis_capacity-direction*W(axis,:));
        cost = sum(R.^2,1)+zeros(13,1);
        cost(~usable) = inf;
        possible = false(1,size(masks,2));
        for module = 1:3
            possible = possible | target_signs(axis,:)*masks(4*(module-1)+(1:4),:) ~= 0;
        end
        cost(:,~possible) = inf;
        [value,diagnostic.selected(:,label)] = min(cost,[],2);
        diagnostic.residual_norm = diagnostic.residual_norm+sum(value);
        diagnostic.capacity(:,label) = direction*W(axis,diagnostic.selected(:,label));
        diagnostic.cross_residual(:,label) = cross_residual(diagnostic.selected(:,label));
        diagnostic.closest_wrench(:,:,label) = W(:,diagnostic.selected(:,label));
        strict = cross_residual <= cfg.pure_tolerance;
        for fault = 0:12
            diagnostic.strict_capacity(fault+1,label) = max([0,direction*W(axis,strict & usable(fault+1,:))]);
        end
        diagnostic.requirement_feasible(:,label) = any(usable & strict & direction*W(axis,:) >= cfg.min_axis_capacity-cfg.pure_tolerance,2);
    end
    diagnostic.coverage = nnz(diagnostic.requirement_feasible);
    diagnostic.residual_norm = sqrt(diagnostic.residual_norm+sum(diagnostic.installation_violation.^2)+diagnostic.bound_violation^2);
end

% 列出覆盖率最佳候选的缺失需求，区分纯组不存在和纯组能力未达到门槛。
function missing = Missing_Requirements(diagnostic,cfg,masks)
    labels = ["-Fx","+Fx","-Fy","+Fy","-Fz","+Fz","-Mx","+Mx","-My","+My","-Mz","+Mz"];
    missing = struct('fault',{},'axis',{},'closest_group',{},'target_capacity',{}, ...
        'cross_residual',{},'capacity_shortfall',{},'best_pure_capacity',{},'closest_wrench',{},'reason',{});
    for fault = 0:12
        for label = 1:12
            if diagnostic.requirement_feasible(fault+1,label)
                continue;
            end
            reason = "该候选没有正向有效的严格纯组";
            if diagnostic.strict_capacity(fault+1,label) > cfg.pure_tolerance
                reason = "该候选存在严格纯组，但能力低于门槛";
            end
            missing(end+1) = struct('fault',fault,'axis',labels(label), ...
                'closest_group',find(masks(:,diagnostic.selected(fault+1,label)))', ...
                'target_capacity',diagnostic.capacity(fault+1,label), ...
                'cross_residual',diagnostic.cross_residual(fault+1,label), ...
                'capacity_shortfall',max(0,cfg.min_axis_capacity-diagnostic.capacity(fault+1,label)), ...
                'best_pure_capacity',diagnostic.strict_capacity(fault+1,label), ...
                'closest_wrench',diagnostic.closest_wrench(:,fault+1,label),'reason',reason); %#ok<AGROW>
        end
    end
end

% 独立枚举全部偶数组，生成每个故障-正负轴的可复算严格纯组证明。
function certificate = Certify_Layout(x,cfg,masks,usable,lower,upper)
    x = reshape(sortrows(reshape(x,4,3)')',1,12);
    [B,r] = Full_Layout(x,cfg.x_face);
    W = (B ./ [ones(3,1);cfg.x_face*ones(3,1)])*masks;
    certificate = struct('feasible',false,'x',x,'B',B,'r',r,'groups',{cell(13,12)}, ...
        'capacity',zeros(13,12),'max_cross_component',0,'worst_capacity',0);
    if any(~isfinite(x)) || any(x < lower-cfg.pure_tolerance | x > upper+cfg.pure_tolerance) || ...
            any(Installation_Residual(B,r,cfg) > cfg.pure_tolerance)
        return;
    end
    for label = 1:12
        axis = ceil(label/2); direction = 2*mod(label+1,2)-1;
        cross_component = max(abs(W(setdiff(1:6,axis),:)),[],1);
        strict = cross_component <= cfg.pure_tolerance & direction*W(axis,:) >= cfg.min_axis_capacity-cfg.pure_tolerance;
        for fault = 0:12
            selected = find(strict & usable(fault+1,:));
            if isempty(selected)
                return;
            end
            [value,index] = max(direction*W(axis,selected));
            selected = selected(index);
            certificate.groups{fault+1,label} = find(masks(:,selected))';
            certificate.capacity(fault+1,label) = value;
            certificate.max_cross_component = max(certificate.max_cross_component,cross_component(selected));
        end
    end
    certificate.worst_capacity = min(certificate.capacity(2:end,:),[],'all');
    certificate.feasible = true;
end

% 逐一通过现有调用器复核156个需求，并记录最简主备分配实际选择的能力。
function [valid,capacity,detail] = Verify_Invocation(certificate,params,cfg)
    test = params;
    test.alloc_mode = 'synchronous_time_division';
    test.allocation_strategy = 'primary_backup';
    % 此处核验组的结构，暂去掉指令幅值相关的最小脉宽门限；不改仿真参数。
    test.t_min = 0;
    if ~isfield(test,'T')
        test.T = 0.4;
    end
    capacity = zeros(13,12);
    valid = false;
    detail = struct('reason',"",'fault',NaN,'axis',NaN,'group',[],'invocation_status',"");
    reference = [params.F_max*ones(3,1);params.F_max*cfg.x_face*ones(3,1)];
    test.strict_group_contract = struct('row_reference',reference,'pure_tolerance',cfg.pure_tolerance, ...
        'min_axis_capacity',cfg.min_axis_capacity,'max_group_size',cfg.max_group_size);
    for fault = 0:12
        faults = fault;
        if fault == 0
            faults = [];
        end
        for label = 1:12
            axis = ceil(label/2);
            command = zeros(6,1);
            command(axis) = (2*mod(label+1,2)-1)*max(1e-5,0.25*certificate.capacity(fault+1,label))*reference(axis);
            [~,info] = Thruster_invocation(command(1:3),command(4:6),params.F_max*certificate.B,faults,test);
            strategy = info.axis_strategy{1+(axis>3),mod(axis-1,3)+1};
            if isempty(strategy.thrusters) || ~strategy.instantaneous_pure || strategy.is_impulse_margin_fallback || ...
                    mod(numel(strategy.thrusters),2) ~= 0 || numel(strategy.thrusters) > cfg.max_group_size || any(ismember(strategy.thrusters,faults))
                detail = struct('reason',"调用器未选择健康偶数瞬时纯组",'fault',fault,'axis',label, ...
                    'group',strategy.thrusters,'invocation_status',string(strategy.status));
                return;
            end
            actual = sum(certificate.B(:,strategy.thrusters),2)./[ones(3,1);cfg.x_face*ones(3,1)];
            capacity(fault+1,label) = (2*mod(label+1,2)-1)*actual(axis);
            if any(~isfinite(actual)) || max(abs(actual(setdiff(1:6,axis)))) > cfg.pure_tolerance || ...
                    capacity(fault+1,label) < cfg.min_axis_capacity-cfg.pure_tolerance
                detail = struct('reason',"实际调用组未满足统一的纯组容差或最低能力门槛",'fault',fault,'axis',label,'group',strategy.thrusters,'invocation_status',string(strategy.status));
                return;
            end
        end
    end
    valid = true;
end

% 通过JSON调用SCIP联合模型，并保留终止状态、上下界和原始二进制组。
function evidence = Run_Joint_Solver(cfg, lower, upper, remaining_time, seeds)
    input_path = [tempname,'.json'];
    output_path = [tempname,'.json'];
    log_path = [tempname,'.log'];
    progress_path = [tempname,'.json'];
    data = rmfield(cfg,'progress_callback');
    data.lower = lower; data.upper = upper; data.max_time = remaining_time; data.verbose = true;
    data.search_seeds = seeds;
    data.fixed_direction_search = cfg.solver == "hybrid";
    data.fair_search = cfg.solver == "fair";
    data.pattern_search = cfg.solver == "patterns";
    data.decomposition_search = cfg.solver == "decomposed";
    if (data.fair_search || data.pattern_search || data.decomposition_search) && ~isfield(data,'search_state') && strlength(cfg.resume_file) > 0 && isfile(cfg.resume_file)
        if endsWith(string(cfg.resume_file),'.json','IgnoreCase',true)
            previous = jsondecode(fileread(cfg.resume_file));
            data.search_state = previous.search_state;
        else
            previous = load(cfg.resume_file,'saved');
            if isfield(previous,'saved') && isfield(previous.saved,'solver_evidence') && isfield(previous.saved.solver_evidence,'search_state')
                data.search_state = previous.saved.solver_evidence.search_state;
            end
        end
    end
    if (data.fair_search || data.pattern_search || data.decomposition_search) && cfg.save_result
        if ~isfolder(cfg.output_dir)
            mkdir(cfg.output_dir);
        end
        [~,id] = fileparts(tempname(cfg.output_dir));
        data.checkpoint_path = fullfile(cfg.output_dir,['Optim_queue_',id,'.json']);
    end
    data.progress_path = progress_path;
    evidence = struct('status',"error",'has_solution',false,'raw_log',"", ...
        'error',struct('identifier',"",'message',"",'report',""));
    process = [];
    cleanup = onCleanup(@()Close_Solver(process,{input_path,output_path,log_path,progress_path,strrep(progress_path,'.json','.tmp')}));
    try
    if ~isfile(cfg.python_executable)
        error('RCDesigner:MissingJointSolver','未找到联合求解环境：%s',cfg.python_executable);
    end
    file = fopen(input_path,'w','n','UTF-8');
    if file < 0
        error('RCDesigner:SolverInputWrite','无法创建联合求解器输入文件。');
    end
    close_file = onCleanup(@()fclose(file));
    fprintf(file,'%s',jsonencode(data));
    clear close_file;
    command_args = java.util.ArrayList();
    if ismac
        [~, arm_hardware] = system('/usr/sbin/sysctl -n hw.optional.arm64');
        if strcmp(strtrim(arm_hardware),'1')
            % Intel版MATLAB在Apple Silicon上会让通用Python默认继承x86架构。
            command_args.add('/usr/bin/arch'); command_args.add('-arm64');
        end
    end
    command_args.add(char(cfg.python_executable));
    command_args.add(fullfile(fileparts(mfilename('fullpath')),'strict_layout_minlp.py'));
    command_args.add(input_path); command_args.add(output_path);
    builder = java.lang.ProcessBuilder(command_args);
    builder.redirectErrorStream(true);
    builder.redirectOutput(java.io.File(log_path));
    process = builder.start();
    process_cleanup = onCleanup(@()Close_Solver(process,{}));
    clock = tic;
    while process.isAlive()
        if toc(clock) > remaining_time+15
            error('RCDesigner:JointSolverTimeout','联合求解进程未在时间上限内退出。');
        end
        if isa(cfg.progress_callback,'function_handle')
            progress = struct('Stage','joint_minlp','Progress',min(0.99,(cfg.max_time-remaining_time+toc(clock))/cfg.max_time));
            if isfile(progress_path)
                detail = jsondecode(fileread(progress_path));
                progress.Stage = char(detail.stage);
                progress.Coverage = detail.coverage;
                progress.EvaluationCount = detail.attempt;
                if isfield(detail,'pending')
                    progress.Pending = detail.pending;
                end
                if isfield(detail,'topologies')
                    progress.Topologies = detail.topologies;
                    progress.ConflictCount = detail.conflicts;
                    progress.MasterCalls = detail.master_calls;
                end
                if isfield(detail,'patterns_seen')
                    progress.PatternsSeen = detail.patterns_seen;
                    progress.PatternId = detail.pattern_id;
                    progress.FreeCount = detail.free_count;
                end
            end
            cfg.progress_callback(progress);
        end
        pause(0.2);
    end
    if process.exitValue() ~= 0 || ~isfile(output_path)
        detail = fileread(log_path);
        error('RCDesigner:JointSolverFailed','联合求解器失败：%s',detail);
    end
    evidence = jsondecode(fileread(output_path));
    catch ME
        if ~isempty(process) && process.isAlive()
            process.destroyForcibly();
            process.waitFor();
        end
        evidence.status = "error";
        evidence.has_solution = false;
        evidence.error = struct('identifier',string(ME.identifier),'message',string(ME.message), ...
            'report',string(getReport(ME,'extended','hyperlinks','off')));
    end
    if isfile(log_path)
        evidence.raw_log = string(fileread(log_path));
    end
    if isfield(data,'checkpoint_path') && isfile(data.checkpoint_path)
        evidence.checkpoint_file = data.checkpoint_path;
        if ~isfield(evidence,'search_state')
            checkpoint = jsondecode(fileread(data.checkpoint_path));
            evidence.search_state = checkpoint.search_state;
        end
    end
end

% 保存搜索队列和诊断候选；候选使用独立字段，不能冒充B_opt可行结果。
function result = Save_Search_Report(result,params)
    cfg = result.constraints;
    lines = ["严格主备布局：首个可行解搜索诊断";"状态代码："+result.status;"结论："+result.message; ...
        "说明：以下缺失项只描述本轮候选，不构成整个连续空间无解的证明。"; ...
        "归一化：力/F_max，力矩/(F_max*x_face)；故障编号0表示标况。"; ...
        sprintf("单轴严格控制组：2:2:%d台；未通过候选允许导入查看，不代表可行布局。",cfg.max_group_size); ...
        sprintf("F_max=%.17g N，x_face=%.17g m，非零能力判据=%.17g，纯组容差=%.3g",params.F_max,cfg.x_face,cfg.min_axis_capacity,cfg.pure_tolerance); ...
        sprintf("总耗时=%.3f s，本地=%.3f s，联合=%.3f s，候选精修=%.3f s，独立复核=%.3f s",result.elapsed_time, ...
            result.stage_times.local,result.stage_times.joint,result.stage_times.polish,result.stage_times.verification); ...
        sprintf("初值已尝试/已完成=%d/%d，固定组选配调用=%d，内层函数评价=%d",result.starts_attempted,result.starts_completed,result.fixed_group_calls,result.function_evaluations); ...
        sprintf("最高覆盖=%d/156，最佳配平残差范数=%.17g，随机种子=%d",result.best_coverage,result.best_balance_residual,cfg.seed); ...
        "本轮恢复来源："+string(cfg.resume_file);"终止阶段："+result.last_stage;"联合求解器状态："+string(result.solver_evidence.status); ...
        "MATLAB："+string(result.environment.matlab);"运行架构："+string(result.environment.architecture); ...
        "完整参数设置（实际初值另存于MAT中的saved.initial_points）：";string(jsonencode(rmfield(cfg,'progress_callback'))); ...
        "覆盖率最佳候选x："+string(mat2str(result.best_coverage_x,17));"残差最佳候选x："+string(mat2str(result.diagnostic_x,17))];
    if isfield(result.solver_evidence,'environment')
        lines(end+1) = "联合求解环境："+string(jsonencode(result.solver_evidence.environment));
    end
    if isfield(result,'history_sources') && ~isempty(result.history_sources)
        lines(end+1) = "自动回读历史布局与候选："+strjoin(result.history_sources,"；");
    end
    if isfield(result.solver_evidence,'nodes')
        lines(end+1) = sprintf("联合求解节点数=%d，候选解数=%d；常数目标的上下界不代表控制能力上下界。", ...
            result.solver_evidence.nodes,result.solver_evidence.solution_count);
    end
    if isfield(result.solver_evidence,'infeasibility_proof')
        lines(end+1) = "结构性无解证据："+string(jsonencode(result.solver_evidence.infeasibility_proof));
    end
    if isfield(result.solver_evidence,'fixed_direction_attempts')
        attempts = result.solver_evidence.fixed_direction_attempts;
        lines(end+1) = sprintf("固定方向方案已检查%d组；该有限搜索不构成连续空间无解证明。",numel(attempts));
        for index = 1:numel(attempts)
            if iscell(attempts)
                trial = attempts{index};
            else
                trial = attempts(index);
            end
            lines(end+1) = "方向方案"+index+"："+string(jsonencode(trial)); %#ok<AGROW>
        end
    end
    if isfield(result.solver_evidence,'search_state')
        state = result.solver_evidence.search_state;
        if isfield(state,'conflicts')
            lines(end+1) = sprintf("组选与几何分解：累计几何检查%d次，已登记%d个待解拓扑，保存%d条可复核冲突，主问题求解%d次。",state.processed,numel(state.pending),numel(state.conflicts),state.master_calls);
            lines(end+1) = "仅精确有理数证据永久排除组合；数值不可行和超时的几何区域均保留。检查点不包含求解器内部完整搜索树。";
        else
        lines(end+1) = sprintf("分区轮转：累计处理%d个区域，待处理%d个，已排除%d个；继续搜索将恢复队列。",state.processed,numel(state.pending),state.pruned);
        lines(end+1) = "分区不是固定拓扑，每区均联合搜索全部健康2/4/6台选组；数值分支搜索不构成有限时间必获解保证。";
        end
        if isfield(result.solver_evidence,'resume_note')
            lines(end+1) = "队列恢复："+string(result.solver_evidence.resume_note);
        end
        if isfield(result.solver_evidence,'checkpoint_file')
            lines(end+1) = "中断续跑检查点："+string(result.solver_evidence.checkpoint_file);
        end
        if isfield(state,'patterns')
            lines(end+1) = sprintf("零/自由模式：共729类，已初筛%d类，有求解调用%d类；自由变量允许落到零，不是729个互斥的完整选组拓扑。", ...
                nnz(string({state.patterns.status})~="pending"),nnz([state.patterns.calls]>0));
        end
    end
    if ~isempty(fieldnames(result.best_coverage_candidate))
        d = result.best_coverage_candidate;
        lines(end+1) = sprintf("覆盖率最佳候选：安装违反量最大值=%.17g，变量越界量=%.17g",max([0;d.installation_violation]),d.bound_violation);
    end
    lines(end+1) = "缺失需求：以下最近配平组不一定就是严格纯组；能力与耦合量均已归一化。";
    for index = 1:numel(result.diagnostic_missing)
        item = result.diagnostic_missing(index);
        lines(end+1) = sprintf("故障=%d，轴=%s，最近组=%s，目标能力=%.9g，非目标最大分量=%.9g，能力缺口=%.9g，严格纯组最大能力=%.9g；%s", ...
            item.fault,item.axis,mat2str(item.closest_group),item.target_capacity,item.cross_residual,item.capacity_shortfall,item.best_pure_capacity,item.reason); %#ok<AGROW>
    end
    if isempty(result.diagnostic_missing)
        lines(end+1) = "无缺失项明细：可能未评估任何候选，或轴向组条件已通过；请同时检查状态、安装约束和调用器复核。";
    end
    for index = 1:numel(result.validation_rejections)
        lines(end+1) = "独立复核未通过："+string(jsonencode(result.validation_rejections(index))); %#ok<AGROW>
    end
    if strlength(result.error.report) > 0
        lines = [lines;"错误标识："+result.error.identifier;"错误信息："+result.error.message;"完整调用堆栈：";result.error.report];
    end
    result.diagnostic_text = strjoin(lines,newline);
    if ~cfg.save_result
        return;
    end
    try
        if ~isfolder(cfg.output_dir)
            mkdir(cfg.output_dir);
        end
        prefix = 'Optim_search_';
        if result.feasible
            prefix = 'Optim_data_';
        end
        [~,unique_id] = fileparts(tempname(cfg.output_dir));
        basename = fullfile(cfg.output_dir,[prefix,char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')),'_',unique_id]);
        result.output_file = string([basename,'.mat']);
        result.diagnostic_file = string([basename,'.txt']);
        if isfield(result.solver_evidence,'raw_log') && strlength(string(result.solver_evidence.raw_log)) > 0
            result.solver_log_file = string([basename,'.solver.log']);
            Write_Search_Text(result.solver_log_file,result.solver_evidence.raw_log);
        end
        Write_Search_Text(result.diagnostic_file,result.diagnostic_text);
        saved = result;
        saved.constraints.progress_callback = [];
        if isfield(result.solver_evidence,'search_state') && isfield(result.solver_evidence.search_state,'conflicts')
            result.conflict_file = string([basename,'.conflicts.json']);
            saved.conflict_file = result.conflict_file;
            proofRecord = struct('method',result.solver_evidence.search_state.method, ...
                'signature',result.solver_evidence.search_state.signature, ...
                'conflicts',result.solver_evidence.search_state.conflicts);
            Write_Search_Text(result.conflict_file,string(jsonencode(proofRecord)));
        end
        if isfield(result.solver_evidence,'search_state') && isfield(result.solver_evidence.search_state,'patterns')
            patterns = result.solver_evidence.search_state.patterns;
            patternMask = arrayfun(@(p)string(sprintf('%d%d%d%d|%d%d%d%d|%d%d%d%d',p.mask)),patterns(:));
            patternTable = table([patterns.id]',patternMask,[patterns.free_count]',[patterns.calls]',string({patterns.status})',[patterns.elapsed]', ...
                'VariableNames',{'PatternId','FreeMask_y_z_alpha_beta','FreeCount','SolverCalls','Status','ElapsedSeconds'});
            result.pattern_table_file = string([basename,'.patterns.csv']);
            saved.pattern_table_file = result.pattern_table_file;
            writetable(patternTable,result.pattern_table_file,'Encoding','UTF-8');
        end
        B_opt = result.B_opt; r_opt = result.r_opt; x_opt = result.x_opt; fval = result.fval;
        candidate_B = []; candidate_r = [];
        if ~isempty(result.best_coverage_x)
            [candidate_B,candidate_r] = Full_Layout(result.best_coverage_x,cfg.x_face);
        end
        save(result.output_file,'saved','B_opt','r_opt','x_opt','fval','candidate_B','candidate_r');
    catch ME
        result.persistence_error = string(getReport(ME,'extended','hyperlinks','off'));
        warning('RCDesigner:DiagnosticSaveFailed','保存诊断失败：%s。完整诊断仍保留在返回值中。',ME.message);
        if ~isfile(result.output_file)
            result.output_file = "";
        end
        if ~isfile(result.diagnostic_file)
            result.diagnostic_file = "";
        end
        if ~isfile(result.solver_log_file)
            result.solver_log_file = "";
        end
    end
end

% 用UTF-8写入诊断文本，确保中文错误消息和文件路径可以直接查看。
function Write_Search_Text(path,body)
    file = fopen(path,'w','n','UTF-8');
    if file < 0
        error('RCDesigner:DiagnosticWrite','无法创建诊断文件：%s',path);
    end
    cleanup = onCleanup(@()fclose(file));
    fprintf(file,'%s',body);
end

% 终止尚未退出的本次子进程，并清理本次创建的三个临时文件。
function Close_Solver(process,paths)
    if ~isempty(process) && process.isAlive()
        process.destroyForcibly();
        process.waitFor();
    end
    for index = 1:numel(paths)
        if isfile(paths{index})
            delete(paths{index});
        end
    end
end
