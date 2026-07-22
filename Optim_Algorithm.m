%% 推力器布局优化设计
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
%             display, save_result, output_dir
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
    base_num = params.Num / 4;   % 每个基准推力器通过对称构形生成4台推力器
    lb = repmat([positionARange(1), positionBRange(1), ...
        opt_cfg.alpha_range(1), opt_cfg.beta_range(1)], 1, base_num);
    ub = repmat([positionARange(2), positionBRange(2), ...
        opt_cfg.alpha_range(2), opt_cfg.beta_range(2)], 1, base_num);
    nvars = 4 * base_num;

    can_run_ga = exist('ga', 'file') == 2 && exist('optimoptions', 'file') == 2;
    if can_run_ga
        try
            options = optimoptions('ga', ...
                'Display', opt_cfg.display, ...
                'PopulationSize', opt_cfg.population_size, ...
                'MaxGenerations', opt_cfg.max_generations, ...
                'FunctionTolerance', opt_cfg.function_tolerance, ...
                'UseParallel', opt_cfg.use_parallel);

            tic;
            [x_opt, fval] = ga(@(x) Optimal_config(x, params), nvars, ...
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
            result.base_thruster_count = base_num;
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

    names = fieldnames(defaults);
    for index = 1:numel(names)
        name = names{index};
        if ~isfield(opt_cfg, name) || isempty(opt_cfg.(name))
            opt_cfg.(name) = defaults.(name);
        end
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

%% 推力器布局优化配置
function [B_all, r] = Thruster_reconfig(x, params)
    if params.Num < 4 || params.Num ~= round(params.Num) || mod(params.Num, 4) ~= 0
        error('RCDesigner:InvalidThrusterCount', ...
            '推力器数量必须为4的整数倍（4n）。');
    end
    base_num = params.Num / 4;
    if numel(x) ~= 4 * base_num
        error('RCDesigner:InvalidOptimizationVector', ...
            '优化变量数量应为4×基准推力器台数。');
    end

    r = zeros(3, params.Num);
    d = zeros(3, params.Num);
    alpha = zeros(1, params.Num);
    beta = zeros(1, params.Num);
    x_face = 2;
    if isfield(params, 'optim_x_face') && isnumeric(params.optim_x_face) && ...
            isscalar(params.optim_x_face) && isfinite(params.optim_x_face) && ...
            params.optim_x_face > 0
        x_face = params.optim_x_face;
    end

    % 报告4.3.1中的通用对称构形：
    % 每个基准推力器先在+X安装面按原点中心对称生成一台，
    % 再镜像至-X安装面，因此每组确定4台推力器。
    for group = 1:base_num
        variableIndex = 4 * (group - 1);
        thrusterIndex = 4 * (group - 1);
        y0 = x(variableIndex + 1);
        z0 = x(variableIndex + 2);
        alpha0 = x(variableIndex + 3);
        beta0 = x(variableIndex + 4);

        plusBase = thrusterIndex + 1;
        plusCenter = thrusterIndex + 2;
        minusBase = thrusterIndex + 3;
        minusCenter = thrusterIndex + 4;

        r(:, plusBase) = [x_face; y0; z0];
        alpha(plusBase) = alpha0;
        beta(plusBase) = beta0;

        r(:, plusCenter) = [x_face; -y0; -z0];
        alpha(plusCenter) = mod(alpha0 + pi, 2*pi);
        beta(plusCenter) = beta0;

        r(:, minusBase) = [-x_face; y0; z0];
        alpha(minusBase) = alpha0;
        beta(minusBase) = beta0;

        r(:, minusCenter) = [-x_face; -y0; -z0];
        alpha(minusCenter) = mod(alpha0 + pi, 2*pi);
        beta(minusCenter) = beta0;
    end

    B_all = zeros(6, params.Num);
    for index = 1:params.Num
        dx = -sign(r(1, index)) * cos(beta(index));
        dy = sin(beta(index)) * cos(alpha(index));
        dz = sin(beta(index)) * sin(alpha(index));
        d(:, index) = [dx; dy; dz];
        B_all(1:3, index) = d(:, index);
        B_all(4:6, index) = cross(r(:, index), d(:, index));
    end
end

%% 最优布局参数目标函数
function J = Optimal_config(x,params)
    [B_all, ~] = Thruster_reconfig(x,params);
    penatly = 0;
    [~, Jc1, ~, Jc, ~,~,~] = Reconfig_eval(params, B_all);
    % [~, Jc2, ~, ~, ~,~,~] = Reconfig_eval(params, B_all,2);
    v_Jc = max(0, 0 - Jc);
    % v_Jc1_f = max(0,0 - Jc1(:,1));
    % v_Jc1_t = max(0,0 - Jc1(:,2));
    penatly = penatly + 10000 * (v_Jc' * v_Jc);
    J = -(Jc1(:,1)'*Jc1(:,1)+Jc1(:,2)'*Jc1(:,2))+penatly;
end

%% 非线性物理约束
function [c, ceq] = Physical_constraints(x, params, opt_cfg)
    [B_all, r] = Thruster_reconfig(x, params);
    minDistance = opt_cfg.min_install_distance;
    minAngleDeg = opt_cfg.min_install_angle_deg;
    c = zeros(0, 1);
    ceq = [];

    % 当最小间距和最小夹角均为0时，不启用这两项非线性约束。
    if minDistance <= 0 && minAngleDeg <= 0
        return;
    end

    directions = B_all(1:3, :);
    directions = directions ./ max(vecnorm(directions, 2, 1), 1e-12);
    pairCount = params.Num * (params.Num - 1) / 2;
    if minDistance > 0
        distanceConstraint = zeros(pairCount, 1);
    else
        distanceConstraint = zeros(0, 1);
    end
    if minAngleDeg > 0
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
    c = [distanceConstraint; angleConstraint];
end 
