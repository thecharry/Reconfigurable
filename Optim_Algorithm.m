%% 推力器布局优化设计
clear; clc;
params = Get_params();
Y_max = 0.6;% 安装面尺寸约束
Z_max = 0.6;
base_num = 2;% params.Num / 4(3个基准推力器)
lb = repmat([0, -Z_max, 0, 0], 1, base_num);
ub = repmat([Y_max, Z_max, 2*pi, pi/2], 1, base_num);
nvars = 4 * base_num;
options = optimoptions('ga', ...
                       'Display', 'iter', ...
                       'PopulationSize', 500, ...% 种群大小1000
                       'MaxGenerations', 50, ...  % 最大迭代代数
                       'FunctionTolerance', 1e-4, ...
                       'UseParallel', true);
tic;
[x_opt, fval] = ga(@(x) Optimal_config(x,params), nvars, [], [], [], [], lb, ub, @(x) Physical_constraints(x,params), options);
toc;
[B_opt, r_opt] = Thruster_reconfig(x_opt,params);
timestamp = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
output_file = sprintf('Optim_data_%s.mat', timestamp);
save(output_file, 'B_opt', 'r_opt', 'fval', 'x_opt');

%% 推力器布局优化配置
function [B_all, r] = Thruster_reconfig(x, params)
    r = zeros(3, params.Num);
    d = zeros(3, params.Num);
    alpha = zeros(1, params.Num);
    beta = zeros(1, params.Num);
    B_all = zeros(6, params.Num);

    %% 2基准面布局
    % 四象限对称法
    % 基准推力器
    r(:,10) = [2; x(1); x(2)];
    alpha(10) = x(3);
    beta(10) = x(4);
    % 第2象限镜像(-Y, Z)
    r(:,9) = [2; -x(1); x(2)];
    alpha(9) = pi - alpha(10);
    beta(9) = beta(10);
    % 第3象限镜像(-Y, -Z)
    r(:,5) = [2; -x(1); -x(2)];
    alpha(5) = pi + alpha(10);
    beta(5) = beta(10);
    % 第4象限镜像(Y, -Z)
    r(:,6) = [2; x(1); -x(2)];
    alpha(6) = -alpha(10);
    beta(6) = beta(10);
    % 半平面分割法
    % 基准推力器(第1半平面)
    r(:,1) = [2; x(5); x(6)];
    alpha(1) = x(7);
    beta(1) = x(8);
    % 第2半平面映射(原点中心对称)
    r(:,2) = [2; -x(5); -x(6)];
    % alpha(2) = -alpha(1);
    alpha(2) = mod(alpha(1) + pi, 2*pi);
    beta(2) = beta(1);
    % X面安装镜像
    idx1 = [1, 2, 9, 10, 6, 5];
    idx2 = [3, 4, 11, 12, 8, 7];
    for k = 1:6
        p = idx1(k);
        m = idx2(k);
        r(:, m) = [-2; r(2, p); r(3, p)];
        alpha(m) = alpha(p);
        beta(m)  = beta(p);
    end
    % 推力器方向矢量与配置矩阵计算
    for i = 1:params.Num
        if r(1, i) > 0
            dx = -cos(beta(i));
        else
            dx = cos(beta(i));
        end
        dy = sin(beta(i)) * cos(alpha(i));
        dz = sin(beta(i)) * sin(alpha(i));

        d(:, i) = [dx; dy; dz];
        B_all(1:3, i) = d(:, i);
        B_all(4:6, i) = cross(r(:, i), d(:, i));
    end

    %% 3基准推力器布局
    % base_num = params.Num / 4;
    % if base_num ~= fix(base_num)
    %     error('统一对称布局要求推力器数量为4的倍数。');
    % end

    % for g = 1:base_num
    %     idx = 4 * (g - 1);
    %     y0 = x(idx + 1);
    %     z0 = x(idx + 2);
    %     alpha0 = x(idx + 3);
    %     beta0 = x(idx + 4);

    %     p = 4 * g - 3;
    %     q = 4 * g - 2;
    %     pm = 4 * g - 1;
    %     qm = 4 * g;

    %     r(:, p) = [2; y0; z0];
    %     alpha(p) = alpha0;
    %     beta(p) = beta0;

    %     r(:, q) = [2; -y0; -z0];
    %     alpha(q) = mod(alpha0 + pi, 2*pi);
    %     beta(q) = beta0;

    %     r(:, pm) = [-2; y0; z0];
    %     alpha(pm) = alpha0;
    %     beta(pm) = beta0;

    %     r(:, qm) = [-2; -y0; -z0];
    %     alpha(qm) = mod(alpha0 + pi, 2*pi);
    %     beta(qm) = beta0;
    % end
    % % 推力器方向矢量与配置矩阵计算
    % for i = 1:params.Num
    %     dx = -sign(r(1, i)) * cos(beta(i));
    %     dy = sin(beta(i)) * cos(alpha(i));
    %     dz = sin(beta(i)) * sin(alpha(i));

    %     d(:, i) = [dx; dy; dz];
    %     B_all(1:3, i) = d(:, i);
    %     B_all(4:6, i) = cross(r(:, i), d(:, i));
    % end
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
function [c, ceq] = Physical_constraints(x, params)
    [~, r] = Thruster_reconfig(x, params);
    D_min = 0.1; % 最小安装间距
    c = zeros(params.Num * (params.Num - 1) / 2, 1);
    ceq = []; 
    c_idx = 1;
    for i = 1:params.Num
        for j = (i+1):params.Num
            dist = norm(r(:, i) - r(:, j));
            c(c_idx) = D_min - dist; 
            c_idx = c_idx + 1;
        end
    end
end 
