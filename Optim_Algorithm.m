%% 推力器布局优化设计
clear; clc;
params = Get_params();
Y_max = 0.6;% 安装面尺寸约束
Z_max = 0.6;
lb = [0, -Z_max, 0, 0, -Y_max, -Z_max, 0, 0];
ub = [Y_max, 0, 2*pi, pi/2, 0, Z_max, 2*pi, pi/2];
options = optimoptions('ga', ...
                       'Display', 'iter', ...
                       'PopulationSize', 500, ...% 种群大小1000
                       'MaxGenerations', 50, ...  % 最大迭代代数
                       'FunctionTolerance', 1e-4, ...
                       'UseParallel', true);
tic;
[x_opt, fval] = ga(@(x) Optimal_config(x,params), 8, [], [], [], [], lb, ub, @(x) Physical_constraints(x,params), options);
% [x_opt, fval] = ga(@(x) Optimal_config(x,params), 4, [], [], [], [], lb, ub, @(x) Physical_constraints(x,params), options);
toc;
[B_opt, r_opt] = Thruster_reconfig(x_opt,params);
save('Optim_config_data1.mat', 'B_opt', 'r_opt', 'fval');

%% 推力器布局优化配置
function [B_all, r] = Thruster_reconfig(x, params)
    r = zeros(3, 12);
    d = zeros(3, 12);
    alpha = zeros(1, 12);
    beta = zeros(1, 12);
    B_all = zeros(6, 12);

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
    alpha(2) = -alpha(1); 
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
    for i = 1:12
        if params.r_all(1, i) > 0
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
end

%% 最优布局参数目标函数
function J = Optimal_config(x,params)
    [B_all, ~] = Thruster_reconfig(x,params);
    w = 0.5;
    penalty = 0;
    for i = 1:params.Num+1
        if i == 1
            eval_fault = [];
        else
            eval_fault = i - 1;
        end
        % [~, Jc_eval, ~, Ja_eval, Jo_eval] = Reconfig_eval(params, B_all, eval_fault);
        v_Jc1 = max(0, params.Jc(i,1) - Jc_eval(1));
        v_Jc2 = max(0, params.Jc(i,2) - Jc_eval(2));
        v_Ja1 = max(0, params.Ja(i,1) - Ja_eval(1));
        v_Ja2 = max(0, params.Ja(i,2) - Ja_eval(2));
        v_Jo  = max(0, params.Jo(i) - Jo_eval);
        % penalty = penalty + 1000 * (v_Jc1^2 + v_Jc2^2 + v_Jo^2) + 100 * (v_Ja1^2 + v_Ja2^2);
        penalty = penalty + v_Jc1+v_Jc2+v_Ja1+v_Ja2+v_Jo;
    end
    % [~,Jc,~,Ja,~] = Reconfig_eval(params,B_all,1);
    % Jc_all = w * Jc(:,1) + (1-w) * Jc(:,2);
    % Ja_all = w * Ja(:,1) + (1-w) * Ja(:,2);
    J = penalty;
end

%% 非线性物理约束
function [c, ceq] = Physical_constraints(x, params)
    [~, r] = Thruster_reconfig(x, params);
    D_min = 0.1; % 最小安装间距
    c = zeros(66, 1); 
    ceq = []; 
    c_idx = 1;
    for i = 1:12
        for j = (i+1):12
            dist = norm(r(:, i) - r(:, j));
            c(c_idx) = D_min - dist; 
            c_idx = c_idx + 1;
        end
    end
end 

% 国外文献优化方法
% % 配置 fmincon 的优化选项 (作为局部精细搜寻)
% options_fmincon = optimoptions('fmincon', ...
%     'Algorithm', 'sqp', ...          % SQP 算法对复杂的非线性约束表现更好
%     'Display', 'iter', ...
%     'MaxFunctionEvaluations', 2000);
% % 配置 GA 的优化选项，并指定 HybridFcn 为 fmincon
% options_ga = optimoptions('ga', ...
%     'Display', 'iter', ...
%     'PopulationSize', 500, ...       % 种群大小
%     'MaxGenerations', 50, ...        % 最大迭代代数
%     'FunctionTolerance', 1e-4, ...
%     'UseParallel', true, ...
%     'HybridFcn', {@fmincon, options_fmincon}); % 关键：GA结束后自动移交 fmincon
% % 执行混合优化
% [x_opt, fval] = ga(@(x) Optimal_config(x, params), 8, [], [], [], [], lb, ub, @(x) Physical_constraints(x, params), options_ga);


% %% 推力器布局理论全局优化设计 (全随机解耦版)
% clear; clc;
% % 假设 Get_params() 包含了 Jc, Ja 目标指标以及 Num 等参数
% params = Get_params(); 
% Y_max = 0.6; % 安装面尺寸约束
% Z_max = 0.6;
% 
% % 12个推力器，每个推力器4个独立变量：[y, z, alpha, beta]
% % 总共 48 个变量
% lb = repmat([-Y_max, -Z_max, 0,    0], 1, 12);
% ub = repmat([ Y_max,  Z_max, 2*pi, pi/2], 1, 12);
% 
% % 配置 GA 的优化选项
% % 注意：48维空间极其庞大，需要更大的种群和迭代次数
% options_ga = optimoptions('ga', ...
%     'Display', 'iter', ...
%     'PopulationSize', 1000, ...       % 扩大种群基数以覆盖高维空间
%     'MaxGenerations', 200, ...        % 增加迭代代数
%     'FunctionTolerance', 1e-4, ...
%     'UseParallel', true);
% 
% % 如果你想在GA跑完后用梯度下降法找极其精确的最优解，可以取消注释下面两行启用混合优化：
% % options_fmincon = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'iter');
% % options_ga.HybridFcn = {@fmincon, options_fmincon};
% 
% tic;
% % 变量维度设为 48
% [x_opt, fval] = ga(@(x) Optimal_config_free(x, params), 48, [], [], [], [], lb, ub, @(x) Physical_constraints_free(x, params), options_ga);
% toc;
% 
% [B_opt, r_opt] = Thruster_reconfig_free(x_opt, params);
% save('Optim_config_data_GlobalFree.mat', 'B_opt', 'r_opt', 'fval', 'x_opt');
% disp('优化完成！理论最优构型已保存。');
% 
% 
% %% 推力器布局配置 (全独立计算)
% function [B_all, r] = Thruster_reconfig_free(x, params)
%     r = zeros(3, 12);
%     d = zeros(3, 12);
%     B_all = zeros(6, 12);
% 
%     % 将 48 维向量解析为 12 个推力器的状态
%     for i = 1:12
%         idx = (i-1)*4;
%         y_val     = x(idx + 1);
%         z_val     = x(idx + 2);
%         alpha_val = x(idx + 3);
%         beta_val  = x(idx + 4);
% 
%         % 分配安装面：前6个在 +X 面，后6个在 -X 面 (符合常见卫星箱体布局)
%         if i <= 6
%             r(:, i) = [2; y_val; z_val];
%             dx = -cos(beta_val); % +X面朝向内推
%         else
%             r(:, i) = [-2; y_val; z_val];
%             dx = cos(beta_val);  % -X面朝向内推
%         end
% 
%         % 计算方向向量
%         dy = sin(beta_val) * cos(alpha_val);
%         dz = sin(beta_val) * sin(alpha_val);
% 
%         d(:, i) = [dx; dy; dz];
% 
%         % 计算控制矩阵 (前3行是力，后3行是力矩)
%         B_all(1:3, i) = d(:, i);
%         B_all(4:6, i) = cross(r(:, i), d(:, i));
%     end
% end
% 
% 
% %% 最优布局参数目标函数 (带软惩罚项)
% function J = Optimal_config_free(x, params)
%     [B_all, ~] = Thruster_reconfig_free(x, params);
%     w = 0.5;
% 
%     % 获取综合评估指标 (基准控制能力)
%     [~, Jc, ~, Ja, ~] = Reconfig_eval(params, B_all, 1);
%     Jc_all = w * Jc(:,1) + (1-w) * Jc(:,2);
%     Ja_all = w * Ja(:,1) + (1-w) * Ja(:,2);
% 
%     % % 基础目标：最大化力矩/力，最小化误差
%     % J_base = -Jc_all + Ja_all;
%     % 
%     % --- 惩罚函数设计 ---
%     % 代替原本会导致死锁的非线性硬约束，通过施加巨大数值惩罚引导算法
%     penalty = 0;
% 
%     for i = 1:params.Num+1
%         [~, Jc_eval, ~, Ja_eval, Jo_eval] = Reconfig_eval(params, B_all, i);
% 
%         % 只有当性能低于要求时，才产生惩罚值 (max保证超标部分不奖励)
%         v_Jc1 = max(0, params.Jc(i,1) - Jc_eval(1));
%         v_Jc2 = max(0, params.Jc(i,2) - Jc_eval(2));
%         % v_Ja1 = max(0, Ja_eval(1) - params.Ja(i,1));
%         % v_Ja2 = max(0, Ja_eval(2) - params.Ja(i,2));
%         % v_Jo  = max(0, 1e-6 - Jo_eval);
% 
%         % 权重系数：力矩(Jc)缺口罚得最重，精度(Ja)次之
%         % penalty = penalty + 1000 * (v_Jc1^2 + v_Jc2^2 + v_Jo^2) + 100 * (v_Ja1^2 + v_Ja2^2);
%         penalty = penalty + 10000 * (v_Jc1^2 + v_Jc2^2);
%     end
%     % 
%     % % 总目标函数 = 原始性能目标 + 违规惩罚
%     % J = J_base + penalty;
%     J = -Jc_all +penalty;
% end
% 
% 
% %% 物理约束 (仅保留绝对不可侵犯的防干涉约束)
% function [c, ceq] = Physical_constraints_free(x, params)
%     [~, r] = Thruster_reconfig_free(x, params);
%     D_min = 0.1; % 最小安装间距
% 
%     % 12个推力器两两之间的距离，共 C(12,2) = 66 个约束
%     c = zeros(66, 1); 
%     ceq = []; 
% 
%     c_idx = 1;
%     for i = 1:12
%         for j = (i+1):12
%             dist = norm(r(:, i) - r(:, j));
%             c(c_idx) = D_min - dist; 
%             c_idx = c_idx + 1;
%         end
%     end
% end