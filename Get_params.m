%% 参数设置
function params = Get_params()
    R_earth = 6371e3;
    h_orbit = 400e3;
    mu = 3.986e14;
    params.t_min = 0.02;% 最小脉宽
    params.F_max = 10;
    params.Num = 12;
    params.T = 0.4;% 系统控制周期
    params.m = 3700;
    params.J = diag([10000, 6000, 13000]);
    params.n = sqrt(mu / (R_earth + h_orbit)^3);
    % 默认调用采用“同步分时复用 + 最简主备分配”。
    % 软件还提供任务书的异步分时复用，以及直接求六维占空比的
    % 联合优化复用，用于在同一布局和指令下进行对比。
    % 一个轴向调用组不能由奇数台推力器构成。故障或最小脉宽剔除
    % 发生时，当前完整调用组整体退出；不得保留其中的残余成员喷气。
    % 健康成员可在重新求得的另一完整备用组中复用，以免因永久关停
    % 某一固定搭档而损失原布局本来具备的单故障重构能力。
    params.alloc_mode = 'synchronous_time_division';
    params.allocation_strategy = 'primary_backup';
    params.alloc_max_group_size = 6;% 搜索纯六维控制的最大调用台数
    params.alloc_candidate_limit = 128;% 每个轴向保留的最简候选上限
    params.alloc_fit_tol = 1e-8;% 纯力/纯力矩六维残差容差
    params.alloc_even_group_only = true;% 每个轴向调用组只能使用偶数台
    params.alloc_prefer_instantaneous_group = true;% 优先选择瞬时纯控制组
    params.alloc_require_instantaneous_group = true;% 标况仅执行瞬时纯控制组
    params.alloc_instantaneous_tol = 1e-8;% 瞬时纯控制判定容差
    % 严格单故障备用不存在时，可采用“冲量裕度耦合备用组”。该组
    % 不再受偶数台约束，但必须消耗少于下列比例的反向纯控制冲量储备。
    params.alloc_impulse_margin_fallback = true;
    params.alloc_impulse_margin_limit = 0.80;
    params.alloc_impulse_margin_force_weight = 0.35;
    params.alloc_impulse_candidate_limit = 512;
    % 注：原 12 台布局并不保证每个正负轴向均有两套互不含同一台
    % 推力器的瞬时纯控制组。因此严格层会在部分单故障轴向失效，
    % 再由冲量裕度层判断能否安全采用耦合备用。布局优化仍应将
    % “每个带符号轴向至少两套独立完整组”作为硬约束。
    % 时序调度层：在保持周期平均调用结果的同时，安排各组喷气时段，
    % 并将轨控瞬时残余力矩约束在保守姿控预留裕度以内。
    params.scheduler_enable = true;
    params.scheduler_peak_torque_ratio = 0.25;
    params.scheduler_peak_force_ratio = inf;% 当前仅监视姿控附带力峰值
    params.scheduler_time_tol = 1e-10;
    % 联合优化复用：min ||D^(-1)(B*u-w)||^2 + rho||u||^2，0<=u<=1。
    % 使用最小脉宽活动集后再求一次有界解，避免输出无法实际执行的
    % 短脉冲。该选项不拆分轨控/姿控，因此不具有“纯组”解释。
    params.joint_opt_regularization = 1e-6;
    params.joint_opt_max_iter = 800;
    params.joint_opt_tolerance = 1e-9;
    params.joint_opt_enforce_min_pulse = true;
    params.joint_opt_reconfig_tolerance = 5e-3;
    [params.B_all, params.r_all] = Thruster_config();
    [params.Z, ~, ~, params.Jc, params.Jo, params.Jt,params.Jf] = Reconfig_eval(params, params.B_all);

    %% 推力器原布局
    function [B_all, r] = Thruster_config()
        r = zeros(3, 12); d = zeros(3, 12);
        r(:,1) = [ 2;-0.6;   0]; d(:,1) = [-0.5; 0.866;  0];
        r(:,2) = [ 2; 0.6;   0]; d(:,2) = [-0.5;-0.866;  0];
        r(:,3) = [-2;-0.6;   0]; d(:,3) = [ 0.5; 0.866;  0];
        r(:,4) = [-2; 0.6;   0]; d(:,4) = [ 0.5;-0.866;  0];
        r(:,5) = [ 2;-0.6; 0.6]; d(:,5) = [-0.707;0;-0.707];
        r(:,6) = [ 2; 0.6; 0.6]; d(:,6) = [-0.707;0;-0.707];
        r(:,7) = [-2;-0.6; 0.6]; d(:,7) = [ 0.707;0;-0.707];
        r(:,8) = [-2; 0.6; 0.6]; d(:,8) = [ 0.707;0;-0.707];
        r(:,9) = [ 2;-0.6;-0.6]; d(:,9) = [-0.707;0; 0.707];
        r(:,10) = [ 2;0.6;-0.6]; d(:,10) = [-0.707;0;0.707];
        r(:,11) = [-2;-0.6;-0.6]; d(:,11) = [0.707;0;0.707];
        r(:,12) = [-2; 0.6;-0.6]; d(:,12) = [0.707;0;0.707];
        B_all = zeros(6, 12);
        for i = 1:12
            B_all(1:3, i) = d(:,i);
            B_all(4:6, i) = cross(r(:,i), d(:,i));
        end
    end
end
