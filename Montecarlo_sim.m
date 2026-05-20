%% 蒙特卡洛打靶仿真结果对比
function MC = Montecarlo_sim(params, B_opt, mc_cfg)
% ============================================================
% 对原布局和优化布局的故障组合进行 Monte Carlo 打靶验证。
%
% 与旧版本相比：
%   旧版本：每种故障数量只选 1 个 Jc 判定不可重构组合。
%   当前默认：每种故障数量分别选 1 个可重构组合和 1 个不可重构组合。
%   如需全组合验证，可设置 mc_cfg.case_selection = "all"。
%
% 输出：
%   MC.orig.Detail / MC.orig.Summary : 原布局明细与汇总结果
%   MC.opt.Detail  / MC.opt.Summary  : 优化布局明细与汇总结果
% ============================================================

    default_cfg = Default_MC_Config(params);
    if nargin < 3 || isempty(mc_cfg)
        mc_cfg = default_cfg;
    else
        mc_cfg = Merge_MC_Config(default_cfg, mc_cfg);
    end

    mc_cfg.fault_count_list = unique(mc_cfg.fault_count_list(:)');
    mc_cfg.fault_count_list = mc_cfg.fault_count_list( ...
        mc_cfg.fault_count_list >= 0 & mc_cfg.fault_count_list <= params.Num & ...
        floor(mc_cfg.fault_count_list) == mc_cfg.fault_count_list);

    if isempty(mc_cfg.fault_count_list)
        error('mc_cfg.fault_count_list 必须包含 0 到 params.Num 之间的故障数量。');
    end

    MC.config = mc_cfg;

    MC.orig = MonteCarlo_Cases_Verify(params, params.B_all, mc_cfg, ...
        '原布局Monte Carlo打靶验证');

    MC.opt = MonteCarlo_Cases_Verify(params, B_opt, mc_cfg, ...
        '优化布局Monte Carlo打靶验证');

    if nargout == 0
        assignin('base', 'MC_Result', MC);
        fprintf('\nMonte Carlo 结果已写入工作区变量 MC_Result。\n');
    end

    function cfg = Default_MC_Config(params)
        cfg.N_trial = 2;                  % 每个故障组合的打靶次数
        cfg.T_sim = 2000;
        cfg.dt = 0.005;

        cfg.pos_tol = 0.5;                % 最终位置误差阈值，单位 m
        cfg.att_tol = deg2rad(2);         % 最终姿态误差阈值，单位 rad

        % 打靶初始条件
        cfg.r0 = [0; 15; 55];
        cfg.euler0 = deg2rad([0; 15; 55]);

        % 目标位置随机范围
        cfg.rt_center = [5; 25; 35];
        cfg.rt_range  = [3; 3; 3];

        % 目标姿态随机范围，单位 deg
        cfg.eulert_center_deg = [5; 25; 35];
        cfg.eulert_range_deg  = [3; 3; 3];

        % 为了验证故障构型本身，默认故障从任务开始就存在
        cfg.fault_time = 0;

        % 默认验证全部非空故障数量；若想包含标况，可设置为 0:params.Num
        cfg.fault_count_list = 1:params.Num;

        % 可选值：
        %   "one_per_status" : 每种故障数量分别选 1 个可重构组合和 1 个不可重构组合
        %   "all"            : 枚举所有组合
        cfg.case_selection = "one_per_status";

        % 可选值：
        %   "all"          : 可重构和不可重构组合都验证
        %   "reconfig"     : 只验证 Jc 判定可重构组合
        %   "unreconfig"   : 只验证 Jc 判定不可重构组合
        cfg.status_filter = "all";

        % 可选值：
        %   "split"    : 只使用旧版 Jc_F 和 Jc_T 分开判据
        %   "joint_6d" : 在 Jc_F/Jc_T 基础上增加归一化六维 Jc_6D 判据
        cfg.reconfig_metric = "joint_6d";
        cfg.jc_ratio_min = 0.0001;          % 相对标况能力至少保留 5%
        cfg.jc_eps = 1e-10;

        % 可选值：
        %   "axis_split" : 保持旧版力/力矩分开分配
        %   "joint_6d"   : 闭环仿真时使用六维联合分配
        cfg.alloc_mode = "axis_split";

        % 固定随机种子，保证原布局和优化布局使用相同打靶目标序列
        cfg.rng_seed = 1;

        % Monte Carlo 仿真输出量较大，默认压制 Closedloop_sim 内部 tic/toc 输出
        cfg.silent_closed_loop = true;
        cfg.verbose_case = true;
        cfg.verbose_trial = false;

        % 图表输出
        cfg.disp_detail = false;
        cfg.disp_summary = true;
        cfg.draw_summary_table = false;
        cfg.draw_success_bar = false;
        cfg.draw_detail_table = false;
    end

    function cfg = Merge_MC_Config(default_cfg, user_cfg)
        cfg = default_cfg;
        names = fieldnames(user_cfg);
        for ii = 1:numel(names)
            cfg.(names{ii}) = user_cfg.(names{ii});
        end
    end

    function MC_Result = MonteCarlo_Cases_Verify(params, B, mc_cfg, title_str)
    % ============================================================
    % 按 mc_cfg.case_selection 选取故障组合，按 Jc 判据分类后逐组合打靶。
    %
    % 判据：
    %   默认要求 Jc_F、Jc_T、归一化 Jc_6D 均不低于标况能力的一定比例；
    %   若 mc_cfg.reconfig_metric = "split"，则退回旧版 Jc_F/Jc_T 判据。
    %
    % Monte Carlo 成功判据：
    %   最终位置误差 < mc_cfg.pos_tol
    %   且最终姿态误差 < mc_cfg.att_tol
    % ============================================================

        rng(mc_cfg.rng_seed);

        N_thr = params.Num;
        Matrix_conf = params.F_max * B;
        nominal_cap = Build_Nominal_Capability(params, Matrix_conf, mc_cfg);
        sim_params = params;
        sim_params.alloc_mode = mc_cfg.alloc_mode;

        total_case_num = Count_Selected_Fault_Cases(params, Matrix_conf, nominal_cap, mc_cfg);
        total_trial_num = total_case_num * mc_cfg.N_trial;

        Fault_Num = zeros(total_case_num, 1);
        Fault_Set = strings(total_case_num, 1);
        Jc_Status = strings(total_case_num, 1);
        Jc_Force = zeros(total_case_num, 1);
        Jc_Torque = zeros(total_case_num, 1);
        Jc_6D = zeros(total_case_num, 1);
        MC_Count = zeros(total_case_num, 1);
        Success_Count = zeros(total_case_num, 1);
        Fail_Count = zeros(total_case_num, 1);
        Success_Ratio = nan(total_case_num, 1);
        Mean_Final_PosErr = nan(total_case_num, 1);
        Mean_Final_AttErr_deg = nan(total_case_num, 1);
        Verdict = strings(total_case_num, 1);

        row = 0;

        fprintf('\n==============================================================\n');
        fprintf('%s\n', title_str);
        fprintf('==============================================================\n');
        fprintf('故障数量范围: [%s]\n', num2str(mc_cfg.fault_count_list));
        fprintf('组合选取模式: %s\n', char(string(mc_cfg.case_selection)));
        fprintf('可重构判据: %s, 相对标况阈值: %.2f%%\n', ...
            char(string(mc_cfg.reconfig_metric)), mc_cfg.jc_ratio_min * 100);
        fprintf('闭环推力分配模式: %s\n', char(string(mc_cfg.alloc_mode)));
        fprintf('实际验证组合数: %d, 每组合打靶次数: %d, 总打靶次数: %d\n', ...
            total_case_num, mc_cfg.N_trial, total_trial_num);

        for k_fault = mc_cfg.fault_count_list
            fault_combs = nchoosek(1:N_thr, k_fault);
            candidate_cases = size(fault_combs, 1);
            fault_combs = Select_Fault_Combs(params, Matrix_conf, nominal_cap, fault_combs, mc_cfg);
            cases = size(fault_combs, 1);

            fprintf('\n故障数 %2d: 候选 %d 个组合，实际验证 %d 个组合。\n', ...
                k_fault, candidate_cases, cases);

            for idx = 1:cases
                faulty_set = fault_combs(idx, :);

                [Jc_F, Jc_T, Jc_6D_case, is_reconfig] = Evaluate_Reconfig_By_Jc( ...
                    params, Matrix_conf, nominal_cap, faulty_set, mc_cfg);
                status_str = Reconfig_Status_String(is_reconfig);

                if ~Should_Run_Status(status_str, mc_cfg.status_filter)
                    continue;
                end

                row = row + 1;

                final_pos_err_list = zeros(mc_cfg.N_trial, 1);
                final_att_err_list = zeros(mc_cfg.N_trial, 1);
                success_list = false(mc_cfg.N_trial, 1);

                for trial = 1:mc_cfg.N_trial
                    sim_cfg = Build_MC_Target_Config(mc_cfg, faulty_set);

                    if mc_cfg.silent_closed_loop
                        evalc('log_mc = Closedloop_sim(sim_params, B, sim_cfg);');
                    else
                        log_mc = Closedloop_sim(sim_params, B, sim_cfg);
                    end

                    [success, final_pos_err, final_att_err] = ...
                        Evaluate_MC_Final_Error(log_mc, mc_cfg);

                    success_list(trial) = success;
                    final_pos_err_list(trial) = final_pos_err;
                    final_att_err_list(trial) = final_att_err;

                    if mc_cfg.verbose_trial
                        fprintf('  故障数 %2d 组合 %s Trial %02d: pos_err = %.4f m, att_err = %.4f deg, result = %s\n', ...
                            k_fault, mat2str(faulty_set), trial, final_pos_err, ...
                            rad2deg(final_att_err), pass_fail_str(success));
                    end
                end

                success_count = sum(success_list);
                fail_count = mc_cfg.N_trial - success_count;

                if mc_cfg.N_trial > 0
                    success_ratio = success_count / mc_cfg.N_trial * 100;
                    mean_pos_err = mean(final_pos_err_list);
                    mean_att_err_deg = mean(rad2deg(final_att_err_list));
                else
                    success_ratio = NaN;
                    mean_pos_err = NaN;
                    mean_att_err_deg = NaN;
                end

                Fault_Num(row) = k_fault;
                Fault_Set(row) = mat2str(faulty_set);
                Jc_Status(row) = status_str;
                Jc_Force(row) = Jc_F;
                Jc_Torque(row) = Jc_T;
                Jc_6D(row) = Jc_6D_case;
                MC_Count(row) = mc_cfg.N_trial;
                Success_Count(row) = success_count;
                Fail_Count(row) = fail_count;
                Success_Ratio(row) = success_ratio;
                Mean_Final_PosErr(row) = mean_pos_err;
                Mean_Final_AttErr_deg(row) = mean_att_err_deg;
                Verdict(row) = Case_Verdict(status_str, success_count, mc_cfg.N_trial);

                if mc_cfg.verbose_case
                    fprintf('  [%4d/%4d] 组合 %-32s Jc=%s, 成功 %d/%d, 成功率 %.2f%%\n', ...
                        idx, cases, mat2str(faulty_set), char(status_str), ...
                        success_count, mc_cfg.N_trial, success_ratio);
                end
            end
        end

        Fault_Num = Fault_Num(1:row);
        Fault_Set = Fault_Set(1:row);
        Jc_Status = Jc_Status(1:row);
        Jc_Force = Jc_Force(1:row);
        Jc_Torque = Jc_Torque(1:row);
        Jc_6D = Jc_6D(1:row);
        MC_Count = MC_Count(1:row);
        Success_Count = Success_Count(1:row);
        Fail_Count = Fail_Count(1:row);
        Success_Ratio = Success_Ratio(1:row);
        Mean_Final_PosErr = Mean_Final_PosErr(1:row);
        Mean_Final_AttErr_deg = Mean_Final_AttErr_deg(1:row);
        Verdict = Verdict(1:row);

        Detail = table(Fault_Num, Fault_Set, Jc_Status, Jc_Force, Jc_Torque, Jc_6D, ...
            MC_Count, Success_Count, Fail_Count, Success_Ratio, ...
            Mean_Final_PosErr, Mean_Final_AttErr_deg, Verdict, ...
            'VariableNames', {'故障数量', '故障组合', 'Jc判定', 'Jc_Force', 'Jc_Torque', 'Jc_6D', ...
                            '打靶次数', '成功次数', '失败次数', '成功率_percent', ...
                            '平均最终位置误差_m', '平均最终姿态误差_deg', '验证结论'});

        Summary = Build_MC_Summary(Detail, mc_cfg.fault_count_list);

        MC_Result.Detail = Detail;
        MC_Result.Summary = Summary;

        if mc_cfg.disp_detail
            fprintf('\n%s：组合明细结果\n', title_str);
            disp(Detail);
        end

        if mc_cfg.disp_summary
            fprintf('\n%s：按故障数量与Jc判定汇总结果\n', title_str);
            disp(Summary);
        end

        if mc_cfg.draw_summary_table
            Draw_MC_Summary_Table(Summary, [title_str, ' 汇总表']);
        end

        if mc_cfg.draw_success_bar
            Plot_MC_Summary_Bar(Summary, title_str);
        end

        if mc_cfg.draw_detail_table
            Draw_MC_Detail_Table(Detail, [title_str, ' 明细表']);
        end

        function s = pass_fail_str(success)
            if success
                s = '成功';
            else
                s = '失败';
            end
        end
    end

    function nominal_cap = Build_Nominal_Capability(params, Matrix_conf, mc_cfg)
        nominal_cap.scale_6d = max(abs(Matrix_conf), [], 2);
        nominal_cap.scale_6d(nominal_cap.scale_6d < 1e-12) = 1;

        [Jc_F0, Jc_T0, Jc_6D0] = Evaluate_Jc_Values(params, Matrix_conf, [], nominal_cap.scale_6d);

        nominal_cap.Jc_Force = Jc_F0;
        nominal_cap.Jc_Torque = Jc_T0;
        nominal_cap.Jc_6D = Jc_6D0;

        % nominal_cap.Jc_Force_min = max(mc_cfg.jc_eps, mc_cfg.jc_ratio_min * Jc_F0);
        % nominal_cap.Jc_Torque_min = max(mc_cfg.jc_eps, mc_cfg.jc_ratio_min * Jc_T0);
        % nominal_cap.Jc_6D_min = max(mc_cfg.jc_eps, mc_cfg.jc_ratio_min * Jc_6D0);
        nominal_cap.Jc_Force_min = mc_cfg.jc_eps;
        nominal_cap.Jc_Torque_min = mc_cfg.jc_eps;
        nominal_cap.Jc_6D_min = mc_cfg.jc_eps;
    end

    function total_case_num = Count_Selected_Fault_Cases(params, Matrix_conf, nominal_cap, mc_cfg)
        total_case_num = 0;
        for kk = mc_cfg.fault_count_list
            fault_combs = nchoosek(1:params.Num, kk);
            fault_combs = Select_Fault_Combs(params, Matrix_conf, nominal_cap, fault_combs, mc_cfg);
            total_case_num = total_case_num + size(fault_combs, 1);
        end
    end

    function selected_combs = Select_Fault_Combs(params, Matrix_conf, nominal_cap, fault_combs, mc_cfg)
        case_selection = lower(string(mc_cfg.case_selection));

        if case_selection == "all"
            selected_combs = fault_combs;
            return;
        elseif case_selection ~= "one_per_status"
            error('未知的 mc_cfg.case_selection: %s', char(case_selection));
        end

        selected_combs = zeros(0, size(fault_combs, 2));
        found_reconfig = false;
        found_unreconfig = false;

        for ii = 1:size(fault_combs, 1)
            candidate = fault_combs(ii, :);
            [~, ~, ~, is_reconfig] = Evaluate_Reconfig_By_Jc( ...
                params, Matrix_conf, nominal_cap, candidate, mc_cfg);
            status_str = Reconfig_Status_String(is_reconfig);

            if ~Should_Run_Status(status_str, mc_cfg.status_filter)
                continue;
            end

            if status_str == "可重构" && ~found_reconfig
                selected_combs = [selected_combs; candidate]; %#ok<AGROW>
                found_reconfig = true;
            elseif status_str == "不可重构" && ~found_unreconfig
                selected_combs = [selected_combs; candidate]; %#ok<AGROW>
                found_unreconfig = true;
            end

            if Should_Stop_Selecting(found_reconfig, found_unreconfig, mc_cfg.status_filter)
                return;
            end
        end
    end

    function should_stop = Should_Stop_Selecting(found_reconfig, found_unreconfig, status_filter)
        status_filter = lower(string(status_filter));

        if status_filter == "all"
            should_stop = found_reconfig && found_unreconfig;
        elseif status_filter == "reconfig" || status_filter == "可重构"
            should_stop = found_reconfig;
        elseif status_filter == "unreconfig" || status_filter == "nonreconfig" || ...
                status_filter == "non-reconfig" || status_filter == "不可重构"
            should_stop = found_unreconfig;
        else
            error('未知的 mc_cfg.status_filter: %s', char(status_filter));
        end
    end

    function [Jc_F, Jc_T, Jc_6D, is_reconfig] = Evaluate_Reconfig_By_Jc(params, Matrix_conf, nominal_cap, faulty_set, mc_cfg)
        [Jc_F, Jc_T, Jc_6D] = Evaluate_Jc_Values(params, Matrix_conf, faulty_set, nominal_cap.scale_6d);

        split_ok = (Jc_F >= nominal_cap.Jc_Force_min) && ...
                   (Jc_T >= nominal_cap.Jc_Torque_min);

        reconfig_metric = lower(string(mc_cfg.reconfig_metric));

        if reconfig_metric == "split"
            is_reconfig = split_ok;
        elseif reconfig_metric == "joint_6d"
            % is_reconfig = split_ok && (Jc_6D >= nominal_cap.Jc_6D_min);
            is_reconfig = Jc_6D >= nominal_cap.Jc_6D_min;
        else
            error('未知的 mc_cfg.reconfig_metric: %s', char(reconfig_metric));
        end
    end

    function [Jc_F, Jc_T, Jc_6D] = Evaluate_Jc_Values(params, Matrix_conf, faulty_set, scale_6d)
        healthy_idx = setdiff(1:params.Num, faulty_set);

        Matrix_conf_F = Matrix_conf(1:3, healthy_idx);
        Matrix_conf_T = Matrix_conf(4:6, healthy_idx);
        Matrix_conf_6D = Matrix_conf(:, healthy_idx) ./ scale_6d;

        [Jc_F, ~] = Capability_For_MC(Matrix_conf_F);
        [Jc_T, ~] = Capability_For_MC(Matrix_conf_T);
        [Jc_6D, ~] = Capability_For_MC(Matrix_conf_6D);
    end

    function status_str = Reconfig_Status_String(is_reconfig)
        if is_reconfig
            status_str = "可重构";
        else
            status_str = "不可重构";
        end
    end

    function should_run = Should_Run_Status(status_str, status_filter)
        status_filter = lower(string(status_filter));

        if status_filter == "all"
            should_run = true;
        elseif status_filter == "reconfig" || status_filter == "可重构"
            should_run = (status_str == "可重构");
        elseif status_filter == "unreconfig" || status_filter == "nonreconfig" || ...
                status_filter == "non-reconfig" || status_filter == "不可重构"
            should_run = (status_str == "不可重构");
        else
            error('未知的 mc_cfg.status_filter: %s', char(status_filter));
        end
    end

    function verdict = Case_Verdict(status_str, success_count, trial_count)
        if trial_count == 0
            verdict = "未执行打靶";
        elseif status_str == "可重构"
            if success_count > 0
                verdict = "打靶支持可重构";
            else
                verdict = "Jc可重构但本任务未命中";
            end
        else
            if success_count == 0
                verdict = "打靶支持不可重构";
            else
                verdict = "存在可达任务，Jc判定偏保守";
            end
        end
    end

    function sim_cfg = Build_MC_Target_Config(mc_cfg, faulty_set)
    % ============================================================
    % 构造一次 Monte Carlo 打靶仿真配置
    % ============================================================

        sim_cfg.r0 = mc_cfg.r0;
        sim_cfg.v0 = [0; 0; 0];

        % 随机目标位置
        sim_cfg.rt = mc_cfg.rt_center + mc_cfg.rt_range .* (2 * rand(3,1) - 1);

        % 随机目标姿态
        eulert_deg = mc_cfg.eulert_center_deg(:) + mc_cfg.eulert_range_deg(:) .* (2 * rand(3,1) - 1);

        sim_cfg.euler0 = mc_cfg.euler0;
        sim_cfg.eulert = deg2rad(eulert_deg);

        sim_cfg.T_sim = mc_cfg.T_sim;
        sim_cfg.dt = mc_cfg.dt;

        sim_cfg.Kp_pos = 10;
        sim_cfg.Kd_pos = 300;
        sim_cfg.Kp_att = 400 * eye(3);
        sim_cfg.Kd_att = 3200 * eye(3);
        sim_cfg.Ki_att = 20 * eye(3);

        sim_cfg.fault_time = mc_cfg.fault_time;
        sim_cfg.faulty_thrusters_fixed = faulty_set;
    end

    function [success, final_pos_err, final_att_err] = Evaluate_MC_Final_Error(log_mc, mc_cfg)
    % ============================================================
    % Monte Carlo 打靶最终误差判断
    % ============================================================

        r_final = log_mc.Y(1:3, end);
        r_ref   = log_mc.R(:, end);

        euler_final = log_mc.Y_euler(:, end);
        euler_ref   = log_mc.E(:, end);

        final_pos_err = norm(r_final - r_ref);

        att_err = euler_final - euler_ref;
        att_err = mod(att_err + pi, 2*pi) - pi;
        final_att_err = norm(att_err);

        if any(isnan(r_final)) || any(isnan(euler_final)) || any(isinf(r_final)) || any(isinf(euler_final))
            success = false;
            return;
        end

        success = (final_pos_err < mc_cfg.pos_tol) && (final_att_err < mc_cfg.att_tol);
    end

    function [umin, umax] = Capability_For_MC(Matrix_sub)
    % ============================================================
    % 基于可达域几何特性的控制能力指标 Jc
    % ============================================================

        N_sub = size(Matrix_sub, 2);

        if N_sub == 0
            umin = 0;
            umax = 0;
            return;
        end

        c = 0.5 * ones(N_sub, 1);
        G = 0.5 * eye(N_sub);

        try
            Z = zonotope(Matrix_sub * c, Matrix_sub * G);
            P = polytope(Z);

            H = P.A;
            w = P.b;

            if isempty(H) || isempty(w)
                umin = 0;
                umax = 0;
                return;
            end

            if any(w < -1e-6)
                umin = 0;
                umax = 0;
            else
                d = w ./ (vecnorm(H, 2, 2) + 1e-12);
                umin = min(d);
                umax = max(d);
            end

        catch
            umin = 0;
            umax = 0;
        end
    end

    function Summary = Build_MC_Summary(Detail, fault_count_list)
        status_order = ["可重构", "不可重构"];
        n_row = numel(fault_count_list) * numel(status_order);

        Fault_Num_S = zeros(n_row, 1);
        Jc_Status_S = strings(n_row, 1);
        Case_Count = zeros(n_row, 1);
        MC_Count_S = zeros(n_row, 1);
        Success_Count_S = zeros(n_row, 1);
        Fail_Count_S = zeros(n_row, 1);
        Success_Ratio_S = nan(n_row, 1);
        Mean_Final_PosErr_S = nan(n_row, 1);
        Mean_Final_AttErr_deg_S = nan(n_row, 1);
        Verdict_S = strings(n_row, 1);

        row_s = 0;

        for kk = fault_count_list
            for ss = 1:numel(status_order)
                status_str = status_order(ss);
                row_s = row_s + 1;

                mask = Detail{:, 1} == kk & string(Detail{:, 3}) == status_str;

                case_count = sum(mask);
                mc_count = sum(Detail{mask, 7});
                success_count = sum(Detail{mask, 8});
                fail_count = sum(Detail{mask, 9});

                Fault_Num_S(row_s) = kk;
                Jc_Status_S(row_s) = status_str;
                Case_Count(row_s) = case_count;
                MC_Count_S(row_s) = mc_count;
                Success_Count_S(row_s) = success_count;
                Fail_Count_S(row_s) = fail_count;

                if mc_count > 0
                    Success_Ratio_S(row_s) = success_count / mc_count * 100;
                    Mean_Final_PosErr_S(row_s) = mean(Detail{mask, 11}, 'omitnan');
                    Mean_Final_AttErr_deg_S(row_s) = mean(Detail{mask, 12}, 'omitnan');
                end

                Verdict_S(row_s) = Summary_Verdict(status_str, case_count, success_count, mc_count);
            end
        end

        % Summary = table(Fault_Num_S, Jc_Status_S, Case_Count, MC_Count_S, ...
        %     Success_Count_S, Fail_Count_S, Success_Ratio_S, ...
        %     Mean_Final_PosErr_S, Mean_Final_AttErr_deg_S, Verdict_S, ...
        %     'VariableNames', {'故障数量', 'Jc判定', '组合数量', '打靶次数', ...
        %                     '成功次数', '失败次数', '成功率_percent', ...
        %                     '平均最终位置误差_m', '平均最终姿态误差_deg', '验证结论'});
        Summary = table(Fault_Num_S, Jc_Status_S, MC_Count_S, ...
            Success_Ratio_S, ...
            Mean_Final_PosErr_S, Mean_Final_AttErr_deg_S, ...
            'VariableNames', {'故障数量', '是否可重构', '打靶次数', ...
                            '成功率', ...
                            '平均最终位置误差', '平均最终姿态误差'});
    end

    function verdict = Summary_Verdict(status_str, case_count, success_count, mc_count)
        if case_count == 0
            verdict = "该类组合不存在";
        elseif mc_count == 0
            verdict = "未执行打靶";
        elseif status_str == "可重构"
            if success_count > 0
                verdict = "总体支持可重构";
            else
                verdict = "可重构组合未命中当前任务集";
            end
        else
            if success_count == 0
                verdict = "总体支持不可重构";
            else
                verdict = "存在可达任务，Jc判定偏保守";
            end
        end
    end

    function Draw_MC_Summary_Table(T_result, table_title)
    % ============================================================
    % 绘制 Monte Carlo 汇总表
    % ============================================================

        data = Table_Cell_For_UITable(T_result);
        colNames = {'故障数量', 'Jc判定', '组合数量', '打靶次数', '成功次数', ...
                    '失败次数', '成功率(%)', '平均位置误差(m)', ...
                    '平均姿态误差(deg)', '验证结论'};

        fig = figure('Name', table_title, 'Color', 'w', 'Position', [100, 100, 1450, 720]);

        uicontrol(fig, 'Style', 'text', 'String', table_title, ...
            'Units', 'normalized', 'Position', [0.02, 0.92, 0.96, 0.06], ...
            'BackgroundColor', 'w', 'FontSize', 15, 'FontWeight', 'bold');

        uitable(fig, 'Data', data, 'ColumnName', colNames, ...
            'Units', 'normalized', 'Position', [0.02, 0.04, 0.96, 0.86]);
    end

    function Draw_MC_Detail_Table(T_result, table_title)
    % ============================================================
    % 绘制 Monte Carlo 明细表。全组合行数很多，默认不启用。
    % ============================================================

        data = Table_Cell_For_UITable(T_result);
        colNames = {'故障数量', '故障组合', 'Jc判定', 'Jc_Force', 'Jc_Torque', 'Jc_6D', ...
                    '打靶次数', '成功次数', '失败次数', '成功率(%)', ...
                    '平均位置误差(m)', '平均姿态误差(deg)', '验证结论'};

        fig = figure('Name', table_title, 'Color', 'w', 'Position', [80, 80, 1600, 820]);

        uicontrol(fig, 'Style', 'text', 'String', table_title, ...
            'Units', 'normalized', 'Position', [0.02, 0.92, 0.96, 0.06], ...
            'BackgroundColor', 'w', 'FontSize', 15, 'FontWeight', 'bold');

        uitable(fig, 'Data', data, 'ColumnName', colNames, ...
            'Units', 'normalized', 'Position', [0.02, 0.04, 0.96, 0.86]);
    end

    function Plot_MC_Summary_Bar(T_result, title_str)
    % ============================================================
    % 绘制不同故障数量下可重构/不可重构组合 Monte Carlo 成功率
    % ============================================================

        fault_nums = unique(T_result{:, 1})';
        status_order = ["可重构", "不可重构"];
        y = nan(numel(fault_nums), numel(status_order));

        for ii = 1:numel(fault_nums)
            for jj = 1:numel(status_order)
                mask = T_result{:, 1} == fault_nums(ii) & string(T_result{:, 2}) == status_order(jj);
                if any(mask)
                    y(ii, jj) = T_result{find(mask, 1), 7};
                end
            end
        end

        y_plot = y;
        y_plot(isnan(y_plot)) = 0;

        figure('Name', [title_str, ' 成功率统计'], 'Color', 'w');
        bar(fault_nums, y_plot, 'grouped');

        grid on;
        xlabel('故障数量');
        ylabel('Monte Carlo 打靶成功率 (%)');
        title([title_str, '：不同故障数量与Jc判定下打靶成功率']);
        legend(cellstr(status_order), 'Location', 'best');
        xticks(fault_nums);
        ylim([0, 100]);
    end

    function data = Table_Cell_For_UITable(T_result)
        data = table2cell(T_result);

        for ii = 1:numel(data)
            if isstring(data{ii})
                data{ii} = char(data{ii});
            end
        end
    end
end
