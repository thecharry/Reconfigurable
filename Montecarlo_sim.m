%% 蒙特卡洛打靶仿真结果对比
function Montecarlo_sim(params, B_opt)

    mc_cfg.N_trial = 5;                  % 每种故障数量打靶次数，汇报够用可取5~10
    mc_cfg.T_sim = 2000;
    mc_cfg.dt = 0.005;

    mc_cfg.pos_tol = 0.5;                % 最终位置误差阈值，单位 m
    mc_cfg.att_tol = deg2rad(2);         % 最终姿态误差阈值，单位 rad

    % 打靶初始条件
    mc_cfg.r0 = [0; 15; 55];
    mc_cfg.euler0 = deg2rad([0; 15; 55]);

    % 目标位置随机范围
    mc_cfg.rt_center = [5; 25; 35];
    mc_cfg.rt_range  = [3; 3; 3];

    % 目标姿态随机范围，单位 deg
    mc_cfg.eulert_center_deg = [5; 25; 35];
    mc_cfg.eulert_range_deg  = [3; 3; 3];

    % 为了验证故障构型本身，建议故障从任务开始就存在
    mc_cfg.fault_time = 0;

    % 固定随机种子，保证结果可复现
    mc_cfg.rng_seed = 1;

    MC_orig = MonteCarlo_Unreconfig_Verify(params, params.B_all, mc_cfg, ...
        '原布局不可重构组合Monte Carlo打靶验证');

    MC_opt = MonteCarlo_Unreconfig_Verify(params, B_opt, mc_cfg, ...
        '优化布局不可重构组合Monte Carlo打靶验证');
    function MC_Result = MonteCarlo_Unreconfig_Verify(params, B, mc_cfg, title_str)
    % ============================================================
    % 每种故障数量选一个 Jc 判定不可重构的故障组合，进行 Monte Carlo 打靶验证
    %
    % 判据：
    %   Jc_F > 0 且 Jc_T > 0 认为可重构；
    %   否则认为不可重构。
    %
    % Monte Carlo 成功判据：
    %   最终位置误差 < mc_cfg.pos_tol
    %   且最终姿态误差 < mc_cfg.att_tol
    %
    % 输出：
    %   MC_Result: table
    % ============================================================

        rng(mc_cfg.rng_seed);

        N_thr = params.Num;

        Fault_Num = zeros(N_thr, 1);
        Fault_Set = strings(N_thr, 1);
        MC_Count = zeros(N_thr, 1);
        Success_Count = zeros(N_thr, 1);
        Fail_Count = zeros(N_thr, 1);
        Success_Ratio = zeros(N_thr, 1);
        Mean_Final_PosErr = nan(N_thr, 1);
        Mean_Final_AttErr_deg = nan(N_thr, 1);
        Verdict = strings(N_thr, 1);

        fprintf('\n==============================================================\n');
        fprintf('%s\n', title_str);
        fprintf('==============================================================\n');

        for k_fault = 1:N_thr

            Fault_Num(k_fault) = k_fault;

            % 1. 找到该故障数量下的一个不可重构组合
            [found_case, faulty_set] = Find_One_Unreconfig_Case_By_Jc(params, B, k_fault);
            [found_case1, faulty_set1] = Find_One_Reconfig_Case_By_Jc(params, B, k_fault);

            if ~found_case
                Fault_Set(k_fault) = "无";
                MC_Count(k_fault) = 0;
                Success_Count(k_fault) = 0;
                Fail_Count(k_fault) = 0;
                Success_Ratio(k_fault) = 0;
                Verdict(k_fault) = "该故障数量下无不可重构组合";

                fprintf('故障数 %2d: 未发现不可重构组合，跳过Monte Carlo验证。\n', k_fault);
                continue;
            end

            Fault_Set(k_fault) = mat2str(faulty_set);

            final_pos_err_list = zeros(mc_cfg.N_trial, 1);
            final_att_err_list = zeros(mc_cfg.N_trial, 1);
            success_list = false(mc_cfg.N_trial, 1);

            fprintf('\n故障数 %2d，选取不可重构组合：%s\n', k_fault, mat2str(faulty_set));

            for trial = 1:mc_cfg.N_trial

                % 2. 随机生成一个打靶目标
                sim_cfg = Build_MC_Target_Config(mc_cfg, faulty_set);

                % 3. 闭环仿真
                log_mc = Closedloop_sim(params, B, sim_cfg);

                % 4. 计算是否打靶成功
                [success, final_pos_err, final_att_err] = Evaluate_MC_Final_Error(log_mc, mc_cfg);

                success_list(trial) = success;
                final_pos_err_list(trial) = final_pos_err;
                final_att_err_list(trial) = final_att_err;

                fprintf('  Trial %02d: pos_err = %.4f m, att_err = %.4f deg, result = %s\n', ...
                    trial, final_pos_err, rad2deg(final_att_err), pass_fail_str(success));
            end

            success_count = sum(success_list);
            fail_count = mc_cfg.N_trial - success_count;
            success_ratio = success_count / mc_cfg.N_trial * 100;

            MC_Count(k_fault) = mc_cfg.N_trial;
            Success_Count(k_fault) = success_count;
            Fail_Count(k_fault) = fail_count;
            Success_Ratio(k_fault) = success_ratio;
            Mean_Final_PosErr(k_fault) = mean(final_pos_err_list);
            Mean_Final_AttErr_deg(k_fault) = mean(rad2deg(final_att_err_list));

            if success_count == 0
                Verdict(k_fault) = "验证支持不可重构";
            else
                Verdict(k_fault) = "存在可达任务，Jc判定偏保守";
            end

            fprintf('  统计结果: 成功 %d / %d，成功率 %.2f%%，结论：%s\n', ...
                success_count, mc_cfg.N_trial, success_ratio, Verdict(k_fault));
        end

        MC_Result = table(Fault_Num, Fault_Set, MC_Count, Success_Count, Fail_Count, ...
            Success_Ratio, Mean_Final_PosErr, Mean_Final_AttErr_deg, Verdict, ...
            'VariableNames', {'故障数量', '验证故障组合', '打靶次数', '成功次数', '失败次数', ...
                            '成功率_percent', '平均最终位置误差_m', '平均最终姿态误差_deg', '验证结论'});

        disp(MC_Result);

        Draw_MC_Verification_Table(MC_Result, title_str);
        Plot_MC_Verification_Bar(MC_Result, title_str);

        function s = pass_fail_str(success)
            if success
                s = '成功';
            else
                s = '失败';
            end
        end
    end
    function [found_case, faulty_set] = Find_One_Reconfig_Case_By_Jc(params, B, k_fault)

        fault_combs = nchoosek(1:params.Num, k_fault);

        found_case = false;
        faulty_set = [];

        Matrix_conf = params.F_max * B;

        for i = 1:size(fault_combs, 1)
            candidate = fault_combs(i, :);
            healthy_idx = setdiff(1:params.Num, candidate);

            Matrix_conf_F = Matrix_conf(1:3, healthy_idx);
            Matrix_conf_T = Matrix_conf(4:6, healthy_idx);

            [Jc_F, ~] = Capability_For_MC(Matrix_conf_F);
            [Jc_T, ~] = Capability_For_MC(Matrix_conf_T);

            is_reconfig = (Jc_F > 1e-10) && (Jc_T > 1e-10);

            if is_reconfig
                found_case = true;
                faulty_set = candidate;
                return;
            end
        end
    end
    function [found_case, faulty_set] = Find_One_Unreconfig_Case_By_Jc(params, B, k_fault)
    % ============================================================
    % 在指定故障数量 k_fault 下，寻找一个 Jc 判定不可重构的组合
    %
    % 判据：
    %   Jc_F > 0 且 Jc_T > 0，则可重构；
    %   否则不可重构。
    % ============================================================

        fault_combs = nchoosek(1:params.Num, k_fault);

        found_case = false;
        faulty_set = [];

        Matrix_conf = params.F_max * B;

        for i = 1:size(fault_combs, 1)
            candidate = fault_combs(i, :);

            healthy_idx = setdiff(1:params.Num, candidate);

            Matrix_conf_F = Matrix_conf(1:3, healthy_idx);
            Matrix_conf_T = Matrix_conf(4:6, healthy_idx);

            [Jc_F, ~] = Capability_For_MC(Matrix_conf_F);
            [Jc_T, ~] = Capability_For_MC(Matrix_conf_T);

            is_reconfig = (Jc_F > 1e-10) && (Jc_T > 1e-10);

            if ~is_reconfig
                found_case = true;
                faulty_set = candidate;
                return;
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
    function Draw_MC_Verification_Table(T_result, table_title)
    % ============================================================
    % 绘制 Monte Carlo 打靶验证表
    % ============================================================

        data = table2cell(T_result);

        colNames = {'故障数量', '验证故障组合', '打靶次数', '成功次数', ...
                    '失败次数', '成功率(%)', '平均位置误差(m)', ...
                    '平均姿态误差(deg)', '验证结论'};

        nRow = size(data, 1);

        figure('Name', table_title, 'Color', 'w', 'Position', [100, 100, 1350, 650]);

        ax = axes;
        axis(ax, 'off');
        hold(ax, 'on');

        text(0.5, 0.96, table_title, ...
            'HorizontalAlignment', 'center', ...
            'FontSize', 15, ...
            'FontWeight', 'bold');

        x0 = 0.03;
        x1 = 0.97;
        y_top = 0.88;
        y_bottom = 0.06;

        colWidth = [0.08, 0.16, 0.08, 0.08, 0.08, 0.10, 0.13, 0.13, 0.16];
        colWidth = colWidth / sum(colWidth) * (x1 - x0);

        colX = x0 + [0, cumsum(colWidth)];
        colCenter = colX(1:end-1) + colWidth / 2;

        rowH = (y_top - y_bottom) / (nRow + 1);

        line([x0, x1], [y_top, y_top], 'Color', 'k', 'LineWidth', 1.8);
        line([x0, x1], [y_top-rowH, y_top-rowH], 'Color', 'k', 'LineWidth', 1.2);
        line([x0, x1], [y_bottom, y_bottom], 'Color', 'k', 'LineWidth', 1.8);

        y_header = y_top - rowH / 2;

        for j = 1:length(colNames)
            text(colCenter(j), y_header, colNames{j}, ...
                'HorizontalAlignment', 'center', ...
                'FontSize', 10, ...
                'FontWeight', 'bold');
        end

        for i = 1:nRow
            y = y_top - rowH * (i + 0.5);

            text(colCenter(1), y, sprintf('%d', data{i,1}), ...
                'HorizontalAlignment', 'center', 'FontSize', 10);

            text(colCenter(2), y, char(data{i,2}), ...
                'HorizontalAlignment', 'center', 'FontSize', 10);

            text(colCenter(3), y, sprintf('%d', data{i,3}), ...
                'HorizontalAlignment', 'center', 'FontSize', 10);

            text(colCenter(4), y, sprintf('%d', data{i,4}), ...
                'HorizontalAlignment', 'center', 'FontSize', 10);

            text(colCenter(5), y, sprintf('%d', data{i,5}), ...
                'HorizontalAlignment', 'center', 'FontSize', 10);

            text(colCenter(6), y, sprintf('%.2f', data{i,6}), ...
                'HorizontalAlignment', 'center', 'FontSize', 10);

            text(colCenter(7), y, sprintf('%.4f', data{i,7}), ...
                'HorizontalAlignment', 'center', 'FontSize', 10);

            text(colCenter(8), y, sprintf('%.4f', data{i,8}), ...
                'HorizontalAlignment', 'center', 'FontSize', 10);

            text(colCenter(9), y, char(data{i,9}), ...
                'HorizontalAlignment', 'center', 'FontSize', 10);
        end

        xlim([0, 1]);
        ylim([0, 1]);
    end
    function Plot_MC_Verification_Bar(T_result, title_str)
    % ============================================================
    % 绘制 Monte Carlo 成功率统计图
    % ============================================================

        figure('Name', [title_str, ' 成功率统计'], 'Color', 'w');

        x = T_result{:, 1};   % 故障数量
        y = T_result{:, 6};   % 成功率_percent

        bar(x, y);

        grid on;
        xlabel('故障数量');
        ylabel('Monte Carlo 打靶成功率 (%)');
        title([title_str, '：不同故障数量下打靶成功率']);

        xticks(x);
        ylim([0, 100]);
    end
end