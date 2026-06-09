%% 仿真系统结果
function Plot_results(log_orig, log_opt, params, B_opt, r_opt, layout_set)
    if nargin < 6 || isempty(layout_set)
        layout_set = struct('name', {'原布局', '优化布局'}, ...
                            'B', {params.B_all, B_opt}, ...
                            'r', {params.r_all, r_opt});
    else
        layout_set = layout_set(:)';
    end

    plot_1();% 推力器布局优化前后对比
    plot_2();% 评价指标优化前后对比
    plot_3();% 闭环仿真结果对比
    plot_4();% 推力器控制脉宽对比
    plot_5();% 推力器分配策略输出
    plot_6();% 诊断结果输出
    plot_7();% 不同数量故障下可重构性判定表
    plot_8();% 单推力器故障综合评价明细表

    %% 推力器布局优化前后对比
    function plot_1()
        figure('Name', '推力器布局优化前后对比', 'Color','w');
        subplot(1,2,1);Plot_place(params.r_all, params.B_all, '原布局示意图');
        subplot(1,2,2);Plot_place(r_opt, B_opt, '优化布局示意图');
        function Plot_place(r, B, title_str)
            hold on; grid on;
            Lx = 2; Ly = 0.6; Lz = 0.6;
            vert = [-Lx -Ly -Lz; Lx -Ly -Lz; Lx Ly -Lz; -Lx Ly -Lz;
                    -Lx -Ly  Lz; Lx -Ly  Lz; Lx Ly  Lz; -Lx Ly  Lz];
            fac = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
            patch('Vertices',vert,'Faces',fac,'FaceColor',[0.8 0.8 0.8], ...
                'FaceAlpha',0.18,'EdgeColor',[0.6 0.6 0.6]);
            % 推力器位置和方向
            for j = 1:params.Num
                dx = B(1,j); dy = B(2,j); dz = B(3,j);
                if r(1,j) > 0
                    beta_rad = acos(-dx);
                else              
                    beta_rad = acos(dx);
                end
                alpha_rad = atan2(dz, dy);
                beta_deg = rad2deg(beta_rad);
                alpha_deg = mod(rad2deg(alpha_rad), 360);
                label_str = sprintf(' %d(\\alpha:%.0f^\\circ, \\beta:%.0f^\\circ)', j, alpha_deg, beta_deg);
                text(r(1,j), r(2,j), r(3,j) + 0.15, label_str, 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k');% 推力器编号
            end
            % 正常推力器用红色实线
            healthy_idx = setdiff(1:params.Num, log_orig.faulty_thrusters);
            plot3(r(1,healthy_idx), r(2,healthy_idx), r(3,healthy_idx), ...
                'o','MarkerSize',7,'MarkerFaceColor',[0.2 0.7 1],'MarkerEdgeColor','k');
            quiver3(r(1,healthy_idx), r(2,healthy_idx), r(3,healthy_idx), ...
                    B(1,healthy_idx), B(2,healthy_idx), B(3,healthy_idx), ...
                    0.45,'r','LineWidth',1.4);
            % 故障推力器用灰色虚线
            if ~isempty(log_orig.faulty_thrusters)
                plot3(r(1, log_orig.faulty_thrusters), r(2, log_orig.faulty_thrusters), r(3, log_orig.faulty_thrusters), ...
                    'o','MarkerSize',7,'MarkerFaceColor',[0.8 0.8 0.8],'MarkerEdgeColor','k');
                quiver3(r(1, log_orig.faulty_thrusters), r(2, log_orig.faulty_thrusters), r(3, log_orig.faulty_thrusters), ...
                        B(1, log_orig.faulty_thrusters), B(2, log_orig.faulty_thrusters), B(3, log_orig.faulty_thrusters), ...
                        0.45, 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5, 'LineStyle', '--');
            end
            title(title_str);
            xlabel('X'); ylabel('Y'); zlabel('Z');
            xlim([-3, 3]); ylim([-1.2, 1.2]); zlim([-1.2, 1.2]);axis equal; grid on;
            set(gca,'XDir','reverse','ZDir','reverse');
            view(3);
        end
    end

    %% 评价指标优化前后对比
    function plot_2()
        figure('Name', '重构评价指标优化前后对比', 'Color','w');
        Matrix_orig = params.F_max * params.B_all;
        Matrix_opt  = params.F_max * B_opt;
        healthy_idx = setdiff(1:params.Num, log_orig.faulty_thrusters);
        M_orig = Matrix_orig(:, healthy_idx);
        M_opt  = Matrix_opt(:, healthy_idx);
        scale_orig = max(abs(Matrix_orig), [], 2);
        scale_orig(scale_orig < 1e-12) = 1;
        scale_opt = max(abs(Matrix_opt), [], 2);
        scale_opt(scale_opt < 1e-12) = 1;
        % 连续域的Zonotope包络
        N_h = length(healthy_idx);
        c = 0.5 * ones(N_h,1);
        G = 0.5 * eye(N_h);
        Z_orig = zonotope(M_orig*c, M_orig*G);
        Z_opt  = zonotope(M_opt*c,  M_opt*G);
        Z1_orig = zonotope((M_orig ./ scale_orig)*c, (M_orig ./ scale_orig)*G);
        Z1_opt  = zonotope((M_opt  ./ scale_opt)*c,  (M_opt  ./ scale_opt)*G);
        % 离散域的全状态点云
        tau = dec2bin(0:2^N_h-1) - '0';
        Pts_orig = M_orig * tau';
        Pts_opt  = M_opt  * tau';  
        Pts1_orig = (M_orig ./ scale_orig) * tau';
        Pts1_opt  = (M_opt  ./ scale_opt)  * tau';
        warning('off','all');
        % 力空间对比
        subplot(3,2,1);hold on;grid on;
        plot(Z_orig,[1 2 3],'FaceColor', [0.7 1 0.7], 'FaceAlpha', 0.15, 'EdgeColor', 'g');
        plot(Z_opt,[1 2 3], 'FaceColor', [0.9 0.7 0.7], 'FaceAlpha', 0.3, 'EdgeColor', 'r');
        plot3(Pts_orig(1,:), Pts_orig(2,:), Pts_orig(3,:), '.', 'Color', [0, 0.8, 0], 'MarkerSize', 4);
        plot3(Pts_opt(1,:), Pts_opt(2,:), Pts_opt(3,:), '.', 'Color', [0.8, 0, 0], 'MarkerSize', 4);
        plot3(0, 0, 0, 'ko', 'MarkerSize', 3, 'MarkerFaceColor', 'k');
        title('力空间包络与离散点云');xlabel('Fx (N)'); ylabel('Fy (N)'); zlabel('Fz (N)');
        axis equal;view(3);
        % 力矩空间对比
        subplot(3,2,2);hold on;grid on;
        plot(Z_orig, [4 5 6], 'FaceColor', [0.7 1 0.7], 'FaceAlpha', 0.15, 'EdgeColor', 'g');
        plot(Z_opt, [4 5 6], 'FaceColor', [0.9 0.7 0.7], 'FaceAlpha', 0.3, 'EdgeColor', 'r');
        plot3(Pts_orig(4,:), Pts_orig(5,:), Pts_orig(6,:), '.', 'Color', [0, 0.8, 0], 'MarkerSize', 4);
        plot3(Pts_opt(4,:), Pts_opt(5,:), Pts_opt(6,:), '.', 'Color', [0.8, 0, 0], 'MarkerSize', 4);
        plot3(0, 0, 0, 'ko', 'MarkerSize', 3, 'MarkerFaceColor', 'k');
        title('力矩空间包络与离散点云');xlabel('Mx (N·m)'); ylabel('My (N·m)'); zlabel('Mz (N·m)');
        axis equal;view(3);
        % 力空间对比（归一化）
        subplot(3,2,3); hold on; grid on;
        plot(Z1_orig, [1 2 3], 'FaceColor', [0.7 1 0.7], 'FaceAlpha', 0.15, 'EdgeColor', 'g');
        plot(Z1_opt,  [1 2 3], 'FaceColor', [0.9 0.7 0.7], 'FaceAlpha', 0.3, 'EdgeColor', 'r');
        plot3(Pts1_orig(1,:), Pts1_orig(2,:), Pts1_orig(3,:), '.', 'Color', [0, 0.8, 0], 'MarkerSize', 4);
        plot3(Pts1_opt(1,:),  Pts1_opt(2,:),  Pts1_opt(3,:),  '.', 'Color', [0.8, 0, 0], 'MarkerSize', 4);
        plot3(0, 0, 0, 'ko', 'MarkerSize', 3, 'MarkerFaceColor', 'k');
        title('归一化力空间包络与离散点云'); xlabel('Fx'); ylabel('Fy'); zlabel('Fz');
        axis equal; view(3);
        % 力矩空间对比（归一化）
        subplot(3,2,4); hold on; grid on;
        plot(Z1_orig, [4 5 6], 'FaceColor', [0.7 1 0.7], 'FaceAlpha', 0.15, 'EdgeColor', 'g');
        plot(Z1_opt,  [4 5 6], 'FaceColor', [0.9 0.7 0.7], 'FaceAlpha', 0.3, 'EdgeColor', 'r');
        plot3(Pts1_orig(4,:), Pts1_orig(5,:), Pts1_orig(6,:), '.', 'Color', [0, 0.8, 0], 'MarkerSize', 4);
        plot3(Pts1_opt(4,:),  Pts1_opt(5,:),  Pts1_opt(6,:),  '.', 'Color', [0.8, 0, 0], 'MarkerSize', 4);
        plot3(0, 0, 0, 'ko', 'MarkerSize', 3, 'MarkerFaceColor', 'k');
        title('归一化力矩空间包络与离散点云'); xlabel('Mx'); ylabel('My'); zlabel('Mz');
        axis equal; view(3);
        warning('on','all');
        % 6维向量夹角热力图
        M_orig = M_orig ./ (vecnorm(M_orig,2,1) + 1e-12);% 归一化
        M_opt = M_opt ./ (vecnorm(M_opt,2,1) + 1e-12);
        cos_orig = M_orig' * M_orig;
        cos_opt  = M_opt'  * M_opt;
        angle_orig = acos(min(max(cos_orig, -1), 1)) * 180 / pi;
        angle_opt  = acos(min(max(cos_opt,  -1), 1)) * 180 / pi;
        subplot(3,2,5);
        h1 = heatmap(healthy_idx, healthy_idx, angle_orig);
        h1.Title = '原布局向量夹角';h1.XLabel = '推进器编号';h1.YLabel = '推进器编号';
        colormap(h1, jet);
        subplot(3,2,6);
        h2 = heatmap(healthy_idx, healthy_idx, angle_opt);
        h2.Title = '优化布局向量夹角';h2.XLabel = '推进器编号';h2.YLabel = '推进器编号';
        colormap(h2, jet);
    end
    
    %% 闭环位置与姿态响应对比
    function plot_3()
        figure('Name', '闭环位置与姿态响应对比','Color','w');
        subplot(2,2,1); Plot_3Axis(log_orig.Time, log_orig.R, log_orig.Y(1:3,:), log_opt.Y(1:3,:), '位置响应', '(m)', log_orig.faulty_time);
        subplot(2,2,2); Plot_3Axis(log_orig.Time, log_orig.E, log_orig.Y_euler, log_opt.Y_euler, '姿态响应', '(rad)', log_orig.faulty_time);
        function Plot_3Axis(t, ref, y_orig, y_opt, title_str, unit_str, fault_time)
            hold on; grid on;
            colors = lines(3);
            for ii = 1:3
                plot(t, ref(ii,:), '--', 'Color', colors(ii,:), 'LineWidth',0.9);
                plot(t, y_orig(ii,:), '-',  'Color', colors(ii,:), 'LineWidth',1.1);
                plot(t, y_opt(ii,:),  ':',  'Color', colors(ii,:), 'LineWidth',1.6);
            end
            xline(fault_time,'--r');
            title(title_str); xlabel('t(s)'); ylabel(unit_str);
            legend('ref','orig','opt');
        end
        % 误差范数
        pos_err_orig = log_orig.Y(1:3,:) - log_orig.R;
        pos_err_opt  = log_opt.Y(1:3,:)  - log_opt.R;
        att_err_orig = mod((log_orig.Y_euler - log_orig.E) + pi, 2*pi) - pi;
        att_err_opt  = mod((log_opt.Y_euler - log_opt.E) + pi, 2*pi) - pi;
        subplot(2,2,3); hold on; grid on;
        plot(log_orig.Time, vecnorm(pos_err_orig,2,1), 'LineWidth',1.2);
        plot(log_orig.Time, vecnorm(pos_err_opt,2,1), 'LineWidth',1.2);
        xline(log_orig.faulty_time,'--r');
        title('位置误差范数'); xlabel('t(s)'); ylabel('||e_r||(m)'); legend('原布局','优化布局');
        subplot(2,2,4); hold on; grid on;
        plot(log_orig.Time, vecnorm(att_err_orig,2,1), 'LineWidth',1.2);
        plot(log_orig.Time, vecnorm(att_err_opt,2,1), 'LineWidth',1.2);
        xline(log_orig.faulty_time,'--r');
        title('姿态误差范数'); xlabel('t(s)'); ylabel('||e_euler||(rad)'); legend('原布局','优化布局');
    end

    %% 推力器控制脉宽对比
    function plot_4()
        figure('Name', '各推力器脉宽','Color','w');
        for i = 1:params.Num
            subplot(3, 4, i);plot(log_orig.Time, log_orig.Pulse_Widths(i, :));
            title(['推力器 ' num2str(i)]);xlabel('时间(s)');ylabel('脉宽(s)');
            grid on;hold on;
            xline(log_orig.faulty_time, '--r');
            hold off;
        end
        % 总喷气时长对比
        figure('Name','总喷气时长对比','Color','w');
        subplot(3,1,1); hold on; grid on;
        bar(1:2, [log_orig.Total_Pulse, log_opt.Total_Pulse]);xticks(1:2);xticklabels({'原布局','优化布局'});
        ylabel('总喷气时长(s)'); title('全任务总喷气时长对比'); ylim([0, max([log_orig.Total_Pulse, log_opt.Total_Pulse])*1.2 + eps]);
        subplot(3,1,2); hold on; grid on;
        per_orig = sum(log_orig.Pulse_History, 2);
        per_opt  = sum(log_opt.Pulse_History, 2);
        bar(1:params.Num, [per_orig, per_opt], 'grouped');
        xlabel('推力器编号'); ylabel('累计脉宽(s)'); title('各推力器累计脉宽对比');legend('原布局','优化布局');
        subplot(3,1,3); hold on; grid on;
        plot(log_orig.Control_Time, sum(log_orig.Pulse_History,1), 'LineWidth',1.0);
        plot(log_opt.Control_Time, sum(log_opt.Pulse_History,1), 'LineWidth',1.0);
        xline(log_orig.faulty_time,'--r');
        xlabel('时间(s)'); ylabel('当前控制周期总脉宽(s)'); title('控制周期总脉宽变化');legend('原布局','优化布局');
    end
   
    %% 推力器分配策略输出
    function plot_5()
        Print_Thruster_Allocation(params.B_all, log_orig.faulty_thrusters, '原布局');
        Print_Thruster_Allocation(B_opt, log_orig.faulty_thrusters, '优化布局');
        function Print_Thruster_Allocation(B, faulty_thrusters, name_str)
            axes_names = {'X', 'Y', 'Z'};
            fprintf('%s推力器分配策略\n', name_str);
            if isempty(faulty_thrusters)
                fprintf('推力器标况\n');
            else
                fprintf('推力器[%s]故障\n', num2str(faulty_thrusters));
            end
            
            fprintf('--------------------------------------------------------------\n');
            fprintf('【轨道控制推力器分配】\n');
            for i = 1:3
                pos_idx = find(B(i, :) > 1e-3);
                neg_idx = find(B(i, :) < -1e-3);

                pos_idx = setdiff(pos_idx, faulty_thrusters);
                neg_idx = setdiff(neg_idx, faulty_thrusters);

                fprintf('+%s轴: [%s]\n', axes_names{i}, num2str(pos_idx));
                fprintf('-%s轴: [%s]\n', axes_names{i}, num2str(neg_idx));
            end

            fprintf('--------------------------------------------------------------\n');
            fprintf('【姿态控制推力器分配】\n');
            for i = 1:3
                pos_idx = find(B(i+3, :) > 1e-3);
                neg_idx = find(B(i+3, :) < -1e-3);

                pos_idx = setdiff(pos_idx, faulty_thrusters);
                neg_idx = setdiff(neg_idx, faulty_thrusters);

                fprintf('+%s轴: [%s]\n', axes_names{i}, num2str(pos_idx));
                fprintf('-%s轴: [%s]\n', axes_names{i}, num2str(neg_idx));
            end
            fprintf('--------------------------------------------------------------\n');
        end
    end

    %% 诊断结果输出
    function plot_6()
        fprintf('随机故障配置: 实际故障推力器 = [%s], 故障时间 = %.2f s\n', ...
                num2str(log_orig.faulty_thrusters), log_orig.faulty_time);
        fprintf('原布局诊断结果: 诊断故障推力器 = [%s], 诊断时间 = %.2f s, 诊断率 = %d\n', ...
                num2str(log_orig.estimated_faults), log_orig.diagnosis_time, log_orig.diagnosis_success);
        fprintf('优化布局诊断结果: 诊断故障推力器 = [%s], 诊断时间 = %.2f s, 诊断率 = %d\n', ...
                num2str(log_opt.estimated_faults), log_opt.diagnosis_time, log_opt.diagnosis_success);
    end

    %% 不同故障数量下可重构性判定表
    function plot_7()
        fault_nums = 1:params.Num;
        [Eval_grid, ~] = samples_combin(layout_set, fault_nums);
        positive_eps = eps;

        for fault_idx = 1:numel(fault_nums)
            for layout_idx = 1:numel(layout_set)
                Eval = Eval_grid{layout_idx, fault_idx};
                rows = 2:size(Eval.MetricRaw, 1);
                Eval.IsReconfig(rows) = all(Eval.MetricRaw(rows, :) > positive_eps, 2);
                Eval.Status(:) = "不可重构";
                Eval.Status(Eval.IsReconfig) = "可重构";
                Eval_grid{layout_idx, fault_idx} = Eval;
            end
        end

        colNames = {'推力器故障数', '是否可重构', '可重构数量', '不可重构数量', '不可重构占比'};
        for layout_idx = 1:numel(layout_set)
            data = strings(numel(fault_nums), numel(colNames));
            for fault_idx = 1:numel(fault_nums)
                Eval = Eval_grid{layout_idx, fault_idx};
                rows = 2:numel(Eval.FaultSets);
                reconfig_num = sum(Eval.IsReconfig(rows));
                nonreconfig_num = numel(rows) - reconfig_num;

                if nonreconfig_num == 0
                    status = "完全可重构";
                elseif reconfig_num == 0
                    status = "不可重构";
                else
                    status = "部分可重构";
                end

                data(fault_idx, :) = [string(fault_nums(fault_idx)), status, ...
                                      string(reconfig_num), string(nonreconfig_num), ...
                                      string(sprintf('%.2f%%', 100 * nonreconfig_num / numel(rows)))];
            end

            Draw_Three_Line_Table([char(layout_set(layout_idx).name) ':不同数量故障下可重构性判定表'], ...
                                  {}, colNames, data, [0.18, 0.18, 0.18, 0.18, 0.18]);
        end
    end

    %% 单推力器故障综合评价明细表
    function plot_8()
        [Eval_case, Raw] = samples_combin(layout_set, 1);
        Weight = Least_Squares_Combined_Weight(Raw, AHP_Weight(params, 4), Entropy_Weight(Raw));
        positive_eps = eps;

        for layout_idx = 1:numel(layout_set)
            Eval = Eval_case{layout_idx};
            rows = 2:size(Eval.MetricRaw, 1);
            Eval.Score(rows) = Eval.MetricRaw(rows, :) * Weight;
            Eval.Weight = Weight;
            Eval.IsReconfig(rows) = all(Eval.MetricRaw(rows, :) > positive_eps, 2);
            Eval.Status(:) = "不可重构";
            Eval.Status(Eval.IsReconfig) = "可重构";
            Eval_case{layout_idx} = Eval;
        end

        layout_avg = zeros(numel(layout_set), 1);
        avg_text = cell(1, numel(layout_set));
        for layout_idx = 1:numel(layout_set)
            layout_avg(layout_idx) = mean(Eval_case{layout_idx}.Score(2:end));
            avg_text{layout_idx} = sprintf('%s %.4f', char(layout_set(layout_idx).name), layout_avg(layout_idx));
        end

        colNames = {'故障推力器', '状态', '综合能力', '控制能力', '可诊断性', '跟踪性能', '能耗效率'};
        weight_text = sprintf('组合权重: 控制 %.3f, 诊断 %.3f, 跟踪 %.3f, 能耗 %.3f', ...
                              Weight(1), Weight(2), Weight(3), Weight(4));
        for layout_idx = 1:numel(layout_set)
            Eval = Eval_case{layout_idx};
            data = strings(params.Num, numel(colNames));
            row = 0;
            for eval_idx = 2:numel(Eval.FaultSets)
                faulty_idx = Eval.FaultSets{eval_idx};
                if numel(faulty_idx) ~= 1
                    continue;
                end
                row = row + 1;
                data(row, :) = [string(faulty_idx), Eval.Status(eval_idx), ...
                                string(sprintf('%.4f', Eval.Score(eval_idx))), ...
                                string(sprintf('%.4f', Eval.MetricRaw(eval_idx, 1))), ...
                                string(sprintf('%.4f', Eval.MetricRaw(eval_idx, 2))), ...
                                string(sprintf('%.4f', Eval.MetricRaw(eval_idx, 3))), ...
                                string(sprintf('%.4f', Eval.MetricRaw(eval_idx, 4)))];
            end
            data = data(1:row, :);
            Draw_Three_Line_Table([char(layout_set(layout_idx).name) ':单推力器故障综合评价指标明细表'], ...
                                  {weight_text, ['单故障平均综合能力: ' strjoin(avg_text, ' | ')]}, ...
                                  colNames, data, [0.12, 0.13, 0.13, 0.125, 0.125, 0.125, 0.125]);
        end
    end

    %% 三个布局的样本组合函数
    function [Eval_grid, Raw] = samples_combin(layout_set, fault_nums)
        Eval_grid = cell(numel(layout_set), numel(fault_nums));
        sample_count = 0;
        for fault_idx = 1:numel(fault_nums)
            if fault_nums(fault_idx) >= params.Num
                sample_count = sample_count + 1;
            else
                sample_count = sample_count + nchoosek(params.Num, fault_nums(fault_idx));
            end
        end

        Raw = zeros(sample_count * numel(layout_set), 4);
        raw_row = 0;
        for fault_idx = 1:numel(fault_nums)
            fault_num = fault_nums(fault_idx);
            for layout_idx = 1:numel(layout_set)
                if fault_num >= params.Num
                    Eval = struct('FaultSets', {{[], 1:params.Num}}, ...
                                  'Jc', zeros(2, 1), 'Jo', zeros(2, 1), ...
                                  'Jt', zeros(2, 1), 'Jf', zeros(2, 1));
                else
                    Eval = Reconfig_eval(params, layout_set(layout_idx).B, fault_num);
                end
                Eval.MetricRaw = [Eval.Jc(:), Eval.Jo(:), Eval.Jt(:), Eval.Jf(:)];
                Eval.Score = nan(size(Eval.MetricRaw, 1), 1);
                Eval.Weight = nan(4, 1);
                Eval.IsReconfig = false(size(Eval.MetricRaw, 1), 1);
                Eval.Status = strings(size(Eval.MetricRaw, 1), 1);
                n_sample = size(Eval.MetricRaw, 1) - 1;
                sample_rows = raw_row + (1:n_sample);
                Raw(sample_rows, :) = Eval.MetricRaw(2:end, :);
                raw_row = raw_row + numel(sample_rows);
                Eval_grid{layout_idx, fault_idx} = Eval;
            end
        end
        Raw = Raw(1:raw_row, :);
    end

    %% AHP赋权函数
    function W = AHP_Weight(params, n)
        if isfield(params, 'reconfig_ahp_weight') && ~isempty(params.reconfig_ahp_weight)
            W = params.reconfig_ahp_weight(:);
            if numel(W) == n && all(isfinite(W)) && sum(max(W, 0)) > 1e-12
                W = max(W, 0);
                W = W / sum(W);
                return;
            end
        end

        if isfield(params, 'reconfig_ahp_matrix') && ~isempty(params.reconfig_ahp_matrix)
            G = params.reconfig_ahp_matrix;
        else
            % 指标顺序：[控制能力, 可诊断性, 跟踪性能, 能耗效率]
            G = [1,   6,   7,   7;
                 1/6, 1,   6,   6;
                 1/7, 1/6,   1,   2;
                 1/7, 1/6,   1/2, 1];
        end

        if size(G, 1) ~= n || size(G, 2) ~= n || any(~isfinite(G(:))) || any(G(:) <= 0)
            W = ones(n, 1) / n;
            return;
        end

        W = prod(G, 2).^(1 / n);
        W = W / sum(W);
        RI = [0, 0, 0.58, 0.90, 1.12, 1.24, 1.32, 1.41, 1.45, 1.49];
        if n > 2 && n <= numel(RI)
            lambda_max = mean((G * W) ./ W);
            CR = ((lambda_max - n) / (n - 1)) / RI(n);
            if CR > 0.10
                warning('AHP判断矩阵一致性比例CR=%.3f，大于0.10。', CR);
            end
        end
    end

    %% 熵权法赋权函数
    function W = Entropy_Weight(X)
        [m, n] = size(X);
        if m <= 1
            W = ones(n, 1) / n;
            return;
        end

        W = zeros(n, 1);
        for col = 1:n
            col_sum = sum(X(:, col));
            if col_sum > 1e-12
                p = X(:, col) / col_sum;
            else
                p = ones(m, 1) / m;
            end
            p = p(p > 1e-12);
            W(col) = 1 + sum(p .* log(p)) / log(m);
        end

        if sum(W) > 1e-12
            W = W / sum(W);
        else
            W = ones(n, 1) / n;
        end
    end

    %% 最小二乘组合赋权函数
    function W = Least_Squares_Combined_Weight(X, U, V)
        [~, n] = size(X);
        s = sum(X.^2, 1)';
        if sum(s) < 1e-12
            W = 0.5 * (U(:) + V(:));
        else
            A = diag(max(s, 1e-12));
            b = 0.5 * (U(:) + V(:)) .* s;
            e = ones(n, 1);
            sol = [A, e; e', 0] \ [b; 1];
            W = sol(1:n);
        end
        W = W(:);
        if numel(W) ~= n || any(~isfinite(W)) || sum(abs(W)) < 1e-12
            W = ones(n, 1) / n;
            return;
        end
        W(W < 0) = 0;
        if sum(W) < 1e-12
            W = ones(n, 1) / n;
        else
            W = W / sum(W);
        end
    end

    %% 三线表表格绘制函数
    function Draw_Three_Line_Table(title_str, subtitle, colNames, data, colWidth)
        subtitle = cellstr(string(subtitle));
        nRow = size(data, 1);
        nCol = numel(colNames);
        if nargin < 5 || numel(colWidth) ~= nCol || sum(colWidth) <= 0
            colWidth = ones(1, nCol);
        end

        fig = figure('Name', title_str, 'Color', 'w', 'Position', [120, 100, 1250, 650]);
        ax = axes(fig);
        axis(ax, 'off');
        hold(ax, 'on');

        text(0.5, 0.96, title_str, 'HorizontalAlignment', 'center', 'FontSize', 15, 'FontWeight', 'bold');
        for idx = 1:numel(subtitle)
            text(0.5, 0.92 - 0.035 * (idx - 1), subtitle{idx}, ...
                 'HorizontalAlignment', 'center', 'FontSize', 10.5);
        end

        x0 = 0.05;
        x1 = 0.95;
        y_top = 0.875 - 0.035 * max(numel(subtitle) - 1, 0);
        y_bottom = 0.06;
        colWidth = colWidth / sum(colWidth) * (x1 - x0);
        colX = x0 + [0, cumsum(colWidth)];
        colCenter = colX(1:end-1) + colWidth / 2;
        rowH = (y_top - y_bottom) / (nRow + 1);

        line([x0, x1], [y_top, y_top], 'Color', 'k', 'LineWidth', 1.8);
        line([x0, x1], [y_top - rowH, y_top - rowH], 'Color', 'k', 'LineWidth', 1.2);
        line([x0, x1], [y_bottom, y_bottom], 'Color', 'k', 'LineWidth', 1.8);

        y_header = y_top - rowH / 2;
        for col = 1:nCol
            text(colCenter(col), y_header, colNames{col}, ...
                 'HorizontalAlignment', 'center', 'FontSize', 11.5, 'FontWeight', 'bold');
        end

        for row = 1:nRow
            y = y_top - rowH * (row + 0.5);
            for col = 1:nCol
                if iscell(data)
                    value = char(string(data{row, col}));
                else
                    value = char(string(data(row, col)));
                end
                text(colCenter(col), y, value, 'HorizontalAlignment', 'center', 'FontSize', 11.5);
            end
        end

        xlim([0, 1]);
        ylim([0, 1]);
    end
end
