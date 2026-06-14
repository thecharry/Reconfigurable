%% 仿真系统结果
function Plot_results(log_orig, log_opt, log_opt1, params, B_opt, r_opt, layout_set)
    if nargin < 7 || isempty(layout_set)
        layout_set = struct('name', {'原布局', '优化布局'}, ...
                            'B', {params.B_all, B_opt}, ...
                            'r', {params.r_all, r_opt});
    else
        layout_set = layout_set(:)';
    end

    % plot_1();% 推力器布局优化前后对比
    % plot_2();% 评价指标优化前后对比
    plot_3();% 闭环仿真结果对比
    plot_3_1();% 全故障工况与标况闭环响应对比
    % plot_4();% 推力器控制脉宽对比
    % plot_5();% 推力器分配策略输出
    % plot_6();% 诊断结果输出
    % plot_7();% 不同数量故障下可重构性判定表
    % plot_7_mc();% 三种布局故障1~3台全部组合Monte Carlo打靶验证
    % plot_8();% 单推力器故障综合评价明细表

    %% 推力器布局优化前后对比
    function plot_1()
        view_defs = struct('name', {'YZ正视图','XY俯视图', 'XZ侧视图' }, ...
                           'axes', {[2 3],[1 2], [1 3]}, ...
                           'xlabel', {'Y','X', 'X'}, ...
                           'ylabel', {'Z','Y', 'Z'}, ...
                           'xlim', { [-1 1],[-2.4 2.4], [-2.4 2.4]}, ...
                           'ylim', {[-1 1],[-1 1], [-1 1]}, ...
                           'xdir', { 'normal','reverse', 'reverse'}, ...
                           'ydir', {'reverse','normal', 'reverse'});

        layout_num = numel(layout_set);
        fig_width = max(1500, 480 * numel(view_defs));
        fig_height = max(900, 300 * layout_num);
        figure('Name', '三个布局推力器三视图对比', 'Color','w', ...
               'Position', [80, 80, fig_width, fig_height]);

        for layout_idx = 1:layout_num
            r = layout_set(layout_idx).r;
            B = layout_set(layout_idx).B;
            layout_name = char(layout_set(layout_idx).name);
            for view_idx = 1:numel(view_defs)
                ax = subplot(layout_num, numel(view_defs), ...
                             (layout_idx - 1) * numel(view_defs) + view_idx);
                Plot_place_view(ax, r, B, view_defs(view_idx), layout_name);
            end
        end

        function Plot_place_view(ax, r, B, view_def, layout_name)
            axes(ax);
            cla(ax);
            hold(ax, 'on'); grid(ax, 'on');
            Plot_Body_Projection(view_def.axes);

            healthy_idx = setdiff(1:params.Num, log_orig.faulty_thrusters);
            faulty_idx = intersect(1:params.Num, log_orig.faulty_thrusters);

            Plot_Thruster_Group(r, B, healthy_idx, view_def.axes, ...
                                [0.2 0.7 1], 'r', '-');
            Plot_Thruster_Group(r, B, faulty_idx, view_def.axes, ...
                                [0.8 0.8 0.8], [0.5 0.5 0.5], '--');

            % title(ax, [layout_name, ' - ', view_def.name]);
            xlabel(ax, view_def.xlabel);
            ylabel(ax, view_def.ylabel);
            axis(ax, 'equal');
            xlim(ax, view_def.xlim);
            ylim(ax, view_def.ylim);
            set(ax, 'XDir', view_def.xdir, 'YDir', view_def.ydir);
        end

        function Plot_Body_Projection(proj_axes)
            half_size = [2, 0.6, 0.6];
            x_half = half_size(proj_axes(1));
            y_half = half_size(proj_axes(2));
            body_x = [-x_half, x_half, x_half, -x_half];
            body_y = [-y_half, -y_half, y_half, y_half];
            patch(body_x, body_y, [0.8 0.8 0.8], ...
                  'FaceAlpha', 0.18, 'EdgeColor', [0.6 0.6 0.6], ...
                  'LineWidth', 1.0);
        end

        function Plot_Thruster_Labels(r, view_def)
            proj_axes = view_def.axes;
            p = r(proj_axes, :);
            half_size = [2, 0.6, 0.6];
            x_half = half_size(proj_axes(1));
            side_gap = max(0.32, 0.18 * diff(view_def.xlim));
            group_gap = 0.11;
            label_x_side = [-x_half - side_gap, x_half + side_gap];

            side_flag = ones(1, params.Num);
            side_flag(p(1, :) < 0) = -1;
            side_flag(abs(p(1, :)) < 1e-9 & p(2, :) < 0) = -1;

            for side_value = [-1, 1]
                side_idx = find(side_flag == side_value);
                if isempty(side_idx)
                    continue;
                end

                [~, order] = sortrows([round(p(2, side_idx)' / group_gap), ...
                                       round(p(1, side_idx)' / group_gap), ...
                                       side_idx(:)]);
                side_idx = side_idx(order);
                desired_y = p(2, side_idx);
                label_y = Spread_Label_Y(desired_y, view_def.ylim, 0.18);
                if side_value > 0
                    label_x = label_x_side(2);
                else
                    label_x = label_x_side(1);
                end

                for local_idx = 1:numel(side_idx)
                    thr_idx = side_idx(local_idx);
                    text(label_x, label_y(local_idx), sprintf('%d', thr_idx), ...
                         'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
                         'FontSize', 9, 'FontWeight', 'bold', 'Color', 'k');
                end
            end
        end

        function label_y = Spread_Label_Y(desired_y, ylim_value, min_gap)
            [sorted_y, sorted_idx] = sort(desired_y(:));
            if isempty(sorted_y)
                label_y = sorted_y;
                return;
            end

            y_low = ylim_value(1) + 0.12;
            y_high = ylim_value(2) - 0.12;
            sorted_y = max(y_low, min(y_high, sorted_y));

            for idx = 2:numel(sorted_y)
                sorted_y(idx) = max(sorted_y(idx), sorted_y(idx - 1) + min_gap);
            end

            overflow = sorted_y(end) - y_high;
            if overflow > 0
                sorted_y = sorted_y - overflow;
            end

            for idx = numel(sorted_y)-1:-1:1
                sorted_y(idx) = min(sorted_y(idx), sorted_y(idx + 1) - min_gap);
            end
            underflow = y_low - sorted_y(1);
            if underflow > 0
                sorted_y = sorted_y + underflow;
            end

            label_y = zeros(size(sorted_y));
            label_y(sorted_idx) = sorted_y;
        end

        function Plot_Thruster_Group(r, B, idx, proj_axes, marker_color, arrow_color, line_style)
            if isempty(idx)
                return;
            end

            plot(r(proj_axes(1), idx), r(proj_axes(2), idx), ...
                 'o', 'MarkerSize', 5, 'MarkerFaceColor', marker_color, ...
                 'MarkerEdgeColor', 'k');
            quiver(r(proj_axes(1), idx), r(proj_axes(2), idx), ...
                   B(proj_axes(1), idx), B(proj_axes(2), idx), ...
                   0.35, 'Color', arrow_color, 'LineWidth', 1.2, ...
                   'LineStyle', line_style, 'MaxHeadSize', 0.8);
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
        % % 力空间对比
        % subplot(3,2,1);hold on;grid on;
        % plot(Z_orig,[1 2 3],'FaceColor', [0.7 1 0.7], 'FaceAlpha', 0.15, 'EdgeColor', 'g');
        % plot(Z_opt,[1 2 3], 'FaceColor', [0.9 0.7 0.7], 'FaceAlpha', 0.3, 'EdgeColor', 'r');
        % plot3(Pts_orig(1,:), Pts_orig(2,:), Pts_orig(3,:), '.', 'Color', [0, 0.8, 0], 'MarkerSize', 4);
        % plot3(Pts_opt(1,:), Pts_opt(2,:), Pts_opt(3,:), '.', 'Color', [0.8, 0, 0], 'MarkerSize', 4);
        % plot3(0, 0, 0, 'ko', 'MarkerSize', 3, 'MarkerFaceColor', 'k');
        % title('力空间包络与离散点云');xlabel('Fx (N)'); ylabel('Fy (N)'); zlabel('Fz (N)');
        % axis equal;view(3);
        % 力矩空间对比
        subplot(1,1,1);hold on;grid on;
        % plot(Z_orig, [4 5 6], 'FaceColor', [0.7 1 0.7], 'FaceAlpha', 0.15, 'EdgeColor', 'g');
        plot(Z_opt, [4 5 6], 'FaceColor', [0.9 0.7 0.7], 'FaceAlpha', 0.3, 'EdgeColor', 'r');
        % plot3(Pts_orig(4,:), Pts_orig(5,:), Pts_orig(6,:), '.', 'Color', [0, 0.8, 0], 'MarkerSize', 4);
        % plot3(Pts_opt(4,:), Pts_opt(5,:), Pts_opt(6,:), '.', 'Color', [0.8, 0, 0], 'MarkerSize', 4);
        plot3(0, 0, 0, 'ko', 'MarkerSize', 3, 'MarkerFaceColor', 'k');
        % Plot_Min_Distance(M_orig(4:6, :), [0, 0.45, 0], '原布局');
        Plot_Min_Distance(M_opt(4:6, :), [0.75, 0, 0], '优化布局');
        % title('力矩空间包络、离散点云与最短边界距离');
        xlabel('Mx (N·m)'); ylabel('My (N·m)'); zlabel('Mz (N·m)');
        % axis equal;
        view(3);
        Plot_Reconfig_Metric_Comparison();
        % % 力空间对比（归一化）
        % subplot(3,2,3); hold on; grid on;
        % plot(Z1_orig, [1 2 3], 'FaceColor', [0.7 1 0.7], 'FaceAlpha', 0.15, 'EdgeColor', 'g');
        % plot(Z1_opt,  [1 2 3], 'FaceColor', [0.9 0.7 0.7], 'FaceAlpha', 0.3, 'EdgeColor', 'r');
        % plot3(Pts1_orig(1,:), Pts1_orig(2,:), Pts1_orig(3,:), '.', 'Color', [0, 0.8, 0], 'MarkerSize', 4);
        % plot3(Pts1_opt(1,:),  Pts1_opt(2,:),  Pts1_opt(3,:),  '.', 'Color', [0.8, 0, 0], 'MarkerSize', 4);
        % plot3(0, 0, 0, 'ko', 'MarkerSize', 3, 'MarkerFaceColor', 'k');
        % title('归一化力空间包络与离散点云'); xlabel('Fx'); ylabel('Fy'); zlabel('Fz');
        % axis equal; view(3);
        % % 力矩空间对比（归一化）
        % subplot(3,2,4); hold on; grid on;
        % plot(Z1_orig, [4 5 6], 'FaceColor', [0.7 1 0.7], 'FaceAlpha', 0.15, 'EdgeColor', 'g');
        % plot(Z1_opt,  [4 5 6], 'FaceColor', [0.9 0.7 0.7], 'FaceAlpha', 0.3, 'EdgeColor', 'r');
        % plot3(Pts1_orig(4,:), Pts1_orig(5,:), Pts1_orig(6,:), '.', 'Color', [0, 0.8, 0], 'MarkerSize', 4);
        % plot3(Pts1_opt(4,:),  Pts1_opt(5,:),  Pts1_opt(6,:),  '.', 'Color', [0.8, 0, 0], 'MarkerSize', 4);
        % plot3(0, 0, 0, 'ko', 'MarkerSize', 3, 'MarkerFaceColor', 'k');
        % M1_orig = M_orig ./ scale_orig;
        % M1_opt = M_opt ./ scale_opt;
        % Plot_Min_Distance(M1_orig(4:6, :), [0, 0.45, 0], '原布局');
        % Plot_Min_Distance(M1_opt(4:6, :), [0.75, 0, 0], '优化布局');
        % title('归一化力矩空间包络、离散点云与最短边界距离'); xlabel('Mx'); ylabel('My'); zlabel('Mz');
        % axis equal; view(3);
        % warning('on','all');
        % % 6维向量夹角热力图
        % M_orig = M_orig ./ (vecnorm(M_orig,2,1) + 1e-12);% 归一化
        % M_opt = M_opt ./ (vecnorm(M_opt,2,1) + 1e-12);
        % cos_orig = M_orig' * M_orig;
        % cos_opt  = M_opt'  * M_opt;
        % angle_orig = acos(min(max(cos_orig, -1), 1)) * 180 / pi;
        % angle_opt  = acos(min(max(cos_opt,  -1), 1)) * 180 / pi;
        % subplot(3,2,5);
        % h1 = heatmap(healthy_idx, healthy_idx, angle_orig);
        % h1.Title = '原布局向量夹角';h1.XLabel = '推进器编号';h1.YLabel = '推进器编号';
        % colormap(h1, jet);
        % subplot(3,2,6);
        % h2 = heatmap(healthy_idx, healthy_idx, angle_opt);
        % h2.Title = '优化布局向量夹角';h2.XLabel = '推进器编号';h2.YLabel = '推进器编号';
        % colormap(h2, jet);

        function Plot_Min_Distance(Matrix_sub, line_color, label_str)
            [d_min, boundary_point] = Closest_Boundary_Point(Matrix_sub);
            if d_min <= 1e-12 || any(~isfinite(boundary_point))
                text(0, 0, 0, sprintf('%s J_c=0', label_str), ...
                     'Color', line_color, 'FontSize', 8, 'FontWeight', 'bold');
                return;
            end

            plot3([0, boundary_point(1)], [0, boundary_point(2)], [0, boundary_point(3)], ...
                  '--', 'Color', line_color, 'LineWidth', 2.0);
            plot3(boundary_point(1), boundary_point(2), boundary_point(3), ...
                  'o', 'MarkerSize', 5, 'MarkerFaceColor', line_color, 'MarkerEdgeColor', 'k');
            label_pos = 0.55 * boundary_point;
            text(label_pos(1), label_pos(2), label_pos(3), ...
                 sprintf('%s J_c=%.3g', label_str, d_min), ...
                 'Color', line_color, 'FontSize', 8, 'FontWeight', 'bold', ...
                 'BackgroundColor', 'w', 'Margin', 1);
        end

        function [d_min, boundary_point] = Closest_Boundary_Point(Matrix_sub)
            d_min = 0;
            boundary_point = [NaN; NaN; NaN];
            n_sub = size(Matrix_sub, 2);
            if n_sub == 0 || isempty(Matrix_sub)
                return;
            end

            c_local = 0.5 * ones(n_sub, 1);
            G_local = 0.5 * eye(n_sub);
            try
                Z_local = zonotope(Matrix_sub * c_local, Matrix_sub * G_local);
                P_local = polytope(Z_local);
                H = P_local.A;
                w = P_local.b;
                if isempty(H) || isempty(w) || any(w < -1e-6)
                    return;
                end

                H_norm = vecnorm(H, 2, 2) + 1e-12;
                distances = w ./ H_norm;
                [d_min, idx_min] = min(distances);
                h_min = H(idx_min, :).';
                boundary_point = (w(idx_min) / (norm(h_min)^2 + 1e-12)) * h_min;
            catch
                d_min = 0;
                boundary_point = [NaN; NaN; NaN];
            end
        end

        function Plot_Reconfig_Metric_Comparison()
            fault_num = 1;
            if isfield(params, 'plot_2_fault_num') && ~isempty(params.plot_2_fault_num)
                fault_num = params.plot_2_fault_num;
            end
            if ~isnumeric(fault_num) || isempty(fault_num) || ~isfinite(fault_num(1))
                fault_num = 1;
            else
                fault_num = fault_num(1);
            end
            fault_num = floor(fault_num);
            fault_num = max(0, min(params.Num, fault_num));

            metric_names = {'控制能力 J_c', '可诊断性 J_o', '控制指令 J_t', '能量消耗 J_f'};
            layout_num = numel(layout_set);
            layout_names = cell(1, layout_num);
            Eval_set = cell(1, layout_num);

            for layout_idx = 1:layout_num
                layout_names{layout_idx} = char(layout_set(layout_idx).name);
                Eval_set{layout_idx} = Reconfig_eval(params, layout_set(layout_idx).B, max(1, fault_num));
            end

            fault_sets = Eval_set{1}.FaultSets;
            if fault_num == 0
                fault_sets = fault_sets(1);
            end

            case_num = numel(fault_sets);
            fault_labels = cell(1, case_num);
            metric_values = zeros(case_num, layout_num, numel(metric_names));

            for case_idx = 1:case_num
                fault_set = Normalize_Plot2_Fault_Set(fault_sets{case_idx});
                fault_labels{case_idx} = Plot2_Fault_Label(fault_set);

                for layout_idx = 1:layout_num
                    eval_idx = Find_Plot2_Fault_Row(Eval_set{layout_idx}.FaultSets, fault_set);
                    metric_values(case_idx, layout_idx, :) = [ ...
                        Eval_set{layout_idx}.Jc(eval_idx), ...
                        Eval_set{layout_idx}.Jo(eval_idx), ...
                        Eval_set{layout_idx}.Jt(eval_idx), ...
                        Eval_set{layout_idx}.Jf(eval_idx)];
                end
            end

            figure('Name', '重构评价四项指标优化前后对比', 'Color', 'w');
            for metric_idx = 1:numel(metric_names)
                ax = subplot(2, 2, metric_idx);
                bar(ax, metric_values(:, :, metric_idx), 'grouped');
                grid(ax, 'on');
                title(ax, metric_names{metric_idx});
                xlabel(ax, '故障工况');
                ylabel(ax, '归一化指标值');
                ylim(ax, [0, 1.05]);
                xticks(ax, 1:case_num);
                xticklabels(ax, fault_labels);
                xtickangle(ax, 45);
                if metric_idx == 1
                    legend(ax, layout_names, 'Location', 'best');
                end
            end

            % if exist('sgtitle', 'file') || exist('sgtitle', 'builtin')
            %     if fault_num == 0
            %         title_str = '标况四项重构评价指标对比';
            %     else
            %         title_str = sprintf('标况与%d推力器故障工况四项重构评价指标对比', fault_num);
            %     end
            %     sgtitle(title_str);
            % end
        end

        function row_idx = Find_Plot2_Fault_Row(fault_sets, target_set)
            target_set = Normalize_Plot2_Fault_Set(target_set);
            for set_idx = 1:numel(fault_sets)
                if isequal(Normalize_Plot2_Fault_Set(fault_sets{set_idx}), target_set)
                    row_idx = set_idx;
                    return;
                end
            end
            error('plot_2未找到故障组合[%s]的评价指标。', num2str(target_set));
        end

        function fault_set = Normalize_Plot2_Fault_Set(fault_set)
            fault_set = unique(fault_set(:)');
            fault_set = fault_set(fault_set >= 1 & fault_set <= params.Num);
        end

        function label = Plot2_Fault_Label(fault_set)
            fault_set = Normalize_Plot2_Fault_Set(fault_set);
            if isempty(fault_set)
                label = '标况';
            elseif numel(fault_set) == 1
                label = sprintf('故障%d', fault_set);
            else
                label = ['故障[' strjoin(cellstr(string(fault_set)), ',') ']'];
            end
        end
    end
    
    %% 闭环位置与姿态响应对比
    function plot_3()
        plot_logs = {log_orig, log_opt, log_opt1};
        plot_labels = Plot3_Layout_Labels(numel(plot_logs));

        figure('Name', '闭环位置与姿态响应对比','Color','w');
        subplot(2,2,1); Plot_3Axis(plot_logs, plot_labels, 'pos', '位置响应', '(m)', log_orig.faulty_time);
        subplot(2,2,2); Plot_3Axis(plot_logs, plot_labels, 'att', '姿态响应', '(rad)', log_orig.faulty_time);
        subplot(2,2,3); Plot_Error_Norm(plot_logs, plot_labels, 'pos', '位置误差范数', '||e_r||(m)', log_orig.faulty_time);
        subplot(2,2,4); Plot_Error_Norm(plot_logs, plot_labels, 'att', '姿态误差范数', '||e_e||(rad)', log_orig.faulty_time);

        function Plot_3Axis(logs, labels, data_type, title_str, unit_str, fault_time)
            ax = gca;
            hold(ax, 'on'); grid(ax, 'on');
            colors = lines(3);
            line_styles = {'-', '--', ':', '-.'};
            line_widths = [1.2, 1.3, 1.7, 1.3];
            axis_names = Plot3_Axis_Names(data_type);
            legend_handles = gobjects(1, numel(logs) * 3);
            legend_labels = cell(1, numel(logs) * 3);
            legend_idx = 0;

            for layout_idx = 1:numel(logs)
                data = Plot3_Response_Data(logs{layout_idx}, data_type);
                line_style = line_styles{min(layout_idx, numel(line_styles))};
                line_width = line_widths(min(layout_idx, numel(line_widths)));
                for axis_idx = 1:3
                    legend_idx = legend_idx + 1;
                    legend_handles(legend_idx) = plot(ax, logs{layout_idx}.Time, data(axis_idx, :), ...
                        line_style, 'Color', colors(axis_idx, :), 'LineWidth', line_width);
                    legend_labels{legend_idx} = [labels{layout_idx} '-' axis_names{axis_idx}];
                end
            end
            Draw_Plot3_Fault_Line(ax, fault_time);
            title(ax, title_str); xlabel(ax, 't(s)'); ylabel(ax, unit_str);
            legend(ax, legend_handles, legend_labels, 'Location', 'best');
        end

        function Plot_Error_Norm(logs, labels, data_type, title_str, ylabel_str, fault_time)
            ax = gca;
            hold(ax, 'on'); grid(ax, 'on');
            line_styles = {'-', '--', ':', '-.'};
            line_widths = [1.2, 1.3, 1.7, 1.3];
            handles = gobjects(1, numel(logs));
            for layout_idx = 1:numel(logs)
                err = Plot3_Error_Data(logs{layout_idx}, data_type);
                handles(layout_idx) = plot(ax, logs{layout_idx}.Time, vecnorm(err, 2, 1), ...
                    line_styles{min(layout_idx, numel(line_styles))}, ...
                    'LineWidth', line_widths(min(layout_idx, numel(line_widths))));
            end
            Draw_Plot3_Fault_Line(ax, fault_time);
            title(ax, title_str); xlabel(ax, 't(s)'); ylabel(ax, ylabel_str);
            legend(ax, handles, labels, 'Location', 'best');
        end

        function data = Plot3_Response_Data(log_data, data_type)
            if strcmp(data_type, 'pos')
                data = log_data.Y(1:3, :);
            else
                data = log_data.Y_euler;
            end
        end

        function err = Plot3_Error_Data(log_data, data_type)
            if strcmp(data_type, 'pos')
                err = log_data.Y(1:3, :) - log_data.R;
            else
                err = mod((log_data.Y_euler - log_data.E) + pi, 2*pi) - pi;
            end
        end

        function axis_names = Plot3_Axis_Names(data_type)
            if strcmp(data_type, 'pos')
                axis_names = {'x', 'y', 'z'};
            else
                axis_names = {'phi', 'theta', 'psi'};
            end
        end

        function labels = Plot3_Layout_Labels(label_count)
            labels = cell(1, label_count);
            default_labels = {'原布局', '方案一', '方案二'};
            for label_idx = 1:label_count
                if numel(layout_set) >= label_idx && isfield(layout_set, 'name')
                    labels{label_idx} = char(layout_set(label_idx).name);
                elseif label_idx <= numel(default_labels)
                    labels{label_idx} = default_labels{label_idx};
                else
                    labels{label_idx} = ['布局' num2str(label_idx)];
                end
            end
        end

        function Draw_Plot3_Fault_Line(ax, fault_time)
            if ~isempty(fault_time) && isfinite(fault_time)
                xline(ax, fault_time, '--r');
            end
        end
    end

    %% 全故障工况与标况闭环位置、姿态响应对比
    function plot_3_1()
        fault_sets = Build_Plot31_Fault_Sets();
        case_num = numel(fault_sets);
        colors = lines(case_num);

        for layout_idx = 1:numel(layout_set)
            layout_name = char(layout_set(layout_idx).name);
            logs = cell(1, case_num);
            labels = cell(1, case_num);

            for case_idx = 1:case_num
                faulty_set = fault_sets{case_idx};
                labels{case_idx} = Fault_Case_Label(faulty_set);
                logs{case_idx} = Get_Plot31_Log(layout_set(layout_idx).B, faulty_set);
            end

            figure('Name', [layout_name, ' 全故障工况与标况位置姿态响应对比'], 'Color','w');
            plot_handles = gobjects(1, case_num);
            pos_names = {'x', 'y', 'z'};
            att_names = {'\phi', '\theta', '\psi'};

            for axis_idx = 1:3
                ax = subplot(2, 3, axis_idx);
                hold(ax, 'on'); grid(ax, 'on');
                for case_idx = 1:case_num
                    line_style = '-';
                    line_width = 0.9;
                    if isempty(fault_sets{case_idx})
                        line_width = 1.8;
                    end
                    h = plot(ax, logs{case_idx}.Time, logs{case_idx}.Y(axis_idx, :), ...
                             line_style, 'Color', colors(case_idx, :), 'LineWidth', line_width);
                    if axis_idx == 1
                        plot_handles(case_idx) = h;
                    end
                end
                Draw_Fault_Time_Line(ax, logs{1});
                % title(ax, [pos_names{axis_idx}, '位置响应']);
                xlabel(ax, 't(s)');
                ylabel(ax, [pos_names{axis_idx},'(m)']);
            end

            for axis_idx = 1:3
                ax = subplot(2, 3, axis_idx + 3);
                hold(ax, 'on'); grid(ax, 'on');
                for case_idx = 1:case_num
                    line_style = '-';
                    line_width = 0.9;
                    if isempty(fault_sets{case_idx})
                        line_width = 1.8;
                    end
                    plot(ax, logs{case_idx}.Time, logs{case_idx}.Y_euler(axis_idx, :), ...
                         line_style, 'Color', colors(case_idx, :), 'LineWidth', line_width);
                end
                Draw_Fault_Time_Line(ax, logs{1});
                % title(ax, [att_names{axis_idx}, '姿态响应']);
                xlabel(ax, 't(s)');
                ylabel(ax, [att_names{axis_idx},'(rad)']);
            end

            legend(plot_handles, labels, 'Location', 'eastoutside');
        end

        function fault_sets = Build_Plot31_Fault_Sets()
            if isfield(params, 'plot_3_1_fault_sets') && ~isempty(params.plot_3_1_fault_sets)
                fault_sets = Normalize_Fault_Set_Input(params.plot_3_1_fault_sets);
            else
                fault_nums = 1;
                if isfield(params, 'plot_3_1_fault_nums') && ~isempty(params.plot_3_1_fault_nums)
                    fault_nums = params.plot_3_1_fault_nums;
                end
                fault_sets = Generate_Fault_Sets(fault_nums);
            end
            fault_sets = [{[]}, fault_sets(:)'];
            fault_sets = Unique_Fault_Sets(fault_sets);
        end

        function fault_sets = Generate_Fault_Sets(fault_nums)
            fault_nums = unique(fault_nums(:)');
            fault_nums = fault_nums(isfinite(fault_nums) & fault_nums >= 0 & ...
                                    fault_nums <= params.Num & floor(fault_nums) == fault_nums);
            fault_sets = {};
            for fault_num = fault_nums
                if fault_num <= 0
                    continue;
                elseif fault_num >= params.Num
                    fault_sets{end + 1} = 1:params.Num; %#ok<AGROW>
                else
                    combs = nchoosek(1:params.Num, fault_num);
                    for comb_idx = 1:size(combs, 1)
                        fault_sets{end + 1} = combs(comb_idx, :); %#ok<AGROW>
                    end
                end
            end
        end

        function fault_sets = Normalize_Fault_Set_Input(raw_fault_sets)
            if iscell(raw_fault_sets)
                fault_sets = raw_fault_sets;
            else
                fault_sets = cell(size(raw_fault_sets, 1), 1);
                for row_idx = 1:size(raw_fault_sets, 1)
                    row_faults = raw_fault_sets(row_idx, :);
                    fault_sets{row_idx} = row_faults(row_faults > 0);
                end
            end
        end

        function fault_sets = Unique_Fault_Sets(fault_sets)
            unique_keys = strings(0, 1);
            unique_sets = {};
            for set_idx = 1:numel(fault_sets)
                fault_set = Normalize_Fault_Set(fault_sets{set_idx});
                key = mat2str(fault_set);
                if ~any(unique_keys == key)
                    unique_keys(end + 1, 1) = key; %#ok<AGROW>
                    unique_sets{end + 1} = fault_set; %#ok<AGROW>
                end
            end
            fault_sets = unique_sets;
        end

        function log_case = Get_Plot31_Log(B_case, faulty_set)
            precomputed_log = Try_Get_Precomputed_Log(B_case, faulty_set);
            if ~isempty(precomputed_log)
                log_case = precomputed_log;
                return;
            end

            params_case = params;
            params_case.true_faults = Normalize_Fault_Set(faulty_set);
            evalc('log_case = Closedloop_sim(params_case, B_case);');
        end

        function precomputed_log = Try_Get_Precomputed_Log(B_case, faulty_set)
            precomputed_log = [];
            if Same_Matrix(B_case, params.B_all) && Same_Fault_Set(log_orig.faulty_thrusters, faulty_set)
                precomputed_log = log_orig;
            elseif Same_Matrix(B_case, B_opt) && Same_Fault_Set(log_opt.faulty_thrusters, faulty_set)
                precomputed_log = log_opt;
            end
        end

        function tf = Same_Matrix(A, B)
            tf = isequal(size(A), size(B)) && norm(A - B, 'fro') < 1e-10;
        end

        function tf = Same_Fault_Set(a, b)
            tf = isequal(Normalize_Fault_Set(a), Normalize_Fault_Set(b));
        end

        function fault_set = Normalize_Fault_Set(fault_set)
            fault_set = unique(fault_set(:)');
            fault_set = fault_set(fault_set >= 1 & fault_set <= params.Num);
        end

        function label = Fault_Case_Label(faulty_set)
            faulty_set = Normalize_Fault_Set(faulty_set);
            if isempty(faulty_set)
                label = '标况';
            else
                label = sprintf('%s号故障', strjoin(cellstr(string(faulty_set)), ','));
            end
        end

        function Draw_Fault_Time_Line(ax, log_case)
            if isfield(log_case, 'faulty_time') && isfinite(log_case.faulty_time)
                xline(ax, log_case.faulty_time, '--r', 'LineWidth', 0.8);
            end
        end
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

        for fault_idx = 1:numel(fault_nums)
            for layout_idx = 1:numel(layout_set)
                Eval = Eval_grid{layout_idx, fault_idx};
                Eval = Apply_Reconfig_Status(Eval);
                Eval_grid{layout_idx, fault_idx} = Eval;
            end
        end

        layout_names = Plot7_Layout_Names();
        colNames = {'推力器故障数'};
        colWidth = 0.08;
        for layout_idx = 1:numel(layout_set)
            colNames = [colNames, ...
                        {[layout_names{layout_idx}], ...
                         ['可重构数'], ...
                         ['不可重构数']}]; %#ok<AGROW>
            colWidth = [colWidth, 0.08, 0.07, 0.08]; %#ok<AGROW>
        end

        data = strings(numel(fault_nums), numel(colNames));
        for fault_idx = 1:numel(fault_nums)
            data(fault_idx, 1) = string(fault_nums(fault_idx));
            for layout_idx = 1:numel(layout_set)
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

                col0 = 2 + (layout_idx - 1) * 3;
                data(fault_idx, col0:(col0 + 2)) = [status, ...
                                                    string(reconfig_num), ...
                                                    string(nonreconfig_num)];
            end
        end

        Draw_Three_Line_Table('不同数量故障下可重构性判断对比表', ...
                              '', colNames, data, colWidth);

        function layout_names = Plot7_Layout_Names()
            layout_names = cell(1, numel(layout_set));
            for idx = 1:numel(layout_set)
                if idx == 1
                    layout_names{idx} = '原布局';
                else
                    layout_names{idx} = ['方案' Plot7_Chinese_Num(idx - 1)];
                end
            end
        end

        function subtitle = Plot7_Subtitle(layout_names)
            subtitle = {};
            if numel(layout_set) <= 1
                return;
            end

            items = cell(1, numel(layout_set) - 1);
            for idx = 2:numel(layout_set)
                items{idx - 1} = sprintf('%s: %s', layout_names{idx}, char(layout_set(idx).name));
            end
            subtitle = {strjoin(items, ' | ')};
        end

        function value = Plot7_Chinese_Num(num)
            names = {'一', '二', '三', '四', '五', '六', '七', '八', '九', '十'};
            if num >= 1 && num <= numel(names)
                value = names{num};
            else
                value = num2str(num);
            end
        end
    end

    %% plot_7前三行中三种布局全部故障组合的Monte Carlo打靶验证
    function plot_7_mc()
        mc_cfg = Plot7_MC_Default_Config();
        if isfield(params, 'plot_7_mc') && ~isempty(params.plot_7_mc)
            mc_cfg = Merge_Struct(mc_cfg, params.plot_7_mc);
        end

        mc_cfg.fault_nums = unique(mc_cfg.fault_nums(:)');
        mc_cfg.fault_nums = mc_cfg.fault_nums(mc_cfg.fault_nums >= 1 & ...
                                              mc_cfg.fault_nums <= params.Num & ...
                                              floor(mc_cfg.fault_nums) == mc_cfg.fault_nums);
        if isempty(mc_cfg.fault_nums)
            mc_cfg.fault_nums = 1:min(3, params.Num);
        end
        mc_cfg.N_trial = max(0, floor(mc_cfg.N_trial));

        mc_cfg.layout_indices = unique(mc_cfg.layout_indices(:)', 'stable');
        mc_cfg.layout_indices = mc_cfg.layout_indices(mc_cfg.layout_indices >= 1 & ...
                                                      mc_cfg.layout_indices <= numel(layout_set) & ...
                                                      floor(mc_cfg.layout_indices) == mc_cfg.layout_indices);
        if isempty(mc_cfg.layout_indices)
            mc_cfg.layout_indices = 1:min(3, numel(layout_set));
        end
        if numel(mc_cfg.layout_indices) < 3
            error('plot_7_mc需要layout_set同时包含原布局、方案一和方案二。');
        end

        [Eval_grid, ~] = samples_combin(layout_set, mc_cfg.fault_nums);
        Eval_grid = Mark_Reconfig_Status(Eval_grid);

        CaseSummary = struct('LayoutIndex', {}, 'LayoutName', {}, 'FaultNum', {}, ...
                             'FaultSet', {}, 'IsReconfig', {}, 'TrialCount', {}, ...
                             'SuccessCount', {}, 'SuccessRate', {}, ...
                             'MeanPosErr', {}, 'MeanAttErrDeg', {});
        Detail = struct('LayoutIndex', {}, 'LayoutName', {}, 'FaultNum', {}, ...
                        'FaultSet', {}, 'Trial', {}, ...
                        'Success', {}, 'FinalPosErr', {}, 'FinalAttErrDeg', {});
        total_case_count = 0;
        total_trials = 0;
        total_success = 0;

        fprintf('\n==============================================================\n');
        fprintf('plot_7前三行中三种布局全部故障组合的Monte Carlo打靶验证\n');
        fprintf('==============================================================\n');
        fprintf('验证布局: %s\n', strjoin(Layout_Name_List(mc_cfg.layout_indices), ' | '));
        fprintf('故障数量范围: [%s]\n', num2str(mc_cfg.fault_nums));
        fprintf('每个布局的故障组合数: %d\n', Count_Fault_Combinations(mc_cfg.fault_nums));
        fprintf('每个组合打靶次数: %d\n', mc_cfg.N_trial);
        fprintf('成功阈值: 位置误差 < %.4g m, 姿态误差 < %.4g deg\n', ...
                mc_cfg.pos_tol, rad2deg(mc_cfg.att_tol));

        for layout_order = 1:numel(mc_cfg.layout_indices)
            layout_idx = mc_cfg.layout_indices(layout_order);
            layout_name = char(layout_set(layout_idx).name);
            layout_case_count = 0;
            layout_trials = 0;
            layout_success = 0;

            fprintf('\n--------------------------------------------------------------\n');
            fprintf('%s: Monte Carlo打靶验证\n', layout_name);
            fprintf('--------------------------------------------------------------\n');

            for fault_idx = 1:numel(mc_cfg.fault_nums)
                fault_num = mc_cfg.fault_nums(fault_idx);
                Eval = Eval_grid{layout_idx, fault_idx};
                rows = 2:numel(Eval.FaultSets);
                reconfig_num = sum(Eval.IsReconfig(rows));
                nonreconfig_num = numel(rows) - reconfig_num;

                fprintf('\n故障 %d 台: 组合 %d 个，可重构 %d 个，不可重构 %d 个。\n', ...
                        fault_num, numel(rows), reconfig_num, nonreconfig_num);

                for local_idx = 1:numel(rows)
                    row_idx = rows(local_idx);
                    faulty_set = Normalize_Fault_Set(Eval.FaultSets{row_idx});
                    is_reconfig = Eval.IsReconfig(row_idx);
                    [success_count, trial_count, mean_pos_err, mean_att_err_deg, Detail] = ...
                        Run_Plot7_MC_Case(layout_idx, layout_name, fault_num, row_idx, ...
                                          faulty_set, mc_cfg, Detail);

                    if trial_count > 0
                        success_ratio = 100 * success_count / trial_count;
                    else
                        success_ratio = NaN;
                    end

                    summary_idx = numel(CaseSummary) + 1;
                    CaseSummary(summary_idx).LayoutIndex = layout_idx;
                    CaseSummary(summary_idx).LayoutName = layout_name;
                    CaseSummary(summary_idx).FaultNum = fault_num;
                    CaseSummary(summary_idx).FaultSet = faulty_set;
                    CaseSummary(summary_idx).IsReconfig = is_reconfig;
                    CaseSummary(summary_idx).TrialCount = trial_count;
                    CaseSummary(summary_idx).SuccessCount = success_count;
                    CaseSummary(summary_idx).SuccessRate = success_ratio;
                    CaseSummary(summary_idx).MeanPosErr = mean_pos_err;
                    CaseSummary(summary_idx).MeanAttErrDeg = mean_att_err_deg;

                    total_case_count = total_case_count + 1;
                    total_trials = total_trials + trial_count;
                    total_success = total_success + success_count;
                    layout_case_count = layout_case_count + 1;
                    layout_trials = layout_trials + trial_count;
                    layout_success = layout_success + success_count;

                    if mc_cfg.print_each_case
                        fprintf('  %3d/%3d 故障 [%s] %-8s: 成功 %d/%d, 成功率 %.2f%%, 平均位置误差 %.4f m, 平均姿态误差 %.4f deg\n', ...
                                local_idx, numel(rows), num2str(faulty_set), ...
                                char(Eval.Status(row_idx)), success_count, trial_count, ...
                                success_ratio, mean_pos_err, mean_att_err_deg);
                    end
                end
            end

            if layout_trials > 0
                layout_success_rate = 100 * layout_success / layout_trials;
            else
                layout_success_rate = NaN;
            end
            fprintf('\n%s完成: 组合 %d 个，累计打靶 %d 次，成功 %d 次，总成功率 %.2f%%。\n', ...
                    layout_name, layout_case_count, layout_trials, layout_success, layout_success_rate);
        end

        fprintf('\n==============================================================\n');
        fprintf('验证完成: 三种布局共跑 %d 个布局-故障工况，累计打靶 %d 次，成功 %d 次。\n', ...
                total_case_count, total_trials, total_success);
        if total_trials > 0
            fprintf('总成功率: %.2f%%\n', 100 * total_success / total_trials);
        end
        fprintf('详细结果已写入工作区变量 Plot7_MC_Result。\n');
        fprintf('==============================================================\n\n');

        MC_Result.Config = mc_cfg;
        MC_Result.Layouts = Layout_Name_List(mc_cfg.layout_indices);
        MC_Result.CaseSummary = CaseSummary;
        MC_Result.Detail = Detail;
        assignin('base', 'Plot7_MC_Result', MC_Result);

        function cfg = Plot7_MC_Default_Config()
            cfg.fault_nums = 1:min(3, params.Num);
            cfg.N_trial = 2;
            cfg.rng_seed = 1;
            cfg.layout_indices = 1:min(3, numel(layout_set));

            cfg.T_sim = 2000;
            cfg.dt = 0.005;
            cfg.faulty_time = 0;

            cfg.pos_tol = 0.5;
            cfg.att_tol = deg2rad(2);

            cfg.r0 = [0; 15; 55];
            cfg.v0 = [0; 0; 0];
            cfg.euler0 = deg2rad([0; 15; 55]);
            cfg.rt_center = [5; 25; 35];
            cfg.rt_range = [3; 3; 3];
            cfg.eulert_center_deg = [5; 25; 35];
            cfg.eulert_range_deg = [3; 3; 3];
            cfg.silent_closed_loop = true;
            cfg.print_each_case = true;
        end

        function cfg = Merge_Struct(cfg, user_cfg)
            names = fieldnames(user_cfg);
            for field_idx = 1:numel(names)
                cfg.(names{field_idx}) = user_cfg.(names{field_idx});
            end
        end

        function Eval_grid = Mark_Reconfig_Status(Eval_grid)
            for fault_idx_local = 1:size(Eval_grid, 2)
                for layout_idx_local = 1:size(Eval_grid, 1)
                    Eval = Eval_grid{layout_idx_local, fault_idx_local};
                    Eval = Apply_Reconfig_Status(Eval);
                    Eval_grid{layout_idx_local, fault_idx_local} = Eval;
                end
            end
        end

        function layout_names = Layout_Name_List(layout_indices)
            layout_names = cell(1, numel(layout_indices));
            for idx = 1:numel(layout_indices)
                layout_names{idx} = char(layout_set(layout_indices(idx)).name);
            end
        end

        function total_count = Count_Fault_Combinations(fault_nums)
            total_count = 0;
            for count_idx = 1:numel(fault_nums)
                fault_num = fault_nums(count_idx);
                if fault_num >= params.Num
                    total_count = total_count + 1;
                else
                    total_count = total_count + nchoosek(params.Num, fault_num);
                end
            end
        end

        function [success_count, trial_count, mean_pos_err, mean_att_err_deg, Detail] = ...
                Run_Plot7_MC_Case(layout_idx, layout_name, fault_num, row_idx, ...
                                  faulty_set, mc_cfg, Detail)
            trial_count = mc_cfg.N_trial;
            success_count = 0;
            pos_err_all = nan(trial_count, 1);
            att_err_all = nan(trial_count, 1);

            sim_params = params;
            if isfield(params, 'alloc_mode')
                sim_params.alloc_mode = params.alloc_mode;
            end
            sim_params.true_faults = faulty_set;

            for trial = 1:trial_count
                rng(mc_cfg.rng_seed + 1000 * fault_num + 100 * row_idx + trial);
                sim_cfg = Build_Plot7_MC_Sim_Config(mc_cfg, faulty_set);

                if mc_cfg.silent_closed_loop
                    evalc('log_mc = Closedloop_sim(sim_params, layout_set(layout_idx).B, sim_cfg);');
                else
                    log_mc = Closedloop_sim(sim_params, layout_set(layout_idx).B, sim_cfg);
                end

                [success, final_pos_err, final_att_err_deg] = Evaluate_Plot7_MC_Result(log_mc, mc_cfg);
                success_count = success_count + success;
                pos_err_all(trial) = final_pos_err;
                att_err_all(trial) = final_att_err_deg;

                detail_row = numel(Detail) + 1;
                Detail(detail_row).LayoutIndex = layout_idx;
                Detail(detail_row).LayoutName = layout_name;
                Detail(detail_row).FaultNum = fault_num;
                Detail(detail_row).FaultSet = mat2str(faulty_set);
                Detail(detail_row).Trial = trial;
                Detail(detail_row).Success = success;
                Detail(detail_row).FinalPosErr = final_pos_err;
                Detail(detail_row).FinalAttErrDeg = final_att_err_deg;
            end

            if trial_count > 0
                mean_pos_err = mean(pos_err_all, 'omitnan');
                mean_att_err_deg = mean(att_err_all, 'omitnan');
            else
                mean_pos_err = NaN;
                mean_att_err_deg = NaN;
            end
        end

        function sim_cfg = Build_Plot7_MC_Sim_Config(mc_cfg, faulty_set)
            sim_cfg.r0 = mc_cfg.r0;
            sim_cfg.v0 = mc_cfg.v0;
            sim_cfg.rt = mc_cfg.rt_center + mc_cfg.rt_range .* (2 * rand(3, 1) - 1);
            sim_cfg.euler0 = mc_cfg.euler0;
            sim_cfg.eulert = deg2rad(mc_cfg.eulert_center_deg(:) + ...
                                      mc_cfg.eulert_range_deg(:) .* (2 * rand(3, 1) - 1));
            sim_cfg.T_sim = mc_cfg.T_sim;
            sim_cfg.dt = mc_cfg.dt;
            sim_cfg.faulty_time = mc_cfg.faulty_time;
            sim_cfg.true_faults = faulty_set;
        end

        function [success, final_pos_err, final_att_err_deg] = Evaluate_Plot7_MC_Result(log_mc, mc_cfg)
            r_final = log_mc.Y(1:3, end);
            r_ref = log_mc.R(:, end);
            euler_final = log_mc.Y_euler(:, end);
            euler_ref = log_mc.E(:, end);

            final_pos_err = norm(r_final - r_ref);
            att_err = mod((euler_final - euler_ref) + pi, 2*pi) - pi;
            final_att_err = norm(att_err);
            final_att_err_deg = rad2deg(final_att_err);

            success = all(isfinite(r_final)) && all(isfinite(euler_final)) && ...
                      final_pos_err < mc_cfg.pos_tol && final_att_err < mc_cfg.att_tol;
        end

        function fault_set = Normalize_Fault_Set(fault_set)
            fault_set = unique(fault_set(:)');
            fault_set = fault_set(fault_set >= 1 & fault_set <= params.Num);
        end
    end

    %% 单推力器故障综合评价明细表
    function plot_8()
        [Eval_case, Raw] = samples_combin(layout_set, 1);
        Weight = Least_Squares_Combined_Weight(Raw, AHP_Weight(params, 4), Entropy_Weight(Raw));

        for layout_idx = 1:numel(layout_set)
            Eval = Eval_case{layout_idx};
            rows = 2:size(Eval.MetricRaw, 1);
            Eval.Score(rows) = Eval.MetricRaw(rows, :) * Weight;
            Eval.Weight = Weight;
            Eval = Apply_Reconfig_Status(Eval);
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
                                  'Jt', zeros(2, 1), 'Jf', zeros(2, 1), ...
                                  'Jc6', zeros(2, 1));
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

    %% 可重构状态判定函数
    function Eval = Apply_Reconfig_Status(Eval)
        positive_eps = eps;
        rows = 2:size(Eval.MetricRaw, 1);
        jc6 = zeros(size(Eval.MetricRaw, 1), 1);
        if isfield(Eval, 'Jc6') && ~isempty(Eval.Jc6)
            jc6_count = min(numel(jc6), numel(Eval.Jc6));
            jc6(1:jc6_count) = Eval.Jc6(1:jc6_count);
        end
        Eval.Jc6 = jc6;

        Eval.IsReconfig(:) = false;
        Eval.IsReconfig(rows) = all(Eval.MetricRaw(rows, :) > positive_eps, 2) & ...
                                jc6(rows) > positive_eps;
        Eval.Status(:) = "不可重构";
        Eval.Status(Eval.IsReconfig) = "可重构";
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

        fig_width = max(1250, min(1800, 90 * nCol + 250));
        table_font_size = max(7.5, min(11.5, 105 / nCol));

        fig = figure('Name', title_str, 'Color', 'w', 'Position', [120, 100, fig_width, 650]);
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
                 'HorizontalAlignment', 'center', 'FontSize', table_font_size, 'FontWeight', 'bold');
        end

        for row = 1:nRow
            y = y_top - rowH * (row + 0.5);
            for col = 1:nCol
                if iscell(data)
                    value = char(string(data{row, col}));
                else
                    value = char(string(data(row, col)));
                end
                text(colCenter(col), y, value, 'HorizontalAlignment', 'center', 'FontSize', table_font_size);
            end
        end

        xlim([0, 1]);
        ylim([0, 1]);
    end
end
