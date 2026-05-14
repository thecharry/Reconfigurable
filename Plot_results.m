%% 仿真系统结果
function Plot_results(log_orig, log_opt, params, B_opt, r_opt)
    %% 推力器布局优化前后对比
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
        % plot3(r(1,:), r(2,:), r(3,:), 'o', 'Color', 'b', 'MarkerSize', 6, 'MarkerFaceColor', '#D9FFFF');
        % for j = 1:params.Num
        %     dx = B(1,j); dy = B(2,j); dz = B(3,j);
        %     if r(1,j) > 0
        %         beta_rad = acos(-dx);
        %     else              
        %         beta_rad = acos(dx);
        %     end
        %     alpha_rad = atan2(dz, dy);
        %     beta_deg = rad2deg(beta_rad);
        %     alpha_deg = mod(rad2deg(alpha_rad), 360);
        %     label_str = sprintf(' %d(\\alpha:%.0f^\\circ, \\beta:%.0f^\\circ)', j, alpha_deg, beta_deg);
        %     text(r(1,j), r(2,j), r(3,j) + 0.15, label_str, 'FontSize', 8, 'FontWeight', 'bold', 'Color', 'k');
        % end
        % 正常推力器用红色实线
        healthy_idx = setdiff(1:params.Num, log_orig.faluty_thrusters);
        plot3(r(1,healthy_idx), r(2,healthy_idx), r(3,healthy_idx), ...
              'o','MarkerSize',7,'MarkerFaceColor',[0.2 0.7 1],'MarkerEdgeColor','k');
        quiver3(r(1,healthy_idx), r(2,healthy_idx), r(3,healthy_idx), ...
                B(1,healthy_idx), B(2,healthy_idx), B(3,healthy_idx), ...
                0.45,'r','LineWidth',1.4);
        % 故障推力器用灰色虚线
        if ~isempty(log_orig.faluty_thrusters)
            plot3(r(1, log_orig.faluty_thrusters), r(2, log_orig.faluty_thrusters), r(3, log_orig.faluty_thrusters), ...
                  'o','MarkerSize',7,'MarkerFaceColor',[0.8 0.8 0.8],'MarkerEdgeColor','k');
            quiver3(r(1, log_orig.faluty_thrusters), r(2, log_orig.faluty_thrusters), r(3, log_orig.faluty_thrusters), ...
                    B(1, log_orig.faluty_thrusters), B(2, log_orig.faluty_thrusters), B(3, log_orig.faluty_thrusters), ...
                    0.45, 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5, 'LineStyle', '--');
        end
        % 推力器编号
        for j = 1:params.Num
            text(r(1,j), r(2,j), r(3,j) + 0.15, sprintf('%d', j), ...
            'FontSize', 8, 'FontWeight', 'bold', 'Color', 'k');
        end
        title(title_str);
        xlabel('X'); ylabel('Y'); zlabel('Z');
        xlim([-3, 3]); ylim([-1.2, 1.2]); zlim([-1.2, 1.2]);axis equal; grid on;
        set(gca,'XDir','reverse','ZDir','reverse');
        view(3);
    end

    %% 评价指标优化前后对比
    figure('Name', '重构评价指标优化前后对比', 'Color','w');
    Matrix_orig = params.F_max * params.B_all;
    Matrix_opt  = params.F_max * B_opt;
    healthy_idx = setdiff(1:params.Num, log_orig.faluty_thrusters);
    M_orig = Matrix_orig(:, healthy_idx);
    M_opt  = Matrix_opt(:, healthy_idx);
    % 连续域的Zonotope包络
    N_h = length(healthy_idx);
    c = 0.5 * ones(N_h,1);
    G = 0.5 * eye(N_h);
    Z_orig = zonotope(M_orig*c, M_orig*G);
    Z_opt  = zonotope(M_opt*c,  M_opt*G);
    % 离散域的全状态点云
    tau = dec2bin(0:2^N_h-1) - '0';
    Pts_orig = M_orig * tau';
    Pts_opt  = M_opt  * tau';  
    warning('off','all');
    % 力空间对比
    subplot(2,2,1);hold on;grid on;
    plot(Z_orig,[1 2 3],'FaceColor', [0.7 1 0.7], 'FaceAlpha', 0.15, 'EdgeColor', 'g');
    plot(Z_opt,[1 2 3], 'FaceColor', [0.9 0.7 0.7], 'FaceAlpha', 0.3, 'EdgeColor', 'r');
    plot3(Pts_orig(1,:), Pts_orig(2,:), Pts_orig(3,:), '.', 'Color', [0, 0.8, 0], 'MarkerSize', 4);
    plot3(Pts_opt(1,:), Pts_opt(2,:), Pts_opt(3,:), '.', 'Color', [0.8, 0, 0], 'MarkerSize', 4);
    plot3(0, 0, 0, 'ko', 'MarkerSize', 3, 'MarkerFaceColor', 'k');
    title('力空间包络与离散点云');xlabel('Fx (N)'); ylabel('Fy (N)'); zlabel('Fz (N)');
    legend('原布局包络','优化布局包络','原布局离散点','优化布局离散点');
    axis equal;view(3);
    % 力矩空间对比
    subplot(2,2,2);hold on;grid on;
    plot(Z_orig, [4 5 6], 'FaceColor', [0.7 1 0.7], 'FaceAlpha', 0.15, 'EdgeColor', 'g');
    plot(Z_opt, [4 5 6], 'FaceColor', [0.9 0.7 0.7], 'FaceAlpha', 0.3, 'EdgeColor', 'r');
    plot3(Pts_orig(4,:), Pts_orig(5,:), Pts_orig(6,:), '.', 'Color', [0, 0.8, 0], 'MarkerSize', 4);
    plot3(Pts_opt(4,:), Pts_opt(5,:), Pts_opt(6,:), '.', 'Color', [0.8, 0, 0], 'MarkerSize', 4);
    plot3(0, 0, 0, 'ko', 'MarkerSize', 3, 'MarkerFaceColor', 'k');
    title('力矩空间包络与离散点云');xlabel('Mx (N·m)'); ylabel('My (N·m)'); zlabel('Mz (N·m)');
    axis equal;view(3);
    warning('on','all');
    % 6维向量夹角热力图
    M_orig = M_orig ./ (vecnorm(M_orig,2,1) + 1e-12);% 归一化
    M_opt = M_opt ./ (vecnorm(M_opt,2,1) + 1e-12);
    cos_orig = M_orig' * M_orig;
    cos_opt  = M_opt'  * M_opt;
    angle_orig = acos(min(max(cos_orig, -1), 1)) * 180 / pi;
    angle_opt  = acos(min(max(cos_opt,  -1), 1)) * 180 / pi;
    subplot(2,2,3);
    h1 = heatmap(healthy_idx, healthy_idx, angle_orig);
    h1.Title = '原布局向量夹角';h1.XLabel = '推进器编号';h1.YLabel = '推进器编号';
    colormap(h1, jet);
    subplot(2,2,4);
    h2 = heatmap(healthy_idx, healthy_idx, angle_opt);
    h2.Title = '优化布局向量夹角';h2.XLabel = '推进器编号';h2.YLabel = '推进器编号';
    colormap(h2, jet);
    
    %% 闭环位置与姿态响应对比
    figure('Name', '闭环位置与姿态响应对比','Color','w');
    subplot(2,2,1); Plot_3Axis(log_orig.Time, log_orig.R, log_orig.Y(1:3,:), log_opt.Y(1:3,:), '位置响应', '(m)', log_orig.faluty_time);
    subplot(2,2,2); Plot_3Axis(log_orig.Time, log_orig.E, log_orig.Y_euler, log_opt.Y_euler, '姿态响应', '(rad)', log_orig.faluty_time);
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
    xline(log_orig.faluty_time,'--r');
    title('位置误差范数'); xlabel('t(s)'); ylabel('||e_r||(m)'); legend('原布局','优化布局');
    subplot(2,2,4); hold on; grid on;
    plot(log_orig.Time, vecnorm(att_err_orig,2,1), 'LineWidth',1.2);
    plot(log_orig.Time, vecnorm(att_err_opt,2,1), 'LineWidth',1.2);
    xline(log_orig.faluty_time,'--r');
    title('姿态误差范数'); xlabel('t(s)'); ylabel('||e_euler||(rad)'); legend('原布局','优化布局');

    %% 推力器控制脉宽对比
    figure('Name', '各推力器脉宽','Color','w');
    for i = 1:params.Num
        subplot(3, 4, i);plot(log_orig.Time, log_orig.Pulse_Widths(i, :));
        title(['推力器 ' num2str(i)]);xlabel('时间(s)');ylabel('脉宽(s)');
        grid on;hold on;
        xline(log_orig.faluty_time, '--r');
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
    xline(log_orig.faluty_time,'--r');
    xlabel('时间(s)'); ylabel('当前控制周期总脉宽(s)'); title('控制周期总脉宽变化');legend('原布局','优化布局');
   
    %% 推力器分配策略输出
    Print_Thruster_Allocation(params.B_all, log_orig.faluty_thrusters, '原布局');
    Print_Thruster_Allocation(B_opt, log_orig.faluty_thrusters, '优化布局');
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
    % fprintf('\n');
    % if log_orig.faluty_thrusters == []
    %     fprintf('======================== 推力器标况下 ========================\n');
    % else
    %     fprintf('=================== 推力器[%s]故障下 ===================\n', num2str(log_orig.faluty_thrusters));
    % end
    % fprintf('推力器按轴分配策略\n');
    % fprintf('--------------------------------------------------------------\n');
    % axes_names = {'X', 'Y', 'Z'};
    % fprintf('【轨道控制推力器分配】\n');
    % for idx1 = 1:3
    %     pos_idx = find(B_opt(idx1, :) > 1e-3);
    %     neg_idx = find(B_opt(idx1, :) < -1e-3);
    %     if log_orig.faluty_thrusters ~= []
    %         pos_idx = setdiff(pos_idx, log_orig.faluty_thrusters);
    %         neg_idx = setdiff(neg_idx, log_orig.faluty_thrusters);
    %     end
    %     fprintf('+%s轴: [%s]\n', axes_names{idx1}, num2str(pos_idx));
    %     fprintf('-%s轴: [%s]\n', axes_names{idx1}, num2str(neg_idx));
    % end
    % fprintf('--------------------------------------------------------------\n');
    % fprintf('【姿态控制推力器分配】\n');
    % for idx2 = 1:3
    %     pos_idx = find(B_opt(idx2+3, :) > 1e-3);
    %     neg_idx = find(B_opt(idx2+3, :) < -1e-3);
    %     if log_orig.faluty_thrusters ~= []
    %         pos_idx = setdiff(pos_idx, log_orig.faluty_thrusters);
    %         neg_idx = setdiff(neg_idx, log_orig.faluty_thrusters);
    %     end
    %     fprintf('+%s轴: [%s]\n', axes_names{idx2}, num2str(pos_idx));
    %     fprintf('-%s轴: [%s]\n', axes_names{idx2}, num2str(neg_idx));
    % end

    %% 不同故障数量下可重构性判定表
    plot_falut_reconfig(params, params.B_all, '原布局:不同数量故障下可重构性判定表');
    plot_falut_reconfig(params, B_opt, '优化布局:不同数量故障下可重构性判定表');
    function plot_falut_reconfig(params, B, title)
        Fault_Num = zeros(params.Num, 1);
        Status = strings(params.Num, 1);
        Reconfig_Count = zeros(params.Num, 1);
        NonReconfig_Count = zeros(params.Num, 1);
        Ratio_Text = strings(params.Num, 1);
        for k_fault = 1:params.Num
            reconfig_num = 0;
            ireconfig_num = 0;
            % 可重构性判定
            [~, Jc_eval, ~, ~, ~, ~] = Reconfig_eval(params, B, k_fault);
            cases = size(nchoosek(1:params.Num, k_fault),1);
            for idx = 1:cases
                Jc_force = Jc_eval(idx+1,1);
                Jc_torque = Jc_eval(idx+1,2);
                is_reconfig = (Jc_force > 1e-10) && (Jc_torque > 1e-10);
                if is_reconfig
                    reconfig_num = reconfig_num + 1;
                else
                    ireconfig_num = ireconfig_num + 1;
                end
            end
            ratio = ireconfig_num / cases * 100;% 不可重构占比

            Fault_Num(k_fault) = k_fault;
            Reconfig_Count(k_fault) = reconfig_num;
            NonReconfig_Count(k_fault) = ireconfig_num;
            Ratio_Text(k_fault) = sprintf('%.2f%%', ratio);

            if ireconfig_num == 0
                Status(k_fault) = "完全可重构";
            elseif reconfig_num == 0
                Status(k_fault) = "不可重构";
            else
                Status(k_fault) = "部分可重构";
            end
        end
        % 绘制表格
        colNames = {'推力器故障数', '是否可重构', '可重构数量', '不可重构数量', '不可重构占比'};
        data = table2cell(table(Fault_Num, Status, Reconfig_Count, NonReconfig_Count, Ratio_Text, ...
            'VariableNames', colNames));
        nRow = size(data, 1);
        fig = figure('Name', title, 'Color', 'w', 'Position', [200, 100, 950, 650]);
        ax = axes(fig);
        axis(ax, 'off');
        hold(ax, 'on');
        % 标题
        text(0.5, 0.96, title, 'HorizontalAlignment', 'center', 'FontSize', 15, 'FontWeight', 'bold');
        % 表格区域参数
        x0 = 0.05;
        x1 = 0.95;
        y_top = 0.88;
        y_bottom = 0.06;
        colWidth = [0.18, 0.18, 0.18, 0.18, 0.18];
        colX = x0 + [0, cumsum(colWidth)];
        colCenter = colX(1:end-1) + colWidth / 2;
        rowH = (y_top - y_bottom) / (nRow + 1);
        % 三线表风格横线
        line([x0, x1], [y_top, y_top], 'Color', 'k', 'LineWidth', 1.8);
        line([x0, x1], [y_top - rowH, y_top - rowH], 'Color', 'k', 'LineWidth', 1.2);
        line([x0, x1], [y_bottom, y_bottom], 'Color', 'k', 'LineWidth', 1.8);
        % 表头
        y_header = y_top - rowH / 2;
        for j = 1:length(colNames)
            text(colCenter(j), y_header, colNames{j}, 'HorizontalAlignment', 'center', 'FontSize', 12, 'FontWeight', 'bold');
        end
        % 表体
        for i = 1:nRow
            y = y_top - rowH * (i + 0.5);
            text(colCenter(1), y, sprintf('%d', data{i,1}), ...
                'HorizontalAlignment', 'center', 'FontSize', 12);
            text(colCenter(2), y, char(data{i,2}), ...
                'HorizontalAlignment', 'center', 'FontSize', 12);
            text(colCenter(3), y, sprintf('%d', data{i,3}), ...
                'HorizontalAlignment', 'center', 'FontSize', 12);
            text(colCenter(4), y, sprintf('%d', data{i,4}), ...
                'HorizontalAlignment', 'center', 'FontSize', 12);
            text(colCenter(5), y, char(data{i,5}), ...
                'HorizontalAlignment', 'center', 'FontSize', 12);
        end
        xlim([0, 1]);
        ylim([0, 1]);
    end
    
    %% 蒙特卡洛打靶仿真结果对比
    % Montecarlo_sim(params, B_opt)
    
end
% %% 综合评价指标输出
    % function [F, W, U_ahp, V_entropy] = Comprehensive(Z)
    %     [m, n] = size(Z);
    %     % 规范化处理
    %     X = zeros(m, n);
    %     for j = 1:n
    %         z_max = max(Z(:, j));
    %         z_min = min(Z(:, j));
    %         if abs(z_max - z_min) < 1e-12
    %             X(:, j) = 1;
    %         else
    %             X(:, j) = (Z(:, j) - z_min) / (z_max - z_min);
    %         end
    %     end
    %     % AHP主观权重
    %     G_AHP = [1,   2,   3,   4;
    %              1/2, 1,   2,   3;
    %              1/3, 1/2, 1,   2;
    %              1/4, 1/3, 1/2, 1];
    %     U_ahp = AHP_Weight(G_AHP);
    %     function U = AHP_Weight(G)
    %         % AHP 主观权重计算（特征向量法）
    %         [V_eig, D_eig] = eig(G);
    %         [~, idx] = max(real(diag(D_eig)));
    %         U = real(V_eig(:, idx));
    %         U = U / sum(U);
    %         % 保证正值
    %         U = abs(U);
    %         U = U / sum(U);
    %     end
    %     % 熵权法客观权重
    %     V_entropy = zeros(n, 1);
    %     for j = 1:n
    %         col_sum = sum(X(:, j));
    %         if col_sum < 1e-12
    %             r = ones(m,1) / m;
    %         else
    %             r = X(:, j) / col_sum;
    %         end
    %         r_valid = r(r > 1e-12);
    %         E_j = -(1 / log(m)) * sum(r_valid .* log(r_valid));
    %         V_entropy(j) = 1 - E_j;
    %     end
    %     if sum(V_entropy) < 1e-12
    %         V_entropy = ones(n,1) / n;
    %     else
    %         V_entropy = V_entropy / sum(V_entropy);
    %     end
    %     % 最小二乘组合赋权
    %     s = sum(X.^2, 1)';
    %     A = diag(s);
    %     B = 0.5 * (U_ahp + V_entropy) .* s;
    %     e = ones(n, 1);
    %     Ainv = inv(A);
    %     W = Ainv * B + ((1 - e' * Ainv * B) / (e' * Ainv * e)) * (Ainv * e);
    %     % 数值修正
    %     W(W < 0) = 0;
    %     if sum(W) < 1e-12
    %         W = ones(n,1) / n;
    %     else
    %         W = W / sum(W);
    %     end
    %     % TOPSIS 综合评价
    %     x_plus  = max(X, [], 1);
    %     x_minus = min(X, [], 1);
    %     L = zeros(m, 1);
    %     D = zeros(m, 1);
    %     F = zeros(m, 1);
    %     for i = 1:m
    %         L(i) = sqrt(sum((W' .* (X(i, :) - x_plus)).^2));
    %         D(i) = sqrt(sum((W' .* (X(i, :) - x_minus)).^2));
    %         F(i) = D(i) / (L(i) + D(i) + 1e-12);
    %     end
    % end
    % %% 输出综合评价指标数据
    % function Print_Layout_Evaluation(StateNames, F_Force_orig, F_Force_opt, ...
    %     F_Torque_orig, F_Torque_opt, F_Total_orig, F_Total_opt, ...
    %     W_Force, W_Torque, W_Total)

    %     nState = length(StateNames);

    %     fprintf('\n==============================================================\n');
    %     fprintf('布局层最终组合权重（统一评价池下）\n');
    %     fprintf('指标顺序：[Jc, Ja, Jf, Jo]\n');
    %     fprintf('--------------------------------------------------------------\n');
    %     fprintf('Force  权重 = [%s]\n', num2str(W_Force',  '%.4f '));
    %     fprintf('Torque 权重 = [%s]\n', num2str(W_Torque', '%.4f '));
    %     fprintf('Total  权重 = [%s]\n', num2str(W_Total',  '%.4f '));
    %     fprintf('==============================================================\n');

    %     fprintf('\n==================== F_layout 状态对比 ====================\n');
    %     fprintf('%-14s | %10s | %10s | %10s\n', '状态', '原布局', '优化布局', '提升(%)');
    %     for i = 1:nState
    %         imp = (F_Total_opt(i) - F_Total_orig(i)) / (F_Total_orig(i) + 1e-12) * 100;
    %         fprintf('%-14s | %10.4f | %10.4f | %+9.2f\n', StateNames{i}, F_Total_orig(i), F_Total_opt(i), imp);
    %     end

    %     fprintf('\n==================== 平均布局综合性能 ====================\n');
    %     fprintf('Force  平均F: 原布局 %.4f, 优化布局 %.4f, 提升 %+6.2f%%\n', ...
    %         mean(F_Force_orig), mean(F_Force_opt), PercentImprove(mean(F_Force_orig), mean(F_Force_opt), true));
    %     fprintf('Torque 平均F: 原布局 %.4f, 优化布局 %.4f, 提升 %+6.2f%%\n', ...
    %         mean(F_Torque_orig), mean(F_Torque_opt), PercentImprove(mean(F_Torque_orig), mean(F_Torque_opt), true));
    %     fprintf('Total  平均F: 原布局 %.4f, 优化布局 %.4f, 提升 %+6.2f%%\n', ...
    %         mean(F_Total_orig), mean(F_Total_opt), PercentImprove(mean(F_Total_orig), mean(F_Total_opt), true));
    % end