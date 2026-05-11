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
    healthy_idx = setdiff(1:params.Num, log_orig.faulty_thrusters);
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
    subplot(2,2,1); Plot_3Axis(log_orig.Time, log_orig.R, log_orig.Y(1:3,:), log_opt.Y(1:3,:), '位置响应', '(m)', log_orig.fault_time);
    subplot(2,2,2); Plot_3Axis(log_orig.Time, log_orig.E, log_orig.Y_euler, log_opt.Y_euler, '姿态响应', '(rad)', log_orig.fault_time);
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
    xline(log_orig.fault_time,'--r');
    title('位置误差范数'); xlabel('t(s)'); ylabel('||e_r||(m)'); legend('原布局','优化布局');
    subplot(2,2,4); hold on; grid on;
    plot(log_orig.Time, vecnorm(att_err_orig,2,1), 'LineWidth',1.2);
    plot(log_orig.Time, vecnorm(att_err_opt,2,1), 'LineWidth',1.2);
    xline(log_orig.fault_time,'--r');
    title('姿态误差范数'); xlabel('t(s)'); ylabel('||e_\euler||(rad)'); legend('原布局','优化布局');

    %% 推力器控制脉宽对比
    figure('Name', '各推力器脉宽','Color','w');
    for i = 1:params.Num
        subplot(3, 4, i);plot(log_orig.Time, log_orig.Pulse_Widths(i, :));
        title(['推力器 ' num2str(i)]);xlabel('时间(s)');ylabel('脉宽(s)');
        grid on;hold on;
        xline(log_orig.fault_time, '--r');
        hold off;
    end
    % 总喷气时长对比
    figure('Name','总喷气时长对比','Color','w');
    subplot(3,1,1); hold on; grid on;
    bar(1:2, [log_orig.Total_Pulse, log_opt.Total_Pulse]);xticks(1:2);xticklabels({'原布局','优化布局'});
    ylabel('总喷气时长(s)'); title('全任务总喷气时长对比'); ylim([0, max(data)*1.2 + eps]);
    subplot(3,1,2); hold on; grid on;
    per_orig = sum(log_orig.Pulse_History, 2);
    per_opt  = sum(log_opt.Pulse_History, 2);
    bar(1:params.Num, [per_orig, per_opt], 'grouped');
    xlabel('推力器编号'); ylabel('累计脉宽(s)'); title('各推力器累计脉宽对比');legend('原布局','优化布局');
    subplot(3,1,3); hold on; grid on;
    plot(log_orig.Control_Time, sum(log_orig.Pulse_History,1), 'LineWidth',1.0);
    plot(log_opt.Control_Time, sum(log_opt.Pulse_History,1), 'LineWidth',1.0);
    xline(log_orig.fault_time,'--r');
    xlabel('时间(s)'); ylabel('当前控制周期总脉宽(s)'); title('控制周期总脉宽变化');legend('原布局','优化布局');
   
    %% 仿真结果数据输出
    fprintf('\n');
    if num_faults == 0
        fprintf('======================== 推力器标况下 ========================\n');
    else
        fprintf('=================== 推力器[%s]故障下 ===================\n', num2str(log_orig.faulty_thrusters));
    end

    fprintf('推力器按轴分配策略\n');
    fprintf('--------------------------------------------------------------\n');
    axes_names = {'X', 'Y', 'Z'};

    fprintf('【轨道控制推力器分配】\n');
    for i = 1:3
        pos_idx = find(B(i, :) > 1e-3);
        neg_idx = find(B(i, :) < -1e-3);
        if num_faults > 0
            pos_idx = setdiff(pos_idx, log_orig.faulty_thrusters);
            neg_idx = setdiff(neg_idx, log_orig.faulty_thrusters);
        end
        fprintf('+%s轴: [%s]\n', axes_names{i}, num2str(pos_idx));
        fprintf('-%s轴: [%s]\n', axes_names{i}, num2str(neg_idx));
    end
    fprintf('--------------------------------------------------------------\n');

    fprintf('【姿态控制推力器分配】\n');
    for i = 1:3
        pos_idx = find(B(i+3, :) > 1e-3);
        neg_idx = find(B(i+3, :) < -1e-3);
        if num_faults > 0
            pos_idx = setdiff(pos_idx, log_orig.faulty_thrusters);
            neg_idx = setdiff(neg_idx, log_orig.faulty_thrusters);
        end
        fprintf('+%s轴: [%s]\n', axes_names{i}, num2str(pos_idx));
        fprintf('-%s轴: [%s]\n', axes_names{i}, num2str(neg_idx));
    end
    fprintf('--------------------------------------------------------------\n');

    fprintf('可重构性综合评价指标 (当前正在仿真的组合)\n');
    fprintf('--------------------------------------------------------------\n');

    actual_data_idx = curr_state_idx + 1; % 补全因为标况(0)带来的索引偏移

    fprintf('->力空间控制能力  :原布局%8.4f N   优化后%8.4f N\n', Jc_F_orig(actual_data_idx), Jc_F_opt(actual_data_idx));
    fprintf('->力矩空间控制能力:原布局%8.4f N*m 优化后%8.4f N*m\n', Jc_T_orig(actual_data_idx), Jc_T_opt(actual_data_idx));
    fprintf('--------------------------------------------------------------\n');
    fprintf('->力空间控制降级  :原布局%8.4f     优化后%8.4f\n', Ja_F_orig(actual_data_idx), Ja_F_opt(actual_data_idx));
    fprintf('->力矩空间控制降级:原布局%8.4f     优化后%8.4f\n', Ja_T_orig(actual_data_idx), Ja_T_opt(actual_data_idx));
    fprintf('--------------------------------------------------------------\n');
    fprintf('->可诊断性        :原布局%8.4f     优化后%8.4f\n', Jo_actual_orig, Jo_actual_opt);
    fprintf('--------------------------------------------------------------\n');

end
