%% 绘图函数
function Plot_results(faulty_thrusters, J, log, params, B_opt, r_opt, B,falut_time)
    %% 系统仿真结果
    figure('Name', '标况与故障动态响应对比', 'Color','w');
    % 推力器状态
    subplot(2,4,1);hold on;grid on;
    Plot_Compare(log.Time, log.R(1:3,:), log.Y(1:3,:), '位置 (m)');
    subplot(2,4,2);hold on;grid on;
    Plot_Compare(log.Time, log.V(1:3,:), log.Y(4:6,:), '速度 (m/s)');
    subplot(2,4,3);hold on;grid on;
    Plot_Compare(log.Time, log.E(1:3,:), log.Y_euler(1:3,:), '欧拉角 (rad)');
    subplot(2,4,4);hold on;grid on;
    Plot_Compare(log.Time, log.O(1:3,:), log.Y(10:12,:), '角速度 (rad/s)');
    subplot(2,4,5);hold on;grid on;
    Plot_Compare(log.Time, log.Y(1:3,:)-log.R(1:3,:),'0', '位置误差 (m)');
    subplot(2,4,6);hold on;grid on;
    Plot_Compare(log.Time, log.Y(4:6,:)-log.V(1:3,:),'0', '速度误差 (m/s)');
    subplot(2,4,7);hold on;grid on;
    Plot_Compare(log.Time, log.Y_euler(1:3,:)-log.E(1:3,:),'0', '欧拉角误差 (rad)');
    subplot(2,4,8);hold on;grid on;
    Plot_Compare(log.Time, log.Y(10:12,:)-log.O(1:3,:),'0', '角速度误差 (rad/s)');

    figure('Name', '推力器控制脉宽');
    for i = 1:params.Num
        subplot(3, 4, i);plot(log.Time, log.Pulse_Widths(i, :));
        title(['推力器 ' num2str(i)]);xlabel('时间 (s)');ylabel('脉宽 (s)');
        grid on;hold on;
        xline(falut_time, '--r');
        hold off;
    end

    % 绘图函数(内部)
    function Plot_Compare(time, Data_exp, Data_act, title_str)
        colors = ["#0072BD", "#D95319", "#77AC30"];
        hold on; grid on;
        p_exp = gobjects(3, 1);
        p_act = gobjects(3, 1);
        if Data_act ~= '0'
            for j = 1:3
                p_exp(j) = plot(time, Data_exp(j, :), '--', 'Color', colors(j));
                p_act(j) = plot(time, Data_act(j, :), '-', 'Color', colors(j));
            end
        else
            for j = 1:3
                p_exp(j) = plot(time, Data_exp(j, :), '-', 'Color', colors(j));
            end
        end
        title(title_str);
        xline(falut_time, '--r');
    end

    %% 推力器布局优化对比
    figure('Name', '推力器布局优化对比', 'Color','w', 'Position', [100, 100, 1400, 800]);
    % 推力器布局
    subplot(2, 3, 1); hold on;
    Plot_place(params.r_all, params.B_all, '优化前-推力器布局示意图');
    subplot(2, 3, 4); hold on;
    Plot_place(r_opt, B_opt, '优化后-推力器布局示意图');

    % 绘图函数(内部)
    function Plot_place(r, B, title_str)
        Lx = 2; Ly = 0.6; Lz = 0.6;
        vert = [-Lx -Ly -Lz; Lx -Ly -Lz; Lx Ly -Lz; -Lx Ly -Lz;
            -Lx -Ly Lz; Lx -Ly Lz; Lx Ly Lz; -Lx Ly Lz];
        fac = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
        patch('Vertices', vert, 'Faces', fac, 'FaceColor', [0.8 0.8 0.8], ...
            'FaceAlpha', 0.2, 'EdgeColor', '#999999');
        plot3(r(1,:), r(2,:), r(3,:), 'o', 'Color', 'b', 'MarkerSize', 6, 'MarkerFaceColor', '#D9FFFF');
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
            text(r(1,j), r(2,j), r(3,j) + 0.15, label_str, 'FontSize', 8, 'FontWeight', 'bold', 'Color', 'k');
        end
        % 利用 setdiff 获取未故障推力器编号
        healthy_idx = setdiff(1:params.Num, faulty_thrusters);
        quiver3(r(1, healthy_idx), r(2, healthy_idx), r(3, healthy_idx), ...
            B(1, healthy_idx), B(2, healthy_idx), B(3, healthy_idx), ...
            0.5, 'r', 'LineWidth', 1.5, 'LineStyle', '-');

        if ~isempty(faulty_thrusters)
            % 支持多台推力器同时变为灰色虚线
            quiver3(r(1, faulty_thrusters), r(2, faulty_thrusters), r(3, faulty_thrusters), ...
                B(1, faulty_thrusters), B(2, faulty_thrusters), B(3, faulty_thrusters), ...
                0.5, 'Color', [0.6 0.6 0.6], 'LineWidth', 1.5, 'LineStyle', '--');
        end
        title(title_str);xlabel('X'); ylabel('Y'); zlabel('Z');
        xlim([-3, 3]); ylim([-1.2, 1.2]); zlim([-1.2, 1.2]);axis equal; grid on;
        set(gca, 'XDir', 'reverse');set(gca, 'ZDir', 'reverse');view(3);
    end

    % 包络空间与离散点云
    Matrix_conf_opt = params.F_max * B_opt;
    healthy_idx = setdiff(1:params.Num, faulty_thrusters); % 获取健康推力器索引
    Mf_all = params.Matrix_conf(:, healthy_idx);
    Mf_opt = Matrix_conf_opt(:, healthy_idx);
    Num_healthy = size(Mf_all, 2);

    % 连续域的Zonotope包络
    Z_all = zonotope(params.Matrix_conf * (0.5 * ones(params.Num, 1)), params.Matrix_conf * (0.5 * diag(ones(1, params.Num))));
    all_fault = zonotope(Mf_all * (0.5 * ones(Num_healthy, 1)), Mf_all * (0.5 * diag(ones(1, Num_healthy))));
    Z_opt = zonotope(Matrix_conf_opt * (0.5 * ones(params.Num,1)), Matrix_conf_opt * (0.5 * diag(ones(1,params.Num))));
    opt_fault = zonotope(Mf_opt * (0.5 * ones(Num_healthy, 1)), Mf_opt * (0.5 * diag(ones(1, Num_healthy))));

    % 离散域的全状态点云
    states = 2^Num_healthy; 
    tau = dec2bin(0:states-1) - '0';
    % 计算所有离散组合下的力和力矩点阵
    Pts_all = Mf_all * tau';% 优化前点云
    Pts_opt = Mf_opt * tau';% 优化后点云

    warning('off','all');
    % --- 力空间对比 ---
    subplot(2, 3, 2); hold on;
    % 包络面
    plot(all_fault, [1 2 3], 'FaceColor', [0.7 1 0.7], 'FaceAlpha', 0.15, 'EdgeColor', 'g');
    plot(opt_fault, [1 2 3], 'FaceColor', [0.9 0.7 0.7], 'FaceAlpha', 0.3, 'EdgeColor', 'r');
    % 离散点
    plot3(Pts_all(1,:), Pts_all(2,:), Pts_all(3,:), '.', 'Color', [0, 0.8, 0], 'MarkerSize', 4);
    plot3(Pts_opt(1,:), Pts_opt(2,:), Pts_opt(3,:), '.', 'Color', [0.8, 0, 0], 'MarkerSize', 4);
    % 原点
    plot3(0, 0, 0, 'ko', 'MarkerSize', 3, 'MarkerFaceColor', 'k');

    if ~isempty(faulty_thrusters)
        title(sprintf('第[%s]台故障-力空间对比', num2str(faulty_thrusters)));
    else
        title('标况-力空间对比');
    end
    xlabel('Fx (N)'); ylabel('Fy (N)'); zlabel('Fz (N)');
    axis equal; grid on; view(3);

    % --- 力矩空间对比 ---
    subplot(2, 3, 5); hold on;
    % 包络面
    plot(all_fault, [4 5 6], 'FaceColor', [0.7 1 0.7], 'FaceAlpha', 0.15, 'EdgeColor', 'g');
    plot(opt_fault, [4 5 6], 'FaceColor', [0.9 0.7 0.7], 'FaceAlpha', 0.3, 'EdgeColor', 'r');
    % 离散点
    plot3(Pts_all(4,:), Pts_all(5,:), Pts_all(6,:), '.', 'Color', [0, 0.8, 0], 'MarkerSize', 4);
    plot3(Pts_opt(4,:), Pts_opt(5,:), Pts_opt(6,:), '.', 'Color', [0.8, 0, 0], 'MarkerSize', 4);
    % 原点
    plot3(0, 0, 0, 'ko', 'MarkerSize', 3, 'MarkerFaceColor', 'k');

    if ~isempty(faulty_thrusters)
        title(sprintf('第[%s]台故障-力矩空间对比', num2str(faulty_thrusters)));
    else
        title('标况-力矩空间对比');
    end
    xlabel('Mx (N·m)'); ylabel('My (N·m)'); zlabel('Mz (N·m)');
    axis equal; grid on; view(3);
    warning('on','all');

    % --- 控制能力 (预计算单故障评估展示，保持不变) ---
    % idx_list = 0:params.Num;
    % Jc_F_orig = zeros(params.Num+1, 1); Jc_F_opt = zeros(params.Num+1, 1);
    % Jc_T_orig = zeros(params.Num+1, 1); Jc_T_opt = zeros(params.Num+1, 1);
    % Ja_F_orig = zeros(params.Num+1, 1); Ja_F_opt = zeros(params.Num+1, 1);
    % Ja_T_orig = zeros(params.Num+1, 1); Ja_T_opt = zeros(params.Num+1, 1);
    % for k = 1:params.Num+1
    %     idx = idx_list(k);
    %     Jc_F_orig(k) = J(idx*5+1, 1); Jc_F_opt(k) = J(idx*5+1, 2);
    %     Jc_T_orig(k) = J(idx*5+2, 1); Jc_T_opt(k) = J(idx*5+2, 2);
    %     Ja_F_orig(k) = J(idx*5+3, 1); Ja_F_opt(k) = J(idx*5+3, 2);
    %     Ja_T_orig(k) = J(idx*5+4, 1); Ja_T_opt(k) = J(idx*5+4, 2);
    % end
    % 
    % % 力空间评估图
    % subplot(2, 3, 3); hold on; grid on;
    % ax1 = gca;
    % yyaxis left;
    % b1 = bar(idx_list, [Jc_F_orig, Jc_F_opt], 'grouped');
    % b1(1).FaceColor = [0.7 1 0.7]; b1(1).EdgeColor = [0 0.8 0]; b1(1).LineWidth = 1;
    % b1(2).FaceColor = [0.9 0.7 0.7]; b1(2).EdgeColor = [0.8 0 0]; b1(2).LineWidth = 1;
    % ylabel('控制能力 Jc (N)'); ax1.YAxis(1).Color = 'k';
    % ylim([0, max([Jc_F_orig; Jc_F_opt]) * 1.2]);
    % yyaxis right;
    % p1 = plot(idx_list, Ja_F_orig * 100, '-o', 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5, 'MarkerSize', 5, 'MarkerEdgeColor', [0 0.8 0], 'MarkerFaceColor', [0 0.8 0]);
    % p2 = plot(idx_list, Ja_F_opt * 100, '-o', 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5, 'MarkerSize', 5, 'MarkerEdgeColor', [0.8 0 0], 'MarkerFaceColor', [0.8 0 0]);
    % ylabel('控制降级 Ja (%)'); ax1.YAxis(2).Color = 'k';
    % ylim([-10, 110]);
    % title('力空间-评价指标对比'); xlabel('故障推力器编号'); xticks(idx_list);
    % h1 = patch(NaN, NaN, [0.7 1 0.7], 'EdgeColor', 'g'); h2 = patch(NaN, NaN, [0.9 0.7 0.7], 'EdgeColor', 'r');
    % h3 = plot(NaN, NaN, '-o', 'Color', [0.7 0.7 0.7], 'MarkerSize', 6, 'MarkerFaceColor', [0 0.8 0], 'MarkerEdgeColor', [0 0.8 0]);
    % h4 = plot(NaN, NaN, '-o', 'Color', [0.7 0.7 0.7], 'MarkerSize', 6, 'MarkerFaceColor', [0.8 0 0], 'MarkerEdgeColor', [0.8 0 0]);
    % legend([h1, h2, h3, h4],{'优化前能力', '优化后能力', '优化前降级', '优化后降级'},'NumColumns', 2, 'FontSize', 8);
    % 
    % % 力矩空间评估图
    % subplot(2, 3, 6); hold on; grid on;
    % ax2 = gca;
    % yyaxis left;
    % b2 = bar(idx_list, [Jc_T_orig, Jc_T_opt], 'grouped');
    % b2(1).FaceColor = [0.7 1 0.7]; b2(1).EdgeColor = [0 0.8 0]; b2(1).LineWidth = 1;
    % b2(2).FaceColor = [0.9 0.7 0.7]; b2(2).EdgeColor = [0.8 0 0]; b2(2).LineWidth = 1;
    % ylabel('控制能力 Jc (N\cdotm)'); ax2.YAxis(1).Color = 'k';
    % ylim([0, max([Jc_T_orig; Jc_T_opt]) * 1.2]);
    % yyaxis right;
    % p3 = plot(idx_list, Ja_T_orig * 100, '-o', 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5, 'MarkerSize', 5, 'MarkerEdgeColor', [0 0.8 0], 'MarkerFaceColor', [0 0.8 0]);
    % p4 = plot(idx_list, Ja_T_opt * 100, '-o', 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5, 'MarkerSize', 5, 'MarkerEdgeColor', [0.8 0 0], 'MarkerFaceColor', [0.8 0 0]);
    % ylabel('控制降级 Ja (%)'); ax2.YAxis(2).Color = 'k';
    % ylim([-10, 110]);
    % title('力矩空间-评价指标对比'); xlabel('故障推力器编号'); xticks(idx_list);
    % 
    % %% 分配策略和评价指标控制台输出
    % fprintf('\n');
    % if isempty(faulty_thrusters)
    %     fprintf('======================== 推力器标况下 ========================\n');
    % else
    %     fprintf('=================== 推力器[%s]故障下 ===================\n', num2str(faulty_thrusters));
    % end
    % 
    % fprintf('推力器按轴分配策略\n');
    % fprintf('--------------------------------------------------------------\n');
    % axes_names = {'X', 'Y', 'Z'};
    % 
    % fprintf('【轨道控制推力器分配】\n');
    % for i = 1:3
    %     pos_idx = find(B(i, :) > 1e-3);
    %     neg_idx = find(B(i, :) < -1e-3);
    %     % 剔除故障推力器
    %     if ~isempty(faulty_thrusters)
    %         pos_idx = setdiff(pos_idx, faulty_thrusters);
    %         neg_idx = setdiff(neg_idx, faulty_thrusters);
    %     end
    %     fprintf('+%s轴: [%s]\n', axes_names{i}, num2str(pos_idx));
    %     fprintf('-%s轴: [%s]\n', axes_names{i}, num2str(neg_idx));
    % end
    % fprintf('--------------------------------------------------------------\n');
    % 
    % fprintf('【姿态控制推力器分配】\n');
    % for i = 1:3
    %     pos_idx = find(B(i+3, :) > 1e-3);
    %     neg_idx = find(B(i+3, :) < -1e-3);
    %     if ~isempty(faulty_thrusters)
    %         pos_idx = setdiff(pos_idx, faulty_thrusters);
    %         neg_idx = setdiff(neg_idx, faulty_thrusters);
    %     end
    %     fprintf('+%s轴: [%s]\n', axes_names{i}, num2str(pos_idx));
    %     fprintf('-%s轴: [%s]\n', axes_names{i}, num2str(neg_idx));
    % end
    % fprintf('--------------------------------------------------------------\n');
    % 
    % % fprintf('可重构性综合评价指标\n');
    % % fprintf('--------------------------------------------------------------\n');
    % 
    % % % 根据故障台数决定是否输出 J 矩阵的特定指标（多故障未计算，需防护）
    % % if isempty(faulty_thrusters) || length(faulty_thrusters) == 1
    % %     % f_idx: 标况为0，单故障为具体台号
    % %     if isempty(faulty_thrusters)
    % %         f_idx = 0;
    % %     else
    % %         f_idx = faulty_thrusters(1);
    % %     end
    % % 
    % %     fprintf('->力空间控制能力  :原布局%8.4f N   优化后%8.4f N\n', J(f_idx*5+1,1), J(f_idx*5+1,2));
    % %     fprintf('->力矩空间控制能力:原布局%8.4f N*m 优化后%8.4f N*m\n', J(f_idx*5+2,1), J(f_idx*5+2,2));
    % %     fprintf('--------------------------------------------------------------\n');
    % %     fprintf('->力空间控制降级  :原布局%8.4f     优化后%8.4f\n', J(f_idx*5+3,1), J(f_idx*5+3,2));
    % %     fprintf('->力矩空间控制降级:原布局%8.4f     优化后%8.4f\n', J(f_idx*5+4,1), J(f_idx*5+4,2));
    % %     fprintf('--------------------------------------------------------------\n');
    % %     fprintf('->可诊断性        :原布局%8.4f     优化后%8.4f\n', J(f_idx*5+5,1), J(f_idx*5+5,2));
    % %     fprintf('--------------------------------------------------------------\n');
    % % else
    % %     fprintf('（当前为多台推力器组合故障，未在J矩阵中预计算其专项指标，故跳过展示）\n');
    % %     fprintf('--------------------------------------------------------------\n');
    % % end

    % =========================================================================
    % 控制能力与控制降级 (按同级故障穷举遍历)
    % =========================================================================
    num_faults = length(faulty_thrusters);

    if num_faults <= 1
        % 【情况A：标况或单台故障】直接使用主程序预计算的 J 矩阵 (0~12)
        num_cases = params.Num;
        idx_list = 0:num_cases;
        Jc_F_orig = zeros(num_cases+1, 1); Jc_F_opt = zeros(num_cases+1, 1);
        Jc_T_orig = zeros(num_cases+1, 1); Jc_T_opt = zeros(num_cases+1, 1);
        Ja_F_orig = zeros(num_cases+1, 1); Ja_F_opt = zeros(num_cases+1, 1);
        Ja_T_orig = zeros(num_cases+1, 1); Ja_T_opt = zeros(num_cases+1, 1);

        for k = 1:num_cases+1
            Jc_F_orig(k) = J(k*6-5, 1); Jc_F_opt(k) = J(k*6-5, 2);
            Jc_T_orig(k) = J(k*6-4, 1); Jc_T_opt(k) = J(k*6-4, 2);
            Ja_F_orig(k) = J(k*6-3, 1); Ja_F_opt(k) = J(k*6-3, 2);
            Ja_T_orig(k) = J(k*6-2, 1); Ja_T_opt(k) = J(k*6-2, 2);
        end

        if num_faults == 0
            curr_state_idx = 0;
            Jo_actual_orig = J(5,1); Jo_actual_opt = J(5,2);
        else
            curr_state_idx = faulty_thrusters(1);
            Jo_actual_orig = J((curr_state_idx*6)+5, 1); 
            Jo_actual_opt = J((curr_state_idx*6)+5, 2);
        end
        x_label_title = '单台故障推力器编号';

    else
        % 【情况B：多台组合故障】动态遍历所有同级故障组合
        all_combs = nchoosek(1:params.Num, num_faults);
        num_cases = size(all_combs, 1);
        idx_list = 0:num_cases; % 0代表标况，1~N代表各种组合

        % 预分配内存 (包含下标0的标况)
        Jc_F_orig = zeros(num_cases+1, 1); Jc_F_opt = zeros(num_cases+1, 1);
        Jc_T_orig = zeros(num_cases+1, 1); Jc_T_opt = zeros(num_cases+1, 1);
        Ja_F_orig = zeros(num_cases+1, 1); Ja_F_opt = zeros(num_cases+1, 1);
        Ja_T_orig = zeros(num_cases+1, 1); Ja_T_opt = zeros(num_cases+1, 1);

        % 提取标况 (从J矩阵的第1组读取)
        Jc_F_orig(1) = J(1, 1); Jc_F_opt(1) = J(1, 2);
        Jc_T_orig(1) = J(2, 1); Jc_T_opt(1) = J(2, 2);
        Ja_F_orig(1) = J(3, 1); Ja_F_opt(1) = J(3, 2);
        Ja_T_orig(1) = J(4, 1); Ja_T_opt(1) = J(4, 2);

        curr_state_idx = -1;
        Jo_actual_orig = 0; Jo_actual_opt = 0;

        % 遍历计算所有组合
        for i = 1:num_cases
            comb = all_combs(i, :);
            [~, Jc_opt_val, ~, Ja_opt_val, Jo_opt_val] = Reconfig_eval(params, B_opt, comb);
            [~, Jc_orig_val, ~, Ja_orig_val, Jo_orig_val] = Reconfig_eval(params, params.B_all, comb);

            Jc_F_orig(i+1) = Jc_orig_val(1); Jc_F_opt(i+1) = Jc_opt_val(1);
            Jc_T_orig(i+1) = Jc_orig_val(2); Jc_T_opt(i+1) = Jc_opt_val(2);
            Ja_F_orig(i+1) = Ja_orig_val(1); Ja_F_opt(i+1) = Ja_opt_val(1);
            Ja_T_orig(i+1) = Ja_orig_val(2); Ja_T_opt(i+1) = Ja_opt_val(2);

            % 记录当前正在发生的这组组合的数据
            if isequal(sort(comb), sort(faulty_thrusters))
                curr_state_idx = i; 
                Jo_actual_orig = Jo_orig_val;
                Jo_actual_opt = Jo_opt_val;
            end
        end
        x_label_title = sprintf('%d台故障的各组合编号', num_faults);
    end
figure;
    % --- 力空间评估图 ---
    subplot(2, 1, 1); 
    hold on; grid on; ax1 = gca;
    % 左Y轴：控制能力
    yyaxis left;
    b1 = bar(idx_list, [Jc_F_orig, Jc_F_opt], 'grouped');
    b1(1).FaceColor = [0.7 1 0.7]; b1(1).EdgeColor = [0 0.8 0]; b1(1).LineWidth = 1;
    b1(2).FaceColor = [0.9 0.7 0.7]; b1(2).EdgeColor = [0.8 0 0]; b1(2).LineWidth = 1;
    ylabel('控制能力 Jc (N)'); ax1.YAxis(1).Color = 'k';
    ylim([0, max([Jc_F_orig; Jc_F_opt]) * 1.2]);

    % 右Y轴：控制降级
    yyaxis right;
    p1 = plot(idx_list, Ja_F_orig * 100, '-o', 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5, 'MarkerSize', 4, 'MarkerEdgeColor', [0 0.8 0], 'MarkerFaceColor', [0 0.8 0]);
    p2 = plot(idx_list, Ja_F_opt * 100, '-o', 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5, 'MarkerSize', 4, 'MarkerEdgeColor', [0.8 0 0], 'MarkerFaceColor', [0.8 0 0]);
    ylabel('控制降级 Ja (%)'); ax1.YAxis(2).Color = 'k'; ylim([-10, 110]);

    title('力空间-同级故障评价指标对比'); xlabel(x_label_title); 

    % 优化 X 轴刻度显示 (避免组合太多时横坐标变成一团黑)
    if num_cases > 20
        xticks(unique([0, 10:10:num_cases, curr_state_idx]));
    else
        xticks(idx_list);
    end

    % 高亮标注当前仿真运行的故障状态
    if num_faults > 0
        xline(curr_state_idx, '--k', sprintf('当前组合[%s]', num2str(faulty_thrusters)), ...
            'LineWidth', 1.5, 'LabelVerticalAlignment', 'top', 'LabelOrientation', 'horizontal');
    else
        xline(0, '--k', '当前标况', 'LineWidth', 1.5, 'LabelVerticalAlignment', 'top');
    end

    h1 = patch(NaN, NaN, [0.7 1 0.7], 'EdgeColor', 'g'); h2 = patch(NaN, NaN, [0.9 0.7 0.7], 'EdgeColor', 'r');
    h3 = plot(NaN, NaN, '-o', 'Color', [0.7 0.7 0.7], 'MarkerSize', 6, 'MarkerFaceColor', [0 0.8 0], 'MarkerEdgeColor', [0 0.8 0]);
    h4 = plot(NaN, NaN, '-o', 'Color', [0.7 0.7 0.7], 'MarkerSize', 6, 'MarkerFaceColor', [0.8 0 0], 'MarkerEdgeColor', [0.8 0 0]);
    legend([h1, h2, h3, h4],{'优化前能力', '优化后能力', '优化前降级', '优化后降级'},'NumColumns', 2, 'FontSize', 8);

    % --- 力矩空间评估图 ---
    % subplot(2, 3, 6); 
    subplot(2, 1, 2);
    hold on; grid on; ax2 = gca;
    % 左Y轴：控制能力
    yyaxis left;
    b2 = bar(idx_list, [Jc_T_orig, Jc_T_opt], 'grouped');
    b2(1).FaceColor = [0.7 1 0.7]; b2(1).EdgeColor = [0 0.8 0]; b2(1).LineWidth = 1;
    b2(2).FaceColor = [0.9 0.7 0.7]; b2(2).EdgeColor = [0.8 0 0]; b2(2).LineWidth = 1;
    ylabel('控制能力 Jc (N\cdotm)'); ax2.YAxis(1).Color = 'k';
    ylim([0, max([Jc_T_orig; Jc_T_opt]) * 1.2]);

    % 右Y轴：控制降级
    yyaxis right;
    p3 = plot(idx_list, Ja_T_orig * 100, '-o', 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5, 'MarkerSize', 4, 'MarkerEdgeColor', [0 0.8 0], 'MarkerFaceColor', [0 0.8 0]);
    p4 = plot(idx_list, Ja_T_opt * 100, '-o', 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5, 'MarkerSize', 4, 'MarkerEdgeColor', [0.8 0 0], 'MarkerFaceColor', [0.8 0 0]);
    ylabel('控制降级 Ja (%)'); ax2.YAxis(2).Color = 'k'; ylim([-10, 110]);

    title('力矩空间-同级故障评价指标对比'); xlabel(x_label_title); 

    % 优化 X 轴刻度显示
    if num_cases > 20
        xticks(unique([0, 10:10:num_cases, curr_state_idx]));
    else
        xticks(idx_list);
    end

    % 高亮标注当前仿真运行的故障状态
    if num_faults > 0
        xline(curr_state_idx, '--k', sprintf('当前组合[%s]', num2str(faulty_thrusters)), ...
            'LineWidth', 1.5, 'LabelVerticalAlignment', 'top', 'LabelOrientation', 'horizontal');
    else
        xline(0, '--k', '当前标况', 'LineWidth', 1.5, 'LabelVerticalAlignment', 'top');
    end

    %% =========================================================================
    % 分配策略和评价指标控制台输出
    % =========================================================================
    fprintf('\n');
    if num_faults == 0
        fprintf('======================== 推力器标况下 ========================\n');
    else
        fprintf('=================== 推力器[%s]故障下 ===================\n', num2str(faulty_thrusters));
    end

    fprintf('推力器按轴分配策略\n');
    fprintf('--------------------------------------------------------------\n');
    axes_names = {'X', 'Y', 'Z'};

    fprintf('【轨道控制推力器分配】\n');
    for i = 1:3
        pos_idx = find(B(i, :) > 1e-3);
        neg_idx = find(B(i, :) < -1e-3);
        if num_faults > 0
            pos_idx = setdiff(pos_idx, faulty_thrusters);
            neg_idx = setdiff(neg_idx, faulty_thrusters);
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
            pos_idx = setdiff(pos_idx, faulty_thrusters);
            neg_idx = setdiff(neg_idx, faulty_thrusters);
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
% function Plot_results(params, B_opt, r_opt, log_orig, log_opt, Perf_orig, Perf_opt, ...
%                             fault_info, Z_Force_orig, Z_Force_opt, Z_Torque_orig, Z_Torque_opt, ...
%                             Z_Total_orig, Z_Total_opt, F_Force_orig, F_Force_opt, ...
%                             F_Torque_orig, F_Torque_opt, F_Total_orig, F_Total_opt, StateNames, eval_state_idx)
% 
%     B_orig = params.B_all;
%     r_orig = params.r_all;
% 
%     faulty_thrusters = fault_info.faulty_thrusters;
%     fault_time = fault_info.fault_time;
% 
%     %% 1. 推力器布局优化前后图
%     Plot_Layout_Compare(params, r_orig, B_orig, r_opt, B_opt, faulty_thrusters);
% 
%     %% 2. 控制能力包络图 Jc + 离散点云 Ja
%     Plot_Envelope_PointCloud(params, B_orig, B_opt, faulty_thrusters);
% 
%     %% 3. 四类指标在所有故障状态下对比
%     Plot_Index_Compare_AllStates(Z_Total_orig, Z_Total_opt, F_Total_orig, F_Total_opt, StateNames);
% 
%     %% 4. Force / Torque 综合指标对比
%     Plot_F_Compare(F_Force_orig, F_Force_opt, F_Torque_orig, F_Torque_opt, ...
%                    F_Total_orig, F_Total_opt, StateNames);
% 
%     %% 5. 闭环位置/姿态及误差对比
%     Plot_ClosedLoop_Response(log_orig, log_opt, fault_time);
% 
%     %% 6. 推力器总喷气时长和脉宽对比
%     Plot_Pulse_Compare(params, log_orig, log_opt, Perf_orig, Perf_opt, fault_time);
% 
%     %% 7. 输出结果表
%     Print_Result_Tables(Z_Total_orig, Z_Total_opt, F_Total_orig, F_Total_opt, ...
%                         Perf_orig, Perf_opt, StateNames, eval_state_idx);
% 
% end
% function Plot_Layout_Compare(params, r_orig, B_orig, r_opt, B_opt, faulty_thrusters)
% 
%     figure('Name','推力器布局优化前后对比','Color','w','Position',[100 100 1300 600]);
% 
%     subplot(1,2,1);
%     Plot_Thruster_Layout(params, r_orig, B_orig, faulty_thrusters, '优化前推力器布局');
% 
%     subplot(1,2,2);
%     Plot_Thruster_Layout(params, r_opt, B_opt, faulty_thrusters, '优化后推力器布局');
% end
% 
% function Plot_Thruster_Layout(params, r, B, faulty_thrusters, title_str)
% 
%     hold on; grid on;
% 
%     Lx = 2; Ly = 0.6; Lz = 0.6;
%     vert = [-Lx -Ly -Lz; Lx -Ly -Lz; Lx Ly -Lz; -Lx Ly -Lz;
%             -Lx -Ly  Lz; Lx -Ly  Lz; Lx Ly  Lz; -Lx Ly  Lz];
%     fac = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
% 
%     patch('Vertices',vert,'Faces',fac,'FaceColor',[0.8 0.8 0.8], ...
%           'FaceAlpha',0.18,'EdgeColor',[0.6 0.6 0.6]);
% 
%     healthy_idx = setdiff(1:params.Num, faulty_thrusters);
% 
%     plot3(r(1,healthy_idx), r(2,healthy_idx), r(3,healthy_idx), ...
%           'o','MarkerSize',7,'MarkerFaceColor',[0.2 0.7 1],'MarkerEdgeColor','k');
% 
%     quiver3(r(1,healthy_idx), r(2,healthy_idx), r(3,healthy_idx), ...
%             B(1,healthy_idx), B(2,healthy_idx), B(3,healthy_idx), ...
%             0.45,'r','LineWidth',1.4);
% 
%     if ~isempty(faulty_thrusters)
%         plot3(r(1,faulty_thrusters), r(2,faulty_thrusters), r(3,faulty_thrusters), ...
%               's','MarkerSize',9,'MarkerFaceColor',[0.5 0.5 0.5],'MarkerEdgeColor','k');
% 
%         quiver3(r(1,faulty_thrusters), r(2,faulty_thrusters), r(3,faulty_thrusters), ...
%                 B(1,faulty_thrusters), B(2,faulty_thrusters), B(3,faulty_thrusters), ...
%                 0.45,'Color',[0.5 0.5 0.5],'LineStyle','--','LineWidth',1.4);
%     end
% 
%     for j = 1:params.Num
%         text(r(1,j), r(2,j), r(3,j)+0.12, sprintf('%d',j), ...
%              'FontSize',9,'FontWeight','bold','Color','k');
%     end
% 
%     title(title_str);
%     xlabel('X'); ylabel('Y'); zlabel('Z');
%     axis equal;
%     xlim([-3 3]); ylim([-1.2 1.2]); zlim([-1.2 1.2]);
%     set(gca,'XDir','reverse','ZDir','reverse');
%     view(3);
% end
% function Plot_Envelope_PointCloud(params, B_orig, B_opt, faulty_thrusters)
% 
%     figure('Name','控制包络与离散点云对比','Color','w','Position',[100 100 1400 650]);
% 
%     Matrix_orig = params.F_max * B_orig;
%     Matrix_opt  = params.F_max * B_opt;
% 
%     healthy_idx = setdiff(1:params.Num, faulty_thrusters);
% 
%     M_orig = Matrix_orig(:, healthy_idx);
%     M_opt  = Matrix_opt(:, healthy_idx);
% 
%     N_h = length(healthy_idx);
%     tau = dec2bin(0:2^N_h-1) - '0';
% 
%     Pts_orig = M_orig * tau';
%     Pts_opt  = M_opt  * tau';
% 
%     c = 0.5 * ones(N_h,1);
%     G = 0.5 * eye(N_h);
% 
%     Z_orig = zonotope(M_orig*c, M_orig*G);
%     Z_opt  = zonotope(M_opt*c,  M_opt*G);
% 
%     warning('off','all');
% 
%     subplot(1,2,1); hold on; grid on;
%     plot(Z_orig,[1 2 3],'FaceColor',[0.6 1 0.6],'FaceAlpha',0.15,'EdgeColor',[0 0.6 0]);
%     plot(Z_opt, [1 2 3],'FaceColor',[1 0.6 0.6],'FaceAlpha',0.20,'EdgeColor',[0.8 0 0]);
%     plot3(Pts_orig(1,:),Pts_orig(2,:),Pts_orig(3,:),'.','Color',[0 0.55 0],'MarkerSize',4);
%     plot3(Pts_opt(1,:), Pts_opt(2,:), Pts_opt(3,:),'.','Color',[0.75 0 0],'MarkerSize',4);
%     plot3(0,0,0,'ko','MarkerFaceColor','k');
%     title('力空间包络与离散点云');
%     xlabel('Fx'); ylabel('Fy'); zlabel('Fz');
%     legend('原布局包络','优化布局包络','原布局离散点','优化布局离散点','原点');
%     axis equal; view(3);
% 
%     subplot(1,2,2); hold on; grid on;
%     plot(Z_orig,[4 5 6],'FaceColor',[0.6 1 0.6],'FaceAlpha',0.15,'EdgeColor',[0 0.6 0]);
%     plot(Z_opt, [4 5 6],'FaceColor',[1 0.6 0.6],'FaceAlpha',0.20,'EdgeColor',[0.8 0 0]);
%     plot3(Pts_orig(4,:),Pts_orig(5,:),Pts_orig(6,:),'.','Color',[0 0.55 0],'MarkerSize',4);
%     plot3(Pts_opt(4,:), Pts_opt(5,:), Pts_opt(6,:),'.','Color',[0.75 0 0],'MarkerSize',4);
%     plot3(0,0,0,'ko','MarkerFaceColor','k');
%     title('力矩空间包络与离散点云');
%     xlabel('Mx'); ylabel('My'); zlabel('Mz');
%     legend('原布局包络','优化布局包络','原布局离散点','优化布局离散点','原点');
%     axis equal; view(3);
% 
%     warning('on','all');
% end
% function Plot_Index_Compare_AllStates(Z_orig, Z_opt, F_orig, F_opt, StateNames)
% 
%     idx = 0:length(F_orig)-1;
% 
%     figure('Name','所有故障状态下综合指标对比','Color','w','Position',[80 80 1400 800]);
% 
%     % names = {'Jc_{all}','Ja_{all}','Jo','Jf','F_{Total}'};
%     names = {'Jc','Ja','Jf','Jo','F'};
%     data_orig = [Z_orig, F_orig];
%     data_opt  = [Z_opt,  F_opt];
% 
%     for j = 1:5
%         subplot(3,2,j); hold on; grid on;
%         bar(idx, [data_orig(:,j), data_opt(:,j)], 'grouped');
%         title([names{j} '对比']);
%         xlabel('故障编号');
%         ylabel(names{j});
%         legend('原布局','优化布局','Location','best');
%         xticks(idx);
%     end
% 
%     % subplot(3,2,6); hold on; grid on;
%     % improvement = (F_opt - F_orig) ./ (abs(F_orig) + 1e-12) * 100;
%     % bar(idx, improvement);
%     % yline(0,'--k');
%     % title('F_{Total} 提升率');
%     % xlabel('故障状态编号');
%     % ylabel('提升率 (%)');
%     % xticks(idx);
% end
% function Plot_F_Compare(F_Force_orig, F_Force_opt, F_Torque_orig, F_Torque_opt, ...
%                         F_Total_orig, F_Total_opt, StateNames)
% 
%     idx = 0:length(F_Total_orig)-1;
% 
%     figure('Name','Force / Torque / Total 综合评价值对比','Color','w','Position',[100 100 1400 700]);
% 
%     subplot(3,1,1); hold on; grid on;
%     bar(idx,[F_Force_orig,F_Force_opt],'grouped');
%     title('Force 综合评价值 F_{Force}');
%     ylabel('F');
%     legend('原布局','优化布局');
%     xticks(idx);
% 
%     subplot(3,1,2); hold on; grid on;
%     bar(idx,[F_Torque_orig,F_Torque_opt],'grouped');
%     title('Torque 综合评价值 F_{Torque}');
%     ylabel('F');
%     legend('原布局','优化布局');
%     xticks(idx);
% 
%     subplot(3,1,3); hold on; grid on;
%     bar(idx,[F_Total_orig,F_Total_opt],'grouped');
%     title('Total 综合评价值 F_{Total}');
%     xlabel('故障状态编号');
%     ylabel('F');
%     legend('原布局','优化布局');
%     xticks(idx);
% end
% function Plot_ClosedLoop_Response(log_orig, log_opt, fault_time)
% 
%     t = log_orig.Time;
% 
%     figure('Name','闭环位置与姿态响应对比','Color','w','Position',[80 80 1450 850]);
% 
%     subplot(2,2,1); hold on; grid on;
%     Plot_3Axis(t, log_orig.R, log_orig.Y(1:3,:), log_opt.Y(1:3,:), '位置响应', 'm', fault_time);
% 
%     subplot(2,2,2); hold on; grid on;
%     Plot_3Axis(t, log_orig.E, log_orig.Y_euler, log_opt.Y_euler, '姿态欧拉角响应', 'rad', fault_time);
% 
%     pos_err_orig = log_orig.Y(1:3,:) - log_orig.R;
%     pos_err_opt  = log_opt.Y(1:3,:)  - log_opt.R;
% 
%     att_err_orig = wrapToPi_local(log_orig.Y_euler - log_orig.E);
%     att_err_opt  = wrapToPi_local(log_opt.Y_euler  - log_opt.E);
% 
%     subplot(2,2,3); hold on; grid on;
%     plot(t, vecnorm(pos_err_orig,2,1), 'LineWidth',1.2);
%     plot(t, vecnorm(pos_err_opt,2,1),  'LineWidth',1.2);
%     xline(fault_time,'--r');
%     title('位置误差范数');
%     xlabel('时间 (s)'); ylabel('||e_r|| (m)');
%     legend('原布局','优化布局');
% 
%     subplot(2,2,4); hold on; grid on;
%     plot(t, vecnorm(att_err_orig,2,1), 'LineWidth',1.2);
%     plot(t, vecnorm(att_err_opt,2,1),  'LineWidth',1.2);
%     xline(fault_time,'--r');
%     title('姿态误差范数');
%     xlabel('时间 (s)'); ylabel('||e_\theta|| (rad)');
%     legend('原布局','优化布局');
% 
%     function Plot_3Axis(t, ref, y_orig, y_opt, title_str, unit_str, fault_time)
%         labels = {'X','Y','Z'};
%         colors = lines(3);
% 
%         for i = 1:3
%             plot(t, ref(i,:), '--', 'Color', colors(i,:), 'LineWidth',0.9);
%             plot(t, y_orig(i,:), '-',  'Color', colors(i,:), 'LineWidth',1.1);
%             plot(t, y_opt(i,:),  ':',  'Color', colors(i,:), 'LineWidth',1.6);
%         end
% 
%         xline(fault_time,'--r');
%         title(title_str);
%         xlabel('时间 (s)');
%         ylabel(unit_str);
%         legend('ref-X','orig-X','opt-X','ref-Y','orig-Y','opt-Y','ref-Z','orig-Z','opt-Z', ...
%                'Location','bestoutside');
%     end
% 
%     function x = wrapToPi_local(x)
%         x = mod(x + pi, 2*pi) - pi;
%     end
% end
% function Plot_Pulse_Compare(params, log_orig, log_opt, Perf_orig, Perf_opt, fault_time)
% 
%     figure('Name','总喷气时长与推力器脉宽对比','Color','w','Position',[100 100 1400 800]);
% 
%     labels = categorical({'优化布局','原布局'});
% subplot(2,2,1); hold on; grid on;
% bar(labels, [Perf_opt.Total_Pulse,Perf_orig.Total_Pulse]);
% ylabel('总喷气时长 (s)');
% title('总喷气时长对比');
% % subplot(2,2,2); hold on; grid on;
% % bar(labels, [Perf_orig.Avg_Pulse_Per_Control, Perf_opt.Avg_Pulse_Per_Control]);
% % ylabel('平均每周期喷气时长 (s)');
% % title('平均喷气时长对比');
%     % subplot(2,2,1); hold on; grid on;
%     % bar([Perf_orig.Total_Pulse, Perf_opt.Total_Pulse]);
%     % set(gca,'XTickLabel',{'原布局','优化布局'});
%     % ylabel('总喷气时长 (s)');
%     % title('全任务总喷气时长对比');
%     % 
%     % subplot(2,2,2); hold on; grid on;
%     % bar([Perf_orig.Avg_Pulse_Per_Control, Perf_opt.Avg_Pulse_Per_Control]);
%     % set(gca,'XTickLabel',{'原布局','优化布局'});
%     % ylabel('平均每周期喷气时长 (s)');
%     % title('平均喷气时长对比');
% 
%     % subplot(2,2,3); hold on; grid on;
%     % total_per_thr_orig = sum(log_orig.Pulse_Widths(:,1:round(params.T/(log_orig.Time(2)-log_orig.Time(1))):end),2);
%     % total_per_thr_opt  = sum(log_opt.Pulse_Widths(:,1:round(params.T/(log_opt.Time(2)-log_opt.Time(1))):end),2);
%     % 
%     % bar(1:params.Num, [total_per_thr_orig, total_per_thr_opt], 'grouped');
%     % xlabel('推力器编号');
%     % ylabel('累计脉宽近似值 (s)');
%     % title('各推力器累计喷气时长对比');
%     % legend('原布局','优化布局');
% 
%     subplot(2,2,2); hold on; grid on;
%     plot(log_orig.Time, sum(log_orig.Pulse_Widths,1), 'LineWidth',1.0);
%     plot(log_opt.Time,  sum(log_opt.Pulse_Widths,1),  'LineWidth',1.0);
%     xline(fault_time,'--r');
%     xlabel('时间 (s)');
%     ylabel('当前周期总脉宽 (s)');
%     title('当前周期总脉宽变化');
%     legend('原布局','优化布局');
% end
% % function Print_Result_Tables(Z_orig, Z_opt, F_orig, F_opt, Perf_orig, Perf_opt, StateNames)
% function Print_Result_Tables(Z_orig, Z_opt, F_orig, F_opt, ...
%                             Perf_orig, Perf_opt, StateNames, eval_state_idx)
% Je_orig = Perf_orig.Precision_Score;
% Je_opt  = Perf_opt.Precision_Score;
%     % fprintf('\n==================== 不同状态综合指标对比表 ====================\n');
%     % fprintf('%-12s | %8s %8s %8s %8s %8s || %8s %8s %8s %8s %8s\n', ...
%     %     '状态','Jc_o','Ja_o','Jo_o','Jf_o','F_o','Jc_p','Ja_p','Jo_p','Jf_p','F_p');
%     fprintf('%-12s | %8s %8s %8s %8s || %8s %8s %8s %8s \n', ...
%     '状态','Jc_o','Ja_o','Jf_o','Jo_o','Jc_p','Ja_p','Jf_p','Jo_p');
% 
%     for i = 1:length(F_orig)
%         fprintf('%-12s | %8.4f %8.4f %8.4f %8.4f || %8.4f %8.4f %8.4f %8.4f \n', ...
%             StateNames{i}, ...
%             Z_orig(i,1), Z_orig(i,2), Z_orig(i,3), Z_orig(i,4),  ...
%             Z_opt(i,1),  Z_opt(i,2),  Z_opt(i,3),  Z_opt(i,4));
%     end
% 
%     fprintf('================================================================\n');
% 
%     fprintf('\n==================== 闭环仿真性能对比表 ====================\n');
%     fprintf('%-28s | %12s | %12s | %12s\n', '指标','原布局','优化布局','改善率(%)');
% 
%     Print_Row('总喷气时长', Perf_orig.Total_Pulse, Perf_opt.Total_Pulse, false);
%     Print_Row('平均每周期喷气时长', Perf_orig.Avg_Pulse_Per_Control, Perf_opt.Avg_Pulse_Per_Control, false);
%     Print_Row('位置 RMSE', Perf_orig.Pos_RMSE, Perf_opt.Pos_RMSE, false);
%     Print_Row('姿态 RMSE', Perf_orig.Att_RMSE, Perf_opt.Att_RMSE, false);
%     Print_Row('综合控制精度得分', Perf_orig.Precision_Score, Perf_opt.Precision_Score, true);
% 
%     fprintf('============================================================\n');
% 
%     function Print_Row(name, v1, v2, benefit)
%         if benefit
%             imp = (v2 - v1) / (abs(v1) + 1e-12) * 100;
%         else
%             imp = (v1 - v2) / (abs(v1) + 1e-12) * 100;
%         end
%         fprintf('%-28s | %12.6f | %12.6f | %+12.2f\n', name, v1, v2, imp);
%     end
% 
% fprintf('\n==================== 系统层综合评价表 ====================\n');
% fprintf('当前评价对应状态：%s\n', StateNames{eval_state_idx});
% fprintf('----------------------------------------------------------\n');
% 
% fprintf('%-12s | %10s | %10s\n', '指标', '原布局', '优化布局');
% 
% fprintf('%-12s | %10.4f | %10.4f\n', 'Je(精度)', Je_orig, Je_opt);
% fprintf('%-12s | %10.4f | %10.4f\n', 'Jc(能力)', Z_orig(eval_state_idx,1), Z_opt(eval_state_idx,1));
% fprintf('%-12s | %10.4f | %10.4f\n', 'Ja(分辨率)', Z_orig(eval_state_idx,2), Z_opt(eval_state_idx,2));
% fprintf('%-12s | %10.4f | %10.4f\n', 'Jf(能耗)', Z_orig(eval_state_idx,3), Z_opt(eval_state_idx,3));
% fprintf('%-12s | %10.4f | %10.4f\n', 'Jo(可诊断)', Z_orig(eval_state_idx,4), Z_opt(eval_state_idx,4));
% 
% fprintf('----------------------------------------------------------\n');
% 
% fprintf('%-12s | %10.4f | %10.4f\n', 'F_total', F_orig(eval_state_idx), F_opt(eval_state_idx));
% 
% fprintf('==========================================================\n');
% end
