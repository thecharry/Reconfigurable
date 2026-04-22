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
            Jc_F_orig(k) = J(k*5-4, 1); Jc_F_opt(k) = J(k*5-4, 2);
            Jc_T_orig(k) = J(k*5-3, 1); Jc_T_opt(k) = J(k*5-3, 2);
            Ja_F_orig(k) = J(k*5-2, 1); Ja_F_opt(k) = J(k*5-2, 2);
            Ja_T_orig(k) = J(k*5-1, 1); Ja_T_opt(k) = J(k*5-1, 2);
        end
        
        if num_faults == 0
            curr_state_idx = 0;
            Jo_actual_orig = J(5,1); Jo_actual_opt = J(5,2);
        else
            curr_state_idx = faulty_thrusters(1);
            Jo_actual_orig = J((curr_state_idx*5)+5, 1); 
            Jo_actual_opt = J((curr_state_idx*5)+5, 2);
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