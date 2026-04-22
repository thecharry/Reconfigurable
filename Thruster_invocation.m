%% 推力器控制效率调用策略（改）
function Prop_Final = Thruster_invocation(v_cmd1,v_cmd2,Matrix_sub,faulty_thrusters,params)
    N = size(Matrix_sub, 2);
    u = zeros(N, 2);
    for v_cmd = 1:2
        u_opt = zeros(N, 1);
        for axis = 1:3
            if v_cmd == 1
                cmd = v_cmd1(axis);
                Matrix_conf = Matrix_sub(1:3, :);
            else
                cmd = v_cmd2(axis);
                Matrix_conf = Matrix_sub(4:6, :);
            end
            if abs(cmd) < 1e-6
                continue; % 该轴无指令
            end
            if cmd > 0
                valid_idx1 = find(Matrix_conf(axis, :) > 1e-3);% 正向推力器组
            else
                valid_idx1 = find(Matrix_conf(axis, :) < -1e-3);% 负向推力器组
            end
            % 故障隔离
            if ~isempty(faulty_thrusters)
                valid_idx1 = setdiff(valid_idx1, faulty_thrusters);
            end
    
            if isempty(valid_idx1)
                continue;% 对应方向无可用推力器
            end
            eff = abs(Matrix_conf(axis, valid_idx1)');% 指令分配
            u_axis = abs(cmd) .* eff ./ sum(eff.^2);% 局部伪逆方式分配，满足 sum(u * eff) = abs(cmd)
            % 部分推力器可能被多个轴复用
            for j = 1:length(valid_idx1)
                u_opt(valid_idx1(j)) = u_opt(valid_idx1(j)) + u_axis(j);
            end
        end
        % 饱和限制与死区截断
        u_opt(u_opt > 1) = 1;
        min_duty = 0.05;
        u_opt(u_opt > 0 & u_opt < min_duty) = 0;
        u(:,v_cmd) = u_opt;
    end
    Prop_F = u(:,1) * params.T;
    Prop_T = u(:,2) * params.T;
    Prop_Final = Thruster_reuse(Prop_F,Prop_T);

    %% 推力器复用策略（改）
    function Prop_Final = Thruster_reuse(Prop_F,Prop_T)
        Prop_Final = zeros(params.Num, 1);
        t_att = min(Prop_T, params.T);
        t_rem = max(0, params.T - t_att);
        normal = 1.0;
        for i = 1:params.Num
            if Prop_F(i) > 1e-6 && Prop_F(i) > t_rem(i)
                current = t_rem(i) / Prop_F(i);% 等比例缩放
                if current < normal
                    normal = current;
                end
            end
        end
        for i = 1:params.Num
            t_orb = Prop_F(i) * normal;
            Prop_Final(i) = t_att(i) + t_orb;
        end
    end
end
% %% 推力器控制效率调用策略
% function u_opt = Thruster_invocation(v_cmd, Matrix_conf, falut_idx)
% N = size(Matrix_conf, 2);
% lb = zeros(N, 1);
% ub = ones(N, 1);
% if falut_idx ~= 1
%     lb(falut_idx - 1) = 0;
%     ub(falut_idx - 1) = 0;
% end
% % W_d = 1e4; % 跟踪误差权重
% % W_u = 1;   % 燃料消耗权重
% % H = 2 * (W_d * (Matrix_conf' * Matrix_conf) + W_u * eye(N));
% % f = -2 * W_d * Matrix_conf' * v_cmd;
% % H = eye(N);% 能量最小化权重
% % options = optimoptions('quadprog', 'Display', 'off');
% % [u_opt, ~, ~] = quadprog(H, [], [], [], Matrix_conf, v_cmd, lb, ub, [], options);
% 
% l = 0.1;% 能量惩罚权重
% C = [Matrix_conf; l * eye(N)];
% d = [v_cmd; zeros(N, 1)];
% options = optimoptions('lsqlin', 'Display', 'off');
% [u_opt, ~, ~] = lsqlin(C, d, [], [], [], [], lb, ub, [], options);
% 
% % 死区截断
% min_duty = 0.05;% params.t_min/params.T
% u_opt(u_opt > 0 & u_opt < min_duty) = 0;
% end

%% 推力器复用策略
% function Prop_Final = Thruster_reuse(Prop_T, Prop_F, Orb_cmds, params)
%     Prop_Final = zeros(12,1);
%     for i = 1:12
%         % 姿态控制优先
%         t_att = min(Prop_T(i), params.T);% 姿态控制时间
%         t_rem = params.T - t_att;        % 剩余时间
%         t_orb = min(Prop_F(i), t_rem);   % 轨道控制时间
%         Prop_Final(i) = t_att + t_orb;   % 推力器总输出时间
%         deltT = Prop_F(i) - t_orb;       % 推力器饱和溢出量
%         if deltT > 1e-6
%             cfg = params.Config_Struct(i);
%             % 推力器复用
%             if cfg.is_multiplexed
%                 cmd = zeros(2,1);
%                 if isfield(Orb_cmds, cfg.axis_names{1})
%                     cmd(1) = abs(Orb_cmds.(cfg.axis_names{1})); 
%                 end
%                 if isfield(Orb_cmds, cfg.axis_names{2})
%                     cmd(2) = abs(Orb_cmds.(cfg.axis_names{2})); 
%                 end
%                 % 复用策略    
%                 sum_cmd = sum(cmd);
%                 if sum_cmd < 1e-9
%                     ratio = [0.5; 0.5];
%                 else
%                     ratio = cmd / sum_cmd;
%                 end
%                 if ~isempty(cfg.orbit_partner)
%                     % 处理第一组配对
%                     if length(cfg.orbit_partner) >= 1
%                         for j = cfg.orbit_partner{1}
%                             Prop_Final(j) = max(0, Prop_Final(j) - deltT * ratio(1));
%                         end
%                     end
%                     % 处理第二组配对
%                     if length(cfg.orbit_partner) >= 2
%                         for j = cfg.orbit_partner{2}
%                             Prop_Final(j) = max(0, Prop_Final(j) - deltT * ratio(2));
%                         end
%                     end
%                 end
%             % 推力器未复用    
%             else          
%                 if ~isempty(cfg.orbit_partner)
%                     if length(cfg.orbit_partner) >= 1
%                         for j = cfg.orbit_partner{1}
%                             Prop_Final(j) = max(0, Prop_Final(j) - deltT);
%                         end
%                     end
%                 end
%             end
%         end
%     end
% end