function [Z,Jc1,Jc2,Jc,Jo,Jt,Jf] = Reconfig_eval(params,Ball,varargin)
% 默认：当前调用策略的时序验证指标。原硬件评价完整保留在本文件。
% 兼容原算法：params.evaluation_model='legacy_hardware'；不作为策略可行性证明。
% 时序积分入口：q=Reconfig_eval('schedule_metrics',M,schedule,params,cmd,faults)。
    if (ischar(params) || isstring(params)) && strcmp(params,'interference_comparison')
        Z=Interference_Comparison(Ball,varargin{:});
        [Jc1,Jc2,Jc,Jo,Jt,Jf]=deal([]); return;
    end
    if (ischar(params) || isstring(params)) && strcmp(params,'interference_margin')
        Z=Interference_Directional_Margin(Ball,varargin{:});
        [Jc1,Jc2,Jc,Jo,Jt,Jf]=deal([]); return;
    end
    if (ischar(params) || isstring(params)) && strcmp(params,'continuous_projection')
        Z=Continuous_Group_Projection(Ball,varargin{:});
        [Jc1,Jc2,Jc,Jo,Jt,Jf]=deal([]); return;
    end
    % 连续分配能力：C=Reconfig_eval('continuous_capability',params,B,faults)。
    if (ischar(params) || isstring(params)) && strcmp(params,'continuous_capability')
        Z=Continuous_Control_Capability(Ball,varargin{:});
        [Jc1,Jc2,Jc,Jo,Jt,Jf]=deal([]); return;
    end
    if (ischar(params) || isstring(params)) && strcmp(params,'schedule_metrics')
        Z=Pulse_Schedule_Metrics(Ball,varargin{:});
        [Jc1,Jc2,Jc,Jo,Jt,Jf]=deal([]); return;
    end
    if isfield(params,'evaluation_model') && strcmp(params.evaluation_model,'continuous_judgment')
        [Z,Jc1,Jc2,Jc,Jo,Jt,Jf]=Continuous_Reconfig_Judgment(params,Ball,varargin{:});
        return;
    end
    if isfield(params,'evaluation_model') && strcmp(params.evaluation_model,'legacy_hardware')
        [Z,Jc1,Jc2,Jc,Jo,Jt,Jf]=Legacy_Reconfig_eval(params,Ball,varargin{:});
    else
        [Z,Jc1,Jc2,Jc,Jo,Jt,Jf]=Policy_Reconfig_eval(params,Ball,varargin{:});
    end
end

%% 可重构基础评价指标函数
% 原入口名称：Reconfig_eval；为保留原实现，改为本文件内兼容入口。
function [Z, Jc1, Jc2, Jc, Jo, Jt, Jf] = Legacy_Reconfig_eval(params, Ball, max_faultys, include_combined_commands)
    if nargin < 3 || isempty(max_faultys)
        max_faultys = 1;
    end
    if nargin < 4 || isempty(include_combined_commands)
        include_combined_commands = true;
    end
    Matrix_conf = params.F_max * Ball;% 推力器矩阵
    % 故障组合列表
    faultysets = {[]};
    if max_faultys > 0
        combs = nchoosek(1:params.Num, max_faultys);
        faultysets = cell(size(combs, 1) + 1, 1);
        faultysets{1} = [];
        for ii = 1:size(combs, 1)
            faultysets{ii + 1, 1} = combs(ii, :);
        end
    end

    n_case = numel(faultysets);
    Jc1 = zeros(n_case, 2);% 控制能力最小值
    Jc2 = zeros(n_case, 2);% 控制能力最大值
    Jc = zeros(n_case, 1);% 6维控制能力
    Jo = zeros(n_case, 1);% 可诊断性
    Jt = zeros(n_case, 1);% 跟踪性能
    Jt_err = zeros(n_case, 1);% 跟踪误差
    Jf = zeros(n_case, 1);% 能耗效率
    invocation_template = struct('axis_callable', false(6, 2), ...
        'strict_axis', false(6, 2), 'axis_capacity', zeros(6, 2), ...
        'force_capacity', 0, 'torque_capacity', 0, 'worst_capacity', 0, ...
        'policy_reconfigurable', false, 'strictly_reconfigurable', false, ...
        'uses_impulse_margin_fallback', false, ...
        'worst_reference_quality', 0, 'reference_commands_callable', false);
    Invocation = repmat(invocation_template, n_case, 1);

    % 评价指标计算
    for i = 1:n_case
        faulty_idx = faultysets{i};
        healthy_idx = setdiff(1:params.Num, faulty_idx);
        Matrix_conf_F = Matrix_conf(1:3, healthy_idx);
        Matrix_conf_T = Matrix_conf(4:6, healthy_idx);
        Matrix_conf_H = Matrix_conf(:, healthy_idx);
        scale = max(abs(Matrix_conf), [], 2);
        scale(scale < 1e-12) = 1;
        
        [Jc1(i, 1), Jc2(i, 1)] = Capability(Matrix_conf_F);
        [Jc1(i, 2), Jc2(i, 2)] = Capability(Matrix_conf_T);
        [Jc(i), ~] = Capability(Matrix_conf_H ./ scale);
        Jo(i) = Diagnosability(Matrix_conf_H);
        [Jt(i), Jf(i), Jt_err(i), Invocation(i)] = Tracking_Energy(Matrix_conf, faulty_idx, scale, params, include_combined_commands);
    end
    % 仅输出原始评价量；跨布局归一化在Plot_results.m中统一完成。
    Z = struct();
    Z.FaultSets = faultysets;
    Z.Jc6 = Jc;
    Z.Jc = Jc1;
    Z.Jo = Jo;
    Z.Jt = Jt;
    Z.Jf = Jf;
    % Jc 表示任意健康推力器占空比组合可达到的硬件上限；Invocation
    % 则只统计现有“最简主备调用 + 分时复用”实际能够执行的能力。
    Z.Invocation = Invocation;
    % Z.Raw = struct('JcForce', Jc1(:, 1), 'JcTorque', Jc1(:, 2), ...
    %                'Jc6', Jc, 'JoAngle', Jo, ...
    %                'JtQuality', Jt, 'JtError', Jt_err, 'JfPulse', Jf);
    Z.Raw = struct('JcForce', Jc1(:, 1), 'JcTorque', Jc1(:, 2), ...
                   'Jc6', Jc, 'JoAngle', Jo, ...
                   'JtQuality', Jt, 'JtError', Jt_err, 'JfPulse', Jf);

    %% 控制能力评价指标
    function [Jc_min, Jc_max] = Capability(Matrix_sub)
        n_sub = size(Matrix_sub, 2);
        if n_sub == 0 || isempty(Matrix_sub)
            Jc_min = 0;
            Jc_max = 0;
            return;
        end

        try
            c = 0.5 * ones(n_sub, 1);
            G = 0.5 * eye(n_sub);
            Z = zonotope(Matrix_sub * c, Matrix_sub * G);
            P = polytope(Z);
            H = P.A;
            w = P.b;
            if isempty(H) || isempty(w) || any(w < -1e-6)
                Jc_min = 0;
                Jc_max = 0;
            else
                d = w ./ (vecnorm(H, 2, 2) + 1e-12);
                Jc_min = min(d);
                Jc_max = max(d);
            end
        catch
            Jc_min = 0;
            Jc_max = 0;
        end
    end

    %% 可诊断性评价指标
    function Jo = Diagnosability(Matrix_sub)
        n_sub = size(Matrix_sub, 2);
        if n_sub < 2 || isempty(Matrix_sub)
            Jo = 0;
            return;
        end

        col_norm = vecnorm(Matrix_sub, 2, 1);
        valid_idx = col_norm > 1e-12;
        if sum(valid_idx) < 2
            Jo = 0;
            return;
        end

        K_nor = Matrix_sub(:, valid_idx) ./ col_norm(valid_idx);
        n_sub = size(K_nor, 2);
        cos_matrix = min(max(K_nor' * K_nor, -1), 1);
        angle_matrix = acos(cos_matrix);
        angle_matrix(logical(eye(n_sub))) = inf;
        min_angle = min(angle_matrix(:));
        if isfinite(min_angle)
            Jo = min_angle;
            % Jo = sin(min_angle / 2);
        else
            Jo = 0;
        end
    end

    %% 跟踪性能和能耗效率评价指标
    function [Jt, Jf, mean_err, invocation] = Tracking_Energy(Matrix_conf, faulty_idx, scale, params, include_combined_commands)
        % 构建典型指令集
        dirs = [eye(3), -eye(3)];
        F_cmds = zeros(3, 0);
        T_cmds = zeros(3, 0);
        cmd_idx = 0;
        weight = 0.2;
        force_level = max(max(abs(Matrix_conf(1:3, :))));
        torque_level = max(max(abs(Matrix_conf(4:6, :))));
        for f = 1:size(dirs, 2)
            cmd_idx = cmd_idx + 1;
            F_cmds(:, cmd_idx) = weight * force_level * dirs(:, f);
            T_cmds(:, cmd_idx) = zeros(3, 1);
        end
        for t = 1:size(dirs, 2)
            cmd_idx = cmd_idx + 1;
            F_cmds(:, cmd_idx) = zeros(3, 1);
            T_cmds(:, cmd_idx) = weight * torque_level * dirs(:, t);
        end
        if include_combined_commands
            for f_idx = 1:size(dirs, 2)
                for t_idx = 1:size(dirs, 2)
                    cmd_idx = cmd_idx + 1;
                    F_cmds(:, cmd_idx) = weight * force_level * dirs(:, f_idx);
                    T_cmds(:, cmd_idx) = weight * torque_level * dirs(:, t_idx);
                end
            end
        end

        n_cmd = size(F_cmds, 2);
        cmd_quality = zeros(n_cmd, 1);
        cmd_err = zeros(n_cmd, 1);
        pulse_sum = zeros(n_cmd, 1);
        invocation = invocation_template;
        for k = 1:n_cmd
            F_cmd = F_cmds(:, k);
            T_cmd = T_cmds(:, k);
            cmd = [F_cmd; T_cmd];
            [Prop_Final, info] = Thruster_invocation( ...
                F_cmd, T_cmd, Matrix_conf, faulty_idx, params);
            actual = info.actual;
            rel_err = norm((cmd - actual) ./ scale) / (norm(cmd ./ scale) + 1e-12);
            cmd_err(k) = rel_err;
            cmd_quality(k) = 1 / (1 + rel_err);
            pulse_sum(k) = sum(Prop_Final);

            % 前12条指令分别是六个正负轴向的独立中等幅值指令。它们
            % 用于检查当前调用策略是否真的能给每个带符号轴安排完整组，
            % 并由组内最大占空比推得该组在一个控制周期内的有效上限。
            if k <= 12
                if k <= 6
                    row_idx = mod(k - 1, 3) + 1;
                    control_type = 1;
                else
                    row_idx = mod(k - 7, 3) + 4;
                    control_type = 2;
                end
                direction_col = 1 + (cmd(row_idx) > 0);
                axis_idx = row_idx - 3 * (control_type - 1);
                strategy = info.axis_strategy{control_type, axis_idx};
                is_group_strategy = isstruct(strategy) && isfield(strategy, 'duty') && ...
                    ~isempty(strategy.duty) && any(strategy.duty > 1e-12);
                is_joint_mode = isfield(info, 'mode') && ...
                    strcmp(info.mode, 'joint_optimization') && any(info.duty > 1e-12);
                axis_ok = rel_err <= 1e-5 && (is_group_strategy || is_joint_mode);
                invocation.axis_callable(row_idx, direction_col) = axis_ok;
                if axis_ok
                    if is_group_strategy
                        max_duty = max(strategy.duty);
                    else
                        max_duty = max(info.duty);
                    end
                    if max_duty > 1e-12
                        capacity = abs(cmd(row_idx)) / max_duty;
                        % 冲量裕度备用只在当前测试幅值下验证过安全性，
                        % 不把它外推到满周期，以免高估故障后的实际能力。
                        if is_group_strategy && strategy.is_impulse_margin_fallback
                            capacity = min(capacity, abs(cmd(row_idx)));
                            invocation.uses_impulse_margin_fallback = true;
                        end
                        invocation.axis_capacity(row_idx, direction_col) = capacity;
                    end
                    if is_group_strategy && isfield(strategy, 'instantaneous_pure')
                        invocation.strict_axis(row_idx, direction_col) = strategy.instantaneous_pure;
                    end
                end
            end
        end
        Jt = mean(cmd_quality);
        mean_err = mean(cmd_err);
        Jf = mean(pulse_sum);
        invocation.force_capacity = min(invocation.axis_capacity(1:3, :), [], 'all');
        invocation.torque_capacity = min(invocation.axis_capacity(4:6, :), [], 'all');
        invocation.worst_capacity = min(invocation.force_capacity, invocation.torque_capacity);
        invocation.policy_reconfigurable = all(invocation.axis_callable(:));
        invocation.strictly_reconfigurable = all(invocation.strict_axis(:));
        invocation.worst_reference_quality = min(cmd_quality);
        invocation.reference_commands_callable = all(cmd_err <= 1e-5);
    end

end

%% 当前策略评价（合并原新增文件，避免独立依赖）
function [Z,Jc1,Jc2,Jc,Jo,Jt,Jf] = Policy_Reconfig_eval(params,Ball,maxFaults,combined)
% Verified policy endpoints, not a continuous envelope or global optimum.
% Hardware: independent continuous duty in [0,1], all non-target rows zero.
    if nargin<3 || isempty(maxFaults), maxFaults=1; end
    if nargin<4 || isempty(combined), combined=true; end
    n=size(Ball,2);
    validateattributes(Ball,{'numeric'},{'size',[6,n],'finite'});
    validateattributes(maxFaults,{'numeric'},{'scalar','integer','>=',0,'<=',n});
    validateattributes(params.T,{'numeric'},{'scalar','positive','finite'});
    validateattributes(params.t_min,{'numeric'},{'scalar','nonnegative','finite'});
    validateattributes(params.F_max,{'numeric'},{'scalar','positive','finite'});
    names={'T','t_min','F_max','alloc_mode','allocation_strategy','strict_group_contract', ...
        'strict_contract_matrix','evaluation_reference_length','evaluation_reference_fraction'};
    p=struct();
    for k=1:numel(names)
        if isfield(params,names{k}), p.(names{k})=params.(names{k}); end
    end
    p.Num=n;
    p.alloc_mode=option(p,'alloc_mode','synchronous_time_division');
    if strcmp(option(p,'allocation_strategy','primary_backup'),'joint_optimization')
        p.alloc_mode='joint_optimization';
    end
    if isfield(p,'strict_contract_matrix') && ~isequal(p.strict_contract_matrix,Ball)
        if isfield(p,'strict_group_contract'), p=rmfield(p,'strict_group_contract'); end
    end
    L=option(p,'evaluation_reference_length',2);
    fraction=option(p,'evaluation_reference_fraction',0.2);
    validateattributes(L,{'numeric'},{'scalar','positive','finite'});
    validateattributes(fraction,{'numeric'},{'scalar','positive','finite'});
    ref=p.F_max*[ones(3,1);L*ones(3,1)];
    key=struct('params',p,'B',Ball,'faults',maxFaults,'combined',combined);
    persistent cache
    for k=1:numel(cache)
        if isequaln(cache(k).key,key)
            Z=cache(k).value; Jc1=Z.Jc; Jc2=Z.JcMax; Jc=Z.Jc6;
            Jo=Z.Jo; Jt=Z.Jt; Jf=Z.Jf; return;
        end
    end
    faultsets={[]};
    if maxFaults>0
        sets=nchoosek(1:n,maxFaults);
        faultsets=[faultsets;mat2cell(sets,ones(size(sets,1),1),maxFaults)];
    end
    M=p.F_max*Ball; count=numel(faultsets);
    Jc1=zeros(count,2); Jc2=Jc1; Jc=zeros(count,1); Jo=Jc; Jt=Jc; Jf=Jc;
    template=struct('axis_callable',false(6,2),'strict_axis',false(6,2), ...
        'axis_capacity',zeros(6,2),'axis_min_verified',nan(6,2), ...
        'axis_witness',{cell(6,2)},'force_capacity',0,'torque_capacity',0, ...
        'worst_capacity',0,'policy_reconfigurable',false,'strictly_reconfigurable',false, ...
        'uses_impulse_margin_fallback',false,'worst_reference_quality',0, ...
        'reference_commands_callable',false,'reference_pass_rate',0, ...
        'reference_error',[],'reference_schedule_valid',[],'envelope_points',zeros(0,6), ...
        'strict_equal_axis',false(6,2),'strict_equal_capacity',zeros(6,2), ...
        'peak_cross_force',0,'peak_cross_torque',0,'end_cross_force_impulse',0, ...
        'end_cross_torque_impulse',0,'peak_cross_force_impulse',0, ...
        'peak_cross_torque_impulse',0,'cross_force_first_moment',0, ...
        'cross_torque_first_moment',0,'energy_valid',false);
    Invocation=repmat(template,count,1);
    Hardware=repmat(struct('axis_capacity',zeros(6,2),'force_capacity',0, ...
        'torque_capacity',0,'worst_capacity',0,'reconfigurable',false),count,1);
    bits=zeros(n,0);
    for g=2:2:min(6,n)
        groups=nchoosek(1:n,g); block=zeros(n,size(groups,1));
        for j=1:size(groups,1), block(groups(j,:),j)=1; end
        bits=[bits,block]; %#ok<AGROW>
    end
    W=(M./ref)*bits; errors=zeros(count,1);
    for f=1:count
        faults=faultsets{f}; healthy=setdiff(1:n,faults); inv=template;
        H=hardwareAxes(M,healthy,ref);
        Hardware(f).axis_capacity=H;
        Hardware(f).force_capacity=min(H(1:3,:),[],'all');
        Hardware(f).torque_capacity=min(H(4:6,:),[],'all');
        Hardware(f).worst_capacity=min(H./ref,[],'all');
        Hardware(f).reconfigurable=all(H./ref>1e-8,'all');
        V=W(:,all(bits(faults,:)==0,1));
        for axis=1:6
            off=setdiff(1:6,axis); pure=max(abs(V(off,:)),[],1)<=1e-9;
            for dir=1:2
                s=2*dir-3; cap=max([0,s*V(axis,pure)]);
                inv.strict_equal_capacity(axis,dir)=cap*ref(axis);
                inv.strict_equal_axis(axis,dir)=cap>=1e-5-1e-9;
                % Actual calls, then re-call at selected-group maximum/minimum
                % endpoints. Switching is non-monotone: no false bisection proof.
                amplitudes=unique([fraction*ref(axis),H(axis,dir)*[1 .75 .5 .25 .1 .05 .01]]);
                endpoints=[];
                for a=reshape(amplitudes(amplitudes>1e-7),1,[])
                    [ok,strict,q,info]=probe(a,axis,s,M,faults,p,ref);
                    inv=keepWitness(inv,ok,strict,q,info,a,axis,dir);
                    strategy=info.axis_strategy{1+(axis>3),mod(axis-1,3)+1};
                    if isstruct(strategy) && isfield(strategy,'duty') && any(strategy.duty>1e-12)
                        d=strategy.duty(strategy.duty>1e-12);
                        endpoints=[endpoints,a/max(d),a*(p.t_min/p.T)/min(d)]; %#ok<AGROW>
                    end
                end
                endpoints=unique(endpoints(endpoints>1e-7 & endpoints<=H(axis,dir)*(1+1e-7)));
                for a=reshape(endpoints,1,[])
                    [ok,strict,q,info]=probe(a,axis,s,M,faults,p,ref);
                    inv=keepWitness(inv,ok,strict,q,info,a,axis,dir);
                end
            end
        end
        commands=[diag(fraction*ref),-diag(fraction*ref)];
        if combined
            for i=[1:3,7:9]
                for j=[4:6,10:12], commands(:,end+1)=commands(:,i)+commands(:,j); end %#ok<AGROW>
            end
        end
        err=zeros(size(commands,2),1); consumption=err;
        good=false(size(err)); valid=good;
        for c=1:size(commands,2)
            cmd=commands(:,c);
            [pulse,info]=Thruster_invocation(cmd(1:3),cmd(4:6),M,faults,p);
            q=Pulse_Schedule_Metrics(M,info.schedule,p,cmd,faults);
            err(c)=norm((cmd-q.average)./ref)/norm(cmd./ref);
            valid(c)=q.valid && max(abs(q.pulse-pulse))<=1e-8;
            good(c)=valid(c) && err(c)<=1e-5; consumption(c)=sum(pulse);
            if good(c), inv.envelope_points(end+1,:)=q.average'; end %#ok<AGROW>
            if c<=12
                inv.peak_cross_force=max(inv.peak_cross_force,q.peak_cross_force);
                inv.peak_cross_torque=max(inv.peak_cross_torque,q.peak_cross_torque);
                inv.end_cross_force_impulse=max(inv.end_cross_force_impulse,norm(q.end_cross_impulse(1:3)));
                inv.end_cross_torque_impulse=max(inv.end_cross_torque_impulse,norm(q.end_cross_impulse(4:6)));
                inv.peak_cross_force_impulse=max(inv.peak_cross_force_impulse,norm(q.peak_prefix_impulse(1:3)));
                inv.peak_cross_torque_impulse=max(inv.peak_cross_torque_impulse,norm(q.peak_prefix_impulse(4:6)));
                inv.cross_force_first_moment=max(inv.cross_force_first_moment,norm(q.cross_first_moment(1:3)));
                inv.cross_torque_first_moment=max(inv.cross_torque_first_moment,norm(q.cross_first_moment(4:6)));
            end
        end
        inv.force_capacity=min(inv.axis_capacity(1:3,:),[],'all');
        inv.torque_capacity=min(inv.axis_capacity(4:6,:),[],'all');
        inv.worst_capacity=min(inv.axis_capacity./ref,[],'all');
        inv.policy_reconfigurable=all(inv.axis_callable,'all');
        inv.strictly_reconfigurable=all(inv.strict_axis,'all');
        inv.reference_commands_callable=all(good); inv.reference_pass_rate=mean(good);
        inv.reference_error=err; inv.reference_schedule_valid=valid;
        inv.worst_reference_quality=max(0,1-max(err)); inv.energy_valid=all(good);
        Invocation(f)=inv;
        Jc1(f,:)=[inv.force_capacity,inv.torque_capacity];
        Jc2(f,:)=[max(inv.axis_capacity(1:3,:),[],'all'),max(inv.axis_capacity(4:6,:),[],'all')];
        Jc(f)=inv.worst_capacity; Jo(f)=columnAngle(M(:,healthy)./ref);
        quality=max(0,1-err); quality(~valid)=0;
        Jt(f)=mean(quality); Jf(f)=mean(consumption); errors(f)=mean(err);
    end
    Z=struct('FaultSets',{faultsets},'Jc',Jc1,'JcMax',Jc2,'Jc6',Jc,'Jo',Jo,'Jt',Jt,'Jf',Jf, ...
        'Invocation',Invocation,'Hardware',Hardware,'ReferenceScale',ref, ...
        'ReferenceFraction',fraction,'PolicyMode',p.alloc_mode, ...
        'CapabilityMeaning','verified single-axis endpoints; not insphere, envelope or global optimum', ...
        'PolicyRelativeTolerance',1e-5,'InstantaneousNormalizedTolerance',1e-9, ...
        'Raw',struct('JcForce',Jc1(:,1),'JcTorque',Jc1(:,2),'Jc6',Jc, ...
        'JoAngle',Jo,'JtQuality',Jt,'JtError',errors,'JfPulse',Jf));
    structural=Continuous_Reconfig_Judgment(p,Ball,maxFaults);
    Z.ContinuousJudgment=structural;
    entry=struct('key',key,'value',Z);
    if isempty(cache), cache=entry; else, cache(end+1)=entry; end
    if numel(cache)>12, cache=cache(end-11:end); end
end

function H=hardwareAxes(M,healthy,ref)
    H=zeros(6,2); if isempty(healthy), return; end
    A=M(:,healthy)./ref;
    opts=optimoptions('linprog','Display','none','ConstraintTolerance',1e-9);
    for k=1:6
        off=setdiff(1:6,k);
        for d=1:2
            s=2*d-3;
            [u,~,flag]=linprog(-s*A(k,:),[],[],A(off,:),zeros(5,1), ...
                zeros(numel(healthy),1),ones(numel(healthy),1),opts);
            if flag~=1 || isempty(u) || max(abs(A(off,:)*u))>1e-7
                error('ReconfigEval:HardwareUnknown','硬件能力求解未可靠完成，不能记为零能力。');
            end
            H(k,d)=max(0,s*A(k,:)*u)*ref(k);
        end
    end
end

function [ok,strict,q,info]=probe(a,axis,s,M,faults,p,ref)
    cmd=zeros(6,1); cmd(axis)=s*a;
    [pulse,info]=Thruster_invocation(cmd(1:3),cmd(4:6),M,faults,p);
    q=Pulse_Schedule_Metrics(M,info.schedule,p,cmd,faults);
    ok=q.valid && max(abs(q.pulse-pulse))<=1e-8 && ...
        norm((q.average-cmd)./ref)<=1e-5*norm(cmd./ref) && s*q.average(axis)>1e-8*ref(axis);
    strict=ok && max(q.peak_wrench./ref)<=1e-9;
end

function inv=keepWitness(inv,ok,strict,q,info,a,axis,dir)
    if ~ok, return; end
    inv.axis_callable(axis,dir)=true;
    inv.strict_axis(axis,dir)=inv.strict_axis(axis,dir)||strict;
    inv.axis_min_verified(axis,dir)=min([inv.axis_min_verified(axis,dir),a],[],'omitnan');
    if abs(q.average(axis))>inv.axis_capacity(axis,dir)
        inv.axis_capacity(axis,dir)=abs(q.average(axis));
        inv.axis_witness{axis,dir}=struct('command',info.command,'schedule',info.schedule, ...
            'actual',q.average,'instantaneous_pure',strict);
    end
    inv.uses_impulse_margin_fallback=inv.uses_impulse_margin_fallback||info.uses_impulse_margin_fallback;
end

function value=columnAngle(A)
    A=A(:,vecnorm(A)>1e-12); value=0;
    if size(A,2)<2, return; end
    A=A./vecnorm(A); angles=acos(max(-1,min(1,A'*A)));
    angles(1:size(A,2)+1:end)=Inf; value=min(angles,[],'all');
end

function value=option(s,name,default)
    value=default; if isfield(s,name) && ~isempty(s.(name)), value=s.(name); end
end

function result = Pulse_Schedule_Metrics(matrix, schedule, params, command, faulty)
% Integrate the actual piecewise-constant BODY-frame schedule, not duty guesses.
% First moments imply displacement only in a frozen, linear dynamics model.
    if nargin < 5, faulty = []; end
    n=size(matrix,2); T=params.T;
    if ~isfield(schedule,'intervals') || numel(schedule.intervals)~=n
        error('PulseMetrics:InvalidSchedule','喷气时序与推力器数量不一致。');
    end
    intervals=schedule.intervals; valid=true; events=[0;T]; pulse=zeros(n,1);
    for i=1:n
        q=intervals{i};
        if isempty(q), continue; end
        if size(q,2)~=2 || any(~isfinite(q),'all') || any(q(:,2)<q(:,1))
            error('PulseMetrics:InvalidSchedule','喷气时序必须为有效的起止时间对。');
        end
        q=sortrows(q,1); merged=q(1,:);
        for j=2:size(q,1)
            valid=valid && q(j,1)>=merged(end,2)-1e-9;
            if q(j,1)<=merged(end,2)+1e-10
                merged(end,2)=max(merged(end,2),q(j,2));
            else
                merged(end+1,:)=q(j,:); %#ok<AGROW>
            end
        end
        valid=valid && all(merged(:,1)>=-1e-9 & merged(:,2)<=T+1e-9) && ...
            all(merged(:,2)-merged(:,1)>=params.t_min-1e-9);
        intervals{i}=merged;
        pulse(i)=sum(merged(:,2)-merged(:,1));
        events=[events;merged(:)]; %#ok<AGROW>
    end
    valid=valid && all(pulse(faulty)<=1e-10);
    events=unique([0;T;max(0,min(T,events))]);
    impulse=zeros(6,1); firstMoment=impulse; peak=impulse; prefixPeak=impulse;
    off=abs(command(:))<=1e-12; forcePeak=0; torquePeak=0;
    wrenchSamples=zeros(6,numel(events)-1);
    for j=1:numel(events)-1
        left=events(j); right=events(j+1); mid=(left+right)/2;
        active=false(n,1);
        for i=1:n
            q=intervals{i};
            if ~isempty(q), active(i)=any(q(:,1)<=mid & q(:,2)>mid); end
        end
        wrench=matrix*double(active); dt=right-left;
        wrenchSamples(:,j)=wrench;
        impulse=impulse+wrench*dt;
        firstMoment=firstMoment+wrench*dt*(T-mid);
        peak=max(peak,abs(wrench)); prefixPeak=max(prefixPeak,abs(impulse));
        cross=wrench.*off;
        forcePeak=max(forcePeak,norm(cross(1:3)));
        torquePeak=max(torquePeak,norm(cross(4:6)));
    end
    result=struct('valid',valid,'pulse',pulse,'average',impulse/T, ...
        'impulse',impulse,'first_moment',firstMoment,'off_axis',off, ...
        'peak_wrench',peak.*off,'peak_prefix_impulse',prefixPeak.*off, ...
        'end_cross_impulse',impulse.*off,'cross_first_moment',firstMoment.*off, ...
        'peak_cross_force',forcePeak,'peak_cross_torque',torquePeak, ...
        'wrench_samples',wrenchSamples,'event_times',events);
end

function result=Continuous_Control_Capability(params,Ball,faults)
% 原单方向口径保留：各方向集合取并集，不进行多方向组的和集运算。
% 多轴连续联动：w=M*R*a, a>=0, R*a<=1；同轴主备候选互斥。
% 不检查最小脉宽或调度时间；合计占空比上限属于物理分配限制。
    if nargin<3, faults=[]; end
    n=size(Ball,2);
    validateattributes(Ball,{'numeric'},{'size',[6,n],'finite'});
    validateattributes(params.F_max,{'numeric'},{'scalar','positive','finite'});
    validateattributes(faults,{'numeric'},{'integer','>=',1,'<=',n});
    faults=unique(faults(:)');
    p=params; p.continuous_capability_only=true;
    if isfield(p,'strict_contract_matrix') && ~isequal(p.strict_contract_matrix,Ball)
        if isfield(p,'strict_group_contract'), p=rmfield(p,'strict_group_contract'); end
    end
    M=params.F_max*Ball;
    [~,allocation]=Thruster_invocation(zeros(3,1),zeros(3,1),M,faults,p);
    endpoints=zeros(6,0); capacity=zeros(6,2);
    independent=strcmp(allocation.allocation_domain,'independent_continuous');
    if independent
        capacity=hardwareAxes(M,allocation.healthy,params.F_max*ones(6,1));
    else
        for j=1:numel(allocation.groups)
            group=allocation.groups(j); endpoints(:,end+1)=group.endpoint; %#ok<AGROW>
            d=1+(group.direction>0);
            capacity(group.axis,d)=max(capacity(group.axis,d),group.direction*group.endpoint(group.axis));
        end
    end
    model=allocation.allocation_domain;
    if ~independent, model='fixed_ratio_joint'; end
    % 原结果标识：'Model',allocation.allocation_domain；'Combination','union'。
    result=struct('Model',model,'Mode',allocation.mode, ...
        'Groups',allocation.groups,'Endpoints',endpoints,'Faults',faults, ...
        'TheoryGenerators',M(:,allocation.healthy),'AxisCapacity',capacity, ...
        'ForceCapacity',min(capacity(1:3,:),[],'all'), ...
        'TorqueCapacity',min(capacity(4:6,:),[],'all'), ...
        'IncludesTimingLimits',false,'Combination','coupled_sum_with_shared_duty', ...
        'Meaning','multi-axis continuous allocation; shared thruster duty <= 1; exclusive alternative groups');
end

function result=Continuous_Group_Projection(capability,rows)
% 精确到浮点容差的三维分区多面体，不使用方向采样。
% 纯轴向组的其他分量为零，故投影只需三个目标轴；其他轴取零即不占资源。
% 每轴可选一个正/负调用组；备用组选在不同分区中互斥，不允许同轴组任意并用。
    validateattributes(rows,{'numeric'},{'vector','numel',3,'integer','>=',1,'<=',6});
    rows=rows(:)';
    if numel(unique(rows))~=3, error('ContinuousCapacity:Rows','投影坐标不能重复。'); end
    groups=capability.Groups; options=cell(1,3); counts=zeros(3,2);
    for k=1:3
        options{k}=find([groups.axis]==rows(k));
        if isempty(options{k}), options{k}=0; end
        for direction=1:2
            counts(k,direction)=sum([groups.axis]==rows(k) & [groups.direction]==2*direction-3);
        end
    end
    cellCount=prod(cellfun(@numel,options));
    if cellCount>20000
        error('ContinuousCapacity:TooManyBranches', ...
            '互斥组选形成%d个分区，超过当前精确绘图上限20000；未用凸包替代。',cellCount);
    end
    n=max([0,groups.thrusters]);
    allPoints=zeros(0,3); pieces=cell(cellCount,1); pieceGroups=cell(cellCount,1); counter=0;
    for ia=options{1}
        for ib=options{2}
            for ic=options{3}
                selected=[ia,ib,ic]; selected=selected(selected>0); m=numel(selected);
                R=zeros(n,m); W=zeros(3,m);
                for k=1:m
                    group=groups(selected(k));
                    cross=group.endpoint; cross(group.axis)=0;
                    if max(abs(cross))>1e-7*max(1,max(abs(group.endpoint)))
                        error('ContinuousCapacity:CoupledGroup','调用组含不可忽略的跨轴分量，不能按纯轴模型投影。');
                    end
                    R(group.thrusters,k)=group.ratio;
                    % 数值纯轴容差下清零小残差，避免把数值噪声误作三维体积。
                    W(rows==group.axis,k)=group.endpoint(group.axis);
                end
                coefficients=Continuous_Cell_Vertices(R);
                points=unique((W*coefficients')','rows');
                counter=counter+1; pieces{counter}=points; pieceGroups{counter}=selected;
                allPoints=[allPoints;points]; %#ok<AGROW>
            end
        end
    end
    % 固定的每个带符号轴仅一组时，集合等于 {W*a:R*a<=1,a>=0}。
    % 同轴正负同时非零可成对减去抵消量，释放占空比且保持总输出，故集合凸。
    % 有互斥备用选项时不能使用这个结论，必须保留各分区的并集。
    convex=all(counts(:)<=1);
    tolerance=1e-9*max(1,max(abs(allPoints),[],'all'));
    allPoints=unique(round(allPoints/tolerance)*tolerance,'rows');
    if convex, pieces={allPoints}; pieceGroups={[]}; end
    result=struct('Vertices',allPoints,'Pieces',{pieces},'PieceGroups',{pieceGroups}, ...
        'Convex',convex,'BranchCount',cellCount,'Rows',rows,'SharedDutyLimit',1, ...
        'Method','active-constraint vertex enumeration; exclusive groups kept separate');
end

function vertices=Continuous_Cell_Vertices(R)
% 每个分区最多三个连续组强度。枚举活跃约束即可得到全部顶点。
    m=size(R,2);
    if m==0, vertices=zeros(1,0); return; end
    resources=unique([R;eye(m)],'rows'); resources=resources(any(resources>0,2),:);
    % 全部资源行右端为1，可删掉被逐分量更强的另一行包含的约束。
    keep=true(size(resources,1),1);
    for i=1:size(resources,1)
        for j=1:size(resources,1)
            if i~=j && all(resources(i,:)<=resources(j,:)) && any(resources(i,:)<resources(j,:))
                keep(i)=false; break;
            end
        end
    end
    A=[resources(keep,:);-eye(m)]; b=[ones(nnz(keep),1);zeros(m,1)];
    combinations=nchoosek(1:size(A,1),m); vertices=zeros(1,m);
    for k=1:size(combinations,1)
        active=combinations(k,:); matrix=A(active,:);
        if rcond(matrix)<1e-12, continue; end
        x=matrix\b(active);
        if all(A*x<=b+1e-9)
            vertices(end+1,:)=max(0,min(1,x))'; %#ok<AGROW>
        end
    end
    vertices=unique(round(vertices/1e-10)*1e-10,'rows');
end

function report=Interference_Comparison(params,Ball,faults)
% 姿轨干扰：相同的26条纯力和26条纯力矩指令，含单/双/三轴联动。
% 纯力指令的非期望力矩、纯力矩指令的非期望力为本页干扰，不含本类别轴间误差。
    if nargin<3, faults=[]; end
    modes={'synchronous_time_division','asynchronous_time_division','joint_optimization'};
    p=params; p.Num=size(Ball,2); p.allocation_strategy='primary_backup';
    if isfield(p,'strict_contract_matrix') && ~isequal(p.strict_contract_matrix,Ball)
        if isfield(p,'strict_group_contract'), p=rmfield(p,'strict_group_contract'); end
    end
    if isfield(p,'continuous_capability_only'), p=rmfield(p,'continuous_capability_only'); end
    validateattributes(p.T,{'numeric'},{'scalar','positive','finite'});
    validateattributes(p.t_min,{'numeric'},{'scalar','nonnegative','finite'});
    ref=p.F_max*[ones(3,1);option(p,'evaluation_reference_length',2)*ones(3,1)];
    fraction=option(p,'evaluation_reference_fraction',0.2);
    % 原测试集保留：commands=[diag(fraction*ref),-diag(fraction*ref)];
    % 仅测单轴会遗漏组间联动导致的同步/异步干扰差异。
    [dx,dy,dz]=ndgrid(-1:1,-1:1,-1:1); directions=[dx(:),dy(:),dz(:)]';
    directions=directions(:,any(directions,1)); directions=directions./vecnorm(directions);
    commands=fraction*diag(ref)*[directions,zeros(3,26);zeros(3,26),directions];
    M=p.F_max*Ball;
    template=struct('Mode','','ForcePeak',0,'TorquePeak',0,'ForceRatio',0,'TorqueRatio',0, ...
        'ForceJudgment','','TorqueJudgment','','EndForceImpulse',0,'EndTorqueImpulse',0, ...
        'ForceCommandsValid',true,'TorqueCommandsValid',true,'ReferenceCommands',commands, ...
        'ForceSamples',zeros(3,0),'TorqueSamples',zeros(3,0),'CommandErrors',zeros(size(commands,2),1));
    values=repmat(template,1,3);
    for mode=1:3
        p.alloc_mode=modes{mode}; item=template; item.Mode=modes{mode};
        for k=1:size(commands,2)
            cmd=commands(:,k);
            [pulse,info]=Thruster_invocation(cmd(1:3),cmd(4:6),M,faults,p);
            q=Pulse_Schedule_Metrics(M,info.schedule,p,cmd,faults);
            err=norm((q.average-cmd)./ref)/norm(cmd./ref); item.CommandErrors(k)=err;
            valid=q.valid && max(abs(q.pulse-pulse))<=1e-8 && err<=1e-5;
            if any(cmd(4:6))
                % 力矩指令造成平动力干扰。
                item.ForceSamples=[item.ForceSamples,q.wrench_samples(1:3,:)]; %#ok<AGROW>
                item.ForcePeak=max(item.ForcePeak,max(vecnorm(q.wrench_samples(1:3,:))));
                item.EndForceImpulse=max(item.EndForceImpulse,norm(q.impulse(1:3)));
                item.ForceCommandsValid=item.ForceCommandsValid && valid;
            else
                item.TorqueSamples=[item.TorqueSamples,q.wrench_samples(4:6,:)]; %#ok<AGROW>
                item.TorquePeak=max(item.TorquePeak,max(vecnorm(q.wrench_samples(4:6,:))));
                item.EndTorqueImpulse=max(item.EndTorqueImpulse,norm(q.impulse(4:6)));
                item.TorqueCommandsValid=item.TorqueCommandsValid && valid;
            end
        end
        cap=Continuous_Control_Capability(p,Ball,faults);
        force=Interference_Directional_Margin(cap,item.ForceSamples,1:3);
        torque=Interference_Directional_Margin(cap,item.TorqueSamples,4:6);
        item.ForceRatio=force.Ratio; item.TorqueRatio=torque.Ratio;
        item.ForceJudgment=Interference_Judgment(force,item.ForceCommandsValid);
        item.TorqueJudgment=Interference_Judgment(torque,item.TorqueCommandsValid);
        values(mode)=item;
    end
    report=struct('Faults',faults,'Modes',values,'ReferenceScale',ref, ...
        'ReferenceFraction',fraction,'Meaning','attitude-to-force / orbit-to-torque coupling; full reverse continuous capacity');
end

function text=Interference_Judgment(margin,commandsValid)
    if ~margin.Known
        text='能力求解未完成';
    elseif ~commandsValid
        text='指令或时序未通过，待复核';
    elseif margin.Ratio>1+1e-6
        text='超过反向控制能力';
    elseif margin.Ratio<=1e-9
        text='无显著干扰';
    else
        text='未超过反向控制能力';
    end
end

function result=Interference_Directional_Margin(capability,samples,rows)
% 对每个真实时段的反向干扰矢量求径向能力，不把不同时间的分量峰值拼接。
% lambdaMax*(-interference) 在连续纯控制集合内；干扰/反向能力 = 1/lambdaMax。
% 当前任务占用未扣除，因此这不是实际剩余补偿裕度或闭环稳定性证明。
    validateattributes(samples,{'numeric'},{'nrows',3,'finite'});
    rows=rows(:)';
    scale=max([1;abs(capability.TheoryGenerators(:))]);
    samples=unique(samples','rows')'; samples=samples(:,vecnorm(samples)>1e-9*scale);
    result=struct('Ratio',0,'Known',true);
    if isempty(samples), return; end
    opts=optimoptions('linprog','Display','none','ConstraintTolerance',1e-9,'OptimalityTolerance',1e-9);
    independent=strcmp(capability.Model,'independent_continuous');
    if ~independent, projection=Continuous_Group_Projection(capability,rows); end
    for k=1:size(samples,2)
        target=-samples(:,k); magnitude=norm(target); direction=target/magnitude;
        maximum=0;
        if independent
            M=capability.TheoryGenerators; n=size(M,2); desired=zeros(6,1); desired(rows)=direction;
            if n==0, result.Ratio=Inf; continue; end
            [~,objective,flag]=linprog([zeros(n,1);-1],[],[],[M,-desired],zeros(6,1), ...
                zeros(n+1,1),[ones(n,1);Inf],opts);
            if flag~=1, result.Known=false; result.Ratio=NaN; return; end
            maximum=max(0,-objective);
        else
            for j=1:numel(projection.Pieces)
                points=projection.Pieces{j}; n=size(points,1);
                [~,objective,flag]=linprog([zeros(n,1);-1],[],[], ...
                    [points',-direction;ones(1,n),0],[zeros(3,1);1], ...
                    zeros(n+1,1),[],opts);
                if flag~=1, result.Known=false; result.Ratio=NaN; return; end
                maximum=max(maximum,max(0,-objective));
            end
        end
        if maximum<=1e-10*scale, ratio=Inf; else, ratio=magnitude/maximum; end
        result.Ratio=max(result.Ratio,ratio);
    end
end

function [Z,Jc1,Jc2,Jc,Jo,Jt,Jf]=Continuous_Reconfig_Judgment(params,Ball,maxFaults,varargin)
% 原点严格位于六维连续实际能力集合内部；不调用执行器或时序验证。
% 独立分配集合凸：十二个纯轴方向正能力等价于原点内点。
% 纯轴绑定组：每个正负方向任选一组，足够小的六轴组合总占空比<=1，
% 从而覆盖原点邻域；同轴候选互斥不影响此局部结论。缺方向则不为内点。
    if nargin<3 || isempty(maxFaults), maxFaults=1; end
    n=size(Ball,2);
    validateattributes(maxFaults,{'numeric'},{'scalar','integer','>=',0,'<=',n});
    ref=params.F_max*[ones(3,1);option(params,'evaluation_reference_length',2)*ones(3,1)];
    validateattributes(ref,{'numeric'},{'positive','finite'});
    faultsets={[]};
    if maxFaults>0
        sets=nchoosek(1:n,maxFaults);
        faultsets=[faultsets;mat2cell(sets,ones(size(sets,1),1),maxFaults)];
    end
    count=numel(faultsets); Jc1=zeros(count,2); Jc2=Jc1; Jc=zeros(count,1);
    Jo=nan(count,1); Jt=Jo; Jf=Jo;
    Invocation=repmat(struct('axis_callable',false(6,2),'axis_capacity',zeros(6,2), ...
        'policy_reconfigurable',false,'origin_interior',false,'normalized_neighborhood_bound',0),count,1);
    Hardware=repmat(struct('axis_capacity',zeros(6,2),'force_capacity',0, ...
        'torque_capacity',0,'reconfigurable',false),count,1);
    for f=1:count
        faults=faultsets{f}; c=Continuous_Control_Capability(params,Ball,faults);
        for j=1:numel(c.Groups)
            g=c.Groups(j); cross=g.endpoint; cross(g.axis)=0;
            if max(abs(cross))>1e-7*max(1,max(abs(g.endpoint)))
                error('ContinuousCapacity:CoupledGroup','组选含非纯轴输出，当前内点判据不适用，不能给出可重构结论。');
            end
        end
        C=c.AxisCapacity;
        H=hardwareAxes(params.F_max*Ball,setdiff(1:n,faults),ref);
        Hardware(f).axis_capacity=H;
        Hardware(f).force_capacity=min(H(1:3,:),[],'all');
        Hardware(f).torque_capacity=min(H(4:6,:),[],'all');
        Hardware(f).reconfigurable=all(H./ref>1e-8,'all');
        Invocation(f).axis_capacity=C;
        Invocation(f).axis_callable=C./ref>1e-8;
        Invocation(f).origin_interior=all(Invocation(f).axis_callable,'all');
        Invocation(f).policy_reconfigurable=Invocation(f).origin_interior;
        Jc1(f,:)=[min(C(1:3,:),[],'all'),min(C(4:6,:),[],'all')];
        Jc2(f,:)=[max(C(1:3,:),[],'all'),max(C(4:6,:),[],'all')];
        Jc(f)=min(C./ref,[],'all');
        % 保守六维欧氏邻域半径下界，不冒充最大内切球半径。
        if Invocation(f).origin_interior
            Invocation(f).normalized_neighborhood_bound=Jc(f)/sqrt(6);
        end
    end
    Z=struct('FaultSets',{faultsets},'Invocation',Invocation,'Hardware',Hardware, ...
        'Jc',Jc1,'JcMax',Jc2,'Jc6',Jc,'Jo',Jo,'Jt',Jt,'Jf',Jf, ...
        'ReferenceScale',ref,'IncludesTimingLimits',false, ...
        'JudgmentModel','continuous_origin_interior','NormalizedTolerance',1e-8);
end
