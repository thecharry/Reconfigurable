%% 推力器调用策略统一入口
function [Prop_Final, info] = Thruster_allocator( ...
        v_cmd1, v_cmd2, Matrix_sub, faulty_thrusters, params)
%THRUSTER_ALLOCATOR Select the reuse/allocation strategy requested by UI.
%
% alloc_mode controls how orbit and attitude requests share one control
% period.  allocation_strategy is kept as a separate UI concept so that
% the joint six-dimensional method can be compared with the primary and
% backup allocation method.

    alloc_mode = char(string(Get_Param(params, 'alloc_mode', ...
        'synchronous_time_division')));
    allocation_strategy = char(string(Get_Param(params, ...
        'allocation_strategy', 'primary_backup')));

    if strcmpi(alloc_mode, 'joint_optimization') || ...
            strcmpi(allocation_strategy, 'joint_optimization')
        [Prop_Final, info] = Thruster_joint_optimization( ...
            v_cmd1, v_cmd2, Matrix_sub, faulty_thrusters, params);
        return;
    end

    if strcmpi(alloc_mode, 'asynchronous_time_division') || ...
            strcmpi(alloc_mode, 'task_book')
        [Prop_Final, info] = Thruster_taskbook_async( ...
            v_cmd1, v_cmd2, Matrix_sub, faulty_thrusters, params);
        return;
    end

    % The existing strategy is the synchronous time-division baseline:
    % minimal primary groups in nominal operation, then healthy backups.
    [Prop_Final, info] = Thruster_invocation( ...
        v_cmd1, v_cmd2, Matrix_sub, faulty_thrusters, params);
    info.mode = 'synchronous_time_division';
    info.reuse_strategy = 'synchronous_time_division';
    info.allocation_strategy = 'primary_backup';

    function value = Get_Param(input_params, field_name, default_value)
        if isfield(input_params, field_name) && ...
                ~isempty(input_params.(field_name))
            value = input_params.(field_name);
        else
            value = default_value;
        end
    end
end
