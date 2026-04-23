%% 参数设置
function params = Get_params()
    R_earth = 6371e3;
    h_orbit = 400e3;
    mu = 3.986e14;
    params.t_min = 0.02;% 最小脉宽
    params.F_max = 10;
    params.Num = 12;
    params.T = 0.4;% 系统控制周期
    params.m = 3700;
    params.J = diag([10000, 6000, 13000]);
    params.n = sqrt(mu / (R_earth + h_orbit)^3);
    [params.B_all, params.r_all] = Thruster_config();
    params.Jc = zeros(params.Num+1, 2);
    params.Ja = zeros(params.Num+1, 2);
    params.Jo = zeros(params.Num+1, 1);
    params.Jf = zeros(params.Num+1, 1);
    for falut_idx = 1:params.Num+1
        if falut_idx == 1
            eval_fault = [];
        else
            eval_fault = falut_idx - 1;
        end
        [params.Matrix_conf,params.Jc(falut_idx, :),~,params.Ja(falut_idx, :),params.Jo(falut_idx),params.Jf(falut_idx)] = Reconfig_eval(params,params.B_all,eval_fault);
    end

    %% 推力器布局设置
    function [B_all, r] = Thruster_config()
        r = zeros(3, 12); d = zeros(3, 12);
        r(:,1) = [ 2;-0.6;   0]; d(:,1) = [-0.5; 0.866;  0];
        r(:,2) = [ 2; 0.6;   0]; d(:,2) = [-0.5;-0.866;  0];
        r(:,3) = [-2;-0.6;   0]; d(:,3) = [ 0.5; 0.866;  0];
        r(:,4) = [-2; 0.6;   0]; d(:,4) = [ 0.5;-0.866;  0];
        r(:,5) = [ 2;-0.6; 0.6]; d(:,5) = [-0.707;0;-0.707];
        r(:,6) = [ 2; 0.6; 0.6]; d(:,6) = [-0.707;0;-0.707];
        r(:,7) = [-2;-0.6; 0.6]; d(:,7) = [ 0.707;0;-0.707];
        r(:,8) = [-2; 0.6; 0.6]; d(:,8) = [ 0.707;0;-0.707];
        r(:,9) = [ 2;-0.6;-0.6]; d(:,9) = [-0.707;0; 0.707];
        r(:,10) = [ 2;0.6;-0.6]; d(:,10) = [-0.707;0;0.707];
        r(:,11) = [-2;-0.6;-0.6]; d(:,11) = [0.707;0;0.707];
        r(:,12) = [-2; 0.6;-0.6]; d(:,12) = [0.707;0;0.707];
        B_all = zeros(6, 12);
        for i = 1:12
            B_all(1:3, i) = d(:,i);
            B_all(4:6, i) = cross(r(:,i), d(:,i));
        end
        % % 推力器复用配置
        % template = struct('is_multiplexed', false, 'axis_names', {{}}, 'orbit_partner', {{}});
        % Config_Struct = repmat(template, 12, 1);
        % Config_Struct(1).is_multiplexed = false; Config_Struct(1).axis_names = {'Tocpy'}; Config_Struct(1).orbit_partner{1} = 3;
        % Config_Struct(2).is_multiplexed = false; Config_Struct(2).axis_names = {'Tocpy'}; Config_Struct(2).orbit_partner{1} = 4;
        % Config_Struct(3).is_multiplexed = false; Config_Struct(3).axis_names = {'Tocpy'}; Config_Struct(3).orbit_partner{1} = 1;
        % Config_Struct(4).is_multiplexed = false; Config_Struct(4).axis_names = {'Tocpy'}; Config_Struct(4).orbit_partner{1} = 2;
        % Config_Struct(5).is_multiplexed = true;  Config_Struct(5).axis_names = {'Tocpx', 'Tocpz'}; Config_Struct(5).orbit_partner{1} = [6,9,10]; Config_Struct(5).orbit_partner{2} = [6,7,8];
        % Config_Struct(6).is_multiplexed = true;  Config_Struct(6).axis_names = {'Tocpx', 'Tocpz'}; Config_Struct(6).orbit_partner{1} = [5,9,10]; Config_Struct(6).orbit_partner{2} = [5,7,8];
        % Config_Struct(7).is_multiplexed = true;  Config_Struct(7).axis_names = {'Tocpx', 'Tocpz'}; Config_Struct(7).orbit_partner{1} = [8,11,12]; Config_Struct(7).orbit_partner{2} = [5,6,8];
        % Config_Struct(8).is_multiplexed = true;  Config_Struct(8).axis_names = {'Tocpx', 'Tocpz'}; Config_Struct(8).orbit_partner{1} = [7,11,12]; Config_Struct(8).orbit_partner{2} = [5,6,7];
        % Config_Struct(9).is_multiplexed = true;  Config_Struct(9).axis_names = {'Tocpx', 'Tocpz'}; Config_Struct(9).orbit_partner{1} = [5,6,10]; Config_Struct(9).orbit_partner{2} = [10,11,12];
        % Config_Struct(10).is_multiplexed = true; Config_Struct(10).axis_names = {'Tocpx', 'Tocpz'}; Config_Struct(10).orbit_partner{1} = [5,6,9]; Config_Struct(10).orbit_partner{2} = [9,11,12];
        % Config_Struct(11).is_multiplexed = true; Config_Struct(11).axis_names = {'Tocpx', 'Tocpz'}; Config_Struct(11).orbit_partner{1} = [7,8,12]; Config_Struct(11).orbit_partner{2} = [9,10,12];
        % Config_Struct(12).is_multiplexed = true; Config_Struct(12).axis_names = {'Tocpx', 'Tocpz'}; Config_Struct(12).orbit_partner{1} = [7,8,11]; Config_Struct(12).orbit_partner{2} = [9,10,11];
    end
end