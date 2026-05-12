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
    [params.Z, params.Jc, ~, params.Ja, params.Jo, params.Jf] = Reconfig_eval(params, params.B_all);

    %% 推力器原布局
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
    end
end