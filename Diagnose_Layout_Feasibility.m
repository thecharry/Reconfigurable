function result = Diagnose_Layout_Feasibility(params,opt_cfg)
%DIAGNOSE_LAYOUT_FEASIBILITY 精确的两层组选必要条件诊断。
% r = Diagnose_Layout_Feasibility(params,struct('max_time',240));
% r.feasible始终为false；必要条件见证不是安装布局。
% 若r.geometry_checkpoint_file非空，可将其作为Optim_Algorithm的resume_file。
    if nargin<1 || isempty(params), params=Get_params(); end
    if nargin<2, opt_cfg=struct(); end
    opt_cfg.solver = 'diagnostic';
    result = Optim_Algorithm(params,opt_cfg);
end
