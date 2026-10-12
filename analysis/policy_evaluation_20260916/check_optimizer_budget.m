root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
backup=fullfile(root,'recovery','before_policy_evaluation_20260916');
folder=tempname; mkdir(folder); cleaner=onCleanup(@()rmdir(folder,'s')); %#ok<NASGU>
addpath(root); oldpwd=pwd; cd(folder); pwdCleaner=onCleanup(@()cd(oldpwd)); %#ok<NASGU>
p=struct('Num',12,'F_max',10); seed=[.2,0,0,pi/4,.6,-.2,pi/2,pi/4,.6,.2,3*pi/2,pi/4];
cfg=struct('initial_x',seed,'start_count',1,'max_time',3,'master_time',1,'geometry_time',1,'save_result',false,'output_dir',folder);
addpath(backup,'-begin'); clear Reconfig_eval Thruster_invocation
fprintf('BEFORE evaluator: %s\n',which('Reconfig_eval'));
before=Optim_Algorithm(p,cfg);
rmpath(backup); clear Reconfig_eval Thruster_invocation
fprintf('AFTER evaluator: %s\n',which('Reconfig_eval'));
after=Optim_Algorithm(p,cfg);
cfg.max_time=20;
longer=Optim_Algorithm(p,cfg);
result=struct('before_status',before.status,'before_method',isfield(before.solver_evidence,'method'), ...
 'after_status',after.status,'after_method',isfield(after.solver_evidence,'method'), ...
 'longer_status',longer.status,'longer_method',isfield(longer.solver_evidence,'method'));
disp(result);
save(fullfile(root,'analysis','policy_evaluation_20260916','optimizer_budget_comparison.mat'),'before','after','longer','result');
f=fopen(fullfile(root,'analysis','policy_evaluation_20260916','optimizer_budget_comparison.json'),'w'); fprintf(f,'%s',jsonencode(result,PrettyPrint=true)); fclose(f);
