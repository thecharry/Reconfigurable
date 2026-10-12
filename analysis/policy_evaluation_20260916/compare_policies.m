root=fileparts(fileparts(fileparts(mfilename('fullpath')))); addpath(root);
d=load(fullfile(root,'analysis','policy_evaluation_20260916','initial_comparison.mat'));
p=d.p; layouts={p.B_all,d.B}; names={'原布局','152/156布局'};
modes={'synchronous_time_division','asynchronous_time_division','joint_optimization'};
summary=struct([]); allResults=cell(2,3); index=0;
for mode=1:3
    p.alloc_mode=modes{mode}; p.allocation_strategy='primary_backup';
    for layout=1:2
        tic; z=Reconfig_eval(p,layouts{layout},1); allResults{layout,mode}=z;
        index=index+1; v=z.Invocation;
        summary(index).layout=names{layout}; summary(index).mode=modes{mode};
        summary(index).hardwareCoverage=sum(arrayfun(@(h)nnz(h.axis_capacity>1e-7),z.Hardware));
        summary(index).policyCoverage=sum(arrayfun(@(q)nnz(q.axis_callable),v));
        summary(index).instantaneousCoverage=sum(arrayfun(@(q)nnz(q.strict_axis),v));
        summary(index).equalPulseCoverage=sum(arrayfun(@(q)nnz(q.strict_equal_axis),v));
        summary(index).completeSingleFaults=nnz([v(2:end).policy_reconfigurable]);
        summary(index).referencePassPercent=100*mean([v.reference_pass_rate]);
        summary(index).peakCrossForce=max([v.peak_cross_force]);
        summary(index).peakCrossTorque=max([v.peak_cross_torque]);
        summary(index).endForceImpulse=max([v.end_cross_force_impulse]);
        summary(index).endTorqueImpulse=max([v.end_cross_torque_impulse]);
        summary(index).forceFirstMoment=max([v.cross_force_first_moment]);
        summary(index).torqueFirstMoment=max([v.cross_torque_first_moment]);
        summary(index).seconds=toc;
        fprintf('%s %s: hardware=%d policy=%d strict=%d ref=%.2f%% %.1fs\n', ...
            names{layout},modes{mode},summary(index).hardwareCoverage,summary(index).policyCoverage, ...
            summary(index).instantaneousCoverage,summary(index).referencePassPercent,summary(index).seconds);
    end
end
save(fullfile(root,'analysis','policy_evaluation_20260916','comparison.mat'),'summary','allResults');
fid=fopen(fullfile(root,'analysis','policy_evaluation_20260916','comparison.json'),'w','n','UTF-8');
fprintf(fid,'%s',jsonencode(summary,PrettyPrint=true)); fclose(fid);
