function tests=test_policy_evaluation
    tests=functiontests(localfunctions);
end

function setupOnce(t)
    t.applyFixture(matlab.unittest.fixtures.PathFixture(fileparts(fileparts(mfilename('fullpath')))));
    t.TestData.p=struct('F_max',10,'T',0.4,'t_min',0.02,'Num',12);
    t.TestData.B=[eye(6),-eye(6)];
end

function testHardwareDoesNotImplyBoundGroupCapability(t)
    p=t.TestData.p;
    z=Reconfig_eval(p,t.TestData.B,0,false);
    verifyEqual(t,z.Hardware.axis_capacity,10*ones(6,2),'AbsTol',1e-8);
    % Each signed basis column occurs once: a nonzero pure even group is impossible.
    verifyEqual(t,nnz(z.Invocation.strict_equal_axis),0);
    verifyEqual(t,nnz(z.Invocation.axis_callable),0);
    verifyEqual(t,z.Jc,[0,0]);
end

function testPolicySwitchAndMinimumImpulseGaps(t)
    p=t.TestData.p; p.alloc_mode='joint_optimization';
    z=Reconfig_eval(p,t.TestData.B,0,false);
    verifyTrue(t,z.Invocation.policy_reconfigurable);
    verifyTrue(t,z.Invocation.reference_commands_callable);
    verifyEqual(t,z.Invocation.axis_capacity,10*ones(6,2),'AbsTol',2e-5);
    p.t_min=0.2; q=Reconfig_eval(p,t.TestData.B,0,false);
    verifyEqual(t,q.Hardware,z.Hardware);
    verifyTrue(t,q.Invocation.policy_reconfigurable);
    verifyFalse(t,q.Invocation.reference_commands_callable);
    % Positive direction capacity does not imply small commands are reachable.
    verifyEqual(t,q.Invocation.reference_pass_rate,0);
end

function testNoFalseReconfigurationFromPositiveScores(t)
    p=t.TestData.p; z=Reconfig_eval(p,t.TestData.B,0,false);
    z.Score=1; z.MetricRaw=ones(1,4); z=RCPlotter.reconfigStatus(z);
    verifyFalse(t,z.IsReconfig); verifyTrue(t,isnan(z.Score));
    p.alloc_mode='joint_optimization'; z=Reconfig_eval(p,t.TestData.B,0,false);
    z.MetricRaw=zeros(1,4); z=RCPlotter.reconfigStatus(z);
    verifyTrue(t,z.IsReconfig);
end

function testAllFaultsPreservesNominal(t)
    p=t.TestData.p; p.alloc_mode='joint_optimization';
    z=Reconfig_eval(p,t.TestData.B,12,false);
    verifyTrue(t,z.Hardware(1).reconfigurable);
    verifyEqual(t,z.Hardware(2).axis_capacity,zeros(6,2));
    verifyFalse(t,z.Invocation(2).policy_reconfigurable);
end

function testImpulseCancellationDoesNotCancelFirstMoment(t)
    p=struct('T',1,'t_min',0.02);
    M=zeros(6,2); M(1,:)=[1,-1];
    schedule=struct('intervals',{{[0,.5];[.5,1]}});
    q=Reconfig_eval('schedule_metrics',M,schedule,p,zeros(6,1),[]);
    verifyTrue(t,q.valid);
    verifyEqual(t,q.end_cross_impulse,zeros(6,1),'AbsTol',1e-12);
    verifyEqual(t,q.first_moment(1),.25,'AbsTol',1e-12);
    verifyEqual(t,q.peak_cross_force,1);
    verifyEqual(t,q.peak_prefix_impulse(1),.5);
end

function testActualScheduleChecksHardwareLimits(t)
    p=struct('T',1,'t_min',.1); M=zeros(6,1); M(1)=1;
    for intervals={{[0,.05]},{[0,.4;.3,.7]},{[0,1.1]}}
        q=Reconfig_eval('schedule_metrics',M,struct('intervals',{intervals{1}}),p,zeros(6,1),[]);
        verifyFalse(t,q.valid);
    end
    q=Reconfig_eval('schedule_metrics',M,struct('intervals',{{[0,.2]}}),p,zeros(6,1),1);
    verifyFalse(t,q.valid);
    % Adjacent segments are one continuous firing, not two short pulses.
    q=Reconfig_eval('schedule_metrics',M,struct('intervals',{{[0,.06;.06,.12]}}),p,zeros(6,1),[]);
    verifyTrue(t,q.valid);
end

function testHistoricalCoverageAndIndependentWitnesses(t)
    p=t.TestData.p;
    x=[.2,0,0,pi/4,.6,-.2,pi/2,pi/4,.6,.2,3*pi/2,pi/4];
    B=zeros(6,12);
    for k=1:3
        y=x(4*k-3); zz=x(4*k-2); a=x(4*k-1); b=x(4*k);
        r=[2,2,-2,-2;y,-y,y,-y;zz,-zz,zz,-zz];
        d=[cos(b)*[-1,-1,1,1];sin(b)*cos(a)*[1,-1,1,-1];sin(b)*sin(a)*[1,-1,1,-1]];
        B(:,4*k-3:4*k)=[d;cross(r,d)];
    end
    z=Reconfig_eval(p,B,1,false);
    verifyEqual(t,sum(arrayfun(@(v)nnz(v.strict_equal_axis),z.Invocation)),152);
    for f=1:13
        inv=z.Invocation(f); upper=z.Hardware(f).axis_capacity;
        verifyLessThanOrEqual(t,inv.axis_capacity,upper+1e-5);
        for k=1:12
            w=inv.axis_witness{k}; if isempty(w), continue; end
            q=Reconfig_eval('schedule_metrics',10*B,w.schedule,p,w.command,z.FaultSets{f});
            verifyTrue(t,q.valid);
            verifyEqual(t,q.average,w.actual,'AbsTol',1e-10);
        end
    end
end

function testSubsetPlotDataPreservesPolicyAndHardware(t)
    p=t.TestData.p; p.alloc_mode='joint_optimization';
    layouts=struct('name','测试','B',t.TestData.B);
    a=RCPlotter.policyComparisonData(p,layouts,{[],1});
    verifySize(t,a.Data,[6,numel(a.ColumnNames)]);
    verifyEqual(t,numel(a.Evaluations{1,1}.Modes),3);
    verifyEqual(t,a.Evaluations{1,1}.Faults,[]);
    verifyEqual(t,a.Evaluations{1,2}.Faults,1);
    verifyEqual(t,a.Data(1:3,3),{'同步分时复用';'异步分时复用';'联合优化复用'});
    verifyFalse(t,any(contains(string(a.ColumnNames),{'综合','方向/','耗量','参考通过'})));
    b=RCPlotter.reconfigSummaryData(p,layouts,0);
    verifyEqual(t,b.Data{1,4},1);
    fig=uifigure('Visible','off'); c=onCleanup(@()delete(fig)); %#ok<NASGU>
    grid=uigridlayout(fig,[1,2]); axesList=[uiaxes(grid),uiaxes(grid)];
    RCPlotter.plotControlEnvelopeComparison(axesList(1),axesList(2),p,layouts,[]);
    verifyEmpty(t,findobj(axesList(1),'Type','bar'));
    verifyEmpty(t,findobj(axesList(1),'Tag','PolicySampleEnvelope'));
    verifyEmpty(t,findobj(axesList(1),'Tag','PolicyVerifiedPoints'));
    verifyEqual(t,numel(findobj(axesList(1),'Tag','ActualContinuousEnvelope')),1);
    verifyEqual(t,numel(findobj(axesList(1),'Type','patch')),2);
    verifyEqual(t,axesList(1).View,[-37.5,30]);
    RCPlotter.plotControlEnvelopeComparison(axesList(1),axesList(2),p,layouts,1:12);
    verifyEmpty(t,findobj(axesList(1),'Type','patch'));
    point=findobj(axesList(1),'Tag','ActualContinuousEnvelope');
    verifyEqual(t,[point.XData(:),point.YData(:),point.ZData(:)],zeros(1,3));
end

function testContinuousUnionIgnoresTimingAndKeepsFixedRatios(t)
    B=[kron([eye(3),-eye(3)],[1,1]);zeros(3,12)];
    p=struct('F_max',10,'alloc_mode','synchronous_time_division');
    a=Reconfig_eval('continuous_capability',p,B,[]);
    verifyEqual(t,a.Model,'fixed_ratio_joint');
    verifyEqual(t,a.AxisCapacity,[20*ones(3,2);zeros(3,2)],'AbsTol',1e-8);
    verifyEqual(t,numel(a.Groups),6);
    for k=1:numel(a.Groups)
        g=a.Groups(k);
        verifyEqual(t,g.ratio,ones(2,1),'AbsTol',1e-10);
        verifyEqual(t,g.endpoint,10*B(:,g.thrusters)*g.ratio,'AbsTol',1e-10);
        verifyEqual(t,max(g.ratio),1);
    end
    p.T=100; p.t_min=99; p.alloc_mode='asynchronous_time_division';
    b=Reconfig_eval('continuous_capability',p,B,[]);
    verifyEqual(t,a.Endpoints,b.Endpoints);
    verifyEqual(t,a.AxisCapacity,b.AxisCapacity);
    fault=Reconfig_eval('continuous_capability',p,B,1);
    verifyTrue(t,all(arrayfun(@(g)~ismember(1,g.thrusters),fault.Groups)));
    verifyFalse(t,fault.IncludesTimingLimits);
    fig=uifigure('Visible','off'); clean=onCleanup(@()delete(fig)); %#ok<NASGU>
    grid=uigridlayout(fig,[1,2]); ax=[uiaxes(grid),uiaxes(grid)];
    RCPlotter.plotControlEnvelopeComparison(ax(1),ax(2),p,struct('name','固定组','B',B),[]);
    actual=findobj(ax(1),'Tag','ActualContinuousEnvelope');
    verifyEqual(t,actual.Type,'patch');
    verifyEqual(t,numel(findobj(ax(1),'Type','patch')),2);
    vertices=actual.Vertices;
    verifyLessThan(t,min(vecnorm(vertices-[20,20,20],2,2)),1e-7);
    verifyLessThan(t,min(vecnorm(vertices-[-20,-20,-20],2,2)),1e-7);
    verifyTrue(t,actual.UserData.Convex);
end

function testContinuousOverlapIsUnionNotConvexHull(t)
    endpoints=[1,2,-3,0,0;0,0,0,4,2;0,0,0,0,0];
    result=RCPlotter.continuousUnionEndpoints(endpoints);
    verifyEqual(t,sortrows(result'),sortrows([2,0,0;-3,0,0;0,4,0]));
end

function testJointContinuousCapabilityEqualsIndependentHardware(t)
    p=struct('F_max',10,'alloc_mode','joint_optimization');
    a=Reconfig_eval('continuous_capability',p,t.TestData.B,[]);
    verifyEqual(t,a.Model,'independent_continuous');
    verifyEqual(t,a.TheoryGenerators,10*t.TestData.B);
    verifyEqual(t,a.AxisCapacity,10*ones(6,2),'AbsTol',1e-8);
    p.T=.001; p.t_min=1;
    b=Reconfig_eval('continuous_capability',p,t.TestData.B,1);
    verifyEqual(t,b.TheoryGenerators,10*t.TestData.B(:,2:end));
    verifyEqual(t,b.AxisCapacity(1,:),[10,0],'AbsTol',1e-8);
end

function testMultiAxisSharedThrusterCutsOffIndependentCorner(t)
    template=struct('axis',1,'direction',1,'thrusters',[1,2], ...
        'ratio',[1;1],'endpoint',[2;0;0;0;0;0],'backup_candidates',false);
    groups=repmat(template,2,1);
    groups(2).axis=2; groups(2).thrusters=[1,3]; groups(2).endpoint=[0;2;0;0;0;0];
    cap=struct('Groups',groups);
    projection=Reconfig_eval('continuous_projection',cap,1:3);
    verifyTrue(t,projection.Convex);
    verifyLessThanOrEqual(t,sum(projection.Vertices(:,1:2),2),2+1e-8);
    verifyEqual(t,sortrows(projection.Vertices),sortrows([0,0,0;2,0,0;0,2,0]),'AbsTol',1e-8);
    % Same groups without a shared thruster permit both axis maxima together.
    cap.Groups(2).thrusters=[3,4];
    independent=Reconfig_eval('continuous_projection',cap,1:3);
    verifyLessThan(t,min(vecnorm(independent.Vertices-[2,2,0],2,2)),1e-8);
end

function testExclusiveBackupGroupsAreNotConvexified(t)
    template=struct('axis',1,'direction',1,'thrusters',1, ...
        'ratio',1,'endpoint',[2;0;0;0;0;0],'backup_candidates',true);
    groups=repmat(template,3,1);
    groups(2).thrusters=2;
    groups(3).axis=2; groups(3).thrusters=[1,2];
    groups(3).ratio=[1;1]; groups(3).endpoint=[0;2;0;0;0;0];
    projection=Reconfig_eval('continuous_projection',struct('Groups',groups),1:3);
    verifyFalse(t,projection.Convex);
    verifyEqual(t,projection.BranchCount,2);
    % Mixing both X alternatives would create (4,0); selecting one never does.
    verifyLessThanOrEqual(t,projection.Vertices(:,1),2+1e-8);
    verifyEqual(t,numel(projection.Pieces),2);
end

function testInterferenceMarginUsesReverseDirectionAndWholeVector(t)
    M=[diag([10,10,1,1,1,1]),-eye(6)];
    cap=struct('Model','independent_continuous','TheoryGenerators',M);
    positive=Reconfig_eval('interference_margin',cap,[2;0;0],1:3);
    negative=Reconfig_eval('interference_margin',cap,[-2;0;0],1:3);
    verifyEqual(t,positive.Ratio,2,'AbsTol',1e-8);
    verifyEqual(t,negative.Ratio,.2,'AbsTol',1e-8);
    diagonal=Reconfig_eval('interference_margin',cap,[1;1;0],1:3);
    verifyEqual(t,diagonal.Ratio,1,'AbsTol',1e-8); % not sqrt(2)/minimum-axis-capacity
    cap.TheoryGenerators=zeros(6,0);
    none=Reconfig_eval('interference_margin',cap,[0;0;0],1:3);
    lost=Reconfig_eval('interference_margin',cap,[1;0;0],1:3);
    verifyEqual(t,none.Ratio,0); verifyEqual(t,lost.Ratio,Inf);
end

function testInterferencePageDoesNotCallFailedCommandsSafe(t)
    p=t.TestData.p; p.allocation_strategy='joint_optimization';
    report=Reconfig_eval('interference_comparison',p,t.TestData.B,[]);
    verifySize(t,report.Modes(1).ReferenceCommands,[6,52]);
    verifyTrue(t,any(sum(abs(report.Modes(1).ReferenceCommands)>0,1)==3));
    verifyEqual(t,{report.Modes.Mode},{'synchronous_time_division', ...
        'asynchronous_time_division','joint_optimization'});
    for k=1:3
        verifyEqual(t,report.Modes(k).ReferenceCommands,report.Modes(1).ReferenceCommands);
    end
    verifyFalse(t,report.Modes(1).ForceCommandsValid);
    verifyTrue(t,contains(report.Modes(1).ForceJudgment,'待复核'));
    verifyTrue(t,report.Modes(3).ForceCommandsValid);
    verifyEqual(t,report.Modes(3).ForcePeak,0,'AbsTol',1e-8);
    verifyEqual(t,report.Modes(3).ForceJudgment,'无显著干扰');
end

function testContinuousJudgmentIgnoresTiming(t)
    p=t.TestData.p; p.alloc_mode='joint_optimization'; p.evaluation_model='continuous_judgment';
    z=Reconfig_eval(p,t.TestData.B,0);
    p.T=.01; p.t_min=10;
    q=Reconfig_eval(p,t.TestData.B,0);
    verifyEqual(t,q,z);
    verifyTrue(t,z.Invocation.origin_interior);
    verifyFalse(t,z.IncludesTimingLimits);
    q=Reconfig_eval(p,t.TestData.B,1);
    verifyFalse(t,any([q.Invocation(2:end).origin_interior]));
end

function testContinuousJudgmentRejectsFullProjectionsOnly(t)
    p=t.TestData.p; p.alloc_mode='joint_optimization'; p.evaluation_model='continuous_judgment';
    % 力和力矩投影均三维包围原点，但F=M使完整空间只有三维。
    B=[eye(3),-eye(3);eye(3),-eye(3)];
    z=Reconfig_eval(p,B,0);
    verifyFalse(t,z.Invocation.origin_interior);
    verifyFalse(t,z.Hardware.reconfigurable);
end
