function tests = test_full12_optimizer
% 验证范围处理、未找到解的输出契约及MATLAB到联合求解器的连接。
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
% 测试运行器可能切换到tests目录，明确将项目主目录加入本次测试路径。
    testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fileparts(fileparts(mfilename('fullpath')))));
end

function testInvalidBounds(testCase)
    p = struct('Num',12,'F_max',10);
    cfg = struct('solver','native','position_a_range',[0.7,0.8],'save_result',false);
    verifyError(testCase,@()Optim_Algorithm(p,cfg),'RCDesigner:EmptyOptimizationRange');
end

function testInitialLayoutMustBeInsideAllTwelveBounds(testCase)
    p = struct('Num',12,'F_max',10);
    cfg = struct('solver','native','initial_x',ones(1,12),'save_result',false);
    verifyError(testCase,@()Optim_Algorithm(p,cfg),'RCDesigner:InvalidInitialLayout');
end

function testNonzeroCapacityMustExceedNumericalTolerance(testCase)
% 不允许利用纯控制容差把几乎零能力的组算作有效控制。
    p = struct('Num',12,'F_max',10);
    cfg = struct('solver','native','min_axis_capacity',1e-10,'save_result',false);
    verifyError(testCase,@()Optim_Algorithm(p,cfg),'RCDesigner:CapacityBelowTolerance');
end

function testNoSolutionIsNotReturnedAsLayout(testCase)
    p = struct('Num',12,'F_max',10);
    cfg = struct('solver','native','start_count',1,'max_iterations',1,'max_time',10,'save_result',false);
    result = Optim_Algorithm(p,cfg);
    verifyFalse(testCase,result.feasible);
    verifyFalse(testCase,result.global_certified);
    verifyEmpty(testCase,result.B_opt);
    verifyEqual(testCase,result.output_file,"");
    verifySize(testCase,result.diagnostic_x,[1,12]);
    verifyEqual(testCase,result.group_count,1485);
    verifyEqual(testCase,result.search_goal,"first_feasible");
    verifyTrue(testCase,isnan(result.fval));
    verifyLessThanOrEqual(testCase,result.fixed_group_calls,1);
    verifyEqual(testCase,numel(result.diagnostic_missing),156-result.best_coverage);
    verifyEqual(testCase,result.best_coverage_candidate.x,result.best_coverage_x);
    for missing = result.diagnostic_missing
        verifyEqual(testCase,mod(numel(missing.closest_group),2),0);
        verifyLessThanOrEqual(testCase,numel(missing.closest_group),6);
        verifyFalse(testCase,ismember(missing.fault,missing.closest_group));
        labels = ["-Fx","+Fx","-Fy","+Fy","-Fz","+Fz","-Mx","+Mx","-My","+My","-Mz","+Mz"];
        signs = [-1,-1,1,1;1,-1,1,-1;1,-1,1,-1;1,1,1,1;-1,1,1,-1;1,-1,-1,1];
        members = reshape(ismember(1:12,missing.closest_group),4,3);
        verifyTrue(testCase,any(signs(ceil(find(labels==missing.axis)/2),:)*members ~= 0));
    end
end

function testSearchRecordIsNotAnOptimizedLayoutFile(testCase)
% 搜索失败的文件单独命名，保存内容仍明确标记为不可用布局。
    folder = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
    p = struct('Num',12,'F_max',10);
    cfg = struct('solver','native','start_count',1,'max_iterations',1,'max_time',10,'output_dir',folder.Folder);
    result = Optim_Algorithm(p,cfg);
    verifyFalse(testCase,result.feasible);
    [~,name] = fileparts(result.output_file);
    verifyTrue(testCase,startsWith(name,'Optim_search_'));
    record = load(result.output_file);
    verifyEmpty(testCase,record.B_opt);
    verifySize(testCase,record.candidate_B,[6,12]);
    verifySize(testCase,record.candidate_r,[3,12]);
    verifyEqual(testCase,record.saved.output_file,result.output_file);
    verifyFalse(testCase,record.saved.feasible);
    verifyTrue(testCase,isfile(result.diagnostic_file));
    report = fileread(result.diagnostic_file);
    verifyTrue(testCase,contains(report,'最高覆盖='));
    verifyTrue(testCase,contains(report,'故障='));
    verifyTrue(testCase,contains(report,'非零能力判据='));
    verifyTrue(testCase,contains(report,'不构成整个连续空间无解'));
    cfg.resume_file = result.output_file;
    cfg.seed = 19;
    cfg.start_count = 3;
    cfg.save_result = false;
    resumed = Optim_Algorithm(p,cfg);
    expected = reshape(sortrows(reshape(result.resume_candidates(1,:),4,3)')',1,12);
    verifyEqual(testCase,resumed.initial_points(1,:),expected);
    verifyEqual(testCase,resumed.constraints.seed,19);
end

function testHighCoverageSeedSurvivesLocalSearch(testCase)
% 按新6台上限重新枚举基线；旧152项数字不能直接沿用。
    seed = [0,0,0,pi/8,.15,0,3*pi/2,pi/8,.15,0,pi/2,pi/8];
    cfg = struct('solver','native','initial_x',seed,'start_count',1,'max_iterations',8,'max_time',10,'save_result',false);
    result = Optim_Algorithm(struct('Num',12,'F_max',10),cfg);
    modules = reshape(seed,4,3);
    A = zeros(6,12);
    for m = 1:3
        y = modules(1,m); z = modules(2,m); alpha = modules(3,m); beta = modules(4,m);
        r = [2,2,-2,-2;y,-y,y,-y;z,-z,z,-z];
        d = [-cos(beta)*[1,1,-1,-1];sin(beta)*cos(alpha)*[1,-1,1,-1];sin(beta)*sin(alpha)*[1,-1,1,-1]];
        A(:,4*(m-1)+(1:4)) = [d;cross(r,d,1)/2];
    end
    groups = zeros(12,0);
    for n = [2,4,6]
        ids = nchoosek(1:12,n);
        g = zeros(12,size(ids,1));
        for i = 1:size(ids,1)
            g(ids(i,:),i) = 1;
        end
        groups = [groups,g]; %#ok<AGROW>
    end
    W = A*groups; covered = 0;
    for fault = 0:12
        healthy = true(1,size(groups,2));
        if fault > 0
            healthy = groups(fault,:) == 0;
        end
        for axis = 1:6
            pure = max(abs(W(setdiff(1:6,axis),:)),[],1) <= 1e-9;
            for sign = [-1,1]
                covered = covered+any(healthy & pure & sign*W(axis,:) >= 1e-5-1e-9);
            end
        end
    end
    verifyGreaterThanOrEqual(testCase,result.best_coverage,covered);
    verifyEqual(testCase,result.constraints.min_axis_capacity,1e-5);
    verifyFalse(testCase,result.feasible);
    verifyEmpty(testCase,result.B_opt);
end

function testHistoricalLayoutsAreReadBack(testCase)
% 真实历史布局按位置/方向回读，不能直接将旧版五变量当作十二变量。
    root = fileparts(fileparts(mfilename('fullpath')));
    assumeTrue(testCase,isfile(fullfile(root,'Optim_data_20260904_161814_topology.mat')));
    cfg = struct('solver','native','start_count',2,'max_iterations',1,'max_time',10,'save_result',false);
    result = Optim_Algorithm(struct('Num',12,'F_max',10),cfg);
    verifyTrue(testCase,any(contains(result.history_sources,'20260904_161814')));
    verifyGreaterThan(testCase,result.best_coverage,0);
end

function testRejectGroupsLargerThanSix(testCase)
    verifyError(testCase,@()Optim_Algorithm(struct('Num',12,'F_max',10),struct('max_group_size',8)),'RCDesigner:InvalidSearchConfig');
end

function testInvalidStageBudget(testCase)
    cfg = struct('fixed_direction_fraction',2,'save_result',false);
    verifyError(testCase,@()Optim_Algorithm(struct('Num',12,'F_max',10),cfg),'RCDesigner:InvalidSearchConfig');
end

function testJointSolverInfeasibilityEvidence(testCase)
    p = struct('Num',12,'F_max',10);
    cfg = struct('solver','scip','beta_range',[0,0],'max_time',10,'save_result',false);
    result = Optim_Algorithm(p,cfg);
    verifyFalse(testCase,result.feasible);
    verifyEqual(testCase,result.status,"infeasible_for_capacity");
    verifyEqual(testCase,string(result.solver_evidence.status),"infeasible");
    verifyEmpty(testCase,result.B_opt);
    verifyEqual(testCase,string(result.solver_evidence.search_goal),"first_feasible");
    verifyTrue(testCase,isfield(result.solver_evidence,'raw_log'));
end

function testMissingSolverProducesActionableDiagnostic(testCase)
% 环境异常应返回错误状态和完整堆栈，不能当成数学不可行。
    folder = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
    cfg = struct('solver','scip','python_executable',fullfile(folder.Folder,'missing_python'),'save_result',false);
    result = Optim_Algorithm(struct('Num',12,'F_max',10),cfg);
    verifyEqual(testCase,result.status,"solver_error");
    verifyEqual(testCase,result.error.identifier,"RCDesigner:MissingJointSolver");
    verifyTrue(testCase,contains(result.diagnostic_text,'完整调用堆栈'));
    verifyFalse(testCase,result.feasible);
    verifyEmpty(testCase,result.B_opt);
end

function testTimeBudgetDoesNotBecomeInfeasibility(testCase)
    cfg = struct('solver','native','max_time',1e-6,'save_result',false);
    result = Optim_Algorithm(struct('Num',12,'F_max',10),cfg);
    verifyEqual(testCase,result.status,"time_limit");
    verifyFalse(testCase,result.global_certified);
    verifyEqual(testCase,result.starts_attempted,0);
    verifyEmpty(testCase,result.resume_candidates);
end

function testDefaultDecompositionAndMatResume(testCase)
    folder = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
    seed = [.2,0,0,pi/4,.6,-.2,pi/2,pi/4,.6,.2,3*pi/2,pi/4];
    cfg = struct('initial_x',seed,'start_count',1,'max_time',3,'master_time',1,'geometry_time',1,'output_dir',folder.Folder);
    p = struct('Num',12,'F_max',10);
    first = Optim_Algorithm(p,cfg);
    verifyEqual(testCase,first.constraints.solver,"decomposed");
    verifyFalse(testCase,first.feasible);
    verifyEmpty(testCase,first.B_opt);
    verifyEqual(testCase,first.fixed_group_calls,0);
    verifyEqual(testCase,string(first.solver_evidence.method),"topology_linear_certificates_v1");
    verifyGreaterThan(testCase,numel(first.solver_evidence.search_state.conflicts),0);
    verifyTrue(testCase,isfile(first.conflict_file));
    proofFile = jsondecode(fileread(first.conflict_file));
    verifyEqual(testCase,numel(proofFile.conflicts),numel(first.solver_evidence.search_state.conflicts));
    cfg.resume_file = first.output_file;
    cfg.max_time = 2;
    second = Optim_Algorithm(p,cfg);
    verifyEqual(testCase,string(second.solver_evidence.resume_note),"resumed_topologies");
    verifyGreaterThanOrEqual(testCase,numel(second.solver_evidence.search_state.conflicts),numel(first.solver_evidence.search_state.conflicts));
    verifyGreaterThanOrEqual(testCase,second.solver_evidence.search_state.master_calls,first.solver_evidence.search_state.master_calls);
    verifyFalse(testCase,second.global_certified);
    verifyTrue(testCase,contains(second.diagnostic_text,'可复核冲突'));
end

function testStrictInvocationUsesCapacityBeforeSmallestGroup(testCase)
    matrix = zeros(6,6);
    matrix(1,:) = [2e-6,2e-6,5e-6,5e-6,5e-6,5e-6];
    matrix(2:4,3:6) = [1,1,-1,-1;1,-1,1,-1;1,-1,-1,1];
    params = struct('t_min',0,'T',.4);
    params.strict_group_contract = struct('row_reference',ones(6,1),'pure_tolerance',1e-9, ...
        'min_axis_capacity',1e-5,'max_group_size',6);
    [~,info] = Thruster_invocation([2e-6;0;0],zeros(3,1),matrix,[],params);
    group = info.axis_strategy{1,1}.thrusters;
    verifyEqual(testCase,numel(group),4);
    verifyGreaterThanOrEqual(testCase,sum(matrix(1,group)),1e-5-1e-9);
    params.strict_group_contract.min_axis_capacity = 2e-6;
    [~,info] = Thruster_invocation([2e-6;0;0],zeros(3,1),matrix,[],params);
    verifyEqual(testCase,numel(info.axis_strategy{1,1}.thrusters),2);
end

function testDefaultStructuralZeroDirectionIsRejected(testCase)
    p = struct('Num',12,'F_max',10);
    cfg = struct('alpha_range',[0,0],'max_time',5,'start_count',1,'save_result',false);
    result = Optim_Algorithm(p,cfg);
    verifyEqual(testCase,string(result.solver_evidence.infeasibility_proof.axis),"Fz");
    verifyEqual(testCase,result.status,"proven_infeasible_for_bounds");
    verifyFalse(testCase,result.feasible);
    verifyEmpty(testCase,result.B_opt);
    verifyEqual(testCase,result.solver_evidence.master_calls,0);
end

function testLayeredDiagnosticReportsProofWithoutReturningLayout(testCase)
    folder = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
    cfg = struct('alpha_range',[0,0],'max_time',5,'output_dir',folder.Folder);
    r = Diagnose_Layout_Feasibility(struct('Num',12,'F_max',10),cfg);
    verifyEqual(testCase,r.status,"diagnostic_unsat_verified");
    verifyTrue(testCase,r.original_problem_infeasibility_verified);
    verifyFalse(testCase,r.feasible);
    verifyFalse(testCase,r.global_certified);
    verifyEmpty(testCase,r.B_opt);
    verifyTrue(testCase,isfile(r.output_file));
    verifyTrue(testCase,isfile(r.solver_evidence.diagnostic_report_file));
    record=load(r.output_file);
    verifyEmpty(testCase,record.B_opt);
    verifyEqual(testCase,record.saved.search_goal,"necessary_conditions_only");
end

function testLayeredDiagnosticUnknownRemainsUnknown(testCase)
    cfg = struct('max_time',2,'diagnostic_stage_time',.5,'diagnostic_compare',false, ...
        'diagnostic_rlimit',1,'save_result',false);
    r = Diagnose_Layout_Feasibility(struct('Num',12,'F_max',10),cfg);
    verifyEqual(testCase,r.status,"diagnostic_unknown");
    verifyFalse(testCase,r.original_problem_infeasibility_verified);
    verifyFalse(testCase,r.feasible);
    verifyEmpty(testCase,r.B_opt);
    verifyEqual(testCase,r.output_file,"");
    verifyEqual(testCase,numel(r.solver_evidence.layers),2);
end

function testLayeredDiagnosticResumesRealVerifiedForceWitness(testCase)
    root=fileparts(fileparts(mfilename('fullpath')));
    previous=fullfile(root,'analysis','layered_240s_20260910','diagnosis.json');
    assumeTrue(testCase,isfile(previous));
    cfg=struct('max_time',2,'diagnostic_stage_time',.5,'diagnostic_compare',false, ...
        'diagnostic_rlimit',1,'save_result',false,'resume_file',previous);
    r=Diagnose_Layout_Feasibility(struct('Num',12,'F_max',10),cfg);
    verifyEqual(testCase,string(r.solver_evidence.resume_note),"resumed_verified_layers");
    layers=r.solver_evidence.layers;
    if iscell(layers), first=layers{1}; else, first=layers(1); end
    verifyEqual(testCase,string(first.status),"sat_verified");
    verifyEqual(testCase,first.verification.checked_requirements,78);
    verifyEmpty(testCase,first.runs);
    verifyFalse(testCase,r.feasible);
    verifyEmpty(testCase,r.B_opt);
    verifyEqual(testCase,r.status,"diagnostic_unknown");
end
