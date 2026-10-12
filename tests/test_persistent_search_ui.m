function tests = test_persistent_search_ui
% 状态显示测试；模拟记录只用于界面，不代表真实必要条件见证。
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root=fileparts(fileparts(mfilename('fullpath')));
    testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
    folder=testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
    source=fileread(fullfile(root,'source','RCDesignerPrototype.m'));
    source=strrep(source,'RCDesignerPrototype','RCDesignerSearchTestApp');
    source=strrep(source,'Access = private','Access = public');
    source=strrep(source,'app.restoreLatestDiagnostic();','');
    fid=fopen(fullfile(folder.Folder,'RCDesignerSearchTestApp.m'),'w','n','UTF-8');
    cleaner=onCleanup(@()fclose(fid));fprintf(fid,'%s',source);clear cleaner;
    testCase.applyFixture(matlab.unittest.fixtures.PathFixture(folder.Folder));
    testCase.TestData.App=RCDesignerSearchTestApp;
    testCase.TestData.App.UIFigure.Visible='off';
    testCase.TestData.Record=jsondecode(fileread(fullfile(root,'analysis','layered_resume_150s_20260910','diagnosis.json')));
    testCase.TestData.Folder=folder.Folder;
end

function teardownOnce(testCase)
    delete(testCase.TestData.App);
end

function setup(testCase)
    app=testCase.TestData.App;
    app.populateParameterControls();app.DropDown1.Value='完整组选必要条件';
    app.EditField1_2.Value='[0,0.6]';app.EditField1_3.Value='[-0.6,0.6]';
    app.EditField1_4.Value='[0,2π]';app.EditField1_5.Value='[0,0.5π]';
    app.DiagnosticRecord=testCase.TestData.Record;
    app.DiagnosticScopeKey=app.searchScopeKey(app.DiagnosticRecord.scope);
    app.DiagnosticTrusted=true;app.SearchRunning=false;
end

function testForceOnlyDoesNotEnableGeometry(testCase)
    app=testCase.TestData.App;app.refreshSearchState();
    verifySubstring(testCase,app.StatusLabel.Text,'78项');
    verifyEqual(testCase,app.StartOptimizationButton_3.Enable,matlab.lang.OnOffSwitchState.on);
    app.DropDown1.Value='安装几何与布局';app.refreshSearchState();
    verifyEqual(testCase,app.StartOptimizationButton_3.Enable,matlab.lang.OnOffSwitchState.off);
end

function testStructuralStageExecutesWithoutGeometryOrDiagnosticWitness(testCase)
    app=testCase.TestData.App;
    app.DropDown1.Value='安装拓扑解析筛选';
    app.DiagnosticTrusted=false;
    app.refreshSearchState();
    verifyEqual(testCase,app.StartOptimizationButton_3.Text,'筛选安装拓扑');
    app.StartOptimizationButtonPushed([]);
    verifyFalse(testCase,app.SearchRunning);
    verifySubstring(testCase,app.ProgressLabel.Text,'115');
    verifySubstring(testCase,app.ProgressLabel.Text,'558');
    verifySubstring(testCase,app.StatusLabel.Text,'尚未证明可行');
end

function testSupportRotationRunsAndResumesWithoutNecessaryWitness(testCase)
    app=testCase.TestData.App;
    oldRoot=app.ProjectRoot; oldBudget=app.GenerationField_3.Value;
    restore=onCleanup(@()restoreSupportSettings(app,oldRoot,oldBudget)); %#ok<NASGU>
    app.ProjectRoot=testCase.TestData.Folder;
    app.DropDown1.Value='115拓扑自动布局搜索';
    app.DiagnosticTrusted=false;app.GenerationField_3.Value=max(10,app.GenerationField_3.Limits(1));
    app.refreshSearchState();
    verifyEqual(testCase,app.StartOptimizationButton_3.Enable,matlab.lang.OnOffSwitchState.on);
    app.StartOptimizationButtonPushed([]);
    verifyFalse(testCase,app.SearchRunning);
    verifyEqual(testCase,app.OptimResult.solver_evidence.support_total,115);
    visits=app.OptimResult.solver_evidence.support_visits;
    verifyGreaterThan(testCase,visits,0);
    verifyGreaterThan(testCase,app.OptimResult.solver_evidence.search_metrics.direct_calls,0);
    verifyGreaterThan(testCase,app.OptimResult.solver_evidence.search_metrics.direct_evaluations,0);
    app.StartOptimizationButtonPushed([]);
    verifyGreaterThan(testCase,app.OptimResult.solver_evidence.support_visits,visits);
    verifySubstring(testCase,app.ProgressLabel.Text,'已保存');
    verifyEqual(testCase,app.StartOptimizationButton_3.Text,'继续拓扑搜索');
end

function restoreSupportSettings(app,root,budget)
    app.ProjectRoot=root;app.GenerationField_3.Value=budget;
end

function testCompleteStateStopsThisStageAndAllowsGeometry(testCase)
    app=testCase.TestData.App;
    app.DiagnosticRecord.status='necessary_conditions_sat';
    app.DiagnosticRecord.geometry_checkpoint_file='UI_STATE_FIXTURE_ONLY.json';
    app.refreshSearchState();
    verifySubstring(testCase,app.StatusLabel.Text,'156/156项精确复核通过');
    verifyEqual(testCase,app.StartOptimizationButton_3.Enable,matlab.lang.OnOffSwitchState.off);
    app.GenerationField_3.Value=1200;app.refreshSearchState();
    verifyEqual(testCase,app.StartOptimizationButton_3.Enable,matlab.lang.OnOffSwitchState.off);
    app.DropDown1.Value='安装几何与布局';app.refreshSearchState();
    verifyEqual(testCase,app.StartOptimizationButton_3.Enable,matlab.lang.OnOffSwitchState.on);
    app.EditField1_2.Value='[0,0.5]';app.SearchSettingsChanged([]);
    verifyEqual(testCase,app.StartOptimizationButton_3.Enable,matlab.lang.OnOffSwitchState.off);
end

function testLoadedClaimRequiresFreshVerification(testCase)
    app=testCase.TestData.App;record=app.DiagnosticRecord;
    record.status='necessary_conditions_sat';record.geometry_checkpoint_file='UI_STATE_FIXTURE_ONLY.json';
    path=fullfile(testCase.TestData.Folder,'ui_fixture.json');
    fid=fopen(path,'w');cleaner=onCleanup(@()fclose(fid));fprintf(fid,'%s',jsonencode(record));clear cleaner;
    app.acceptDiagnosticRecord(path,false);
    verifySubstring(testCase,app.StatusLabel.Text,'先复核证据');
    verifyEqual(testCase,app.StartOptimizationButton_3.Enable,matlab.lang.OnOffSwitchState.on);
    app.DropDown1.Value='安装几何与布局';app.refreshSearchState();
    verifyEqual(testCase,app.StartOptimizationButton_3.Enable,matlab.lang.OnOffSwitchState.off);
end

function testProgressMeansCertifiedMassNotTimeAndReviewStopsRepeating(testCase)
    app=testCase.TestData.App;
    p=struct('certified_percent',12.5,'closed_branches',2,'conflicts',5,'solver_calls',20, ...
        'pending_branches',3,'review_branches',1);
    app.DiagnosticRecord.branch_progress=p;app.refreshSearchState();
    verifyEqual(testCase,app.SearchProgressFill.Position(3),.125,'AbsTol',1e-12);
    app.beginOptimizationProgress(16);
    verifyEqual(testCase,app.LoadSearchRecordButton.Enable,matlab.lang.OnOffSwitchState.off);
    app.updateOptimizationProgress(struct('Progress',.99,'BranchProgress',p));
    verifyEqual(testCase,app.SearchProgressFill.Position(3),.125,'AbsTol',1e-12);
    app.finishOptimizationProgress(false);
    app.DiagnosticRecord.status='diagnostic_review_required';app.refreshSearchState();
    verifyEqual(testCase,app.StartOptimizationButton_3.Enable,matlab.lang.OnOffSwitchState.off);
    verifySubstring(testCase,app.StatusLabel.Text,'尚未认证无解');
end
