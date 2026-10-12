function tests = test_comparison_ui
% 比较界面的回归测试。私有成员仅在临时测试副本中开放，生产源码不变。
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
    folder = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
    source = fileread(fullfile(root, 'source', 'RCDesignerPrototype.m'));
    source = strrep(source, 'RCDesignerPrototype', 'RCDesignerComparisonTestApp');
    source = strrep(source, 'Access = private', 'Access = public');
    source = strrep(source, "app.UIFigure.Visible = 'on';", ...
        "app.UIFigure.Visible = 'off';");
    source = strrep(source, 'app.restoreLatestDiagnostic();', '');
    source = strrep(source, 'uialert(app.UIFigure, ME.message, titleText);', 'rethrow(ME);');
    path = fullfile(folder.Folder, 'RCDesignerComparisonTestApp.m');
    fid = fopen(path, 'w', 'n', 'UTF-8');
    assert(fid ~= -1, '无法创建临时测试类。');
    cleaner = onCleanup(@()fclose(fid));
    fprintf(fid, '%s', source);
    clear cleaner;
    testCase.applyFixture(matlab.unittest.fixtures.PathFixture(folder.Folder));
    testCase.TestData.App = RCDesignerComparisonTestApp;
    testCase.TestData.App.UIFigure.Visible = 'off';
    testCase.TestData.Root = root;
end

function teardownOnce(testCase)
    if isfield(testCase.TestData, 'App') && isvalid(testCase.TestData.App)
        delete(testCase.TestData.App);
    end
end

function setup(testCase)
    app = testCase.TestData.App;
    app.LayoutEntries = struct('Name', '原布局', 'B', app.ReferenceB, ...
        'r', app.ReferenceR, 'Kind', 'original');
    app.CurrentB = app.ReferenceB;
    app.CurrentR = app.ReferenceR;
    app.refreshLayoutDropDown('原布局');
    app.FmaxField_5.Value = 1;
    app.FmaxField_6.Value = 1;
    app.SimulationTimeField.Value = 1.0;
    app.MinPulseField_4.Value = 0.05;
    app.MinPulseField_5.Value = 0;
    app.SimulationLog = [];
end

function testFaultCountZeroAndFaultIdZero(testCase)
    app = testCase.TestData.App;
    params = app.readParamsFromUI();
    verifyEqual(testCase, app.FmaxField_5.Limits(1), 0);
    app.FmaxField_5.Value = 0;
    verifyEqual(testCase, app.requestedFaultCount(params.Num), 0);
    cases = app.getFaultCases(params, 1, 0);
    verifyEqual(testCase, cases, {[]});
    [cases, labels] = app.simulationFaultCases(params, cases);
    verifyEqual(testCase, cases, {[]});
    verifyEqual(testCase, labels, {'标况'});
    verifyEqual(testCase, app.selectedSummaryFaultNumbers(params.Num), 0);
    verifyEmpty(testCase, app.selectedMetricsFaultSet(params.Num));

    for faultCount = [1, 2]
        app.FmaxField_5.Value = faultCount;
        app.FmaxField_6.Value = 0;
        cases = app.getFaultCases(params, 0, faultCount);
        [cases, labels] = app.simulationFaultCases(params, cases);
        verifyEqual(testCase, numel(cases), params.Num + 1);
        verifyEmpty(testCase, cases{1});
        verifyEqual(testCase, cellfun(@(c)c, cases(2:end)), 1:params.Num);
        verifyEqual(testCase, labels, [{'标况'}, ...
            arrayfun(@(k)sprintf('故障%d', k), 1:params.Num, 'UniformOutput', false)]);
    end
end

function testOriginalAndSelectedLayoutOnly(testCase)
    app = testCase.TestData.App;
    original = app.selectedAllocationLayout();
    verifyEqual(testCase, numel(app.buildComparisonLayoutSet(original)), 1);
    installTestLayouts(app);
    selected = app.selectedSimulationLayout();
    layouts = app.buildComparisonLayoutSet(selected);
    verifyEqual(testCase, {layouts.name}, {'原布局', '测试布局'});
    verifyEqual(testCase, layouts(1).B, app.ReferenceB);
    verifyEqual(testCase, layouts(2).B, app.LayoutEntries(2).B);
end

function testAllocationColumnsAndWholeRowHighlight(testCase)
    app = testCase.TestData.App;
    params = app.readParamsFromUI();
    params.allocation_strategy = 'primary_backup';
    original = app.selectedAllocationLayout();
    strategy = RCPlotter.allocationComparisonData(params, original, 1);
    expectedHeaders = {'控制类型', '标况下调用（原布局）', ...
        '故障下调用（原布局）', '调用状态（原布局）'};
    verifyEqual(testCase, strategy.ColumnNames, expectedHeaders);
    verifySize(testCase, strategy.Data, [12, 4]);
    verifyEqual(testCase, string(strategy.Data(:, 1)), ...
        ["轨道控制+X"; "轨道控制-X"; "轨道控制+Y"; "轨道控制-Y"; ...
         "轨道控制+Z"; "轨道控制-Z"; "姿态控制+X"; "姿态控制-X"; ...
         "姿态控制+Y"; "姿态控制-Y"; "姿态控制+Z"; "姿态控制-Z"]);

    installTestLayouts(app);
    layouts = app.buildComparisonLayoutSet(app.selectedAllocationLayout());
    strategy = RCPlotter.allocationComparisonData(params, layouts, 1);
    verifyEqual(testCase, strategy.ColumnNames, [expectedHeaders, ...
        {'标况下调用（测试布局）', '故障下调用（测试布局）', '调用状态（测试布局）'}]);
    verifySize(testCase, strategy.Data, [12, 7]);
    expectedFaultRows = false(12, 1);
    for row = 1:12
        for column = [2, 3, 5, 6]
            ids = str2double(regexp(strategy.Data{row, column}, '\d+', 'match'));
            expectedFaultRows(row) = expectedFaultRows(row) || any(ids == 1);
        end
    end
    verifyTrue(testCase, any(expectedFaultRows));
    verifyEqual(testCase, strategy.FaultRows, expectedFaultRows);
    app.updateAllocationStrategyTable(strategy);
    verifyEqual(testCase, app.AllocationFaultRows, expectedFaultRows);
    configs = app.AllocationStrategyTable.StyleConfigurations;
    redRows = [];
    for row = 1:height(configs)
        style = configs.Style(row);
        if iscell(style), style = style{1}; end
        color = style.BackgroundColor;
        if isnumeric(color) && color(1) > color(2) && color(1) > color(3)
            verifyEqual(testCase, string(configs.Target(row)), "row");
            indices = configs.TargetIndex{row};
            redRows = [redRows, indices(:)']; %#ok<AGROW>
        end
    end
    verifyEqual(testCase, unique(redRows(:)), find(expectedFaultRows));

    nominal = RCPlotter.allocationComparisonData(params, layouts, []);
    verifyEqual(testCase, nominal.Data(:, 2), nominal.Data(:, 3));
    verifyEqual(testCase, nominal.Data(:, 5), nominal.Data(:, 6));
    verifyFalse(testCase, any(nominal.FaultRows));
    app.updateAllocationStrategyTable(nominal);
    verifyFalse(testCase, any(app.AllocationFaultRows));
end

function testNoFaultSimulationHasOnlyNominal(testCase)
    app = testCase.TestData.App;
    app.FmaxField_5.Value = 0;
    app.RunSimulationButtonPushed([]);
    assertSimulationCompleted(testCase, app);
    verifyEqual(testCase, app.SimulationLog.ComparisonLabels, {'标况（原布局）'});
    verifySize(testCase, app.SimulationCaseViews, [5, 1]);
    verifyEqual(testCase, app.SimulationActualVerificationTable.Data(:, 1), {'标况（原布局）'});
    verifyTrue(testCase, all(arrayfun(@(view)view.Rendered, app.SimulationCaseViews(:))));
    for view = 1:5
        verifyEqual(testCase, numel(app.SimulationCaseViews(view, 1).Tab.Parent.Children), 1);
    end
    verifyEmpty(testCase, app.SimulationLog.ComparisonLogs{1}.faulty_thrusters);
end

function testTwoLayoutsAllComparisonViewsAndPhysicalOutputs(testCase)
    app = testCase.TestData.App;
    installTestLayouts(app);
    app.RunSimulationButtonPushed([]);
    assertSimulationCompleted(testCase, app);
    labels = {'标况（原布局）', '故障1（原布局）', ...
        '标况（测试布局）', '故障1（测试布局）'};
    verifyEqual(testCase, app.SimulationLog.ComparisonLabels, labels);
    verifyEqual(testCase, app.SimulationActualVerificationTable.Data(:, 1), labels');
    verifySize(testCase, app.SimulationCaseViews, [5, 4]);
    expectedMainTabs = ["状态响应对比", "推力器实际输出对比", ...
        "位置姿态视图对比", "推力器实际脉宽对比", "推力器调用时序对比", "实际干扰对比"];
    actualMainTabs = string({app.SimulationViewTabGroup.Children.Title});
    verifyEqual(testCase, sort(actualMainTabs), sort(expectedMainTabs));
    verifyFalse(testCase, any(actualMainTabs == "故障工况对比"));
    counts = [4, 6, 4, 12, 1];
    for view = 1:5
        for caseIndex = 1:4
            app.renderSimulationCase(view, caseIndex);
            caseView = app.SimulationCaseViews(view, caseIndex);
            verifyEqual(testCase, caseView.Tab.Title, labels{caseIndex});
            verifyEqual(testCase, numel(caseView.Tab.Parent.Children), 4);
            verifyTrue(testCase, caseView.Rendered);
            verifyEqual(testCase, numel(caseView.Axes), counts(view));
            for axis = caseView.Axes
                verifyFalse(testCase, any(contains(string(axis.Title.String), ...
                    ["标况", "故障1", "原布局", "测试布局"])));
            end
        end
    end
    for caseIndex = [2, 4]
        log = app.SimulationLog.ComparisonLogs{caseIndex};
        verifyEqual(testCase, log.faulty_thrusters, 1);
        verifyTrue(testCase, all(log.Pulse_Actual_History(1, :) == 0));
        verifyTrue(testCase, all(log.Pulse_Widths(1, :) == 0));
        plot = findobj(app.SimulationCaseViews(4, caseIndex).Axes(1), 'Type', 'stair');
        verifyEqual(testCase, numel(plot), 1);
        verifyTrue(testCase, all(plot.YData == 0));
    end
    for caseIndex = 1:4
        log = app.SimulationLog.ComparisonLogs{caseIndex};
        B = app.SimulationLog.LayoutSet(app.SimulationLog.CaseLayoutIndices(caseIndex)).B;
        expected = app.CurrentParams.F_max * B * ...
            (log.Pulse_Actual_History / log.Control_Period);
        verifyEqual(testCase, log.Actual_Wrench_History, expected, 'AbsTol', 1e-12);
    end
    app.navigateToPage('闭环仿真');
    viewTabs = [app.SimulationPositionTab, app.SimulationCommandTab, ...
        app.SimulationTrajectoryTab, app.SimulationPulseTab, app.SimulationScheduleTab];
    viewNames = {'state', 'output', 'trajectory', 'pulse', 'schedule'};
    artifactFolder = fullfile(testCase.TestData.Root, 'artifacts');
    if ~isfolder(artifactFolder), mkdir(artifactFolder); end
    for view = 1:5
        app.SimulationViewTabGroup.SelectedTab = viewTabs(view);
        caseView = app.SimulationCaseViews(view, 4);
        caseView.Tab.Parent.SelectedTab = caseView.Tab;
        drawnow;
        exportapp(app.UIFigure, fullfile(artifactFolder, ['comparison_ui_', viewNames{view}, '.png']));
    end
end

function testAllSingleFaultTabsIncludeNominal(testCase)
    app = testCase.TestData.App;
    app.FmaxField_5.Value = 2;
    app.FmaxField_6.Value = 0;
    app.RunSimulationButtonPushed([]);
    assertSimulationCompleted(testCase, app);
    labels = [{'标况（原布局）'}, arrayfun(@(k)sprintf('故障%d（原布局）', k), ...
        1:12, 'UniformOutput', false)];
    verifyEqual(testCase, app.SimulationLog.ComparisonLabels, labels);
    verifySize(testCase, app.SimulationCaseViews, [5, 13]);
    verifyEqual(testCase, app.SimulationActualVerificationTable.Data(:, 1), labels');
    for view = 1:5
        tabTitles = arrayfun(@(v)v.Tab.Title, app.SimulationCaseViews(view, :), ...
            'UniformOutput', false);
        verifyEqual(testCase, tabTitles, labels);
        app.renderSimulationCase(view, 13);
        verifyEqual(testCase, app.SimulationCaseViews(view, 13).Tab.Title, '故障12（原布局）');
        verifyTrue(testCase, app.SimulationCaseViews(view, 13).Rendered);
    end
end

function testEachErrorAxisUsesMatchingResponseColor(testCase)
    app = testCase.TestData.App;
    fig = uifigure('Visible', 'off');
    cleaner = onCleanup(@()delete(fig)); %#ok<NASGU>
    grid = uigridlayout(fig, [2, 2]);
    axesList = gobjects(1, 4);
    for index = 1:4, axesList(index) = uiaxes(grid); end
    log = struct('Time', [0, 1, 2], 'Y', zeros(12, 3), ...
        'R', zeros(3, 3), 'Y_euler', zeros(3, 3), 'E', zeros(3, 3));
    log.Y(1:3, :) = [-1, 2, -3; 4, -5, 6; -7, 8, -9];
    log.Y_euler = [-0.1, 0.2, -0.3; 0.4, -0.5, 0.6; -0.7, 0.8, -0.9];
    options = struct('ShowFaultTime', false);
    RCPlotter.plotResponseComponents(axesList(1), log, {''}, 'pos', options);
    RCPlotter.plotResponseComponents(axesList(2), log, {''}, 'att', options);
    RCPlotter.plotErrorNorm(axesList(3), log, {''}, 'pos', options);
    RCPlotter.plotErrorNorm(axesList(4), log, {''}, 'att', options);
    colors = lines(3);
    expectedErrors = {abs(log.Y(1:3, :)), abs(log.Y_euler)};
    for type = 1:2
        response = findobj(axesList(type), 'Type', 'line');
        errors = findobj(axesList(type + 2), 'Type', 'line');
        verifyEqual(testCase, numel(response), 3);
        verifyEqual(testCase, numel(errors), 3);
        for component = 1:3
            responseIndex = find(arrayfun(@(line)isequal(line.Color, colors(component, :)), response));
            errorIndex = find(arrayfun(@(line)isequal(line.Color, colors(component, :)), errors));
            verifyEqual(testCase, numel(responseIndex), 1);
            verifyEqual(testCase, numel(errorIndex), 1);
            verifyEqual(testCase, errors(errorIndex).YData, expectedErrors{type}(component, :), 'AbsTol', 1e-12);
        end
    end
    verifyTrue(testCase, isvalid(app));
end

function testInterferenceUsesEachLayoutsMatrix(testCase)
    app = testCase.TestData.App;
    log = struct('Control_Time', 0, 'Requested_Wrench_History', ones(6, 1), ...
        'Actual_Wrench_History', zeros(6, 1), 'faulty_thrusters', [], ...
        'Alloc_Mode_History', "synchronous", 'Control_Period', 1, ...
        'Pulse_Orbit_History', ones(12, 1), 'Pulse_Att_History', ones(12, 1), ...
        'Raw_Max_Pulse_History', 0);
    result = app.buildActualInvocationVerification({ones(6, 12), 2 * ones(6, 12)}, ...
        {log, log}, {'标况（原布局）', '标况（测试布局）'});
    first = sscanf(result.Data{1, 4}, '%f');
    second = sscanf(result.Data{2, 4}, '%f');
    verifyEqual(testCase, second, 2 * first, 'RelTol', 1e-3);
    verifyEqual(testCase, result.Summary, '实际干扰对比');
end

function testPolicySelectionIsSharedAndClearsOldEvaluation(testCase)
    app=testCase.TestData.App;
    cleaner=onCleanup(@()set(app.AllocationModeDropDown_2,'Value','同步分时复用')); %#ok<NASGU>
    displays={'同步分时复用','异步分时复用','联合优化复用'};
    modes={'synchronous_time_division','asynchronous_time_division','joint_optimization'};
    for k=1:3
        app.AllocationModeDropDown_2.Value=displays{k};
        p=app.readParamsFromUI();
        verifyEqual(testCase,p.alloc_mode,modes{k});
    end
    app.EvaluationResult=struct('Name','旧评价');
    app.AllocationModeDropDownValueChanged([]);
    verifyEmpty(testCase,app.EvaluationResult);
    verifyEmpty(testCase,app.SingleFaultTable.Data);
    app.FmaxField_5.Value=0;
    app.EvaluateButtonPushed([]);
    verifyTrue(testCase,startsWith(app.StatusLabel.Text,'可重构判断完成'),app.StatusLabel.Text);
    verifySize(testCase,app.EvaluationTable.Data,[1,8]);
    verifyFalse(testCase,any(contains(string(app.EvaluationTable.ColumnName),["时序","参考通过"])));
    verifyTrue(testCase,contains(string(app.EvaluationTable.ColumnName(5)),'理论'));
end

function testPolicyMetricsInterface(testCase)
    app=testCase.TestData.App;
    cleaner=onCleanup(@()set(app.AllocationModeDropDown_2,'Value','同步分时复用')); %#ok<NASGU>
    app.AllocationModeDropDown_2.Value='联合优化复用';
    app.FmaxField_5.Value=0;
    app.MetricsEvaluateButtonPushed([]);
    verifyTrue(testCase,startsWith(app.StatusLabel.Text,'可重构设计指标计算完成'),app.StatusLabel.Text);
    verifyEmpty(testCase,app.MetricsPolicyTab);
    verifyEmpty(testCase,app.PolicyDetailTable);
    verifyFalse(testCase,any(strcmp({app.MetricsViewTabGroup.Children.Title},'姿轨干扰')));
    verifyFalse(testCase,isfield(app.EvaluationResult,'Interference'));
    verifyEqual(testCase,app.EvaluationResult.PolicyEvaluation.PolicyMode,'joint_optimization');
    verifyTrue(testCase,any(contains(string(app.SingleFaultTable.ColumnName),'综合评价')));
    verifyEqual(testCase,app.EvaluationResult.Score,app.SingleFaultTable.Data{end,2});
    app.navigateToPage('可重构评价');
    app.MetricsViewTabGroup.SelectedTab=app.MetricsSingleFaultTab;
    drawnow;
    exportapp(app.UIFigure,fullfile(testCase.TestData.Root,'artifacts','policy_evaluation_table.png'));
    app.MetricsViewTabGroup.SelectedTab=app.MetricsControlTab;
    drawnow;
    exportapp(app.UIFigure,fullfile(testCase.TestData.Root,'artifacts','policy_evaluation_capacity.png'));
    app.AllocationModeDropDown_2.Value='同步分时复用';
    app.MetricsEvaluateButtonPushed([]);
    verifyTrue(testCase,startsWith(app.StatusLabel.Text,'可重构设计指标计算完成'),app.StatusLabel.Text);
    app.MetricsViewTabGroup.SelectedTab=app.MetricsControlTab;
    drawnow;
    exportapp(app.UIFigure,fullfile(testCase.TestData.Root,'artifacts','continuous_group_capacity.png'));
    exportapp(app.UIFigure,fullfile(testCase.TestData.Root,'artifacts','evaluation_without_interference.png'));
end

function installTestLayouts(app)
    order = [2:12, 1];
    app.LayoutEntries(2) = struct('Name', '测试布局', ...
        'B', app.ReferenceB(:, order), 'r', app.ReferenceR(:, order), 'Kind', 'imported');
    app.LayoutEntries(3) = struct('Name', '未选布局', ...
        'B', app.ReferenceB, 'r', app.ReferenceR, 'Kind', 'imported');
    app.refreshLayoutDropDown('测试布局');
end

function assertSimulationCompleted(testCase, app)
    assertTrue(testCase, startsWith(app.StatusLabel.Text, '闭环仿真完成'), app.StatusLabel.Text);
    assertTrue(testCase, isstruct(app.SimulationLog));
    assertTrue(testCase, isfield(app.SimulationLog, 'ComparisonLogs'));
end

function testJudgmentUsesContinuousInterior(testCase)
    app=testCase.TestData.App; p=app.readParamsFromUI();
    p.alloc_mode='joint_optimization'; p.allocation_strategy='joint_optimization';
    B=[eye(6),-eye(6)];
    a=app.evaluateReconfigJudgment('测试',B,p,{[],1});
    p.T=.001; p.t_min=10;
    b=app.evaluateReconfigJudgment('测试',B,p,{[],1});
    verifyEqual(testCase,a,b);
    verifyTrue(testCase,a(1).PolicyReconfigurable);
    verifyFalse(testCase,a(2).PolicyReconfigurable);
    verifyTrue(testCase,contains(a(1).Conclusion,'原点在实际六维包络内部'));
end
