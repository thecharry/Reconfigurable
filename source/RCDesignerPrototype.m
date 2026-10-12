classdef RCDesignerPrototype < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                        matlab.ui.Figure
        RootGrid                        matlab.ui.container.GridLayout
        HeaderPanel                     matlab.ui.container.Panel
        GridLayout1                     matlab.ui.container.GridLayout
        Label1                          matlab.ui.control.Label
        OpenButton                      matlab.ui.control.Button
        ImportButton                    matlab.ui.control.Button
        BodyGrid                        matlab.ui.container.GridLayout
        NavigationPanel                 matlab.ui.container.Panel
        GridLayout3                     matlab.ui.container.GridLayout
        NavigationList                  matlab.ui.control.ListBox
        Button1                         matlab.ui.control.Button
        PageHostPanel                   matlab.ui.container.Panel
        TabGroup                        matlab.ui.container.TabGroup
        SimulationTab                   matlab.ui.container.Tab
        GridLayout10                    matlab.ui.container.GridLayout
        StartOptimizationButton_11      matlab.ui.control.Button
        StartOptimizationButton_10      matlab.ui.control.Button
        GenerateAllocationButton_2      matlab.ui.control.Button
        SimulationViewTabGroup          matlab.ui.container.TabGroup
        SimulationScheduleTab           matlab.ui.container.Tab
        SimulationScheduleGrid          matlab.ui.container.GridLayout
        SimulationScheduleControlGrid   matlab.ui.container.GridLayout
        SimulationScheduleCycleCountField  matlab.ui.control.NumericEditField
        SimulationScheduleCycleCountLabel  matlab.ui.control.Label
        SimulationScheduleStartTimeField  matlab.ui.control.NumericEditField
        SimulationScheduleStartTimeLabel  matlab.ui.control.Label
        SimulationScheduleAxes          matlab.ui.control.UIAxes
        SimulationPulseTab              matlab.ui.container.Tab
        SimulationPulseGrid             matlab.ui.container.GridLayout
        SimulationPulsePlaceholderAxes  matlab.ui.control.UIAxes
        SimulationTrajectoryTab         matlab.ui.container.Tab
        SimulationTrajectoryGrid        matlab.ui.container.GridLayout
        SimulationTrajectoryCaseTabGroup  matlab.ui.container.TabGroup
        SimulationNominalTrajectoryTab  matlab.ui.container.Tab
        SimulationNominalTrajectoryGrid  matlab.ui.container.GridLayout
        SimulationNominalTrajectoryYZAxes  matlab.ui.control.UIAxes
        SimulationNominalTrajectoryXZAxes  matlab.ui.control.UIAxes
        SimulationNominalTrajectoryXYAxes  matlab.ui.control.UIAxes
        SimulationNominalTrajectory3DAxes  matlab.ui.control.UIAxes
        SimulationCommandTab            matlab.ui.container.Tab
        SimulationCommandGrid           matlab.ui.container.GridLayout
        SimulationCommandCaseTabGroup   matlab.ui.container.TabGroup
        SimulationNominalCommandTab     matlab.ui.container.Tab
        SimulationNominalCommandGrid    matlab.ui.container.GridLayout
        SimulationTorqueZAxes           matlab.ui.control.UIAxes
        SimulationTorqueYAxes           matlab.ui.control.UIAxes
        SimulationTorqueXAxes           matlab.ui.control.UIAxes
        SimulationForceZAxes            matlab.ui.control.UIAxes
        SimulationForceYAxes            matlab.ui.control.UIAxes
        SimulationForceXAxes            matlab.ui.control.UIAxes
        SimulationPositionCaseTabGroup matlab.ui.container.TabGroup
        SimulationNominalPositionTab matlab.ui.container.Tab
        SimulationPositionHostGrid matlab.ui.container.GridLayout
        SimulationPulseCaseTabGroup matlab.ui.container.TabGroup
        SimulationNominalPulseTab matlab.ui.container.Tab
        SimulationPulseHostGrid matlab.ui.container.GridLayout
        SimulationScheduleCaseTabGroup matlab.ui.container.TabGroup
        SimulationNominalScheduleTab matlab.ui.container.Tab
        SimulationNominalScheduleGrid matlab.ui.container.GridLayout
        SimulationPositionTab           matlab.ui.container.Tab
        SimulationPositionGrid          matlab.ui.container.GridLayout
        SimulationAttitudeErrorAxes     matlab.ui.control.UIAxes
        SimulationPositionErrorAxes     matlab.ui.control.UIAxes
        SimulationAttitudeAxes          matlab.ui.control.UIAxes
        SimulationAxes                  matlab.ui.control.UIAxes
        Label10                         matlab.ui.control.Label
        AllocationTab                   matlab.ui.container.Tab
        GridLayout9                     matlab.ui.container.GridLayout
        StartOptimizationButton_9       matlab.ui.control.Button
        StartOptimizationButton_8       matlab.ui.control.Button
        GenerateAllocationButton        matlab.ui.control.Button
        AllocationViewTabGroup          matlab.ui.container.TabGroup
        AllocationStrategyTab           matlab.ui.container.Tab
        AllocationStrategyGrid          matlab.ui.container.GridLayout
        AllocationStrategyTable         matlab.ui.control.Table
        Label9                          matlab.ui.control.Label
        ReconfigDesignTab               matlab.ui.container.Tab
        GridLayout25                    matlab.ui.container.GridLayout
        StartOptimizationButton_7       matlab.ui.control.Button
        StartOptimizationButton_6       matlab.ui.control.Button
        MetricsViewTabGroup             matlab.ui.container.TabGroup
        MetricsPolicyTab                matlab.ui.container.Tab
        MetricsPolicyGrid               matlab.ui.container.GridLayout
        PolicyDetailTable               matlab.ui.control.Table
        PolicyInfoLabel                 matlab.ui.control.Label
        MetricsSingleFaultTab           matlab.ui.container.Tab
        MetricsSingleFaultGrid          matlab.ui.container.GridLayout
        SingleFaultTable                matlab.ui.control.Table
        SingleFaultInfoLabel            matlab.ui.control.Label
        MetricsChartsTab                matlab.ui.container.Tab
        MetricsChartsGrid               matlab.ui.container.GridLayout
        MetricsJfAxes                   matlab.ui.control.UIAxes
        MetricsJtAxes                   matlab.ui.control.UIAxes
        MetricsJoAxes                   matlab.ui.control.UIAxes
        MetricsJcAxes                   matlab.ui.control.UIAxes
        MetricsAnglesTab                matlab.ui.container.Tab
        MetricsAnglesGrid               matlab.ui.container.GridLayout
        MetricsAngleAxes3               matlab.ui.control.UIAxes
        MetricsAngleAxes2               matlab.ui.control.UIAxes
        MetricsAngleAxes1               matlab.ui.control.UIAxes
        MetricsControlTab               matlab.ui.container.Tab
        MetricsControlGrid              matlab.ui.container.GridLayout
        MetricsForceAxes                matlab.ui.control.UIAxes
        MetricsTorqueAxes               matlab.ui.control.UIAxes
        Label41                         matlab.ui.control.Label
        MetricsEvaluateButton           matlab.ui.control.Button
        EvaluationTab                   matlab.ui.container.Tab
        GridLayout8                     matlab.ui.container.GridLayout
        StartOptimizationButton_5       matlab.ui.control.Button
        StartOptimizationButton_4       matlab.ui.control.Button
        StartOptimizationButton_2       matlab.ui.control.Button
        Label8                          matlab.ui.control.Label
        JudgmentViewTabGroup            matlab.ui.container.TabGroup
        JudgmentSummaryTab              matlab.ui.container.Tab
        JudgmentSummaryGrid             matlab.ui.container.GridLayout
        JudgmentSummaryTable            matlab.ui.control.Table
        JudgmentDetailTab               matlab.ui.container.Tab
        JudgmentDetailGrid              matlab.ui.container.GridLayout
        EvaluationTable                 matlab.ui.control.Table
        OptimizationTab                 matlab.ui.container.Tab
        GridLayout7                     matlab.ui.container.GridLayout
        ParameterFileGrid_3             matlab.ui.container.GridLayout
        StartOptimizationButton_3       matlab.ui.control.Button
        LoadSearchRecordButton          matlab.ui.control.Button
        DeleteLayoutButton              matlab.ui.control.Button
        Label7                          matlab.ui.control.Label
        ThrusterPanel_2                 matlab.ui.container.Panel
        ThrusterGrid_2                  matlab.ui.container.GridLayout
        Label34_4                       matlab.ui.control.Label
        NumericEditField2_4             matlab.ui.control.NumericEditField
        Label33_2                       matlab.ui.control.Label
        NumericEditField1_2             matlab.ui.control.NumericEditField
        FmaxField_2                     matlab.ui.control.NumericEditField
        Label49_2                       matlab.ui.control.Label
        Label24_19                      matlab.ui.control.Label
        Label24_20                      matlab.ui.control.Label
        Label24_22                      matlab.ui.control.Label
        Label24_23                      matlab.ui.control.Label
        Label34_5                       matlab.ui.control.Label
        NumericEditField2_5             matlab.ui.control.NumericEditField
        Label24_24                      matlab.ui.control.Label
        Label34_6                       matlab.ui.control.Label
        NumericEditField2_6             matlab.ui.control.NumericEditField
        Label24_25                      matlab.ui.control.Label
        Label49_4                       matlab.ui.control.Label
        FmaxField_4                     matlab.ui.control.NumericEditField
        Label24_33                      matlab.ui.control.Label
        Label33_3                       matlab.ui.control.Label
        Label24_34                      matlab.ui.control.Label
        Label33_4                       matlab.ui.control.Label
        Label24_35                      matlab.ui.control.Label
        Label33_5                       matlab.ui.control.Label
        Label24_36                      matlab.ui.control.Label
        Label33_6                       matlab.ui.control.Label
        Label24_37                      matlab.ui.control.Label
        EditField1_2                    matlab.ui.control.EditField
        EditField1_3                    matlab.ui.control.EditField
        EditField1_4                    matlab.ui.control.EditField
        EditField1_5                    matlab.ui.control.EditField
        Panel10_3                       matlab.ui.container.Panel
        GridLayout21_3                  matlab.ui.container.GridLayout
        Label35_3                       matlab.ui.control.Label
        PopulationField                 matlab.ui.control.NumericEditField
        Label36_3                       matlab.ui.control.Label
        GenerationField_3               matlab.ui.control.NumericEditField
        MinimumAxisCapacityField        matlab.ui.control.NumericEditField
        MinimumAxisCapacityLabel        matlab.ui.control.Label
        Label37                         matlab.ui.control.Label
        DropDown1                       matlab.ui.control.DropDown
        LayoutViewTabGroup              matlab.ui.container.TabGroup
        LayoutParametersTab             matlab.ui.container.Tab
        LayoutParametersGrid            matlab.ui.container.GridLayout
        LayoutTable                     matlab.ui.control.Table
        LayoutViewsTab                  matlab.ui.container.Tab
        LayoutViewsGrid                 matlab.ui.container.GridLayout
        LayoutYZAxes                    matlab.ui.control.UIAxes
        LayoutXYAxes                    matlab.ui.control.UIAxes
        LayoutXZAxes                    matlab.ui.control.UIAxes
        Layout3DTab                     matlab.ui.container.Tab
        Layout3DGrid                    matlab.ui.container.GridLayout
        LayoutAxes                      matlab.ui.control.UIAxes
        ApplyLayoutButton_2             matlab.ui.control.Button
        LayoutTemplateDropDown          matlab.ui.control.DropDown
        ParameterTab                    matlab.ui.container.Tab
        GridLayout5                     matlab.ui.container.GridLayout
        ThrusterPanel_4                 matlab.ui.container.Panel
        ThrusterGrid_4                  matlab.ui.container.GridLayout
        FmaxField_6                     matlab.ui.control.NumericEditField
        Label50_5                       matlab.ui.control.Label
        MinPulseField_5                 matlab.ui.control.NumericEditField
        Label50_4                       matlab.ui.control.Label
        FmaxField_5                     matlab.ui.control.NumericEditField
        Label49_5                       matlab.ui.control.Label
        Label24_40                      matlab.ui.control.Label
        Label24_39                      matlab.ui.control.Label
        Label4                          matlab.ui.control.Label
        ControllerPanel                 matlab.ui.container.Panel
        ControllerGrid                  matlab.ui.container.GridLayout
        Label47_12                      matlab.ui.control.Label
        FmaxField_12                    matlab.ui.control.NumericEditField
        FmaxField_11                    matlab.ui.control.NumericEditField
        FmaxField_10                    matlab.ui.control.NumericEditField
        FmaxField_9                     matlab.ui.control.NumericEditField
        FmaxField_8                     matlab.ui.control.NumericEditField
        FmaxField_7                     matlab.ui.control.NumericEditField
        Label47_11                      matlab.ui.control.Label
        Label47_10                      matlab.ui.control.Label
        Label47_9                       matlab.ui.control.Label
        Label47_8                       matlab.ui.control.Label
        Label47_7                       matlab.ui.control.Label
        ControlPeriodField              matlab.ui.control.NumericEditField
        Label47                         matlab.ui.control.Label
        Label24_11                      matlab.ui.control.Label
        SpacecraftPanel                 matlab.ui.container.Panel
        SpacecraftGrid                  matlab.ui.container.GridLayout
        JzzField                        matlab.ui.control.NumericEditField
        Label46                         matlab.ui.control.Label
        JyyField                        matlab.ui.control.NumericEditField
        Label45                         matlab.ui.control.Label
        JxxField                        matlab.ui.control.NumericEditField
        Label44                         matlab.ui.control.Label
        MassField                       matlab.ui.control.NumericEditField
        Label43                         matlab.ui.control.Label
        Label24_7                       matlab.ui.control.Label
        Label24_8                       matlab.ui.control.Label
        Label24_9                       matlab.ui.control.Label
        Label24_10                      matlab.ui.control.Label
        Panel8                          matlab.ui.container.Panel
        GridLayout19                    matlab.ui.container.GridLayout
        AllocationModeDropDown_2        matlab.ui.control.DropDown
        Label50_6                       matlab.ui.control.Label
        Label38                         matlab.ui.control.Label
        SimulationTimeField             matlab.ui.control.NumericEditField
        Label24                         matlab.ui.control.Label
        EditField1                      matlab.ui.control.EditField
        Label25                         matlab.ui.control.Label
        EditField2                      matlab.ui.control.EditField
        Label26                         matlab.ui.control.Label
        EditField3                      matlab.ui.control.EditField
        Label27                         matlab.ui.control.Label
        EditField4                      matlab.ui.control.EditField
        Label24_2                       matlab.ui.control.Label
        Label24_3                       matlab.ui.control.Label
        Label24_4                       matlab.ui.control.Label
        Label24_5                       matlab.ui.control.Label
        Label24_6                       matlab.ui.control.Label
        Label38_5                       matlab.ui.control.Label
        Label24_32                      matlab.ui.control.Label
        MinPulseField_4                 matlab.ui.control.NumericEditField
        ThrusterPanel_3                 matlab.ui.container.Panel
        ThrusterGrid_3                  matlab.ui.container.GridLayout
        MinPulseField_3                 matlab.ui.control.NumericEditField
        Label50_3                       matlab.ui.control.Label
        FmaxField_3                     matlab.ui.control.NumericEditField
        Label49_3                       matlab.ui.control.Label
        Label24_26                      matlab.ui.control.Label
        Label24_27                      matlab.ui.control.Label
        Label24_28                      matlab.ui.control.Label
        ParameterFileGrid_2             matlab.ui.container.GridLayout
        Button2_2                       matlab.ui.control.Button
        Button2_4                       matlab.ui.control.Button
        HomeTab                         matlab.ui.container.Tab
        GridLayout4                     matlab.ui.container.GridLayout
        Label3                          matlab.ui.control.Label
        Panel2                          matlab.ui.container.Panel
        GridLayout13                    matlab.ui.container.GridLayout
        Label12                         matlab.ui.control.Label
        Label13                         matlab.ui.control.Label
        Panel3                          matlab.ui.container.Panel
        GridLayout14                    matlab.ui.container.GridLayout
        Label14                         matlab.ui.control.Label
        Label15                         matlab.ui.control.Label
        Panel5                          matlab.ui.container.Panel
        GridLayout16                    matlab.ui.container.GridLayout
        Label18                         matlab.ui.control.Label
        Label19                         matlab.ui.control.Label
        Panel6                          matlab.ui.container.Panel
        GridLayout17                    matlab.ui.container.GridLayout
        Label20                         matlab.ui.control.Label
        Label21                         matlab.ui.control.Label
        Panel7                          matlab.ui.container.Panel
        GridLayout18                    matlab.ui.container.GridLayout
        Label22                         matlab.ui.control.Label
        Label23                         matlab.ui.control.Label
        HomeSimulationPanel             matlab.ui.container.Panel
        HomeSimulationGrid              matlab.ui.container.GridLayout
        HomeSimulationDesc              matlab.ui.control.Label
        HomeSimulationTitle             matlab.ui.control.Label
        Panel1                          matlab.ui.container.Panel
        GridLayout2                     matlab.ui.container.GridLayout
        ProgressLabel                   matlab.ui.control.Label
        SearchProgressTrack             matlab.ui.container.Panel
        SearchProgressFill              matlab.ui.container.Panel
        StatusLabel                     matlab.ui.control.Label
        Label2                          matlab.ui.control.Label
    end


    properties (Access = private)
        ProjectRoot
        CurrentParams
        ReferenceB
        ReferenceR
        CurrentB
        CurrentR
        OptimResult
        DiagnosticRecord = []
        DiagnosticResumeFile = ''
        DiagnosticScopeKey = ''
        DiagnosticTrusted = false
        GeometryScopeKey = ''
        SearchRunning = false
        ReconfigJudgmentResult
        EvaluationResult
        SimulationLog
        SimulationCaseViews = struct('Tab', {}, 'Grid', {}, 'Axes', {}, 'Rendered', {})
        AllocationFaultRows = []
        % Runtime-only results tab. It is intentionally not part of the
        % App Designer component model and is recreated by startupFcn.
        SimulationActualVerificationTab       matlab.ui.container.Tab
        SimulationActualVerificationGrid      matlab.ui.container.GridLayout
        SimulationActualVerificationSummaryLabel matlab.ui.control.Label
        SimulationActualVerificationTable     matlab.ui.control.Table
        LastParameterFile = ''
        ReportLines = {}
        LayoutEntries = struct('Name', {}, 'B', {}, 'r', {}, 'Kind', {})
        ImportedLayoutCount = 0
        OptimizedLayoutCount = 0
        MetricsDiagnosticDynamicAxes = gobjects(0)
        SimulationPulseDynamicAxes = gobjects(0)
        ProgressStartTic = []
        OptimizationCurrentGeneration = 0
        OptimizationMaxGenerations = 1
        OptimizationBestCoverage = 0
    end
    
    methods (Access = private)

        function initializeApp(app)
            app.ProjectRoot = app.resolveProjectRoot();
            addpath(app.ProjectRoot, '-begin');
            app.createActualInvocationVerificationTab();
            app.wireCallbacks();
            app.refreshNavigationItems();
            app.configureLayoutProjectionAxes();
            app.loadDefaultState();
            [navigationLabels, navigationTabs] = app.availableNavigationTabs();
            if ~isempty(navigationLabels)
                app.NavigationList.Value = navigationLabels{1};
                app.TabGroup.SelectedTab = navigationTabs{1};
            end
            app.resizeMainTabGroup();
        end

        function projectRoot = resolveProjectRoot(app)
            sourcePath = fileparts(mfilename('fullpath'));
            candidates = {pwd, fileparts(pwd), sourcePath, fileparts(sourcePath)};
            projectRoot = pwd;
            for index = 1:numel(candidates)
                candidate = candidates{index};
                if isfolder(candidate) && isfile(fullfile(candidate, 'Get_params.m'))
                    projectRoot = candidate;
                    return;
                end
            end
        end

        function [labels, tabs] = availableNavigationTabs(app)
            candidates = { ...
                '项目首页', 'HomeTab'; ...
                '输入条件', 'ParameterTab'; ...
                '优化设计', 'OptimizationTab'; ...
                '可重构判断', 'EvaluationTab'; ...
                '可重构评价', 'ReconfigDesignTab'; ...
                '调用策略', 'AllocationTab'; ...
                '闭环仿真', 'SimulationTab'; ...
                '结果报告', 'ReportTab'};
            labels = {};
            tabs = {};
            for index = 1:size(candidates, 1)
                propertyName = candidates{index, 2};
                if ~isprop(app, propertyName)
                    continue;
                end
                tab = app.(propertyName);
                if isempty(tab) || ~isvalid(tab)
                    continue;
                end
                labels{end + 1} = candidates{index, 1}; %#ok<AGROW>
                tabs{end + 1} = tab; %#ok<AGROW>
            end
        end

        function refreshNavigationItems(app)
            [labels, ~] = app.availableNavigationTabs();
            app.NavigationList.Items = labels;
            if isempty(labels)
                return;
            end
            if ~any(strcmp(labels, app.NavigationList.Value))
                app.NavigationList.Value = labels{1};
            end
        end

        function navigateToPage(app, pageLabel)
            [labels, tabs] = app.availableNavigationTabs();
            index = find(strcmp(labels, pageLabel), 1, 'first');
            if isempty(index)
                return;
            end
            app.NavigationList.Value = labels{index};
            app.TabGroup.SelectedTab = tabs{index};
            if strcmp(labels{index}, '优化设计')
                app.drawLayout();
            end
        end

        function wireCallbacks(app)
            app.NavigationList.ValueChangedFcn = createCallbackFcn(app, @NavigationListValueChanged, true);
            app.OpenButton.ButtonPushedFcn = createCallbackFcn(app, @OpenButtonPushed, true);
            app.ImportButton.ButtonPushedFcn = createCallbackFcn(app, @ImportButtonPushed, true);
            app.Button1.ButtonPushedFcn = createCallbackFcn(app, @HelpButtonPushed, true);
            app.Button2_2.ButtonPushedFcn = createCallbackFcn(app, @ResetParameterButtonPushed, true);
            app.Button2_4.ButtonPushedFcn = createCallbackFcn(app, @LoadParameterButtonPushed, true);
            app.NumericEditField1_2.ValueChangedFcn = createCallbackFcn(app, @LayoutBoundaryValueChanged, true);
            app.NumericEditField2_4.ValueChangedFcn = createCallbackFcn(app, @LayoutBoundaryValueChanged, true);
            app.LayoutTemplateDropDown.ValueChangedFcn = createCallbackFcn(app, @LayoutTemplateDropDownValueChanged, true);
            app.LayoutTable.CellEditCallback = createCallbackFcn(app, @ApplyLayoutButtonPushed, true);
            app.ApplyLayoutButton_2.ButtonPushedFcn = createCallbackFcn(app, @ImportLayoutButtonPushed, true);
            app.DeleteLayoutButton.ButtonPushedFcn = createCallbackFcn(app, @DeleteLayoutButtonPushed, true);
            app.StartOptimizationButton_3.ButtonPushedFcn = createCallbackFcn(app, @StartOptimizationButtonPushed, true);
            app.LoadSearchRecordButton.ButtonPushedFcn = createCallbackFcn(app, @LoadSearchRecordButtonPushed, true);
            searchControls = {'DropDown1','EditField1_2','EditField1_3','EditField1_4','EditField1_5', ...
                'FmaxField_4','FmaxField_2','NumericEditField2_5','NumericEditField2_6'};
            for index = 1:numel(searchControls)
                app.(searchControls{index}).ValueChangedFcn = createCallbackFcn(app, @SearchSettingsChanged, true);
            end
            app.StartOptimizationButton_2.ButtonPushedFcn = createCallbackFcn(app, @EvaluateButtonPushed, true);
            app.MetricsEvaluateButton.ButtonPushedFcn = createCallbackFcn(app, @MetricsEvaluateButtonPushed, true);
            app.GenerateAllocationButton.ButtonPushedFcn = createCallbackFcn(app, @GenerateAllocationButtonPushed, true);
            app.AllocationModeDropDown_2.ValueChangedFcn = createCallbackFcn(app, @AllocationModeDropDownValueChanged, true);
            app.SimulationScheduleStartTimeField.ValueChangedFcn = createCallbackFcn(app, @SimulationScheduleWindowValueChanged, true);
            app.SimulationScheduleCycleCountField.ValueChangedFcn = createCallbackFcn(app, @SimulationScheduleWindowValueChanged, true);
            app.GenerateAllocationButton_2.ButtonPushedFcn = createCallbackFcn(app, @RunSimulationButtonPushed, true);
            conditionButtons = {'StartOptimizationButton_5', 'StartOptimizationButton_7', ...
                'StartOptimizationButton_9', 'StartOptimizationButton_11'};
            layoutButtons = {'StartOptimizationButton_4', 'StartOptimizationButton_6', ...
                'StartOptimizationButton_8', 'StartOptimizationButton_10'};
            for index = 1:numel(conditionButtons)
                app.(conditionButtons{index}).ButtonPushedFcn = createCallbackFcn(app, @NavigateToParameterButtonPushed, true);
                app.(layoutButtons{index}).ButtonPushedFcn = createCallbackFcn(app, @NavigateToOptimizationButtonPushed, true);
            end
            app.PageHostPanel.SizeChangedFcn = createCallbackFcn(app, @PageHostPanelSizeChanged, true);
        end

        function loadDefaultState(app)
            try
                app.CurrentParams = Get_params();
            catch ME
                uialert(app.UIFigure, ['默认参数加载失败：', ME.message], '参数错误');
                rethrow(ME);
            end
            app.ReferenceB = app.CurrentParams.B_all;
            app.ReferenceR = app.CurrentParams.r_all;
            app.CurrentB = app.CurrentParams.B_all;
            app.CurrentR = app.CurrentParams.r_all;
            app.LayoutEntries = struct('Name', '原布局', ...
                'B', app.ReferenceB, 'r', app.ReferenceR, 'Kind', 'original');
            app.ImportedLayoutCount = 0;
            app.OptimizedLayoutCount = 0;
            app.OptimResult = [];
            app.ReconfigJudgmentResult = [];
            app.EvaluationResult = [];
            app.SimulationLog = [];
            app.resetSimulationComparisonViews();
            if ~isempty(app.SimulationPulseDynamicAxes)
                delete(app.SimulationPulseDynamicAxes( ...
                    isgraphics(app.SimulationPulseDynamicAxes)));
            end
            app.SimulationPulseDynamicAxes = gobjects(0);
            if isgraphics(app.SimulationPulsePlaceholderAxes)
                cla(app.SimulationPulsePlaceholderAxes, 'reset');
                app.SimulationPulsePlaceholderAxes.Visible = 'off';
            end
            app.populateParameterControls();
            app.refreshLayoutDropDown('原布局');
            app.prepareDiagnosticAxes(1);
            app.populateLayoutTable(app.CurrentB, app.CurrentR);
            app.drawLayout();
            app.updateAllocationStrategyTable([]);
            app.updateActualInvocationVerification([]);
            app.ProgressStartTic = [];
            if isprop(app, 'ProgressGauge')
                app.ProgressGauge.Value = 0;
            end
            app.ProgressLabel.Text = '等待运行｜0%｜耗时 00:00';
            app.updateReport("已载入默认参数和原始推力器布局。");
            app.setStatus('就绪｜已载入默认参数');
            app.restoreLatestDiagnostic();
        end

        function populateParameterControls(app)
            app.resetMetricsComparisonTabs();
            params = app.CurrentParams;
            app.MassField.Value = params.m;
            app.JxxField.Value = params.J(1, 1);
            app.JyyField.Value = params.J(2, 2);
            app.JzzField.Value = params.J(3, 3);
            app.ControlPeriodField.Value = params.T;
            app.FmaxField_2.Value = params.Num;
            app.FmaxField_3.Value = params.F_max;
            app.MinPulseField_3.Value = params.t_min;

            app.EditField1.Value = '[0,15,55]';
            app.EditField2.Value = '[5,25,35]';
            app.EditField3.Value = '[0,15,55]';
            app.EditField4.Value = '[5,25,35]';
            app.SimulationTimeField.Value = 2000;
            app.MinPulseField_4.Value = 0.005;
            app.FmaxField_7.Value = app.paramOrDefault(params, 'Kp_pos', 10);
            app.FmaxField_8.Value = app.paramOrDefault(params, 'Kd_pos', 300);
            app.FmaxField_9.Value = app.paramOrDefault(params, 'Kp_att', 400);
            app.FmaxField_10.Value = app.paramOrDefault(params, 'Ki_att', 20);
            app.FmaxField_11.Value = app.paramOrDefault(params, 'Kd_att', 3200);
            app.FmaxField_4.Value = 2;
            app.NumericEditField1_2.Value = 0.6;
            app.NumericEditField2_4.Value = 0.6;
            app.NumericEditField2_5.Value = 0;
            app.NumericEditField2_6.Value = 0;
            app.PopulationField.Value = 16;
            app.GenerationField_3.Value = 600;
            app.MinimumAxisCapacityField.Value = 1e-5;
            app.MinimumAxisCapacityField.Enable = 'off';
            app.FmaxField_5.Value = 1;
            app.FmaxField_6.Value = 1;
            app.MinPulseField_5.Value = 1000;
            app.AllocationModeDropDown_2.Value = '同步分时复用';
            app.syncAllocationStrategyControl();
        end

        function params = readParamsFromUI(app)
            if isempty(app.CurrentParams)
                params = Get_params();
            else
                params = app.CurrentParams;
            end

            params.m = app.MassField.Value;
            params.J(1, 1) = app.JxxField.Value;
            params.J(2, 2) = app.JyyField.Value;
            params.J(3, 3) = app.JzzField.Value;
            params.Num = round(app.FmaxField_2.Value);
            params.F_max = app.FmaxField_3.Value;
            params.T = app.ControlPeriodField.Value;
            params.t_min = app.MinPulseField_3.Value;
            params.alloc_mode = app.mapAllocationMode(app.AllocationModeDropDown_2.Value);
            params.allocation_strategy = 'primary_backup';
            params.evaluation_reference_length = 2;
            params.R_earth = app.paramOrDefault(params, 'R_earth', 6371000);
            params.h_orbit = app.paramOrDefault(params, 'h_orbit', 400000);
            params.mu = app.paramOrDefault(params, 'mu', 3.986e14);
            params.n = sqrt(params.mu / (params.R_earth + params.h_orbit)^3);
            params.Kp_pos = app.FmaxField_7.Value;
            params.Kd_pos = app.FmaxField_8.Value;
            params.Kp_att = app.FmaxField_9.Value;
            params.Ki_att = app.FmaxField_10.Value;
            params.Kd_att = app.FmaxField_11.Value;

            if ~isempty(app.CurrentB) && size(app.CurrentB, 2) == params.Num
                params.B_all = app.CurrentB;
                params.r_all = app.CurrentR;
            elseif ~isempty(app.ReferenceB) && size(app.ReferenceB, 2) == params.Num
                params.B_all = app.ReferenceB;
                params.r_all = app.ReferenceR;
            end
            % 严格组契约随其布局保留，额定推力变化时更新物理归一化。
            if isstruct(app.OptimResult) && isfield(app.OptimResult,'feasible') && app.OptimResult.feasible && ...
                    isfield(app.OptimResult,'B_opt') && isfield(app.OptimResult,'constraints') && ...
                    all(isfield(app.OptimResult.constraints,{'x_face','pure_tolerance','min_axis_capacity','max_group_size'})) && ...
                    isfield(params,'B_all') && isequal(params.B_all,app.OptimResult.B_opt)
                cfg = app.OptimResult.constraints;
                params.strict_group_contract = struct('row_reference',params.F_max*[ones(3,1);cfg.x_face*ones(3,1)], ...
                    'reference_length',cfg.x_face,'pure_tolerance',cfg.pure_tolerance, ...
                    'min_axis_capacity',cfg.min_axis_capacity,'max_group_size',cfg.max_group_size);
                params.strict_contract_matrix = params.B_all;
            elseif isfield(params,'strict_group_contract') && (~isfield(params,'strict_contract_matrix') || ...
                    ~isfield(params,'B_all') || ~isequal(params.B_all,params.strict_contract_matrix))
                params = rmfield(params,'strict_group_contract');
                if isfield(params,'strict_contract_matrix'), params = rmfield(params,'strict_contract_matrix'); end
            end
            if isfield(params,'strict_group_contract') && isfield(params.strict_group_contract,'reference_length')
                params.strict_group_contract.row_reference = params.F_max*[ones(3,1);params.strict_group_contract.reference_length*ones(3,1)];
            end
            app.CurrentParams = params;
        end

        function value = paramOrDefault(app, params, fieldName, defaultValue)
            value = defaultValue;
            if isstruct(params) && isfield(params, fieldName) && ...
                    isnumeric(params.(fieldName)) && isscalar(params.(fieldName)) && ...
                    isfinite(params.(fieldName))
                value = params.(fieldName);
            end
        end

        function populateLayoutTable(app, B, r)
            if isempty(B) || isempty(r)
                return;
            end
            nThruster = size(B, 2);
            app.setLayoutTableData([(1:nThruster)', r', B(1:3, :)']);
        end

        function setLayoutTableData(app, numericData)
            % UITable 的单一数值矩阵不能按列保存不同数据类型。
            % 因此使用 cell 数组：第一列为真正的 uint32，其余列保持 double。
            numericData = double(numericData);
            numericData(:, 1) = round(numericData(:, 1));
            tableData = num2cell(numericData);
            tableData(:, 1) = num2cell(uint32(numericData(:, 1)));
            app.LayoutTable.Data = tableData;

            % 第一列按整数文本显示；后六列显示 4 位小数。
            % 如果需要 2 位小数，将后六个 'short' 改为 'bank'。
            app.LayoutTable.ColumnFormat = ...
                {'char', 'short', 'short', 'short', 'short', 'short', 'short'};
        end

        function data = layoutTableNumericData(app)
            rawData = app.LayoutTable.Data;
            if istable(rawData)
                rawData = table2cell(rawData);
            end
            if ~iscell(rawData)
                data = double(rawData);
                return;
            end

            data = zeros(size(rawData));
            for row = 1:size(rawData, 1)
                for column = 1:size(rawData, 2)
                    value = rawData{row, column};
                    if (isnumeric(value) || islogical(value)) && isscalar(value)
                        data(row, column) = double(value);
                    elseif (ischar(value) || (isstring(value) && isscalar(value)))
                        parsedValue = str2double(string(value));
                        if ~isfinite(parsedValue)
                            error('布局表第 %d 行第 %d 列不是有效数值。', row, column);
                        end
                        data(row, column) = parsedValue;
                    else
                        error('布局表第 %d 行第 %d 列不是有效标量数值。', row, column);
                    end
                end
            end
        end

        function [B, r] = readLayoutFromTable(app)
            data = app.layoutTableNumericData();
            r = data(:, 2:4)';
            d = data(:, 5:7)';
            for index = 1:size(d, 2)
                nrm = norm(d(:, index));
                if nrm < 1e-12
                    error('第 %d 台推力器方向向量不能为零。', index);
                end
                d(:, index) = d(:, index) / nrm;
            end
            B = zeros(6, size(data, 1));
            B(1:3, :) = d;
            for index = 1:size(data, 1)
                B(4:6, index) = cross(r(:, index), d(:, index));
            end
        end

        function drawLayout(app)
            if isempty(app.CurrentB) || isempty(app.CurrentR)
                return;
            end
            bodyHalfSize = app.currentBodyHalfSize();
            options = struct('BodyHalfSize', bodyHalfSize, ...
                'BodyFaceColor', [0.8, 0.8, 0.8], ...
                'BodyFaceAlpha', 0.18, 'BodyLineWidth', 1.6);
            RCPlotter.plotLayout3D( ...
                app.LayoutAxes, app.CurrentB, app.CurrentR, options);
            app.drawCurrentLayoutProjections();
            app.validateLayout();
        end

        function bodyHalfSize = currentBodyHalfSize(app)
            xHalf = 2;
            if ~isempty(app.CurrentR)
                currentX = max(abs(app.CurrentR(1, :)));
                if isfinite(currentX) && currentX > 1e-9
                    xHalf = currentX;
                end
            end
            yHalf = max(app.NumericEditField1_2.Value, 1e-6);
            zHalf = max(app.NumericEditField2_4.Value, 1e-6);
            bodyHalfSize = [xHalf, yHalf, zHalf];
        end

        function layoutSet = buildLayoutSet(app)
            layoutSet = app.buildCompatibleLayoutSet(3, size(app.CurrentB, 2));
        end

        function layoutSet = buildAllCompatibleLayoutSet(app, targetCount)
            if nargin < 2 || isempty(targetCount)
                targetCount = size(app.CurrentB, 2);
            end
            layoutSet = app.buildCompatibleLayoutSet(inf, targetCount);
        end

        function layoutSet = buildCompatibleLayoutSet(app, maximumLayouts, targetCount)
            entries = app.LayoutEntries;
            if isempty(entries)
                entries = struct('Name', '原布局', ...
                    'B', app.ReferenceB, 'r', app.ReferenceR, 'Kind', 'original');
            end
            compatible = arrayfun(@(entry)size(entry.B, 2) == targetCount && ...
                size(entry.r, 2) == targetCount, entries);
            entries = entries(compatible);
            if isempty(entries)
                entries = struct('Name', app.LayoutTemplateDropDown.Value, ...
                    'B', app.CurrentB, 'r', app.CurrentR, 'Kind', 'current');
            end
            if isfinite(maximumLayouts)
                entries = entries(1:min(maximumLayouts, numel(entries)));
            end
            layoutSet = repmat(struct('name', '', 'B', [], 'r', []), 1, numel(entries));
            for index = 1:numel(entries)
                layoutSet(index) = struct('name', entries(index).Name, ...
                    'B', entries(index).B, 'r', entries(index).r);
            end
        end

        function axesList = prepareDiagnosticAxes(app, layoutCount)
            if layoutCount < 1
                error('可诊断性页至少需要一个布局。');
            end
            if ~isempty(app.MetricsDiagnosticDynamicAxes)
                validAxes = app.MetricsDiagnosticDynamicAxes( ...
                    isgraphics(app.MetricsDiagnosticDynamicAxes));
                if ~isempty(validAxes)
                    delete(validAxes);
                end
            end
            app.MetricsDiagnosticDynamicAxes = gobjects(0);

            baseAxes = [app.MetricsAngleAxes1, app.MetricsAngleAxes2, ...
                app.MetricsAngleAxes3];
            for index = 1:numel(baseAxes)
                baseAxes(index).Visible = 'off';
                baseAxes(index).Layout.Row = 1;
                baseAxes(index).Layout.Column = 1;
            end

            if layoutCount <= 3
                columnCount = layoutCount;
                rowCount = 1;
            else
                columnCount = ceil(sqrt(layoutCount));
                rowCount = ceil(layoutCount / columnCount);
            end
            app.MetricsAnglesGrid.ColumnWidth = repmat({'1x'}, 1, columnCount);
            app.MetricsAnglesGrid.RowHeight = repmat({'1x'}, 1, rowCount);

            extraCount = max(0, layoutCount - numel(baseAxes));
            dynamicAxes = gobjects(1, extraCount);
            for index = 1:extraCount
                dynamicAxes(index) = uiaxes(app.MetricsAnglesGrid);
            end
            app.MetricsDiagnosticDynamicAxes = dynamicAxes;
            axesList = [baseAxes(1:min(layoutCount, numel(baseAxes))), dynamicAxes];
            for index = 1:layoutCount
                axesList(index).Visible = 'on';
                axesList(index).Layout.Row = ceil(index / columnCount);
                axesList(index).Layout.Column = mod(index - 1, columnCount) + 1;
            end
        end

        function axesList = prepareThrusterPulseAxes(app, thrusterCount)
            if thrusterCount < 1
                error('脉宽显示至少需要一台推力器。');
            end
            if ~isempty(app.SimulationPulseDynamicAxes)
                validAxes = app.SimulationPulseDynamicAxes( ...
                    isgraphics(app.SimulationPulseDynamicAxes));
                if ~isempty(validAxes)
                    delete(validAxes);
                end
            end
            app.SimulationPulseDynamicAxes = gobjects(0);

            columnCount = min(3, thrusterCount);
            rowCount = ceil(thrusterCount / columnCount);
            app.SimulationPulseGrid.ColumnWidth = ...
                repmat({'1x'}, 1, columnCount);
            if isprop(app.SimulationPulseGrid, 'Scrollable')
                app.SimulationPulseGrid.Scrollable = 'on';
                app.SimulationPulseGrid.RowHeight = repmat({210}, 1, rowCount);
            else
                app.SimulationPulseGrid.RowHeight = ...
                    repmat({'1x'}, 1, rowCount);
            end

            axesList = gobjects(1, thrusterCount);
            axesList(1) = app.SimulationPulsePlaceholderAxes;
            axesList(1).Visible = 'on';
            extraAxes = gobjects(1, max(0, thrusterCount - 1));
            for index = 2:thrusterCount
                extraAxes(index - 1) = uiaxes(app.SimulationPulseGrid);
                axesList(index) = extraAxes(index - 1);
            end
            app.SimulationPulseDynamicAxes = extraAxes;
            for index = 1:thrusterCount
                axesList(index).Layout.Row = ceil(index / columnCount);
                axesList(index).Layout.Column = mod(index - 1, columnCount) + 1;
                axesList(index).Visible = 'on';
            end
        end

        function refreshLayoutDropDown(app, selectedName)
            if isempty(app.LayoutEntries)
                names = {'原布局'};
                selectedName = '原布局';
                app.updateLayoutSelectors(names, selectedName);
                return;
            end
            names = {app.LayoutEntries.Name};
            if nargin < 2 || ~any(strcmp(names, selectedName))
                selectedName = names{1};
            end
            app.updateLayoutSelectors(names, selectedName);
        end

        function updateLayoutSelectors(app, names, selectedName)
            selectorNames = {'LayoutTemplateDropDown'};
            for selectorIndex = 1:numel(selectorNames)
                propertyName = selectorNames{selectorIndex};
                if ~isprop(app, propertyName)
                    continue;
                end
                selector = app.(propertyName);
                if isempty(selector) || ~isvalid(selector)
                    continue;
                end
                previousValue = char(string(selector.Value));
                selector.Items = names;
                if any(strcmp(names, selectedName))
                    selector.Value = selectedName;
                elseif any(strcmp(names, previousValue))
                    selector.Value = previousValue;
                else
                    selector.Value = names{1};
                end
            end
        end

        function selectedName = selectedLayoutName(app, selectorName)
            selectedName = app.LayoutTemplateDropDown.Value;
            if isprop(app, selectorName)
                selector = app.(selectorName);
                if ~isempty(selector) && isvalid(selector)
                    selectedName = selector.Value;
                end
            end
        end

        function layout = selectedJudgmentLayout(app)
            selectedName = app.selectedLayoutName('LayoutTemplateDropDown');
            index = find(strcmp({app.LayoutEntries.Name}, selectedName), 1, 'first');
            if isempty(index)
                error('未找到优化设计页选择的布局“%s”。', selectedName);
            end
            layout = struct('name', app.LayoutEntries(index).Name, ...
                'B', app.LayoutEntries(index).B, 'r', app.LayoutEntries(index).r);
        end

        function layout = selectedAllocationLayout(app)
            selectedName = app.selectedLayoutName('LayoutTemplateDropDown');
            index = find(strcmp({app.LayoutEntries.Name}, selectedName), 1, 'first');
            if isempty(index)
                error('未找到优化设计页选择的布局“%s”。', selectedName);
            end
            layout = struct('name', app.LayoutEntries(index).Name, ...
                'B', app.LayoutEntries(index).B, 'r', app.LayoutEntries(index).r);
        end

        function layoutSet = buildComparisonLayoutSet(app, selectedLayout)
            layoutSet = struct('name', '原布局', 'B', app.ReferenceB, 'r', app.ReferenceR);
            if ~strcmp(selectedLayout.name, '原布局')
                layoutSet(end + 1) = selectedLayout;
            end
        end

        function label = comparisonCaseLabel(app, layoutName, faultSet)
            if isempty(faultSet)
                condition = '标况';
            else
                condition = ['故障', app.faultSetText(faultSet)];
            end
            if ~contains(layoutName, '布局'), layoutName = [layoutName, '布局']; end
            label = sprintf('%s（%s）', condition, layoutName);
        end

        function layout = selectedSimulationLayout(app)
            selectedName = app.selectedLayoutName('LayoutTemplateDropDown');
            index = find(strcmp({app.LayoutEntries.Name}, selectedName), 1, 'first');
            if isempty(index)
                error('未找到优化设计页选择的布局“%s”。', selectedName);
            end
            layout = struct('name', app.LayoutEntries(index).Name, ...
                'B', app.LayoutEntries(index).B, 'r', app.LayoutEntries(index).r);
        end

        function name = addLayoutEntry(app, kind, B, r)
            [B, r] = app.normalizeLayoutData(B, r);
            switch lower(kind)
                case 'imported'
                    app.ImportedLayoutCount = app.ImportedLayoutCount + 1;
                    name = sprintf('导入布局%d', app.ImportedLayoutCount);
                case 'optimized'
                    app.OptimizedLayoutCount = app.OptimizedLayoutCount + 1;
                    name = sprintf('优化布局%d', app.OptimizedLayoutCount);
                case 'diagnostic'
                    app.ImportedLayoutCount = app.ImportedLayoutCount + 1;
                    name = sprintf('待验证候选%d', app.ImportedLayoutCount);
                otherwise
                    error('未知布局类型：%s', kind);
            end
            entry = struct('Name', name, 'B', B, 'r', r, 'Kind', lower(kind));
            app.LayoutEntries(end + 1) = entry;
            app.CurrentB = B;
            app.CurrentR = r;
            app.CurrentParams.Num = size(B, 2);
            app.FmaxField_2.Value = size(B, 2);
            app.refreshLayoutDropDown(name);
            app.populateLayoutTable(B, r);
            app.drawLayout();
        end

        function replaceOriginalLayout(app, B, r)
            [B, r] = app.normalizeLayoutData(B, r);
            app.ReferenceB = B;
            app.ReferenceR = r;
            if isempty(app.LayoutEntries)
                app.LayoutEntries = struct('Name', '原布局', ...
                    'B', B, 'r', r, 'Kind', 'original');
            else
                app.LayoutEntries(1) = struct('Name', '原布局', ...
                    'B', B, 'r', r, 'Kind', 'original');
            end
            app.CurrentB = B;
            app.CurrentR = r;
            app.refreshLayoutDropDown('原布局');
        end

        function updateSelectedLayoutEntry(app)
            selected = app.LayoutTemplateDropDown.Value;
            index = find(strcmp({app.LayoutEntries.Name}, selected), 1, 'first');
            if isempty(index)
                return;
            end
            app.LayoutEntries(index).B = app.CurrentB;
            app.LayoutEntries(index).r = app.CurrentR;
            if index == 1
                app.ReferenceB = app.CurrentB;
                app.ReferenceR = app.CurrentR;
            end
        end

        function [B, r] = normalizeLayoutData(app, B, r)
            B = double(B);
            r = double(r);
            if size(r, 1) ~= 3 && size(r, 2) == 3
                r = r';
            end
            if size(B, 1) ~= 3 && size(B, 1) ~= 6 && ...
                    (size(B, 2) == 3 || size(B, 2) == 6)
                B = B';
            end
            if size(r, 1) ~= 3 || size(B, 2) ~= size(r, 2) || ...
                    ~ismember(size(B, 1), [3 6])
                error('布局数据尺寸应为 r=3×N，B=3×N 或 6×N。');
            end
            directions = B(1:3, :);
            norms = vecnorm(directions, 2, 1);
            if any(norms < 1e-12) || any(~isfinite(B(:))) || any(~isfinite(r(:)))
                error('布局数据包含非法数值或零方向向量。');
            end
            directions = directions ./ norms;
            fullB = zeros(6, size(r, 2));
            fullB(1:3, :) = directions;
            for index = 1:size(r, 2)
                fullB(4:6, index) = cross(r(:, index), directions(:, index));
            end
            B = fullB;
        end

        function [B, r, note] = readLayoutFile(app, filePath)
            note = "";
            [~, ~, ext] = fileparts(filePath);
            if strcmpi(ext, '.mat')
                data = load(filePath);
                if isfield(data, 'B_opt') && isfield(data, 'r_opt') && ~isempty(data.B_opt) && ~isempty(data.r_opt)
                    B = data.B_opt;
                    r = data.r_opt;
                elseif isfield(data, 'B') && isfield(data, 'r')
                    B = data.B;
                    r = data.r;
                elseif isfield(data, 'B_all') && isfield(data, 'r_all')
                    B = data.B_all;
                    r = data.r_all;
                elseif isfield(data, 'B_all') && isfield(data, 'r')
                    B = data.B_all;
                    r = data.r;
                elseif isfield(data,'candidate_B') && isfield(data,'candidate_r') && ~isempty(data.candidate_B)
                    B = data.candidate_B;
                    r = data.candidate_r;
                    note = "待验证候选，未通过全部硬约束，不能视为可行优化结果。";
                elseif isfield(data,'saved') && isfield(data.saved,'best_coverage_x') && numel(data.saved.best_coverage_x) == 12
                    % 兼容旧搜索记录：空B_opt不是无几何数据，候选保存在12个安装参数中。
                    x = reshape(data.saved.best_coverage_x,4,3);
                    a = data.saved.constraints.x_face;
                    r = zeros(3,12); B = zeros(3,12);
                    for module = 1:3
                        y = x(1,module); z = x(2,module); alpha = x(3,module); beta = x(4,module);
                        columns = 4*(module-1)+(1:4);
                        r(:,columns) = [a,a,-a,-a;y,-y,y,-y;z,-z,z,-z];
                        B(:,columns) = [-cos(beta)*[1,1,-1,-1];sin(beta)*cos(alpha)*[1,-1,1,-1];sin(beta)*sin(alpha)*[1,-1,1,-1]];
                    end
                    note = "待验证候选，未通过全部硬约束；旧覆盖率不代表新6台上限下的覆盖率。";
                else
                    error('MAT 文件中未找到 B/r、B_opt/r_opt 或 B_all/r_all。');
                end
                if isfield(data,'saved') && isfield(data.saved,'feasible') && ~data.saved.feasible && isfield(data.saved,'solver_evidence') && isfield(data.saved.solver_evidence,'search_state')
                    app.OptimResult = data.saved;
                    app.OptimResult.output_file = string(filePath);
                    note = note + " 再次优化时将恢复此记录的待搜索队列；约束改变时重新建队列。";
                end
            else
                tableData = readtable(filePath, 'TextType', 'string');
                if width(tableData) < 6
                    error('布局表至少需要 X、Y、Z、Dx、Dy、Dz 六列。');
                end
                startColumn = width(tableData) - 5;
                numericData = zeros(height(tableData), 6);
                for row = 1:height(tableData)
                    for column = 1:6
                        value = tableData{row, startColumn + column - 1};
                        numericData(row, column) = app.toScalar(value, NaN);
                    end
                end
                if any(~isfinite(numericData(:)))
                    error('布局表的后六列必须全部为有效数值。');
                end
                r = numericData(:, 1:3)';
                B = numericData(:, 4:6)';
            end
            [B, r] = app.normalizeLayoutData(B, r);
        end

        function configureLayoutProjectionAxes(app)
            projectionAxes = [app.LayoutYZAxes, app.LayoutXYAxes, app.LayoutXZAxes];
            for axesIndex = 1:numel(projectionAxes)
                projectionAxes(axesIndex).Visible = 'on';
            end
        end

        function drawCurrentLayoutProjections(app)
            faultyIndices = app.parseIndexList(app.FmaxField_6.Value);
            viewDefinitions = struct( ...
                'name', {'YZ正视图', 'XY俯视图', 'XZ侧视图'}, ...
                'axes', {[2 3], [1 2], [1 3]}, ...
                'xlabel', {'Y / m', 'X / m', 'X / m'}, ...
                'ylabel', {'Z / m', 'Y / m', 'Z / m'}, ...
                'xlim', {[-1 1], [-2.4 2.4], [-2.4 2.4]}, ...
                'ylim', {[-1 1], [-1 1], [-1 1]}, ...
                'xdir', {'normal', 'reverse', 'reverse'}, ...
                'ydir', {'reverse', 'normal', 'reverse'});
            axesGroups = {app.LayoutYZAxes, app.LayoutXYAxes, app.LayoutXZAxes};
            for viewIndex = 1:numel(viewDefinitions)
                options = struct('LayoutName', app.LayoutTemplateDropDown.Value, ...
                    'FaultyIndices', faultyIndices, 'ShowLabels', false, ...
                    'ShowTitle', true, 'BodyHalfSize', app.currentBodyHalfSize());
                RCPlotter.plotLayoutProjection(axesGroups{viewIndex}, ...
                    app.CurrentB, app.CurrentR, viewDefinitions(viewIndex), options);
            end
        end

        function validateLayout(app)
            if isempty(app.CurrentR)
                return;
            end
            boundaryY = app.NumericEditField1_2.Value;
            boundaryZ = app.NumericEditField2_4.Value;
            minDistance = app.NumericEditField2_5.Value;
            minAngleDeg = app.NumericEditField2_6.Value;
            r = app.CurrentR;
            isInside = all(abs(r(2, :)) <= boundaryY + 1e-9) && ...
                all(abs(r(3, :)) <= boundaryZ + 1e-9);
            actualMinDistance = inf;
            actualMinAngleDeg = inf;
            directions = app.CurrentB(1:3, :);
            directions = directions ./ max(vecnorm(directions, 2, 1), 1e-12);
            for i = 1:size(r, 2)
                for j = (i + 1):size(r, 2)
                    actualMinDistance = min(actualMinDistance, norm(r(:, i) - r(:, j)));
                    directionCosine = min(1, max(-1, dot(directions(:, i), directions(:, j))));
                    actualMinAngleDeg = min(actualMinAngleDeg, acosd(directionCosine));
                end
            end
            isSeparated = minDistance <= 0 || actualMinDistance >= minDistance - 1e-9;
            isAngleValid = minAngleDeg <= 0 || actualMinAngleDeg >= minAngleDeg - 1e-9;
            % 新版优化页已取消独立的约束状态灯；如果以后重新加入同名
            % 组件，仍可直接恢复此处的可视化提示。
            if isprop(app, 'LayoutStatusLamp') && isprop(app, 'LayoutStatusLabel')
                if isInside && isSeparated && isAngleValid
                    app.LayoutStatusLamp.Color = [0.2 0.65 0.4];
                    app.LayoutStatusLabel.Text = sprintf('满足约束｜间距 %.3f m｜夹角 %.2f°', ...
                        actualMinDistance, actualMinAngleDeg);
                else
                    app.LayoutStatusLamp.Color = [0.9 0.25 0.2];
                    app.LayoutStatusLabel.Text = sprintf('存在约束风险｜间距 %.3f m｜夹角 %.2f°', ...
                        actualMinDistance, actualMinAngleDeg);
                end
            end
        end

        function simCfg = readSimulationConfig(app)
            simCfg = struct();
            simCfg.r0 = app.parseVector(app.EditField1.Value, 3);
            simCfg.rt = app.parseVector(app.EditField2.Value, 3);
            simCfg.euler0 = deg2rad(app.parseVector(app.EditField3.Value, 3));
            simCfg.eulert = deg2rad(app.parseVector(app.EditField4.Value, 3));
            simCfg.Kp_pos = app.FmaxField_7.Value;
            simCfg.Kd_pos = app.FmaxField_8.Value;
            simCfg.Kp_att = app.FmaxField_9.Value;
            simCfg.Ki_att = app.FmaxField_10.Value;
            simCfg.Kd_att = app.FmaxField_11.Value;
            simCfg.T_sim = app.SimulationTimeField.Value;
            simCfg.faulty_time = app.MinPulseField_5.Value;
            % 零故障时不触发故障事件；故障时刻可以保留在输入框中，
            % 但不能因为它落在仿真区间之外而阻止标况仿真。
            noFaultCase = app.requestedFaultCount() == 0;
            if ~noFaultCase && (~isfinite(simCfg.faulty_time) || simCfg.faulty_time < 0 || ...
                    simCfg.faulty_time > simCfg.T_sim)
                error('故障时刻应为0至仿真时间 %.3g s之间的数值。', simCfg.T_sim);
            end
            if noFaultCase
                simCfg.faulty_time = Inf;
            end
            simCfg.dt = max(eps, app.MinPulseField_4.Value);
            simCfg.true_faults = app.parseIndexList(app.FmaxField_6.Value);
        end

        function tableData = buildParameterFileTable(app)
            app.readParamsFromUI();
            params = app.CurrentParams;
            simCfg = app.readSimulationConfig();
            layoutData = app.layoutTableNumericData();

            rows = {};
            rows = app.addParameterRow(rows, '飞行器参数', 'm', params.m, 'kg', '飞行器质量');
            rows = app.addParameterRow(rows, '飞行器参数', 'Jxx', params.J(1, 1), 'kg*m^2', 'X轴转动惯量');
            rows = app.addParameterRow(rows, '飞行器参数', 'Jyy', params.J(2, 2), 'kg*m^2', 'Y轴转动惯量');
            rows = app.addParameterRow(rows, '飞行器参数', 'Jzz', params.J(3, 3), 'kg*m^2', 'Z轴转动惯量');
            rows = app.addParameterRow(rows, '推力器参数', 'Num', params.Num, '台', '推力器数量（应为4n）');
            rows = app.addParameterRow(rows, '推力器参数', 'F_max', params.F_max, 'N', '单台推力器最大推力');
            rows = app.addParameterRow(rows, '推力器参数', 't_min', params.t_min, 's', '最小脉宽');
            rows = app.addParameterRow(rows, '控制器参数', 'T', params.T, 's', '控制周期');
            rows = app.addParameterRow(rows, '控制器参数', 'Kp_pos', simCfg.Kp_pos, '', '位置比例系数');
            rows = app.addParameterRow(rows, '控制器参数', 'Kd_pos', simCfg.Kd_pos, '', '位置微分系数');
            rows = app.addParameterRow(rows, '控制器参数', 'Kp_att', simCfg.Kp_att, '', '姿态比例系数');
            rows = app.addParameterRow(rows, '控制器参数', 'Ki_att', simCfg.Ki_att, '', '姿态积分系数');
            rows = app.addParameterRow(rows, '控制器参数', 'Kd_att', simCfg.Kd_att, '', '姿态微分系数');
            rows = app.addParameterRow(rows, '任务场景', 'r0', mat2str(simCfg.r0(:)', 8), 'm', '初始位置');
            rows = app.addParameterRow(rows, '任务场景', 'rt', mat2str(simCfg.rt(:)', 8), 'm', '目标位置');
            rows = app.addParameterRow(rows, '任务场景', 'euler0_deg', mat2str(rad2deg(simCfg.euler0(:)'), 8), 'deg', '初始姿态角');
            rows = app.addParameterRow(rows, '任务场景', 'eulert_deg', mat2str(rad2deg(simCfg.eulert(:)'), 8), 'deg', '目标姿态角');
            rows = app.addParameterRow(rows, '任务场景', 'T_sim', simCfg.T_sim, 's', '仿真时长');
            rows = app.addParameterRow(rows, '任务场景', 'dt', simCfg.dt, 's', '仿真步长');
            rows = app.addParameterRow(rows, '故障设置', 'fault_condition', ...
                app.faultCountControlValue(), '', '故障数量');
            rows = app.addParameterRow(rows, '故障设置', 'fault_indices', ...
                app.FmaxField_6.Value, '', '故障推力器编号');
            rows = app.addParameterRow(rows, '故障设置', 'fault_time', ...
                simCfg.faulty_time, 's', '故障发生时刻');
            rows = app.addParameterRow(rows, '调用策略', 'reuse_strategy', ...
                app.AllocationModeDropDown_2.Value, '', '推力器复用策略');
            rows = app.addParameterRow(rows, '轨道参数', 'n', params.n, 'rad/s', '轨道角速度');
            rows = app.addParameterRow(rows, '轨道参数', 'R_earth', params.R_earth, 'm', '地球半径');
            rows = app.addParameterRow(rows, '轨道参数', 'h_orbit', params.h_orbit, 'm', '轨道高度');
            rows = app.addParameterRow(rows, '轨道参数', 'mu', params.mu, 'm^3/s^2', '地球引力常数');
            rows = app.addParameterRow(rows, '安装与优化参数', 'install_face_X', app.FmaxField_4.Value, '个', 'X方向安装面数量');
            rows = app.addParameterRow(rows, '安装与优化参数', 'install_boundary_L', app.NumericEditField1_2.Value, 'm', '安装边界L（y_max）');
            rows = app.addParameterRow(rows, '安装与优化参数', 'install_boundary_W', app.NumericEditField2_4.Value, 'm', '安装边界W（z_max）');
            rows = app.addParameterRow(rows, '安装与优化参数', 'min_install_distance', app.NumericEditField2_5.Value, 'm', '最小安装间距');
            rows = app.addParameterRow(rows, '安装与优化参数', 'min_install_angle_deg', app.NumericEditField2_6.Value, 'deg', '最小安装夹角');

            for index = 1:size(layoutData, 1)
                key = sprintf('thruster_%02d', index);
                rows = app.addParameterRow(rows, '推力器布局', key, ...
                    mat2str(layoutData(index, 2:7), 8), ...
                    '[x y z dx dy dz]', sprintf('第%d台推力器的位置与方向', index));
            end

            tableData = cell2table(rows, 'VariableNames', ...
                {'Module', 'Name', 'Value', 'Unit', 'Description'});
        end

        function rows = addParameterRow(app, rows, moduleName, name, value, unit, description)
            if isnumeric(value)
                if isscalar(value)
                    valueText = sprintf('%.12g', value);
                else
                    valueText = mat2str(value, 8);
                end
            else
                valueText = char(string(value));
            end
            rows(end + 1, :) = {char(moduleName), char(name), valueText, ...
                char(unit), char(description)};
        end

        function [names, values, units, modules, descriptions] = ...
                parameterFileColumns(app, tableData)
            if isempty(tableData) || ~istable(tableData) || width(tableData) < 2
                error('参数文件格式错误：至少需要参数名称和参数值两列。');
            end

            headers = lower(string(tableData.Properties.VariableNames));
            nameIndex = find(ismember(headers, lower(["name", "parameter", ...
                "参数名称", "参数名"])), 1, 'first');
            valueIndex = find(ismember(headers, lower(["value", ...
                "参数值", "数值"])), 1, 'first');
            moduleIndex = find(ismember(headers, lower(["module", ...
                "section", "模块", "参数模块"])), 1, 'first');
            unitIndex = find(ismember(headers, lower(["unit", "单位"])), 1, 'first');
            descriptionIndex = find(ismember(headers, lower(["description", ...
                "说明", "含义"])), 1, 'first');

            % 新格式为“模块-名称-数值-单位-说明”；旧格式仍按
            % “Name-Value-Unit”读取，保证已有参数文件可以继续使用。
            if isempty(nameIndex) || isempty(valueIndex)
                if width(tableData) >= 5
                    moduleIndex = 1;
                    nameIndex = 2;
                    valueIndex = 3;
                    unitIndex = 4;
                    descriptionIndex = 5;
                else
                    nameIndex = 1;
                    valueIndex = 2;
                    if width(tableData) >= 3
                        unitIndex = 3;
                    end
                end
            end

            names = string(tableData{:, nameIndex});
            values = string(tableData{:, valueIndex});
            units = strings(height(tableData), 1);
            modules = repmat("参数", height(tableData), 1);
            descriptions = strings(height(tableData), 1);
            if ~isempty(unitIndex)
                units = string(tableData{:, unitIndex});
            end
            if ~isempty(moduleIndex)
                modules = string(tableData{:, moduleIndex});
            end
            if ~isempty(descriptionIndex)
                descriptions = string(tableData{:, descriptionIndex});
            end
            names(ismissing(names)) = "";
            values(ismissing(values)) = "";
            units(ismissing(units)) = "";
            modules(ismissing(modules) | strlength(strtrim(modules)) == 0) = "参数";
            descriptions(ismissing(descriptions)) = "";
        end

        function applyParameterFileTable(app, tableData)
            if isempty(tableData)
                return;
            end
            if ~istable(tableData)
                error('参数文件格式错误：应为包含 Name/Value 列的表格。');
            end
            [names, values] = app.parameterFileColumns(tableData);
            params = app.CurrentParams;
            if isempty(params)
                params = Get_params();
            end

            params.m = app.getParameterValue(names, values, 'm', params.m);
            params.J(1, 1) = app.getParameterValue(names, values, 'Jxx', params.J(1, 1));
            params.J(2, 2) = app.getParameterValue(names, values, 'Jyy', params.J(2, 2));
            params.J(3, 3) = app.getParameterValue(names, values, 'Jzz', params.J(3, 3));
            params.Num = round(app.getParameterValue(names, values, 'Num', params.Num));
            params.F_max = app.getParameterValue(names, values, 'F_max', params.F_max);
            params.T = app.getParameterValue(names, values, 'T', params.T);
            params.t_min = app.getParameterValue(names, values, 't_min', params.t_min);
            params.n = app.getParameterValue(names, values, 'n', params.n);
            params.R_earth = app.getParameterValue(names, values, 'R_earth', ...
                app.paramOrDefault(params, 'R_earth', 6371000));
            params.h_orbit = app.getParameterValue(names, values, 'h_orbit', ...
                app.paramOrDefault(params, 'h_orbit', 400000));
            params.mu = app.getParameterValue(names, values, 'mu', ...
                app.paramOrDefault(params, 'mu', 3.986e14));
            params.Kp_pos = app.getParameterValue(names, values, 'Kp_pos', ...
                app.paramOrDefault(params, 'Kp_pos', 10));
            params.Kd_pos = app.getParameterValue(names, values, 'Kd_pos', ...
                app.paramOrDefault(params, 'Kd_pos', 300));
            params.Kp_att = app.getParameterValue(names, values, 'Kp_att', ...
                app.paramOrDefault(params, 'Kp_att', 400));
            params.Ki_att = app.getParameterValue(names, values, 'Ki_att', ...
                app.paramOrDefault(params, 'Ki_att', 20));
            params.Kd_att = app.getParameterValue(names, values, 'Kd_att', ...
                app.paramOrDefault(params, 'Kd_att', 3200));
            app.CurrentParams = params;
            app.populateParameterControls();

            app.EditField1.Value = app.getParameterText(names, values, 'r0', app.EditField1.Value);
            app.EditField2.Value = app.getParameterText(names, values, 'rt', app.EditField2.Value);
            app.EditField3.Value = app.getParameterText(names, values, 'euler0_deg', app.EditField3.Value);
            app.EditField4.Value = app.getParameterText(names, values, 'eulert_deg', app.EditField4.Value);
            app.SimulationTimeField.Value = app.getParameterValue(names, values, 'T_sim', app.SimulationTimeField.Value);
            app.MinPulseField_4.Value = app.getParameterValue(names, values, 'dt', app.MinPulseField_4.Value);
            app.FmaxField_6.Value = app.getParameterValue(names, values, ...
                'fault_indices', app.FmaxField_6.Value);
            faultCondition = app.getParameterText(names, values, ...
                'fault_condition', app.faultCountControlValue());
            switch char(string(faultCondition))
                case '单台故障'
                    faultCondition = '1';
                case '两台故障'
                    faultCondition = '2';
                case '自定义故障组合'
                    faultCondition = string(max(0, numel( ...
                        app.parseIndexList(app.FmaxField_6.Value))));
            end
            app.setFaultCountControlValue(faultCondition);
            app.MinPulseField_5.Value = app.getParameterValue(names, values, ...
                'fault_time', app.MinPulseField_5.Value);
            reuseStrategy = app.getParameterText(names, values, ...
                'reuse_strategy', app.AllocationModeDropDown_2.Value);
            if any(strcmp(app.AllocationModeDropDown_2.Items, reuseStrategy))
                app.AllocationModeDropDown_2.Value = reuseStrategy;
            end
            app.FmaxField_4.Value = app.getParameterValue(names, values, 'install_face_X', app.FmaxField_4.Value);
            app.NumericEditField1_2.Value = app.getParameterValue(names, values, 'install_boundary_L', ...
                app.getParameterValue(names, values, 'install_boundary', app.NumericEditField1_2.Value));
            app.NumericEditField2_4.Value = app.getParameterValue(names, values, 'install_boundary_W', app.NumericEditField2_4.Value);
            app.NumericEditField2_5.Value = app.getParameterValue(names, values, 'min_install_distance', app.NumericEditField2_5.Value);
            app.NumericEditField2_6.Value = app.getParameterValue(names, values, 'min_install_angle_deg', app.NumericEditField2_6.Value);

            thrusterRows = startsWith(names, "thruster_");
            if any(thrusterRows)
                thrusterNames = names(thrusterRows);
                thrusterValues = values(thrusterRows);
                [~, order] = sort(thrusterNames);
                layoutData = zeros(numel(order), 7);
                for localIndex = 1:numel(order)
                    vec = app.parseVector(thrusterValues(order(localIndex)), 6);
                    layoutData(localIndex, :) = [localIndex, vec(:)'];
                end
                app.setLayoutTableData(layoutData);
                [app.CurrentB, app.CurrentR] = app.readLayoutFromTable();
                app.CurrentParams.Num = size(app.CurrentB, 2);
                app.FmaxField_2.Value = app.CurrentParams.Num;
                app.replaceOriginalLayout(app.CurrentB, app.CurrentR);
                app.drawLayout();
            end
        end

        function value = getParameterValue(app, names, values, key, defaultValue)
            textValue = app.getParameterText(names, values, key, '');
            if isempty(textValue)
                value = defaultValue;
            else
                value = app.toScalar(textValue, defaultValue);
            end
        end

        function textValue = getParameterText(app, names, values, key, defaultValue)
            index = find(names == string(key), 1, 'first');
            if isempty(index) || ismissing(values(index))
                textValue = defaultValue;
            else
                textValue = char(values(index));
            end
        end

        function LoadParameterFile(app, filePath)
            [~, ~, ext] = fileparts(filePath);
            switch lower(ext)
                case '.txt'
                    % 明确指定制表符，避免含空单位或向量参数时被自动
                    % 识别成空格分隔，导致一行被错误拆成十几列。
                    importOptions = detectImportOptions(filePath, ...
                        'FileType', 'text', 'Delimiter', '\t');
                    importOptions = setvartype(importOptions, ...
                        importOptions.VariableNames, 'string');
                    tableData = readtable(filePath, importOptions);
                case '.csv'
                    importOptions = detectImportOptions(filePath, ...
                        'FileType', 'text', 'Delimiter', ',');
                    importOptions = setvartype(importOptions, ...
                        importOptions.VariableNames, 'string');
                    tableData = readtable(filePath, importOptions);
                case {'.xlsx', '.xls'}
                    importOptions = detectImportOptions(filePath);
                    importOptions = setvartype(importOptions, ...
                        importOptions.VariableNames, 'string');
                    tableData = readtable(filePath, importOptions);
                otherwise
                    error('不支持的参数文件类型：%s', ext);
            end
            if width(tableData) < 2
                error('参数文件至少需要两列：Name 和 Value。');
            end
            app.applyParameterFileTable(tableData);
            app.LastParameterFile = filePath;
            app.updateReport("已加载参数文件：" + string(filePath));
            app.setStatus(['已加载参数文件｜', filePath]);
        end

        function lines = parameterTableToText(app, tableData, heading)
            [names, values, units, modules, descriptions] = ...
                app.parameterFileColumns(tableData);
            lines = [string(heading); "------------------------------"];
            previousModule = "";
            for row = 1:height(tableData)
                moduleName = strtrim(modules(row));
                if moduleName ~= previousModule
                    lines(end + 1, 1) = ""; %#ok<AGROW>
                    lines(end + 1, 1) = "【" + moduleName + "】"; %#ok<AGROW>
                    previousModule = moduleName;
                end
                line = names(row) + " = " + values(row);
                if strlength(strtrim(units(row))) > 0
                    line = line + "  [" + units(row) + "]";
                end
                if strlength(strtrim(descriptions(row))) > 0
                    line = line + "  - " + descriptions(row);
                end
                if strlength(strtrim(names(row))) > 0
                    lines(end + 1, 1) = line; %#ok<AGROW>
                else
                    lines(end + 1, 1) = ""; %#ok<AGROW>
                end
            end
        end

        function SaveParameterFile(app, filePath)
            tableData = app.buildParameterFileTable();
            [~, ~, ext] = fileparts(filePath);
            switch lower(ext)
                case {'.xlsx', '.xls'}
                    writetable(tableData, filePath);
                case '.csv'
                    writetable(tableData, filePath);
                case '.txt'
                    writetable(tableData, filePath, 'Delimiter', '\t', 'FileType', 'text');
                otherwise
                    error('不支持的参数文件类型：%s', ext);
            end
            app.LastParameterFile = filePath;
            app.updateReport("已保存参数文件：" + string(filePath));
            app.setStatus(['已保存参数文件｜', filePath]);
        end

        function SaveButtonPushed(app, event)
            app.SaveParameterButtonPushed(event);
        end

        function SaveParameterButtonPushed(app, event)
            [file, path] = uiputfile({'*.xlsx', 'Excel 工作簿 (*.xlsx)'; '*.txt', '文本文件 (*.txt)'; '*.csv', 'CSV 文件 (*.csv)'}, ...
                '保存参数文件', fullfile(app.ProjectRoot, 'RCDesigner_parameters.xlsx'));
            if isequal(file, 0)
                return;
            end
            try
                app.SaveParameterFile(fullfile(path, file));
            catch ME
                app.showError('参数保存失败', ME);
            end
        end

        function ExportDataButtonPushed(app, event)
            [file, path] = uiputfile({'*.mat', 'MAT 数据文件 (*.mat)'; '*.csv', 'CSV 摘要文件 (*.csv)'}, ...
                '导出设计数据', fullfile(app.ProjectRoot, 'RCDesigner_result.mat'));
            if isequal(file, 0)
                return;
            end
            try
                filePath = fullfile(path, file);
                [~, ~, ext] = fileparts(filePath);
                params = app.readParamsFromUI();
                B = app.CurrentB;
                r = app.CurrentR;
                optimResult = app.OptimResult;
                reconfigJudgmentResult = app.ReconfigJudgmentResult;
                evaluationResult = app.EvaluationResult;
                simulationLog = app.SimulationLog;
                if strcmpi(ext, '.csv')
                    if isempty(evaluationResult)
                        app.MetricsEvaluateButtonPushed(event);
                        evaluationResult = app.EvaluationResult;
                    end
                    summaryTable = cell2table(app.summaryToCell(evaluationResult), ...
                        'VariableNames', {'CaseName', 'Jc', 'Jo', 'Jt', 'Jf', 'Score'});
                    writetable(summaryTable, filePath);
                else
                    save(filePath, 'params', 'B', 'r', 'optimResult', ...
                        'reconfigJudgmentResult', 'evaluationResult', 'simulationLog');
                end
                app.setStatus(['结果数据已导出｜', filePath]);
            catch ME
                app.showError('数据导出失败', ME);
            end
        end

        function GenerateReportButtonPushed(app, event)
            [file, path] = uiputfile({'*.txt', '文本报告 (*.txt)'}, ...
                '生成报告文本', fullfile(app.ProjectRoot, 'RCDesigner_report.txt'));
            if isequal(file, 0)
                return;
            end
            try
                filePath = fullfile(path, file);
                lines = app.ReportLines;
                if isempty(lines)
                    app.updateReport("软件当前状态。");
                    lines = app.ReportLines;
                end
                fid = fopen(filePath, 'w', 'n', 'UTF-8');
                if fid < 0
                    error('无法写入报告文件：%s', filePath);
                end
                cleanup = onCleanup(@()fclose(fid));
                for index = 1:numel(lines)
                    fprintf(fid, '%s\n', lines{index});
                end
                clear cleanup;
                app.setStatus(['报告文本已生成｜', filePath]);
            catch ME
                app.showError('报告生成失败', ME);
            end
        end

        function faultSets = getFaultCases(app, params, faultValue, faultCount)
            if nargin < 3
                faultValue = app.FmaxField_6.Value;
            end
            if nargin < 4 || isempty(faultCount)
                faultCount = app.requestedFaultCount(params.Num);
            end
            % 零台故障优先于故障编号，旧编号不会影响标况计算。
            if faultCount == 0
                faultSets = {[]};
                return;
            end
            rawFaults = app.parseVector(faultValue, []);
            % 编号 0 表示标况和每一台单故障，与故障台数无关。
            if isscalar(rawFaults) && rawFaults == 0
                faultSets = [{[]}, num2cell(1:params.Num)];
                return;
            end
            typedFaults = app.parseIndexList(faultValue);
            if any(typedFaults > params.Num)
                error('故障编号不能大于当前布局的推力器总数 %d。', params.Num);
            end

            if numel(typedFaults) == faultCount
                faultSets = {typedFaults};
                return;
            end

            if faultCount > params.Num
                error('故障数量不能大于推力器总数。');
            end
            combs = nchoosek(1:params.Num, faultCount);
            faultSets = mat2cell(combs, ones(size(combs, 1), 1), faultCount);
        end

% 原实现保留（口径或界面已调整；冲突部分停用）：
%                 'PolicyReconfigurable', false, 'Conclusion', '');
% 原实现保留（口径或界面已调整；冲突部分停用）：
%                     isReconfig = forceMargin > 1e-8 && torqueMargin > 1e-8 && ...
%                         jointMargin > 1e-8 && policyReconfigurable;
% 原实现保留（口径或界面已调整；冲突部分停用）：
%                     if isReconfig
%                         item.Conclusion = '可重构';
%                     elseif forceMargin > 1e-8 && torqueMargin > 1e-8 && jointMargin > 1e-8
%                         item.Conclusion = '调用策略不可重构';
% 原实现保留（口径或界面已调整；冲突部分停用）：
%                         item.Conclusion = '不可重构';
        function results = evaluateReconfigJudgment(app, layoutName, B, params, faultSets)
            layoutParams = params;
            layoutParams.Num = size(B, 2);
            layoutParams.evaluation_model = 'continuous_judgment';
            template = struct('LayoutName', '', 'FaultSet', '', ...
                'JcForce', NaN, 'JcTorque', NaN, 'Jc6D', NaN, ...
                'PolicyReconfigurable', false, 'HardwareForce',0,'HardwareTorque',0, ...
                'AxisCount',0,'StrictCount',0,'ReferencePass',0,'Conclusion', '');
            results = repmat(template, 0, 1);
            faultCounts = unique(cellfun(@numel, faultSets));

            for countIndex = 1:numel(faultCounts)
                faultCount = faultCounts(countIndex);
                selected = find(cellfun(@numel, faultSets) == faultCount);
                [Z, Jc1, ~, Jc] = Reconfig_eval(layoutParams, B, faultCount);
                for localIndex = 1:numel(selected)
                    faultSet = sort(faultSets{selected(localIndex)});
                    row = app.findFaultSetRow(Z.FaultSets, faultSet);
                    if isempty(row)
                        continue;
                    end
                    forceMargin = Jc1(row, 1);
                    torqueMargin = Jc1(row, 2);
                    jointMargin = Jc(row);
                    policyReconfigurable = isfield(Z, 'Invocation') && ...
                        Z.Invocation(row).policy_reconfigurable;
                    isReconfig = policyReconfigurable;
                    item = template;
                    item.LayoutName = char(layoutName);
                    item.FaultSet = app.faultSetText(faultSet);
                    item.JcForce = forceMargin;
                    item.JcTorque = torqueMargin;
                    item.Jc6D = jointMargin;
                    item.PolicyReconfigurable = policyReconfigurable;
                    item.HardwareForce = Z.Hardware(row).force_capacity;
                    item.HardwareTorque = Z.Hardware(row).torque_capacity;
                    item.AxisCount = nnz(Z.Invocation(row).axis_callable);
%                     item.StrictCount = nnz(Z.Invocation(row).strict_equal_axis);
%                     item.ReferencePass = 100*Z.Invocation(row).reference_pass_rate;
%                     if isReconfig && Z.Invocation(row).reference_commands_callable
%                         item.Conclusion = '12向及参考指令通过';
%                     elseif isReconfig
%                         item.Conclusion = '12向可用；参考指令未全通过';
%                     elseif Z.Hardware(row).reconfigurable
%                         item.Conclusion = '硬件可用；策略12向未证实';
%                     else
%                         item.Conclusion = '硬件纯轴能力不足';
%                     end
                    if isReconfig
                        item.Conclusion = '可重构：原点在实际六维包络内部';
                    elseif Z.Hardware(row).reconfigurable
                        item.Conclusion = '不可重构：原点不在实际六维包络内部';
                    else
                        item.Conclusion = '不可重构：理论六维包络也无原点内邻域';
                    end
                    results(end + 1, 1) = item; %#ok<AGROW>
                end
            end
        end

        function row = findFaultSetRow(app, allFaultSets, faultSet)
            row = [];
            for index = 1:numel(allFaultSets)
                candidate = sort(allFaultSets{index});
                if isequal(candidate(:)', faultSet(:)')
                    row = index;
                    return;
                end
            end
        end

        function text = faultSetText(app, faultSet)
            if isempty(faultSet)
                text = '标况';
            else
                text = mat2str(faultSet);
            end
        end

% 原实现保留（口径或界面已调整；冲突部分停用）：
%             % The current judgment table has five visible columns. Keep
%             % text values as character vectors because R2023b UITable does
%             % not accept string scalars inside a mixed cell array.
%             data = cell(numel(results), 5);
% 原实现保留（口径或界面已调整；冲突部分停用）：
%                 data(index, :) = {results(index).FaultSet, ...
%                     results(index).JcForce, results(index).JcTorque, ...
%                     results(index).Jc6D, results(index).Conclusion};
        function data = judgmentToCell(app, results)
% 原十列表格保留：data = cell(numel(results), 10);
            data = cell(numel(results), 8);
            for index = 1:numel(results)
                r = results(index);
% 原列保留：r.HardwareForce,r.HardwareTorque,r.AxisCount,r.StrictCount,r.ReferencePass,r.Conclusion
                data(index,:) = {r.FaultSet,r.JcForce,r.JcTorque,r.Jc6D, ...
                    r.HardwareForce,r.HardwareTorque,r.AxisCount,r.Conclusion};
            end
        end

% 原实现保留（口径或界面已调整；冲突部分停用）：
%                 rowNumbers = find(contains(conclusions, "不可重构"));
% 原实现保留（口径或界面已调整；冲突部分停用）：
%         function result = evaluateLayoutSummary(app, name, B, params, faultLevel)
        function highlightUnreconfigurableRows(app, results)
            % 每次判断后重新生成样式，避免切换故障条件时旧行仍被标红。
            try
                removeStyle(app.EvaluationTable);
                if isempty(results)
                    return;
                end
                conclusions = string({results.Conclusion});
% 原判据：rowNumbers = find(~strcmp(conclusions, "12向及参考指令通过"));
                rowNumbers = find(~[results.PolicyReconfigurable]);
                if isempty(rowNumbers)
                    return;
                end
                warningStyle = uistyle( ...
                    'BackgroundColor', [1.00 0.84 0.84], ...
                    'FontColor', [0.72 0.05 0.05], ...
                    'FontWeight', 'bold');
                addStyle(app.EvaluationTable, warningStyle, 'row', rowNumbers);
            catch
                % 个别旧版MATLAB不支持表格样式时，不影响判断数据本身显示。
            end
        end

% 原实现保留（口径或界面已调整；冲突部分停用）：
%             [~, Jc1, ~, Jc, Jo, Jt, Jf] = Reconfig_eval(layoutParams, B, faultLevel);
% 原实现保留（口径或界面已调整；冲突部分停用）：
%             result.Score = result.Jc + 0.1 * result.Jo + result.Jt - 0.01 * result.Jf;
        function result = evaluateLayoutSummary(app, name, B, params, faultLevel, faultCases)
            layoutParams = params;
            layoutParams.Num = size(B, 2);
            if faultLevel > layoutParams.Num
                error('故障数量不能大于布局“%s”的推力器总数。', char(name));
            end
            [Z, Jc1, ~, Jc, Jo, Jt, Jf] = Reconfig_eval(layoutParams, B, faultLevel);
            rows=1:numel(Z.FaultSets);
            if nargin>=6
                rows=[];
                for i=1:numel(faultCases)
                    rows(end+1)=app.findFaultSetRow(Z.FaultSets,faultCases{i}); %#ok<AGROW>
                end
            end
            Jc=Jc(rows); Jc1=Jc1(rows,:); Jo=Jo(rows); Jt=Jt(rows); Jf=Jf(rows);
            Z.SelectedRows=rows;
            result = struct();
            result.Name = char(name);
            result.Jc = min(Jc);
            result.JcForce = min(Jc1(:, 1));
            result.JcTorque = min(Jc1(:, 2));
            result.Jo = min(Jo);
            result.Jt = mean(Jt);
            result.Jf = mean(Jf);
            result.Score = NaN; % 不同量纲相加不是可重构判据。
            result.PolicyEvaluation = Z;
        end

        function data = summaryToCell(app, summaries)
            data = cell(numel(summaries), 6);
            for index = 1:numel(summaries)
                data(index, :) = {summaries(index).Name, summaries(index).Jc, ...
                    summaries(index).Jo, summaries(index).Jt, summaries(index).Jf, ...
                    summaries(index).Score};
            end
        end

        function updateAllocationStrategyTable(app, strategy)
            if isempty(strategy)
                axesText = repmat({'+X'; '-X'; '+Y'; '-Y'; '+Z'; '-Z'}, 2, 1);
                types = [repmat({'轨道控制'}, 6, 1); repmat({'姿态控制'}, 6, 1)];
                strategy.ColumnNames = {'控制类型', '标况下调用（原布局）', ...
                    '故障下调用（原布局）', '调用状态（原布局）'};
                strategy.Data = [cellstr(strcat(string(types), string(axesText))), ...
                    repmat({'等待生成'}, 12, 3)];
                strategy.FaultRows = false(12, 1);
            end
            if ~isfield(strategy, 'FaultRows')
                strategy.FaultRows = false(size(strategy.Data, 1), 1);
            end
            app.AllocationStrategyTable.ColumnName = strategy.ColumnNames;
            app.AllocationStrategyTable.ColumnWidth = ...
                [{130}, repmat({'auto'}, 1, numel(strategy.ColumnNames) - 1)];
            app.AllocationStrategyTable.Data = strategy.Data;
            app.AllocationFaultRows = strategy.FaultRows;
            app.applyAllocationControlColors();
        end

        function strategy = buildAllocationStrategyComparison(app, params, layoutSet, faultCases)
            if ~iscell(faultCases), faultCases = {faultCases}; end
            strategy = [];
            for caseIndex = 1:numel(faultCases)
                caseResult = RCPlotter.allocationComparisonData(params, layoutSet, faultCases{caseIndex});
                if numel(faultCases) > 1
                    if isempty(faultCases{caseIndex})
                        condition = '标况';
                    else
                        condition = ['故障', app.faultSetText(faultCases{caseIndex})];
                    end
                    % Keep the requested 4/7-column contract in all-fault mode.
                    % A full-width section row identifies each 12-row case block.
                    heading = repmat({''}, 1, numel(caseResult.ColumnNames));
                    heading{1} = condition;
                    caseResult.Data = [heading; caseResult.Data];
                    caseResult.FaultRows = [false; caseResult.FaultRows(:)];
                end
                if isempty(strategy)
                    strategy = caseResult;
                else
                    strategy.Data = [strategy.Data; caseResult.Data]; %#ok<AGROW>
                    strategy.FaultRows = [strategy.FaultRows; caseResult.FaultRows]; %#ok<AGROW>
                end
            end
        end

        function applyAllocationControlColors(app)
            removeStyle(app.AllocationStrategyTable);
            data = app.AllocationStrategyTable.Data;
            if isempty(data), return; end
            controlTypes = string(data(:, 1));
            attitudeRows = find(startsWith(controlTypes, "姿态控制"));
            if ~isempty(attitudeRows)
                addStyle(app.AllocationStrategyTable, ...
                    uistyle('BackgroundColor', [0.90 0.91 0.92]), 'row', attitudeRows);
            end
            faultRows = find(app.AllocationFaultRows);
            if ~isempty(faultRows)
                addStyle(app.AllocationStrategyTable, ...
                    uistyle('BackgroundColor', [1 0.83 0.83], ...
                    'FontColor', [0.78 0.04 0.04]), 'row', faultRows);
            end
        end

        function resetSimulationComparisonViews(app)
            groups = {app.SimulationPositionCaseTabGroup, app.SimulationCommandCaseTabGroup, ...
                app.SimulationTrajectoryCaseTabGroup, app.SimulationPulseCaseTabGroup, ...
                app.SimulationScheduleCaseTabGroup};
            for index = 1:numel(groups)
                group = groups{index};
                group.SelectionChangedFcn = [];
                tabs = group.Children;
                for tabIndex = 1:numel(tabs)
                    if strcmp(tabs(tabIndex).Tag, 'RCComparisonRuntime')
                        delete(tabs(tabIndex));
                    else
                        tabs(tabIndex).Title = '标况（原布局）';
                    end
                end
                axesList = findall(group, 'Type', 'axes');
                for axisIndex = 1:numel(axesList), cla(axesList(axisIndex), 'reset'); end
            end
            app.SimulationCaseViews = struct('Tab', {}, 'Grid', {}, 'Axes', {}, 'Rendered', {});
        end

        function plotSimulationResult(app, logs, labels, varargin)
            % 兼容旧版调用签名 (stateLogs,stateLabels,faultLogs,faultLabels)。
            % 新版直接传入按布局/工况展开的 logs 与 labels。
            if numel(varargin) >= 2 && iscell(varargin{1}) && iscell(varargin{2})
                logs = varargin{1};
                labels = varargin{2};
            end
            groups = {app.SimulationPositionCaseTabGroup, ...
                app.SimulationCommandCaseTabGroup, app.SimulationTrajectoryCaseTabGroup, ...
                app.SimulationPulseCaseTabGroup, app.SimulationScheduleCaseTabGroup};
            baseTabs = {app.SimulationNominalPositionTab, app.SimulationNominalCommandTab, ...
                app.SimulationNominalTrajectoryTab, app.SimulationNominalPulseTab, ...
                app.SimulationNominalScheduleTab};
            baseGrids = {app.SimulationPositionGrid, app.SimulationNominalCommandGrid, ...
                app.SimulationNominalTrajectoryGrid, app.SimulationPulseGrid, ...
                app.SimulationNominalScheduleGrid};
            baseAxes = {[app.SimulationAxes, app.SimulationAttitudeAxes, ...
                app.SimulationPositionErrorAxes, app.SimulationAttitudeErrorAxes], ...
                [app.SimulationForceXAxes, app.SimulationForceYAxes, app.SimulationForceZAxes, ...
                app.SimulationTorqueXAxes, app.SimulationTorqueYAxes, app.SimulationTorqueZAxes], ...
                [app.SimulationNominalTrajectory3DAxes, app.SimulationNominalTrajectoryXYAxes, ...
                app.SimulationNominalTrajectoryXZAxes, app.SimulationNominalTrajectoryYZAxes], ...
                app.SimulationPulsePlaceholderAxes, app.SimulationScheduleAxes};
            emptyView = struct('Tab', [], 'Grid', [], 'Axes', [], 'Rendered', false);
            app.SimulationCaseViews = repmat(emptyView, 5, numel(logs));
            for viewIndex = 1:5
                group = groups{viewIndex};
                group.SelectionChangedFcn = [];
                tabs = group.Children;
                for tabIndex = 1:numel(tabs)
                    if ~isequal(tabs(tabIndex), baseTabs{viewIndex}), delete(tabs(tabIndex)); end
                end
                for caseIndex = 1:numel(logs)
                    if caseIndex == 1
                        caseTab = baseTabs{viewIndex};
                        grid = baseGrids{viewIndex};
                        axesList = baseAxes{viewIndex};
                        for ax = axesList
                            cla(ax, 'reset');
                        end
                    else
                        caseTab = uitab(group, 'Tag', 'RCComparisonRuntime');
                        grid = [];
                        axesList = [];
                    end
                    caseTab.Title = labels{caseIndex};
                    caseTab.UserData = caseIndex;
                    app.SimulationCaseViews(viewIndex, caseIndex) = ...
                        struct('Tab', caseTab, 'Grid', grid, 'Axes', axesList, 'Rendered', false);
                end
                group.SelectedTab = baseTabs{viewIndex};
                group.SelectionChangedFcn = @(~, event) ...
                    app.renderSimulationCase(viewIndex, event.NewValue.UserData);
            end
            % UI tab order is the reverse of the Children array.
            app.SimulationViewTabGroup.Children = [ ...
                app.SimulationActualVerificationTab; app.SimulationScheduleTab; ...
                app.SimulationPulseTab; app.SimulationTrajectoryTab; ...
                app.SimulationCommandTab; app.SimulationPositionTab];
            app.SimulationScheduleStartTimeField.Value = ...
                RCPlotter.selectFullPeriodScheduleStart(logs{1});
            for viewIndex = 1:5
                app.renderSimulationCase(viewIndex, 1);
            end
        end

        function renderSimulationCase(app, viewIndex, caseIndex)
            if isempty(app.SimulationCaseViews) || isempty(app.SimulationLog), return; end
            caseView = app.SimulationCaseViews(viewIndex, caseIndex);
            if caseView.Rendered, return; end
            logData = app.SimulationLog.ComparisonLogs{caseIndex};
            counts = [4, 6, 4, size(logData.Pulse_Widths, 1), 1];
            columns = [2, 3, 2, min(3, counts(4)), 1];
            if isempty(caseView.Grid) || ~isvalid(caseView.Grid)
                caseView.Grid = uigridlayout(caseView.Tab);
                caseView.Grid.Padding = [8 8 8 8];
            end
            grid = caseView.Grid;
            grid.ColumnWidth = repmat({'1x'}, 1, columns(viewIndex));
            rowCount = ceil(counts(viewIndex) / columns(viewIndex));
            grid.RowHeight = repmat({'1x'}, 1, rowCount);
            if viewIndex == 4
                grid.Scrollable = 'on';
                grid.RowHeight = repmat({210}, 1, rowCount);
                if caseIndex == 1
                    caseView.Axes = app.prepareThrusterPulseAxes(counts(4));
                end
            end
            if isempty(caseView.Axes)
                axesList = gobjects(1, counts(viewIndex));
                for axisIndex = 1:numel(axesList)
                    axesList(axisIndex) = uiaxes(grid);
                    axesList(axisIndex).Layout.Row = ceil(axisIndex / columns(viewIndex));
                    axesList(axisIndex).Layout.Column = mod(axisIndex - 1, columns(viewIndex)) + 1;
                end
                caseView.Axes = axesList;
            end
            axesList = caseView.Axes;
            showFault = isfield(logData, 'faulty_thrusters') && ~isempty(logData.faulty_thrusters);
            options = struct('ShowLegend', true, 'ShowFaultTime', showFault, 'HideCasePrefix', true);
            switch viewIndex
                case 1
                    RCPlotter.plotResponseComponents(axesList(1), logData, {''}, 'pos', options);
                    RCPlotter.plotResponseComponents(axesList(2), logData, {''}, 'att', options);
                    RCPlotter.plotErrorNorm(axesList(3), logData, {''}, 'pos', options);
                    RCPlotter.plotErrorNorm(axesList(4), logData, {''}, 'att', options);
                case 2
                    RCPlotter.plotBodyWrenchHistory(axesList, logData, '', options);
                case 3
                    options.BodyHalfSize = app.currentBodyHalfSize();
                    if isfield(app.SimulationLog, 'CaseBodyHalfSizes')
                        options.BodyHalfSize = app.SimulationLog.CaseBodyHalfSizes{caseIndex};
                    end
                    options.SnapshotCount = 9;
                    options.AutoScaleBody = true;
                    RCPlotter.plotTrajectoryViews(axesList, logData, '', options);
                case 4
                    options.UseControlHistory = true;
                    options.LayoutName = '';
                    RCPlotter.plotThrusterPulseWidths(axesList, logData, options);
                case 5
                    options.ControlPeriod = logData.Control_Period;
                    options.StartTime = app.SimulationScheduleStartTimeField.Value;
                    options.CycleCount = app.SimulationScheduleCycleCountField.Value;
                    options.Title = '推力器调用时序';
                    RCPlotter.plotThrusterSchedule(axesList, logData, options);
            end
            caseView.Rendered = true;
            app.SimulationCaseViews(viewIndex, caseIndex) = caseView;
        end

        function createActualInvocationVerificationTab(app)
            % 在闭环结果区动态创建实际调用效果表，避免它依赖设计期的固定验证指令。
            if ~isempty(app.SimulationActualVerificationTab) && ...
                    isvalid(app.SimulationActualVerificationTab)
                return;
            end
            app.SimulationActualVerificationTab = ...
                uitab(app.SimulationViewTabGroup);
            app.SimulationActualVerificationTab.Title = '实际干扰对比';
            app.SimulationActualVerificationGrid = uigridlayout( ...
                app.SimulationActualVerificationTab);
            app.SimulationActualVerificationGrid.ColumnWidth = {'1x'};
            app.SimulationActualVerificationGrid.RowHeight = {36, '1x'};
            app.SimulationActualVerificationGrid.Padding = [12 12 12 12];
            app.SimulationActualVerificationSummaryLabel = uilabel( ...
                app.SimulationActualVerificationGrid);
            app.SimulationActualVerificationSummaryLabel.FontWeight = 'bold';
            app.SimulationActualVerificationSummaryLabel.FontColor = ...
                [0.10 0.27 0.42];
            app.SimulationActualVerificationSummaryLabel.Layout.Row = 1;
            app.SimulationActualVerificationSummaryLabel.Layout.Column = 1;
            app.SimulationActualVerificationTable = uitable( ...
                app.SimulationActualVerificationGrid);
            app.SimulationActualVerificationTable.ColumnWidth = {180, 150, ...
                155, 145, 130, 170};
            app.SimulationActualVerificationTable.RowName = {};
            app.SimulationActualVerificationTable.Layout.Row = 2;
            app.SimulationActualVerificationTable.Layout.Column = 1;
            app.updateActualInvocationVerification([]);
            app.SimulationViewTabGroup.Children = [ ...
                app.SimulationActualVerificationTab; app.SimulationScheduleTab; ...
                app.SimulationPulseTab; app.SimulationTrajectoryTab; ...
                app.SimulationCommandTab; app.SimulationPositionTab];
        end

        function updateActualInvocationVerification(app, verification)
            if isempty(verification)
                app.SimulationActualVerificationSummaryLabel.Text = ...
                    '等待闭环仿真';
                app.SimulationActualVerificationTable.ColumnName = { ...
                    '工况', '标况平均相对控制残差', ...
                    '故障后平均相对控制残差', '轨控最大干扰力矩', ...
                    '姿控最大干扰力', '脉宽超控制周期次数'};
                app.SimulationActualVerificationTable.Data = { ...
                    '等待仿真', '-', '-', '-', '-', '-'};
                return;
            end
            app.SimulationActualVerificationSummaryLabel.Text = ...
                verification.Summary;
            app.SimulationActualVerificationTable.ColumnName = ...
                verification.ColumnNames;
            app.SimulationActualVerificationTable.Data = verification.Data;
        end

        function verification = buildActualInvocationVerification(~, ...
                matrixConf, logs, labels)
            % 只统计实际闭环结果；故障工况的指标从故障时刻开始计算。
            rowCount = numel(logs);
            data = cell(rowCount, 6);
            for caseIndex = 1:rowCount
                logData = logs{caseIndex};
                cycleCount = min([numel(logData.Control_Time), ...
                    size(logData.Requested_Wrench_History, 2), ...
                    size(logData.Actual_Wrench_History, 2)]);
                if cycleCount < 1
                    data(caseIndex, :) = {labels{caseIndex}, '-', '-', ...
                        '-', '-', 0};
                    continue;
                end

                requested = logData.Requested_Wrench_History(:, 1:cycleCount);
                actual = logData.Actual_Wrench_History(:, 1:cycleCount);
                if iscell(matrixConf)
                    caseMatrix = matrixConf{caseIndex};
                else
                    caseMatrix = matrixConf;
                end
                rowScale = max(abs(caseMatrix), [], 2);
                rowScale(rowScale < 1e-12) = 1;
                requestedMagnitude = sqrt(sum( ...
                    bsxfun(@rdivide, requested, rowScale).^2, 1));
                residualMagnitude = sqrt(sum( ...
                    bsxfun(@rdivide, requested - actual, rowScale).^2, 1));
                activeCycles = requestedMagnitude > 1e-9;
                isFaultCase = isfield(logData, 'faulty_thrusters') && ...
                    ~isempty(logData.faulty_thrusters);
                evaluatedCycles = true(1, cycleCount);
                if isFaultCase && isfield(logData, 'faulty_time')
                    evaluatedCycles = logData.Control_Time(1:cycleCount) >= ...
                        logData.faulty_time;
                end
                residualCycles = activeCycles & evaluatedCycles;
                nominalResidualText = '-';
                faultResidualText = '-';
                if any(residualCycles)
                    meanResidual = mean(residualMagnitude(residualCycles) ./ ...
                        requestedMagnitude(residualCycles));
                    if isFaultCase
                        faultResidualText = sprintf('%.3e', meanResidual);
                    else
                        nominalResidualText = sprintf('%.3e', meanResidual);
                    end
                end

                isJoint = isfield(logData, 'Alloc_Mode_History') && ...
                    any(strcmpi(string(logData.Alloc_Mode_History(1:cycleCount)), ...
                    'joint_optimization'));
                if isJoint
                    orbitCrossTorqueText = '联合优化';
                    attitudeCrossForceText = '联合优化';
                else
                    controlPeriod = max(logData.Control_Period, eps);
                    orbitWrench = caseMatrix * ...
                        (logData.Pulse_Orbit_History(:, 1:cycleCount) / controlPeriod);
                    attitudeWrench = caseMatrix * ...
                        (logData.Pulse_Att_History(:, 1:cycleCount) / controlPeriod);
                    orbitCrossTorque = sqrt(sum(orbitWrench(4:6, :).^2, 1));
                    attitudeCrossForce = sqrt(sum(attitudeWrench(1:3, :).^2, 1));
                    if any(evaluatedCycles)
                        orbitCrossTorqueText = sprintf('%.3e N m', ...
                            max(orbitCrossTorque(evaluatedCycles)));
                        attitudeCrossForceText = sprintf('%.3e N', ...
                            max(attitudeCrossForce(evaluatedCycles)));
                    else
                        orbitCrossTorqueText = '-';
                        attitudeCrossForceText = '-';
                    end
                end

                if isfield(logData, 'Raw_Max_Pulse_History')
                    rawMaxPulse = logData.Raw_Max_Pulse_History(1:cycleCount);
                else
                    rawMaxPulse = max(logData.Pulse_Command_History(:, ...
                        1:cycleCount), [], 1);
                end
                rawPulseOverPeriodCount = sum(rawMaxPulse(evaluatedCycles) > ...
                    logData.Control_Period + 1e-9);
                data(caseIndex, :) = {labels{caseIndex}, nominalResidualText, ...
                    faultResidualText, orbitCrossTorqueText, ...
                    attitudeCrossForceText, rawPulseOverPeriodCount};
            end
            verification.Data = data;
            verification.ColumnNames = {'工况', '标况平均相对控制残差', ...
                '故障后平均相对控制残差', '轨控最大干扰力矩', ...
                '姿控最大干扰力', '脉宽超控制周期次数'};
            verification.Summary = '实际干扰对比';
        end

        function refreshSimulationSchedulePlot(app)
            if isempty(app.SimulationLog) || isempty(app.SimulationCaseViews), return; end
            app.SimulationScheduleCycleCountField.Value = ...
                max(1, round(app.SimulationScheduleCycleCountField.Value));
            for caseIndex = 1:size(app.SimulationCaseViews, 2)
                app.SimulationCaseViews(5, caseIndex).Rendered = false;
            end
            selected = app.SimulationScheduleCaseTabGroup.SelectedTab;
            for caseIndex = 1:size(app.SimulationCaseViews, 2)
                if isequal(app.SimulationCaseViews(5, caseIndex).Tab, selected)
                    app.renderSimulationCase(5, caseIndex);
                    break;
                end
            end
        end

        function logData = runClosedloopCase(app, params, B, simCfg, faultSet)
            layoutParams = params;
            layoutParams.Num = size(B, 2);
            layoutParams.B_all = B;
            simCfg.true_faults = faultSet;
            layoutParams.true_faults = faultSet;
            evalc('logData = Closedloop_sim(layoutParams, B, simCfg);');
        end

        function [faultSets, labels] = simulationFaultCases(app, ~, configuredFaultCases)
            faultSets = {[]};
            labels = {'标况'};
            for index = 1:numel(configuredFaultCases)
                faultSet = configuredFaultCases{index};
                if isempty(faultSet) || ~isempty(app.findFaultSetRow(faultSets, faultSet))
                    continue;
                end
                faultSets{end + 1} = faultSet; %#ok<AGROW>
                labels{end + 1} = ['故障', app.faultSetText(faultSet)]; %#ok<AGROW>
            end
        end

        function faultNumbers = selectedSummaryFaultNumbers(app, maximumFaults)
            faultCount = app.requestedFaultCount(maximumFaults);
            rawFaults = app.parseVector(app.FmaxField_6.Value, []);
            if faultCount == 0
                faultNumbers = 0;
            elseif isscalar(rawFaults) && rawFaults == 0
                faultNumbers = [0, 1];
            else
                faultNumbers = faultCount;
            end
        end

        function faultLevel = getFaultLevel(app)
            faultLevel = app.requestedFaultCount();
        end

        function [faultyIndices, faultCases] = selectedMetricsFaultSet(app, maximumIndex)
            expectedCount = app.requestedFaultCount(maximumIndex);
            rawFaults = app.parseVector(app.FmaxField_6.Value, []);
            if expectedCount == 0 || (isscalar(rawFaults) && rawFaults == 0)
                params = struct('Num', maximumIndex);
                faultCases = app.getFaultCases(params, app.FmaxField_6.Value, expectedCount);
                faultyIndices = [];
                return;
            end
            faultyIndices = app.parseIndexList(app.FmaxField_6.Value);
            if isempty(faultyIndices)
                error('请输入评价图使用的故障推力器编号，例如1或[1 3]。');
            end
            if any(faultyIndices > maximumIndex)
                error('故障编号不能大于当前布局的推力器总数 %d。', ...
                    maximumIndex);
            end
            if numel(faultyIndices) ~= expectedCount
                error('%d台故障需要输入%d个故障编号，当前输入了%d个。', ...
                    expectedCount, expectedCount, ...
                    numel(faultyIndices));
            end
            faultCases = {faultyIndices};
        end

        function mode = mapAllocationMode(~, displayMode)
            switch char(string(displayMode))
                case '异步分时复用'
                    mode = 'asynchronous_time_division';
                case '同步分时复用'
                    mode = 'synchronous_time_division';
                case '联合优化复用'
                    mode = 'joint_optimization';
                otherwise
                    mode = 'synchronous_time_division';
            end
        end

        function syncAllocationStrategyControl(app)
            % 分配策略固定为最简主备，输入条件页只保留可切换的复用策略。
        end

        function AllocationModeDropDownValueChanged(app, event)
            app.syncAllocationStrategyControl();
            app.ReconfigJudgmentResult = [];
            app.EvaluationResult = [];
            app.EvaluationTable.Data = {};
            app.JudgmentSummaryTable.Data = {};
            app.SingleFaultTable.Data = {};
%             app.PolicyDetailTable.Data = {};
%             app.PolicyInfoLabel.Text = '调用策略已改变，请重新评价。';
            app.resetMetricsComparisonTabs();
            app.SingleFaultInfoLabel.Text = '调用策略已改变，请重新运行判断和评价。';
            app.setStatus('调用策略已改变：旧评价已清除，请重新计算。');
        end

        function value = faultCountControlValue(app)
            value = sprintf('%d', max(0, round(app.FmaxField_5.Value)));
        end

        function setFaultCountControlValue(app, value)
            count = app.toScalar(value, 1);
            app.FmaxField_5.Value = max(0, round(count));
        end

        function faultCount = requestedFaultCount(app, maximumFaults)
            rawValue = strtrim(app.faultCountControlValue());
            switch rawValue
                case {'单台故障', '1台故障'}
                    faultCount = 1;
                case {'两台故障', '2台故障'}
                    faultCount = 2;
                case '自定义故障组合'
                    faultCount = numel(app.parseIndexList( ...
                        app.FmaxField_6.Value));
                otherwise
                    faultCount = str2double(rawValue);
            end
            if ~isscalar(faultCount) || ~isfinite(faultCount) || ...
                    faultCount < 0 || floor(faultCount) ~= faultCount
                error('故障数量应为大于等于0的整数。');
            end
            if nargin >= 2 && ~isempty(maximumFaults) && ...
                    faultCount > maximumFaults
                error('故障数量不能大于推力器总数 %d。', maximumFaults);
            end
            app.setFaultCountControlValue(sprintf('%d', faultCount));
        end

        function text = faultConditionText(app)
            faultCount = app.requestedFaultCount();
            rawFaults = app.parseVector(app.FmaxField_6.Value, []);
            if faultCount == 0
                text = '标况';
            elseif isscalar(rawFaults) && rawFaults == 0
                text = '标况及全部单台故障';
            else
                text = sprintf('%d台故障', faultCount);
            end
        end

        function indices = parseIndexList(app, value)
            nums = app.parseVector(value, []);
            nums = nums(:)';
            nums = nums(nums >= 1 & floor(nums) == nums);
            indices = unique(nums);
        end

        function vec = parseVector(app, value, expectedLength)
            if isnumeric(value)
                vec = value(:);
            else
                text = char(string(value));
                text = strrep(text, '[', ' ');
                text = strrep(text, ']', ' ');
                text = strrep(text, ',', ' ');
                text = strrep(text, ';', ' ');
                vec = sscanf(text, '%f');
            end
            if nargin >= 3 && ~isempty(expectedLength) && numel(vec) ~= expectedLength
                error('参数 "%s" 应包含 %d 个数值。', char(string(value)), expectedLength);
            end
        end

        function range = parseMathRange(app, value, fieldLabel)
            textValue = char(string(value));
            textValue = strrep(textValue, '[', ' ');
            textValue = strrep(textValue, ']', ' ');
            parts = regexp(strtrim(textValue), '[,;\s]+', 'split');
            parts = parts(~cellfun('isempty', parts));
            if numel(parts) ~= 2
                error('%s应输入两个边界值，例如 [0,0.6] 或 [0,2π]。', fieldLabel);
            end
            range = [app.parseMathScalar(parts{1}, fieldLabel), ...
                app.parseMathScalar(parts{2}, fieldLabel)];
            if range(1) > range(2)
                error('%s的下限不能大于上限。', fieldLabel);
            end
        end

        function value = parseMathScalar(app, inputText, fieldLabel)
            token = lower(strtrim(char(string(inputText))));
            token = strrep(token, 'π', 'pi');
            token = strrep(token, '*', '');
            if contains(token, 'pi')
                coefficientText = strrep(token, 'pi', '');
                if isempty(coefficientText) || strcmp(coefficientText, '+')
                    coefficient = 1;
                elseif strcmp(coefficientText, '-')
                    coefficient = -1;
                else
                    coefficient = str2double(coefficientText);
                end
                value = coefficient * pi;
            else
                value = str2double(token);
            end
            if ~isfinite(value)
                error('%s中的“%s”不是有效数值。', fieldLabel, inputText);
            end
        end

        function value = toScalar(app, value, defaultValue)
            vec = app.parseVector(value, []);
            if isempty(vec)
                value = defaultValue;
            else
                value = vec(1);
            end
        end

        function setBusy(app, message)
            app.setStatus(message);
            drawnow;
        end

        function setStatus(app, message)
            app.StatusLabel.Text = char(message);
        end

        function beginOptimizationProgress(app, maxGenerations)
            app.ProgressStartTic = tic;
            app.OptimizationCurrentGeneration = 0;
            app.OptimizationBestCoverage = 0;
            app.OptimizationMaxGenerations = max(1, round(maxGenerations));
            app.SearchRunning = true;
            app.setSearchControlsEnabled(false);
            if strcmp(app.DropDown1.Value,'115拓扑自动布局搜索')
                app.ProgressLabel.Text='正在读取拓扑队列；已有记录会自动接续…';
                app.setStatus('自适应搜索｜单任务120至300秒｜先搜索几何候选，再求解组选');
                drawnow;
                return;
            end
            if strcmp(app.DropDown1.Value,'完整组选必要条件')
                app.ProgressLabel.Text = '正在读取并复核搜索记录…';
                app.setStatus('正在寻找完整组选必要条件｜结果和未完成分支将自动保存');
                drawnow;
                return;
            end
            if isprop(app, 'SearchProgressFill') && ~isempty(app.SearchProgressFill) && isvalid(app.SearchProgressFill)
                app.SearchProgressFill.Visible = 'off';
            end
            app.Label2.Text = '几何阶段显示本轮时间预算；必要条件通过仍不等于物理布局可行。';
            if isprop(app, 'ProgressGauge')
                app.ProgressGauge.Value = 0;
            end
            app.ProgressLabel.Text = '主备组配平求解｜0%｜耗时 00:00';
            app.setStatus('正在搜索首个可行布局｜正在建立十二变量主备组模型...');
            drawnow;
        end

        function updateOptimizationProgress(app, info)
            if isempty(app.ProgressStartTic)
                app.ProgressStartTic = tic;
            end
            elapsedSeconds = toc(app.ProgressStartTic);
            if isfield(info,'SupportTotal')
                fraction=info.SupportSeen/max(1,info.SupportTotal);
                app.SearchProgressFill.Position=[0,0,max(eps,fraction),1];
                app.SearchProgressFill.Visible='on';
                app.ProgressLabel.Text=sprintf('已访问 %d/%d 拓扑｜累计 %d 次轮转｜当前 %s｜最高 %d/156',info.SupportSeen,info.SupportTotal,info.SupportVisits,info.SupportCurrent,info.Coverage);
                if isfield(info,'SearchMetrics')
                    m=info.SearchMetrics;
                    app.ProgressLabel.Text=sprintf('旧轮转%d｜新直接几何%d次｜计算%d次｜节点%d｜候选检查%d｜最高%d/156',m.legacy_visits,m.direct_calls,m.direct_evaluations,m.nodes_since_upgrade,m.candidate_checks,info.Coverage);
                end
                app.setStatus(sprintf('自适应搜索｜当前 %s｜本轮 %.0f / %.0f 秒｜已访问不等于已排除',info.SupportCurrent,elapsedSeconds,app.GenerationField_3.Value));
                drawnow limitrate nocallbacks;
                return;
            end
            if strcmp(app.DropDown1.Value,'完整组选必要条件')
                if isfield(info,'BranchProgress')
                    app.showBranchProgress(info.BranchProgress);
                    app.setStatus(sprintf('完整组选搜索中｜待搜索 %d 分支｜待证明复核 %d｜本轮已用 %.0f / %.0f s', ...
                        info.BranchProgress.pending_branches,info.BranchProgress.review_branches,elapsedSeconds,app.GenerationField_3.Value));
                else
                    app.ProgressLabel.Text = sprintf('分层诊断与记录复核｜本轮已用 %.0f / %.0f s',elapsedSeconds,app.GenerationField_3.Value);
                end
                drawnow limitrate nocallbacks;
                return;
            end
            evaluationCount = 0;
            if isfield(info, 'EvaluationCount') && isfinite(info.EvaluationCount)
                evaluationCount = max(0, round(info.EvaluationCount));
            end
            if isfield(info, 'Generation') && isfinite(info.Generation)
                app.OptimizationCurrentGeneration = max(0, ...
                    min(app.OptimizationMaxGenerations, round(info.Generation)));
            end
            if isfield(info, 'Coverage') && isfinite(info.Coverage)
                app.OptimizationBestCoverage = max(app.OptimizationBestCoverage,info.Coverage);
            end
            if isfield(info, 'Progress') && isfinite(info.Progress)
                progressValue = 100 * max(0, min(1, info.Progress));
            else
                progressValue = 100 * app.OptimizationCurrentGeneration / ...
                    app.OptimizationMaxGenerations;
            end
            if isprop(app, 'ProgressGauge')
                app.ProgressGauge.Value = progressValue;
            end

            elapsedText = app.formatElapsedTime(elapsedSeconds);
            stageText = '主备组配平求解';
            if isfield(info, 'Stage') && strcmp(info.Stage, 'fault_backup')
                stageText = '单故障严格备用复核';
            elseif isfield(info, 'Stage') && strcmp(info.Stage, 'joint_minlp')
                stageText = '几何与主备组联合求解';
            elseif isfield(info, 'Stage') && strcmp(info.Stage, 'joint_polish')
                stageText = '候选固定组精修';
            elseif isfield(info, 'Stage') && strcmp(info.Stage, 'fair_partition')
                stageText = '分区轮转与主备组选配';
            elseif isfield(info, 'Stage') && strcmp(info.Stage, 'seed_screen')
                stageText = '初值全扫描';
            elseif isfield(info, 'Stage') && strcmp(info.Stage, 'topology_master')
                stageText = '组选相容性搜索';
            elseif isfield(info, 'Stage') && strcmp(info.Stage, 'topology_geometry')
                stageText = '固定组选的安装几何求解';
            elseif isfield(info, 'Stage') && strcmp(info.Stage, 'topology_complete')
                stageText = '组选搜索本轮完成';
            elseif isfield(info, 'Stage') && strcmp(info.Stage, 'zero_free_patterns')
                stageText = '729类零/自由模式搜索';
            end
            app.ProgressLabel.Text = sprintf('%s｜%d%%｜耗时 %s', ...
                stageText, floor(progressValue), elapsedText);
            if isfield(info,'Topologies')
                app.StatusLabel.Text = sprintf('待解组选 %d 个｜已证明冲突 %d 条｜几何检查 %d 次｜最高覆盖 %d/156', ...
                    info.Topologies,info.ConflictCount,evaluationCount,app.OptimizationBestCoverage);
            elseif isfield(info,'PatternsSeen')
                app.StatusLabel.Text = sprintf('已初筛 %d/729 类｜当前第%d类，%d个自由变量｜最高覆盖 %d/156', ...
                    info.PatternsSeen,info.PatternId,info.FreeCount,app.OptimizationBestCoverage);
            elseif isfield(info,'Pending')
                app.StatusLabel.Text = sprintf('已处理 %d 个分区｜待搜索 %d 个｜最高覆盖 %d/156',evaluationCount,info.Pending,app.OptimizationBestCoverage);
            elseif evaluationCount > 0
                app.StatusLabel.Text = sprintf( ...
                    '可行性搜索｜已评估 %d 个候选｜最高覆盖 %d/156', evaluationCount,app.OptimizationBestCoverage);
            else
                app.StatusLabel.Text = sprintf('可行性搜索｜%s｜最高覆盖 %d/156', stageText,app.OptimizationBestCoverage);
            end
            drawnow limitrate nocallbacks;
        end

        function finishOptimizationProgress(app, completed)
            app.SearchRunning = false;
            app.setSearchControlsEnabled(true);
            if isempty(app.ProgressStartTic)
                elapsedSeconds = 0;
            else
                elapsedSeconds = toc(app.ProgressStartTic);
            end
            elapsedText = app.formatElapsedTime(elapsedSeconds);
            if completed
                if isprop(app, 'ProgressGauge')
                    app.ProgressGauge.Value = 100;
                end
                app.ProgressLabel.Text = sprintf('已完成｜100%%｜耗时 %s', elapsedText);
            else
                app.ProgressLabel.Text = sprintf('运行中止｜耗时 %s', elapsedText);
            end
            app.ProgressStartTic = [];
            drawnow;
        end

        function cfg = readSearchSettings(app)
            cfg = struct('optimization_target','首个可行布局','x_face',app.FmaxField_4.Value, ...
                'start_count',round(app.PopulationField.Value),'max_time',app.GenerationField_3.Value, ...
                'min_axis_capacity',1e-5,'pure_tolerance',1e-9,'max_group_size',6, ...
                'y_max',app.NumericEditField1_2.Value,'z_max',app.NumericEditField2_4.Value, ...
                'min_install_distance',app.NumericEditField2_5.Value,'min_install_angle_deg',app.NumericEditField2_6.Value, ...
                'save_result',true,'output_dir',app.ProjectRoot);
            cfg.position_a_range = app.parseMathRange(app.EditField1_2.Value,'位置参数 a 范围');
            cfg.position_b_range = app.parseMathRange(app.EditField1_3.Value,'位置参数 b 范围');
            cfg.alpha_range = app.parseMathRange(app.EditField1_4.Value,'方位角 α 范围');
            cfg.beta_range = app.parseMathRange(app.EditField1_5.Value,'俯仰角 β 范围');
        end

        function key = searchScopeKey(app, scope)
            if nargin < 2
                cfg = app.readSearchSettings();
                lo = repmat([max(-cfg.y_max,cfg.position_a_range(1)),max(-cfg.z_max,cfg.position_b_range(1)),cfg.alpha_range(1),cfg.beta_range(1)],1,3);
                hi = repmat([min(cfg.y_max,cfg.position_a_range(2)),min(cfg.z_max,cfg.position_b_range(2)),cfg.alpha_range(2),cfg.beta_range(2)],1,3);
                numbers = [lo,hi,cfg.x_face,cfg.min_install_distance,cfg.min_install_angle_deg,cfg.min_axis_capacity,cfg.pure_tolerance,cfg.max_group_size];
            else
                values = [string(scope.lower(:));string(scope.upper(:));string(scope.x_face); ...
                    string(scope.min_install_distance);string(scope.min_install_angle_deg); ...
                    string(scope.min_axis_capacity);string(scope.pure_tolerance);string(scope.max_group_size)];
                numbers = zeros(1,numel(values));
                for j = 1:numel(values)
                    parts = split(values(j),'/');
                    numbers(j) = str2double(parts(1));
                    if numel(parts)==2, numbers(j)=numbers(j)/str2double(parts(2)); end
                    if numel(parts)>2 || ~isfinite(numbers(j)), error('搜索记录的边界无效。'); end
                end
            end
            key = jsonencode(numbers);
        end

        function setSearchControlsEnabled(app, enabled)
            value = matlab.lang.OnOffSwitchState(enabled);
            controls = {'StartOptimizationButton_3','LoadSearchRecordButton','DropDown1', ...
                'EditField1_2','EditField1_3','EditField1_4','EditField1_5','FmaxField_4','FmaxField_2', ...
                'NumericEditField1_2','NumericEditField2_4','NumericEditField2_5','NumericEditField2_6', ...
                'GenerationField_3','PopulationField'};
            for j=1:numel(controls)
                if isprop(app, controls{j}) && ~isempty(app.(controls{j})) && isvalid(app.(controls{j}))
                    app.(controls{j}).Enable=value;
                end
            end
        end

        function recoverInterruptedSearch(app)
            if ~isvalid(app) || ~app.SearchRunning, return; end
            app.SearchRunning=false;app.ProgressStartTic=[];
            app.setSearchControlsEnabled(true);
            if strcmp(app.DropDown1.Value,'完整组选必要条件')
                app.restoreLatestDiagnostic();
            else
                app.setStatus('搜索已中断｜可从已经保存的记录继续');
            end
        end

        function showBranchProgress(app, p)
            fraction = max(0,min(1,p.certified_percent/100));
            if isprop(app, 'SearchProgressFill') && ~isempty(app.SearchProgressFill) && isvalid(app.SearchProgressFill)
                app.SearchProgressFill.Position = [0,0,max(eps,fraction),1];
                app.SearchProgressFill.Visible = matlab.lang.OnOffSwitchState(fraction>0);
            end
            app.ProgressLabel.Text = sprintf('已认证排除 %.6g%%｜关闭 %d｜待搜索 %d｜待复核 %d｜冲突 %d｜累计求解 %d 次', ...
                p.certified_percent,p.closed_branches,p.pending_branches,p.review_branches,p.conflicts,p.solver_calls);
            app.Label2.Text = '进度条 = 已认证排除的组选组合比例；超时不计完成，可能长期接近0%，不代表剩余时间。';
        end

        function acceptDiagnosticRecord(app, path, trusted)
            record = jsondecode(fileread(path));
            if isfield(record,'diagnostic_report_file') && ~isfield(record,'scope')
                path = char(record.diagnostic_report_file);
                record = jsondecode(fileread(path));
            end
            if ~isfield(record,'method') || ~strcmp(record.method,'layered_rational_diagnosis_v1') || ~isfield(record,'scope')
                error('请选择必要条件搜索的 diagnosis.json 或 Optim_diagnosis_latest.json。');
            end
            key = app.searchScopeKey(record.scope);
            app.DiagnosticRecord = record;
            app.DiagnosticResumeFile = char(path);
            app.DiagnosticScopeKey = key;
            app.DiagnosticTrusted = trusted;
            app.refreshSearchState();
        end

        function restoreLatestDiagnostic(app)
            app.DiagnosticRecord=[];app.DiagnosticResumeFile='';app.DiagnosticTrusted=false;
            path = fullfile(app.ProjectRoot,'Optim_diagnosis_latest.json');
            if ~isfile(path)
                records = dir(fullfile(app.ProjectRoot,'Optim_diagnosis_*','diagnosis.json'));
                if ~isempty(records)
                    [~,j]=max([records.datenum]);path=fullfile(records(j).folder,records(j).name);
                else
                    path=fullfile(app.ProjectRoot,'analysis','layered_resume_150s_20260910','diagnosis.json');
                end
            end
            if isfile(path)
                try
                    app.acceptDiagnosticRecord(path,false);
                catch ME
                    app.updateReport("未能读取上次搜索记录："+string(ME.message));
                    app.setStatus('搜索记录读取失败｜可用“载入搜索记录”选择完好的 diagnosis.json');
                end
            else
                app.refreshSearchState();
            end
        end

        function refreshSearchState(app)
            if app.SearchRunning, return; end
            app.StartOptimizationButton_3.Enable='on';
            app.GenerationField_3.Enable='on';
            if strcmp(app.DropDown1.Value,'115拓扑自动布局搜索')
                app.StartOptimizationButton_3.Text='开始／继续拓扑搜索';
                app.PopulationField.Enable='off';
                app.SearchProgressFill.Visible='off';
                app.Label2.Text='已有任务优先；每次120至300秒，含直接几何搜索；已访问不代表已排除。';
                app.ProgressLabel.Text='直接搜索解析筛选后的拓扑，无需先手动完成完整组选必要条件。';
                if isstruct(app.OptimResult) && isfield(app.OptimResult,'feasible') && app.OptimResult.feasible && strcmp(app.GeometryScopeKey,app.searchScopeKey())
                    app.StartOptimizationButton_3.Text='已验证完整可行布局';
                    app.StartOptimizationButton_3.Enable='off';
                end
                return;
            end
            if strcmp(app.DropDown1.Value,'安装拓扑解析筛选')
                app.StartOptimizationButton_3.Text='筛选安装拓扑';
                app.PopulationField.Enable='off';
                app.GenerationField_3.Enable='off';
                app.SearchProgressFill.Visible='off';
                app.Label2.Text='严格零／非零，禁止安装点重合，允许2/4/6台；只排除有解析证明的模式。';
                app.ProgressLabel.Text='有限规则扫描；不进行布局优化，也不沿用旧729自由模式进度。';
                return;
            end
            diagnostic=strcmp(app.DropDown1.Value,'完整组选必要条件');
            app.PopulationField.Enable=matlab.lang.OnOffSwitchState(~diagnostic);
            same=~isempty(app.DiagnosticRecord) && strcmp(app.searchScopeKey(),app.DiagnosticScopeKey);
            if same && app.DiagnosticTrusted && isfield(app.DiagnosticRecord,'branch_progress')
                app.showBranchProgress(app.DiagnosticRecord.branch_progress);
            else
                if isprop(app, 'SearchProgressFill') && ~isempty(app.SearchProgressFill) && isvalid(app.SearchProgressFill)
                app.SearchProgressFill.Visible='off';
                end
                app.ProgressLabel.Text='已认证排除 0%｜尚无当前边界的分支统计';
                app.Label2.Text='必要条件与安装几何分两步检查；时限耗尽会保存记录，再次点击可以续跑。';
            end
            if diagnostic
                app.StartOptimizationButton_3.Text='查找必要条件';
                if same
                    app.StartOptimizationButton_3.Text='继续查找';
                    status=string(app.DiagnosticRecord.status);
                    if ~app.DiagnosticTrusted
                        app.setStatus('已载入上次搜索记录｜点击后先复核证据，再继续未完成分支');
                    elseif status=="necessary_conditions_sat"
                        app.StartOptimizationButton_3.Enable='off';
                        app.StartOptimizationButton_3.Text='必要条件已找到';
                        app.setStatus('完整组选必要条件已找到｜156/156项精确复核通过｜可停止此阶段，切换安装几何与布局');
                    elseif status=="diagnostic_unsat_verified"
                        app.StartOptimizationButton_3.Enable='off';
                        app.setStatus('当前约束下已认证无解｜无需继续点击，改变约束后方可重新搜索');
                    elseif status=="diagnostic_review_required" || status=="diagnostic_unsat_reported" || status=="diagnostic_solver_disagreement"
                        app.StartOptimizationButton_3.Enable='off';
                        app.setStatus('需要复核已保存的证明材料｜尚未认证无解，重复点击不会推进');
                    else
                        app.setStatus('本轮未判定｜搜索记录已保存，可保持时限点击“继续查找”');
                        if isfield(app.DiagnosticRecord,'layers')
                            layers=app.DiagnosticRecord.layers;
                            for j=1:numel(layers)
                                if iscell(layers), item=layers{j}; else, item=layers(j); end
                                if strcmp(item.layer,'force') && strcmp(item.status,'sat_verified')
                                    app.setStatus('仅力配平已通过（78项）｜完整组选必要条件尚未找到，可点击“继续查找”');
                                end
                            end
                        end
                    end
                else
                    app.setStatus('准备搜索当前约束｜新边界单独建队列；旧记录保留，有效冲突会重新复核后复用');
                end
            else
                app.StartOptimizationButton_3.Text='搜索安装布局';
                if same && app.DiagnosticTrusted && strcmp(app.DiagnosticRecord.status,'necessary_conditions_sat')
                    if isfield(app.DiagnosticRecord,'geometry_checkpoint_file')
                        app.setStatus('完整必要条件已通过｜点击搜索满足安装约束的物理布局');
                    else
                        app.StartOptimizationButton_3.Enable='off';
                        app.setStatus('必要条件已通过，但当前角度范围不支持几何接续｜请查看诊断报告');
                    end
                else
                    app.StartOptimizationButton_3.Enable='off';
                    app.setStatus('请先完成并复核当前约束的“完整组选必要条件”阶段');
                end
                if isprop(app, 'SearchProgressFill') && ~isempty(app.SearchProgressFill) && isvalid(app.SearchProgressFill)
                    app.SearchProgressFill.Visible='off';
                end
                app.Label2.Text='安装几何阶段还需满足单位方向、位置、间距、夹角和全部156项；通过后才会生成布局。';
                app.ProgressLabel.Text='安装几何阶段｜等待运行';
            end
        end

        function textValue = formatElapsedTime(app, elapsedSeconds)
            elapsedSeconds = max(0, floor(elapsedSeconds));
            hours = floor(elapsedSeconds / 3600);
            minutes = floor(mod(elapsedSeconds, 3600) / 60);
            seconds = mod(elapsedSeconds, 60);
            if hours > 0
                textValue = sprintf('%02d:%02d:%02d', hours, minutes, seconds);
            else
                textValue = sprintf('%02d:%02d', minutes, seconds);
            end
        end

        function updateReport(app, message)
            params = app.CurrentParams;
            if isempty(params)
                paramsText = '参数尚未加载';
            else
                paramsText = sprintf('推力器数量：%d，最大推力：%.3g N，控制周期：%.3g s', ...
                    params.Num, params.F_max, params.T);
            end
            if isempty(app.CurrentB)
                layoutText = '布局尚未加载';
            else
                layoutText = sprintf('当前布局：%d 台推力器，位置/方向表已更新', size(app.CurrentB, 2));
            end
            app.ReportLines = {
                '设计结果摘要';
                '';
                ['当前状态：', char(string(message))];
                paramsText;
                layoutText;
                '';
                '软件输入：任务场景、飞行器参数、控制器参数、推力器参数、故障状态和优化设计参数。';
                '软件输出：配置矩阵、三维布局图、优化布局、可重构判断、可重构设计指标、调用策略、喷气脉宽、闭环响应曲线和结果数据。';
                '参数文件：支持 txt、csv、xls、xlsx 格式，可保存当前参数和推力器布局。'};
        end

        function showError(app, titleText, ME)
            app.setStatus(['错误｜', ME.message]);
            uialert(app.UIFigure, ME.message, titleText);
        end

        function resizeMainTabGroup(app)
            if isempty(app.PageHostPanel) || isempty(app.TabGroup) || ...
                    ~isvalid(app.PageHostPanel) || ~isvalid(app.TabGroup)
                return;
            end
            % GridLayout 在 App Designer 反序列化和应用启动的第一个
            % 布局周期中会暂时报告 260×221 的默认尺寸。若直接使用该
            % 尺寸，主页面会先缩在左下角，待 SizeChanged 回调后才铺满。
            % 根据已定义的网格行列计算页面宿主的目标尺寸，使设计视图
            % 与运行时从第一帧开始保持一致。
            figurePosition = app.UIFigure.InnerPosition;
            rootPadding = app.RootGrid.Padding;
            rootRowHeight = app.RootGrid.RowHeight;
            fixedRootHeight = 0;
            for index = 1:numel(rootRowHeight)
                if isnumeric(rootRowHeight{index})
                    fixedRootHeight = fixedRootHeight + rootRowHeight{index};
                end
            end
            bodyHeight = figurePosition(4) - rootPadding(2) - ...
                rootPadding(4) - fixedRootHeight - ...
                app.RootGrid.RowSpacing * (numel(rootRowHeight) - 1);

            bodyPadding = app.BodyGrid.Padding;
            bodyColumns = app.BodyGrid.ColumnWidth;
            fixedBodyWidth = 0;
            for index = 1:numel(bodyColumns)
                if isnumeric(bodyColumns{index})
                    fixedBodyWidth = fixedBodyWidth + bodyColumns{index};
                end
            end
            bodyWidth = figurePosition(3) - rootPadding(1) - rootPadding(3);
            hostWidth = bodyWidth - bodyPadding(1) - bodyPadding(3) - ...
                fixedBodyWidth - app.BodyGrid.ColumnSpacing * ...
                (numel(bodyColumns) - 1);
            hostHeight = bodyHeight - bodyPadding(2) - bodyPadding(4);

            tabBarHeight = 31;
            app.TabGroup.Units = 'pixels';
            app.TabGroup.Position = [0, -tabBarHeight, ...
                max(1, hostWidth), max(1, hostHeight) + tabBarHeight];
        end

        function NewButtonPushed(app, event)
            app.LoadParameterButtonPushed(event);
        end

        function ExportButtonPushed(app, event)
            app.GenerateReportButtonPushed(event);
        end

        function ApplyLayoutButtonPushed(app, event)
            try
                [app.CurrentB, app.CurrentR] = app.readLayoutFromTable();
                params = app.readParamsFromUI();
                params.Num = size(app.CurrentB, 2);
                app.FmaxField_2.Value = params.Num;
                app.CurrentParams = params;
                app.updateSelectedLayoutEntry();
                app.drawLayout();
                app.updateReport("已应用当前推力器布局。");
                app.setStatus('布局已应用并刷新');
            catch ME
                app.showError('布局应用失败', ME);
            end
        end

    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            app.initializeApp();
        end

        % Value changed function: NavigationList
        function NavigationListValueChanged(app, event)
            [labels, tabs] = app.availableNavigationTabs();
            index = find(strcmp(labels, app.NavigationList.Value), 1, 'first');
            if ~isempty(index)
                app.TabGroup.SelectedTab = tabs{index};
                if strcmp(labels{index}, '优化设计')
                    app.drawLayout();
                end
            end
        end

        % Size changed function: PageHostPanel
        function PageHostPanelSizeChanged(app, event)
            app.resizeMainTabGroup();
        end

        % Button pushed function: Button2_4
        function LoadParameterButtonPushed(app, event)
            [file, path] = uigetfile({'*.xlsx;*.xls;*.csv;*.txt', '参数文件 (*.xlsx,*.xls,*.csv,*.txt)'}, ...
                '加载参数文件', app.ProjectRoot);
            if isequal(file, 0)
                return;
            end
            try
                app.LoadParameterFile(fullfile(path, file));
            catch ME
                app.showError('参数加载失败', ME);
            end
        end

        % Button pushed function: OpenButton
        function OpenButtonPushed(app, event)
            app.SaveParameterButtonPushed(event);
        end

        % Button pushed function: ImportButton
        function ImportButtonPushed(app, event)
            [file, path] = uiputfile({'*.mat', 'MAT 布局文件 (*.mat)'; ...
                '*.xlsx', 'Excel 布局表 (*.xlsx)'; ...
                '*.txt', '文本布局表 (*.txt)'; ...
                '*.csv', 'CSV 布局表 (*.csv)'}, ...
                '导出当前布局', fullfile(app.ProjectRoot, 'RCDesigner_layout.mat'));
            if isequal(file, 0)
                return;
            end
            filePath = fullfile(path, file);
            try
                [~, ~, ext] = fileparts(filePath);
                if strcmpi(ext, '.mat')
                    B = app.CurrentB; %#ok<NASGU>
                    r = app.CurrentR; %#ok<NASGU>
                    layoutName = app.LayoutTemplateDropDown.Value; %#ok<NASGU>
                    save(filePath, 'B', 'r', 'layoutName');
                else
                    data = app.layoutTableNumericData();
                    layoutTable = array2table(data, 'VariableNames', ...
                        {'No', 'X', 'Y', 'Z', 'Dx', 'Dy', 'Dz'});
                    if strcmpi(ext, '.txt')
                        writetable(layoutTable, filePath, 'Delimiter', '\t', 'FileType', 'text');
                    else
                        writetable(layoutTable, filePath);
                    end
                end
                app.updateReport("已导出当前布局：" + string(filePath));
                app.setStatus(['已导出布局｜', filePath]);
            catch ME
                app.showError('布局导出失败', ME);
            end
        end

        % Button pushed function: Button1
% 原实现保留（口径或界面已调整；冲突部分停用）：
%                 '6. 在“可重构评价”页计算 Jc、Jo、Jt、Jf 和综合指标；\n', ...
% 旧口径保留：连续分配视图不再使用时序样本凸包。
%                 '6. 在“可重构评价”页对比策略能力、硬件上限、参考指令通过率与实际干扰；\n', ...
        function HelpButtonPushed(app, event)
            message = sprintf(['使用说明：\n', ...
                '1. 在“输入条件”页设置参数；\n', ...
                '2. 推力器数量必须为4的整数倍；L/W分别对应Y/Z安装面尺寸，d和θ为可选约束；\n', ...
                '3. 在“优化设计”页运行调用可重构优先的布局优化，并在右侧查看当前布局、三视图和布局参数；\n', ...
                '4. 可用右上角下拉框切换布局，导入布局后会自动加入方案列表；\n', ...
                '5. 在“输入条件”页设置故障台数、编号、时刻和复用策略；调用策略页直接生成当前配置的方案；\n', ...
                '6. 在“可重构评价”页对比连续分配下的实际/理论控制能力，实际干扰结合闭环仿真分析；\n', ...
                '7. 判断、调用和仿真页右上角可返回条件设置或布局选择，并可直接运行当前模块。']);
            uialert(app.UIFigure, message, 'RC-Designer 使用说明');
        end

        % Button pushed function: Button2_2
        function ResetParameterButtonPushed(app, event)
            app.loadDefaultState();
        end

        % Value changed function: NumericEditField1_2, NumericEditField2_4
        function LayoutBoundaryValueChanged(app, event)
            if ~isempty(app.CurrentB) && ~isempty(app.CurrentR)
                app.drawLayout();
                app.setStatus('安装边界已更新｜布局外形已刷新');
            end
            app.refreshSearchState();
        end

        % Value changed function: search settings and stage
        function SearchSettingsChanged(app, event)
            try
                app.refreshSearchState();
            catch ME
                app.setStatus(['请检查搜索范围：',ME.message]);
            end
        end

        % Button pushed function: LoadSearchRecordButton
        function LoadSearchRecordButtonPushed(app, event)
            [file,path]=uigetfile({'*.json','必要条件搜索记录 (*.json)'},'载入搜索记录',app.ProjectRoot);
            if isequal(file,0), return; end
            try
                app.acceptDiagnosticRecord(fullfile(path,file),false);
                app.updateReport("已选择搜索记录："+string(app.DiagnosticResumeFile)+"；续跑前会重新复核证据。");
            catch ME
                app.showError('搜索记录读取失败',ME);
            end
        end

        % Value changed function: LayoutTemplateDropDown
        function LayoutTemplateDropDownValueChanged(app, event)
            try
                index = find(strcmp({app.LayoutEntries.Name}, ...
                    app.LayoutTemplateDropDown.Value), 1, 'first');
                if isempty(index)
                    error('未找到所选布局数据。');
                end
                app.CurrentB = app.LayoutEntries(index).B;
                app.CurrentR = app.LayoutEntries(index).r;
                app.CurrentParams.Num = size(app.CurrentB, 2);
                app.FmaxField_2.Value = size(app.CurrentB, 2);
                app.populateLayoutTable(app.CurrentB, app.CurrentR);
                app.drawLayout();
                app.setStatus(['已切换布局｜', app.LayoutTemplateDropDown.Value]);
            catch ME
                app.showError('布局切换失败', ME);
            end
        end

        % Value changed function: SimulationScheduleCycleCountField,
        % ...and 1 other component
        function SimulationScheduleWindowValueChanged(app, event)
            app.refreshSimulationSchedulePlot();
        end

        % Button pushed function: ApplyLayoutButton_2
        function ImportLayoutButtonPushed(app, event)
            [file, path] = uigetfile({'*.mat;*.xlsx;*.xls;*.csv;*.txt', ...
                '布局文件 (*.mat,*.xlsx,*.xls,*.csv,*.txt)'}, ...
                '导入推力器布局', app.ProjectRoot);
            if isequal(file, 0)
                return;
            end
            filePath = fullfile(path, file);
            try
                [B, r, note] = app.readLayoutFile(filePath);
                if strlength(note) > 0
                    name = app.addLayoutEntry('diagnostic', B, r);
                else
                    name = app.addLayoutEntry('imported', B, r);
                end
                app.updateReport("已导入" + string(name) + "：" + string(filePath) + "｜" + note);
                app.setStatus(['已导入布局｜', name]);
            catch ME
                app.showError('布局导入失败', ME);
            end
        end

        % Button pushed function: DeleteLayoutButton
        function DeleteLayoutButtonPushed(app, event)
            try
                selectedName = app.LayoutTemplateDropDown.Value;
                index = find(strcmp({app.LayoutEntries.Name}, selectedName), ...
                    1, 'first');
                if isempty(index)
                    error('未找到当前选择的布局。');
                end
                if index == 1 || strcmpi(app.LayoutEntries(index).Kind, 'original')
                    error('“原布局”是基础布局，不能删除。');
                end

                deletedName = app.LayoutEntries(index).Name;
                app.LayoutEntries(index) = [];
                fallbackIndex = min(max(1, index - 1), numel(app.LayoutEntries));
                fallback = app.LayoutEntries(fallbackIndex);
                app.CurrentB = fallback.B;
                app.CurrentR = fallback.r;
                app.CurrentParams.Num = size(fallback.B, 2);
                app.CurrentParams.B_all = fallback.B;
                app.CurrentParams.r_all = fallback.r;
                app.FmaxField_2.Value = size(fallback.B, 2);
                app.refreshLayoutDropDown(fallback.Name);
                app.populateLayoutTable(fallback.B, fallback.r);
                app.drawLayout();

                if ~any(strcmpi({app.LayoutEntries.Kind}, 'optimized'))
                    app.OptimResult = [];
                end
                app.updateReport("已删除布局：" + string(deletedName));
                app.setStatus(['已删除布局｜', deletedName, ...
                    '｜当前布局：', fallback.Name]);
            catch ME
                app.showError('删除布局失败', ME);
            end
        end

        % Button pushed function: StartOptimizationButton_3
        function StartOptimizationButtonPushed(app, event)
            if app.SearchRunning, return; end
            try
                params = app.readParamsFromUI();
                if params.Num ~= 12
                    error('当前严格主备布局优化要求12台推力器。');
                end
                if app.FmaxField_4.Value ~= 2
                    error('当前对称布局算法使用 +X/-X 两个安装面，“安装面X”请设为2。');
                end
                optCfg = app.readSearchSettings();
                if strcmp(app.DropDown1.Value,'安装拓扑解析筛选')
                    optCfg.solver='structural'; optCfg.max_group_size=6;
                    app.SearchRunning=true;
                    app.setSearchControlsEnabled(false);
                    runCleanup=onCleanup(@()app.recoverInterruptedSearch()); %#ok<NASGU>
                    app.setStatus('正在枚举严格零／非零模式并核对解析排除条件…');
                    drawnow;
                    result=Optim_Algorithm(params,optCfg);
                    app.SearchRunning=false;
                    app.setSearchControlsEnabled(true);
                    app.refreshSearchState();
                    app.SearchProgressFill.Position=[0,0,1,1];
                    app.SearchProgressFill.Visible='on';
                    app.ProgressLabel.Text=sprintf('本版解析规则扫描完成｜剩余 %d 种代表模式，%d 种带编号模式',result.representative_remaining,result.labelled_remaining);
                    app.setStatus(result.message);
                    app.updateReport(string(result.message)+"｜模式与证明清单："+string(result.report_file));
                    return;
                end
                scopeKey = app.searchScopeKey();
                diagnostic = strcmp(app.DropDown1.Value,'完整组选必要条件');
                app.MinimumAxisCapacityField.Value = optCfg.min_axis_capacity;
                if strcmp(app.DropDown1.Value,'115拓扑自动布局搜索')
                    optCfg.solver='supports'; optCfg.max_group_size=6;
                    optCfg.support_slice_seconds=120;
                    optCfg.support_max_slice_seconds=300;
                elseif diagnostic
                    optCfg.solver='diagnostic';
                    optCfg.diagnostic_compare=false;
                    optCfg.diagnostic_persistent=true;
                    optCfg.diagnostic_branch_time=min(30,optCfg.max_time/3);
                    optCfg.diagnostic_latest_path=fullfile(app.ProjectRoot,'Optim_diagnosis_latest.json');
                    if ~isempty(app.DiagnosticResumeFile), optCfg.resume_file=app.DiagnosticResumeFile; end
                elseif ~app.DiagnosticTrusted || ~strcmp(scopeKey,app.DiagnosticScopeKey) || ...
                        ~strcmp(app.DiagnosticRecord.status,'necessary_conditions_sat') || ...
                        ~isfield(app.DiagnosticRecord,'geometry_checkpoint_file')
                    error('请先在当前约束下完成完整组选必要条件的搜索和精确复核。');
                elseif isstruct(app.OptimResult) && isfield(app.OptimResult,'constraints') && ...
                        strcmp(scopeKey,app.GeometryScopeKey)
                    optCfg.seed = mod(app.OptimResult.constraints.seed+1,2^32);
                    if isfield(app.OptimResult,'output_file') && isfile(app.OptimResult.output_file)
                        optCfg.resume_file = app.OptimResult.output_file;
                    end
                else
                    optCfg.resume_file=app.DiagnosticRecord.geometry_checkpoint_file;
                end
                app.beginOptimizationProgress(optCfg.start_count);
                runCleanup = onCleanup(@()app.recoverInterruptedSearch()); %#ok<NASGU>
                optCfg.progress_callback = @(info) ...
                    app.updateOptimizationProgress(info);
                evalc('result = Optim_Algorithm(params, optCfg);');
                if diagnostic
                    app.finishOptimizationProgress(false);
                    app.updateReport(string(result.message)+"｜诊断报告："+string(result.diagnostic_file));
                    if isfield(result.solver_evidence,'diagnostic_report_file')
                        app.acceptDiagnosticRecord(result.solver_evidence.diagnostic_report_file,true);
                    else
                        app.restoreLatestDiagnostic();
                        app.setStatus(result.message);
                    end
                    return;
                end
                app.OptimResult = result;
                app.GeometryScopeKey = scopeKey;
                if ~result.feasible
                    reportLine = string(result.message);
                    if strlength(result.output_file) > 0
                        reportLine = reportLine + "｜搜索记录：" + result.output_file;
                    end
                    if strlength(result.diagnostic_file) > 0
                        reportLine = reportLine + "｜诊断报告：" + result.diagnostic_file;
                    end
                    app.updateReport(reportLine);
                    app.finishOptimizationProgress(false);
                    app.ProgressLabel.Text = sprintf('搜索结束｜耗时 %.1f s',result.elapsed_time);
                    app.setStatus(sprintf('未生成可行新布局，当前布局保持不变｜状态：%s｜最高覆盖%d/156',result.status,result.best_coverage));
                    if strcmp(optCfg.solver,'supports')
                        app.StartOptimizationButton_3.Text='继续拓扑搜索';
                        app.setStatus(result.message);
                        if isfield(result.solver_evidence,'support_total')
                            e=result.solver_evidence;
                            app.ProgressLabel.Text=sprintf('已保存｜已访问 %d/%d 拓扑｜累计 %d 次轮转｜点击继续；不代表无解',e.support_seen,e.support_total,e.support_visits);
                            if isfield(e,'search_metrics')
                                m=e.search_metrics;
                                app.ProgressLabel.Text=sprintf('已保存｜新直接几何%d次｜计算%d次｜节点%d｜候选检查%d｜最高%d/156',m.direct_calls,m.direct_evaluations,m.nodes_since_upgrade,m.candidate_checks,result.best_coverage);
                            end
                        end
                    end
                    return;
                end
                params.Num = size(result.B_opt, 2);
                params.B_all = result.B_opt;
                params.r_all = result.r_opt;
                app.CurrentParams = params;
                name = app.addLayoutEntry('optimized', result.B_opt, result.r_opt);
                app.LayoutViewTabGroup.SelectedTab = app.Layout3DTab;
                reportLine = "首个可行布局：" + string(name) + ...
                    "｜来源：" + string(result.source) + ...
                    "｜最弱故障轴归一化能力：" + string(result.fault_min_normalized_capacity) + ...
                    sprintf("｜计算时间：%.3f s｜", result.elapsed_time) + ...
                    string(result.message);
                if isfield(result, 'output_file') && strlength(string(result.output_file)) > 0
                    reportLine = reportLine + "｜结果文件：" + string(result.output_file);
                end
                app.updateReport(reportLine);
                app.finishOptimizationProgress(true);
                app.setStatus('首个可行布局已生成｜156项及调用器复核通过，未进行能力最大化');
                if strcmp(optCfg.solver,'supports')
                    app.StartOptimizationButton_3.Text='已验证完整可行布局';
                    app.StartOptimizationButton_3.Enable='off';
                end
            catch ME
                app.finishOptimizationProgress(false);
                if exist('diagnostic','var') && diagnostic
                    app.restoreLatestDiagnostic();
                end
                % 输入校验等尚未进入优化器的异常，也保存完整堆栈以便后续定位。
                try
                    errorFile = fullfile(app.ProjectRoot,['Optim_error_',char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')),'.txt']);
                    file = fopen(errorFile,'w','n','UTF-8');
                    if file < 0
                        error('RCDesigner:DiagnosticWrite','无法创建错误报告。');
                    end
                    closeFile = onCleanup(@()fclose(file));
                    fprintf(file,'%s\n',getReport(ME,'extended','hyperlinks','off'));
                    if exist('optCfg','var')
                        if isfield(optCfg,'progress_callback')
                            optCfg.progress_callback = [];
                        end
                        fprintf(file,'输入设置：%s\n',jsonencode(optCfg));
                    end
                    clear closeFile;
                    app.updateReport("搜索异常："+string(ME.message)+"｜错误报告："+string(errorFile));
                catch saveError
                    app.updateReport("搜索异常："+string(ME.message)+"｜报告未能保存："+string(saveError.message));
                end
                app.showError('可行布局搜索失败', ME);
            end
        end

        % Button pushed function: StartOptimizationButton_2
        function EvaluateButtonPushed(app, event)
            try
                app.setBusy('正在进行故障可重构判断...');
                params = app.readParamsFromUI();
                selectedLayout = app.selectedJudgmentLayout();
                params.Num = size(selectedLayout.B, 2);
                faultSets = app.getFaultCases(params);
                results = app.evaluateReconfigJudgment( ...
                    selectedLayout.name, selectedLayout.B, params, faultSets);
                app.ReconfigJudgmentResult = results;
                app.EvaluationTable.Data = app.judgmentToCell(results);
                app.highlightUnreconfigurableRows(results);

                comparisonLayouts = app.buildAllCompatibleLayoutSet(params.Num);
                faultNumbers = app.selectedSummaryFaultNumbers(params.Num);
                summary = RCPlotter.reconfigSummaryData( ...
                    params, comparisonLayouts, faultNumbers);
                app.JudgmentSummaryTable.ColumnName = summary.ColumnNames;
                app.JudgmentSummaryTable.Data = summary.Data;
                app.JudgmentViewTabGroup.SelectedTab = app.JudgmentDetailTab;
                app.updateReport("可重构判断完成：选择布局为" + ...
                    string(selectedLayout.name) + "，对比布局数量为" + ...
                    string(numel(comparisonLayouts)) + "。");
                app.setStatus(sprintf('可重构判断完成｜选择布局：%s｜对比%d个布局', ...
                    selectedLayout.name, numel(comparisonLayouts)));
            catch ME
                app.showError('可重构判断失败', ME);
            end
        end

        % Button pushed function: MetricsEvaluateButton
% 原实现保留（口径或界面已调整；冲突部分停用）：
%                     'Jt', NaN, 'Jf', NaN, 'Score', NaN), 0, 1);
% 原实现保留（口径或界面已调整；冲突部分停用）：
%                         layoutSet(layoutIndex).B, params, faultLevel);
% 原实现保留（口径或界面已调整；冲突部分停用）：
%                 app.SingleFaultInfoLabel.Text = singleFault.WeightText;
% 原实现保留（口径或界面已调整；冲突部分停用）：
%                     "个布局的控制能力、可诊断性、归一化指标和综合评价。");
% 旧口径保留：连续分配视图不再使用时序样本凸包。
%                     '策略能力为已验证端点；未证实不代表无解，12向通过不等于闭环成功。干扰须结合通过率阅读。'], ...
% 旧口径保留：连续分配视图不再使用时序样本凸包。
%                     "个布局的策略能力、硬件对照、参考指令及实际时序干扰。");
% 原说明保留：'实际/理论能力按连续分配计算；时序验证列另计脉宽和调度。图中各组选取并集，不填补方向间空隙。'
% 原策略明细显示实现保留：
%                 app.PolicyDetailTable.ColumnWidth = [{130,90,120},repmat({105},1,numel(policyTable.ColumnNames)-3)];
%                 app.PolicyInfoLabel.Text = sprintf(['%s｜参考单轴幅值：%.3g N / %.3g N·m；含36条姿轨组合指令。', ...
%                     '实际/理论能力按连续分配计算；多轴输出叠加且单台总占空比≤1。时序验证另计脉宽和调度。'], ...
%                     app.AllocationModeDropDown_2.Value,0.2*params.F_max,0.4*params.F_max);
        function MetricsEvaluateButtonPushed(app, event)
            try
                app.setBusy('正在计算可重构设计指标...');
                params = app.readParamsFromUI();
                layoutSet = app.buildAllCompatibleLayoutSet(size(app.CurrentB, 2));
                params.Num = size(layoutSet(1).B, 2);
                [~, faultCases] = app.selectedMetricsFaultSet(params.Num);
                faultLevels = cellfun(@numel, faultCases);
                faultLevel = max(faultLevels);
                summaries = repmat(struct('Name', '', 'Jc', NaN, ...
                    'JcForce', NaN, 'JcTorque', NaN, 'Jo', NaN, ...
                    'Jt', NaN, 'Jf', NaN, 'Score', NaN, 'PolicyEvaluation',struct()), 0, 1);
                for layoutIndex = 1:numel(layoutSet)
                    item = app.evaluateLayoutSummary(layoutSet(layoutIndex).name, ...
                        layoutSet(layoutIndex).B, params, faultLevel, faultCases);
                    summaries(end + 1, 1) = item; %#ok<AGROW>
                end
                app.EvaluationResult = summaries;

                % 同一故障台数只评价一次，各子页筛选自己的确切故障编号。
                comparisonLevels = unique(faultLevels);
                comparisons = cell(size(comparisonLevels));
                for levelIndex = 1:numel(comparisonLevels)
                    comparisons{levelIndex} = RCPlotter.metricComparisonData( ...
                        params, layoutSet, comparisonLevels(levelIndex));
                end

                app.resetMetricsComparisonTabs();
                parentTabs = {app.MetricsChartsTab, ...
                    app.MetricsControlTab, app.MetricsAnglesTab};
                baseGrids = {app.MetricsChartsGrid, ...
                    app.MetricsControlGrid, app.MetricsAnglesGrid};
                caseGroups = cell(1, numel(parentTabs));
                for groupIndex = 1:numel(parentTabs)
                    host = uigridlayout(parentTabs{groupIndex}, [1, 1]);
                    host.Padding = [0, 0, 0, 0];
                    host.Tag = 'RCMetricsComparisonHost';
                    caseGroups{groupIndex} = uitabgroup(host);
                    caseGroups{groupIndex}.Tag = 'RCComparisonRuntime';
                end
                baseAngleAxes = app.prepareDiagnosticAxes(numel(layoutSet));

                for caseIndex = 1:numel(faultCases)
                    faultyIndices = faultCases{caseIndex};
                    if isempty(faultyIndices)
                        conditionText = '标况';
                    else
                        conditionText = ['故障', app.faultSetText(faultyIndices)];
                    end
                    app.StatusLabel.Text = sprintf('正在绘制可重构评价｜%s｜%d/%d', ...
                        conditionText, caseIndex, numel(faultCases));
                    caseGrids = cell(1, numel(caseGroups));
                    for groupIndex = 1:numel(caseGroups)
                        caseTab = uitab(caseGroups{groupIndex}, 'Title', conditionText);
                        if caseIndex == 1
                            caseGrids{groupIndex} = baseGrids{groupIndex};
                            caseGrids{groupIndex}.Parent = caseTab;
                        else
                            caseGrids{groupIndex} = uigridlayout(caseTab);
                        end
                    end
                    if caseIndex == 1
                        metricAxes = [app.MetricsJcAxes, app.MetricsJoAxes, ...
                            app.MetricsJtAxes, app.MetricsJfAxes];
                        controlAxes = [app.MetricsForceAxes, app.MetricsTorqueAxes];
                        angleAxes = baseAngleAxes;
                    else
                        caseGrids{1}.RowHeight = {'1x', '1x'};
                        caseGrids{1}.ColumnWidth = {'1x', '1x'};
                        metricAxes = gobjects(1, 4);
                        for metricIndex = 1:4
                            metricAxes(metricIndex) = uiaxes(caseGrids{1});
                            metricAxes(metricIndex).Layout.Row = ceil(metricIndex / 2);
                            metricAxes(metricIndex).Layout.Column = mod(metricIndex - 1, 2) + 1;
                        end
                        caseGrids{2}.RowHeight = {'1x'};
                        caseGrids{2}.ColumnWidth = {'1x', '1x'};
                        controlAxes = gobjects(1, 2);
                        for axesIndex = 1:2
                            controlAxes(axesIndex) = uiaxes(caseGrids{2});
                            controlAxes(axesIndex).Layout.Row = 1;
                            controlAxes(axesIndex).Layout.Column = axesIndex;
                        end
                        columnCount = min(3, numel(layoutSet));
                        rowCount = ceil(numel(layoutSet) / columnCount);
                        caseGrids{3}.ColumnWidth = repmat({'1x'}, 1, columnCount);
                        caseGrids{3}.RowHeight = repmat({'1x'}, 1, rowCount);
                        angleAxes = gobjects(1, numel(layoutSet));
                        for layoutIndex = 1:numel(layoutSet)
                            angleAxes(layoutIndex) = uiaxes(caseGrids{3});
                            angleAxes(layoutIndex).Layout.Row = ceil(layoutIndex / columnCount);
                            angleAxes(layoutIndex).Layout.Column = mod(layoutIndex - 1, columnCount) + 1;
                        end
                    end
                    levelIndex = find(comparisonLevels == numel(faultyIndices), 1);
                    comparison = comparisons{levelIndex};
                    selectedRow = app.findFaultSetRow( ...
                        comparison.Evaluations{1}.FaultSets, faultyIndices);
                    comparison.MetricValues = comparison.MetricValues(selectedRow, :, :);
                    comparison.FaultLabels = comparison.FaultLabels(selectedRow);
                    for metricIndex = 1:4
                        RCPlotter.plotMetricBars( ...
                            metricAxes(metricIndex), comparison, metricIndex);
                    end
                    RCPlotter.plotControlEnvelopeComparison( ...
                        controlAxes(1), controlAxes(2), params, layoutSet, faultyIndices);
                    for layoutIndex = 1:numel(layoutSet)
                        options = struct('Title', sprintf('%s｜%s：6维向量夹角', ...
                            layoutSet(layoutIndex).name, conditionText));
                        RCPlotter.plotDirectionAngleMatrix( ...
                            angleAxes(layoutIndex), layoutSet(layoutIndex).B, ...
                            faultyIndices, options);
                    end
                    drawnow limitrate nocallbacks;
                end

                singleFault = RCPlotter.singleFaultEvaluationData(params, layoutSet, faultCases);
                app.SingleFaultTable.ColumnName = singleFault.ColumnNames;
                app.SingleFaultTable.Data = singleFault.Data;
                app.SingleFaultTable.ColumnWidth = 'auto';
                app.SingleFaultInfoLabel.Text = [singleFault.WeightText, ...
                    '。按当前策略计算；参考指令未全通过时不参与综合评分。平均分只含通过工况，综合评分不代替闭环仿真。'];
                for layoutIndex=1:numel(layoutSet)
                    app.EvaluationResult(layoutIndex).Score=mean(singleFault.Evaluations{layoutIndex}.Score,'omitnan');
                end
                % 姿轨干扰页已撤下：不再运行该页的额外测试指令或导出该表。
%                 policyTable = RCPlotter.policyComparisonData(params, layoutSet, faultCases);
%                 app.PolicyDetailTable.ColumnName = policyTable.ColumnNames;
%                 app.PolicyDetailTable.Data = policyTable.Data;
%                 app.PolicyDetailTable.ColumnWidth = {90,60,120,100,120,185,185,145,160,160,180};
%                 app.PolicyInfoLabel.Text = sprintf(['三策略同条件：各26条纯力(%.3g N)/纯力矩(%.3g Nm)，含单/双/三轴联动。', ...
%                     '逐时段比较反向连续总能力，取最不利比值；未扣除当前任务占用，不代表即时可抵消。', ...
%                     '指令或时序未通过时标记待复核，低干扰不等于控制有效。'], ...
%                     0.2*params.F_max,0.4*params.F_max);
%                 for layoutIndex=1:numel(layoutSet)
%                     app.EvaluationResult(layoutIndex).Interference=policyTable.Evaluations(layoutIndex,:);
%                 end
                app.MetricsViewTabGroup.SelectedTab = app.MetricsSingleFaultTab;
                faultText = app.faultConditionText();
                app.updateReport("可重构设计指标计算完成：" + ...
                    string(faultText) + "，已对比" + string(numel(layoutSet)) + ...
                    "个布局的实际/理论控制能力、可诊断性和综合指标。");
                app.setStatus(sprintf('可重构设计指标计算完成｜%s｜%d个布局', ...
                    faultText, numel(layoutSet)));
            catch ME
                app.showError('设计指标计算失败', ME);
            end
        end

        function resetMetricsComparisonTabs(app)
            parentTabs = {app.MetricsChartsTab, ...
                app.MetricsControlTab, app.MetricsAnglesTab};
            baseGrids = {app.MetricsChartsGrid, ...
                app.MetricsControlGrid, app.MetricsAnglesGrid};
            for index = 1:numel(parentTabs)
                % 保留 App Designer 的基础组件，只清除运行时工况子页。
                baseGrids{index}.Parent = parentTabs{index};
                baseGrids{index}.Visible = 'on';
                runtimeHosts = findall(parentTabs{index}, ...
                    'Tag', 'RCMetricsComparisonHost');
                if ~isempty(runtimeHosts)
                    delete(runtimeHosts);
                end
            end
        end

        % Button pushed function: GenerateAllocationButton
        function GenerateAllocationButtonPushed(app, event)
            try
                app.setBusy('正在生成推力器调用策略...');
                params = app.readParamsFromUI();
                layout = app.selectedAllocationLayout();
                params.Num = size(layout.B, 2);
                params.alloc_mode = app.mapAllocationMode(app.AllocationModeDropDown_2.Value);
                params.allocation_strategy = 'primary_backup';
                faultValue = app.FmaxField_6.Value;
                faultInputText = strtrim(char(string(faultValue)));
                explicitFaultCount = app.requestedFaultCount(params.Num);
                faultCases = app.getFaultCases( ...
                    params, faultValue, explicitFaultCount);
                if isempty(faultInputText) || numel(faultCases) > 1
                    faultDescription = sprintf('%s（%d种工况）', ...
                        app.faultConditionText(), numel(faultCases));
                else
                    faultDescription = app.faultSetText(faultCases{1});
                end
                strategy = app.buildAllocationStrategyComparison( ...
                    params, app.buildComparisonLayoutSet(layout), faultCases);
                app.updateAllocationStrategyTable(strategy);
                app.AllocationViewTabGroup.SelectedTab = ...
                    app.AllocationStrategyTab;
                app.updateReport(sprintf(['调用策略生成完成：布局=%s，故障=%s，', ...
                    '复用策略=%s。'], layout.name, faultDescription, ...
                    app.AllocationModeDropDown_2.Value));
                if numel(faultCases) > 1
                    app.setStatus(sprintf('调用策略生成完成｜%s｜%d种工况', ...
                        app.faultConditionText(), numel(faultCases)));
                else
                    app.setStatus('调用策略生成完成');
                end
            catch ME
                app.showError('调用策略生成失败', ME);
            end
        end

        % Callback function: not associated with a component
        function RunSimulationButtonPushed(app, event)
            try
                app.setBusy('正在运行闭环仿真...');
                params = app.readParamsFromUI();
                params.alloc_mode = app.mapAllocationMode(app.AllocationModeDropDown_2.Value);
                params.allocation_strategy = 'primary_backup';
                selectedLayout = app.selectedSimulationLayout();
                layoutSet = app.buildComparisonLayoutSet(selectedLayout);
                simCfg = app.readSimulationConfig();
                logs = {};
                labels = {};
                allFaultSets = {};
                matrices = {};
                bodySizes = {};
                layoutIndices = [];
                caseSets = cell(1, numel(layoutSet));
                for layoutIndex = 1:numel(layoutSet)
                    layoutParams = params;
                    layoutParams.Num = size(layoutSet(layoutIndex).B, 2);
                    configured = app.getFaultCases(layoutParams);
                    caseSets{layoutIndex} = app.simulationFaultCases(layoutParams, configured);
                end
                totalCases = sum(cellfun(@numel, caseSets));
                for layoutIndex = 1:numel(layoutSet)
                    layout = layoutSet(layoutIndex);
                    faultSets = caseSets{layoutIndex};
                    for caseIndex = 1:numel(faultSets)
                        label = app.comparisonCaseLabel(layout.name, faultSets{caseIndex});
                        app.setBusy(sprintf('正在仿真 %d/%d｜%s', numel(logs) + 1, totalCases, label));
                        logs{end + 1} = app.runClosedloopCase(params, layout.B, simCfg, faultSets{caseIndex}); %#ok<AGROW>
                        labels{end + 1} = label; %#ok<AGROW>
                        allFaultSets{end + 1} = faultSets{caseIndex}; %#ok<AGROW>
                        matrices{end + 1} = params.F_max * layout.B; %#ok<AGROW>
                        halfSize = app.currentBodyHalfSize();
                        halfSize(1) = max(max(abs(layout.r(1, :))), 1e-6);
                        bodySizes{end + 1} = halfSize; %#ok<AGROW>
                        layoutIndices(end + 1) = layoutIndex; %#ok<AGROW>
                    end
                end
                app.SimulationLog = struct( ...
                    'LayoutSet', layoutSet, 'SelectedLayoutName', selectedLayout.name, ...
                    'SelectedFaults', {allFaultSets}, 'FaultTime', simCfg.faulty_time, ...
                    'ReuseStrategy', app.AllocationModeDropDown_2.Value, ...
                    'AllocationStrategy', '最简主备分配', ...
                    'Controller', struct('Kp_pos', simCfg.Kp_pos, 'Kd_pos', simCfg.Kd_pos, ...
                        'Kp_att', simCfg.Kp_att, 'Ki_att', simCfg.Ki_att, 'Kd_att', simCfg.Kd_att), ...
                    'StateLog', logs{1}, 'ComparisonLogs', {logs}, 'ComparisonLabels', {labels}, ...
                    'CaseLayoutIndices', layoutIndices, 'CaseBodyHalfSizes', {bodySizes}, ...
                    'FaultSets', {allFaultSets}, 'FaultLogs', {logs}, 'FaultLabels', {labels});
                app.plotSimulationResult(logs, labels);
                verification = app.buildActualInvocationVerification(matrices, logs, labels);
                app.updateActualInvocationVerification(verification);
                app.SimulationViewTabGroup.SelectedTab = app.SimulationPositionTab;
                app.updateReport(sprintf('闭环仿真完成：%d个布局，%d个工况；故障时刻=%.3g s，复用策略=%s。', ...
                    numel(layoutSet), numel(logs), simCfg.faulty_time, app.AllocationModeDropDown_2.Value));
                app.setStatus(sprintf('闭环仿真完成｜%d个布局｜%d个工况', numel(layoutSet), numel(logs)));
            catch ME
                app.showError('闭环仿真失败', ME);
            end
        end

        % Callback function
        function NavigateToParameterButtonPushed(app, event)
            app.navigateToPage('输入条件');
        end

        % Callback function
        function NavigateToOptimizationButtonPushed(app, event)
            app.navigateToPage('优化设计');
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
% 原实现保留（口径或界面已调整；冲突部分停用）：
%             app.MetricsControlTab.Title = '控制能力';
% 原实现保留（口径或界面已调整；冲突部分停用）：
%             app.EvaluationTable.ColumnName = {'故障编号'; '力裕度JcF'; '力矩裕度JcT'; '六维裕度Jc'; '判断结果'};
% 原实现保留（口径或界面已调整；冲突部分停用）：
%             app.Label8.Text = '可重构性判断';
% 原实现保留（口径或界面已调整；冲突部分停用）：
%             app.Label21.Text = '评价指标｜综合评价';
% 旧口径保留：连续分配视图不再使用时序样本凸包。
%             app.EvaluationTable.ColumnName = {'故障编号'; '策略力/N'; '策略力矩/Nm'; '归一化最弱轴'; '硬件纯力/N'; '硬件纯矩/Nm'; '策略方向/12'; '等脉宽方向/12'; '参考通过/%'; '判断结果'};
% 旧口径保留：连续分配视图不再使用时序样本凸包。
%             app.Label21.Text = '策略能力｜硬件对照｜实际干扰';
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Color = [0.96 0.97 0.98];
            app.UIFigure.Position = [70 70 1280 800];
            app.UIFigure.Name = 'RC-Designer 可重构性设计软件';

            % Create RootGrid
            app.RootGrid = uigridlayout(app.UIFigure);
            app.RootGrid.ColumnWidth = {'1x'};
            app.RootGrid.RowHeight = {72, '1x', 82};
            app.RootGrid.RowSpacing = 0;
            app.RootGrid.Padding = [0 0 0 0];

            % Create Panel1
            app.Panel1 = uipanel(app.RootGrid);
            app.Panel1.BorderType = 'none';
            app.Panel1.BackgroundColor = [0.9 0.93 0.95];
            app.Panel1.Layout.Row = 3;
            app.Panel1.Layout.Column = 1;

            % Create GridLayout2
            app.GridLayout2 = uigridlayout(app.Panel1);
            app.GridLayout2.ColumnWidth = {'1x', 250};
            app.GridLayout2.RowHeight = {24, 18, 20};
            app.GridLayout2.RowSpacing = 3;
            app.GridLayout2.Padding = [18 5 18 5];

            % Create Label2
            app.Label2 = uilabel(app.GridLayout2);
            app.Label2.HorizontalAlignment = 'left';
            app.Label2.FontColor = [0.38 0.46 0.53];
            app.Label2.Layout.Row = 3;
            app.Label2.Layout.Column = [1 2];
            app.Label2.Text = '搜索记录自动保存；超时不代表无解。';

            % Create StatusLabel
            app.StatusLabel = uilabel(app.GridLayout2);
            app.StatusLabel.Layout.Row = 1;
            app.StatusLabel.Layout.Column = [1 2];
            app.StatusLabel.Text = '就绪｜已载入默认参数';

            % Create ProgressLabel
            app.ProgressLabel = uilabel(app.GridLayout2);
            app.ProgressLabel.HorizontalAlignment = 'left';
            app.ProgressLabel.FontColor = [0.25 0.36 0.45];
            app.ProgressLabel.Layout.Row = 2;
            app.ProgressLabel.Layout.Column = 1;
            app.ProgressLabel.Text = '等待运行｜0%｜耗时 00:00';

            app.SearchProgressTrack = uipanel(app.GridLayout2);
            app.SearchProgressTrack.Layout.Row = 2;
            app.SearchProgressTrack.Layout.Column = 2;
            app.SearchProgressTrack.BorderType = 'none';
            app.SearchProgressTrack.BackgroundColor = [0.78 0.83 0.87];
            app.SearchProgressTrack.AutoResizeChildren = 'off';
            app.SearchProgressFill = uipanel(app.SearchProgressTrack);
            app.SearchProgressFill.Units = 'normalized';
            app.SearchProgressFill.Position = [0 0 eps 1];
            app.SearchProgressFill.BorderType = 'none';
            app.SearchProgressFill.BackgroundColor = [0.12 0.52 0.43];
            app.SearchProgressFill.Visible = 'off';

            % Create BodyGrid
            app.BodyGrid = uigridlayout(app.RootGrid);
            app.BodyGrid.ColumnWidth = {185, '1x'};
            app.BodyGrid.RowHeight = {'1x'};
            app.BodyGrid.ColumnSpacing = 12;
            app.BodyGrid.Padding = [12 12 12 10];
            app.BodyGrid.Layout.Row = 2;
            app.BodyGrid.Layout.Column = 1;
            app.BodyGrid.BackgroundColor = [0.96 0.97 0.98];

            % Create PageHostPanel
            app.PageHostPanel = uipanel(app.BodyGrid);
            app.PageHostPanel.AutoResizeChildren = 'off';
            app.PageHostPanel.BorderType = 'none';
            app.PageHostPanel.BackgroundColor = [0.96 0.97 0.98];
            app.PageHostPanel.SizeChangedFcn = createCallbackFcn(app, @PageHostPanelSizeChanged, true);
            app.PageHostPanel.Layout.Row = 1;
            app.PageHostPanel.Layout.Column = 2;

            % Create TabGroup
            app.TabGroup = uitabgroup(app.PageHostPanel);
            app.TabGroup.TabLocation = 'bottom';
            app.TabGroup.Position = [0 -31 1059 699];

            % Create SimulationTab
            app.SimulationTab = uitab(app.TabGroup);
            app.SimulationTab.Title = '闭环仿真';

            % Create GridLayout10
            app.GridLayout10 = uigridlayout(app.SimulationTab);
            app.GridLayout10.ColumnWidth = {'1x', 104, 104, 200};
            app.GridLayout10.RowHeight = {55, '1x'};
            app.GridLayout10.Padding = [22 18 22 18];

            % Create Label10
            app.Label10 = uilabel(app.GridLayout10);
            app.Label10.FontSize = 22;
            app.Label10.FontWeight = 'bold';
            app.Label10.FontColor = [0.1 0.27 0.42];
            app.Label10.Layout.Row = 1;
            app.Label10.Layout.Column = 1;
            app.Label10.Text = '闭环仿真验证';

            % Create SimulationViewTabGroup
            app.SimulationViewTabGroup = uitabgroup(app.GridLayout10);
            app.SimulationViewTabGroup.Layout.Row = 2;
            app.SimulationViewTabGroup.Layout.Column = [1 4];

            % Create SimulationScheduleTab
            app.SimulationScheduleTab = uitab(app.SimulationViewTabGroup);
            app.SimulationScheduleTab.Title = '推力器调用时序对比';

            % Create SimulationScheduleGrid
            app.SimulationScheduleGrid = uigridlayout(app.SimulationScheduleTab);
            app.SimulationScheduleGrid.ColumnWidth = {'1x'};
            app.SimulationScheduleGrid.RowHeight = {34, '1x'};
            app.SimulationScheduleGrid.Padding = [8 8 8 8];

            % Create SimulationScheduleAxes
            app.SimulationScheduleCaseTabGroup = uitabgroup(app.SimulationScheduleGrid);
            app.SimulationScheduleCaseTabGroup.Layout.Row = 2;
            app.SimulationNominalScheduleTab = uitab(app.SimulationScheduleCaseTabGroup);
            app.SimulationNominalScheduleTab.Title = '标况（原布局）';
            app.SimulationNominalScheduleGrid = uigridlayout(app.SimulationNominalScheduleTab, [1 1]);
            app.SimulationScheduleAxes = uiaxes(app.SimulationNominalScheduleGrid);
            app.SimulationScheduleAxes.Layout.Row = 1;
            app.SimulationScheduleAxes.Layout.Column = 1;

            % Create SimulationScheduleControlGrid
            app.SimulationScheduleControlGrid = uigridlayout(app.SimulationScheduleGrid);
            app.SimulationScheduleControlGrid.ColumnWidth = {'fit', 120, 'fit', 90, '1x'};
            app.SimulationScheduleControlGrid.RowHeight = {'1x'};
            app.SimulationScheduleControlGrid.Padding = [0 0 0 0];
            app.SimulationScheduleControlGrid.Layout.Row = 1;
            app.SimulationScheduleControlGrid.Layout.Column = 1;

            % Create SimulationScheduleStartTimeLabel
            app.SimulationScheduleStartTimeLabel = uilabel(app.SimulationScheduleControlGrid);
            app.SimulationScheduleStartTimeLabel.HorizontalAlignment = 'right';
            app.SimulationScheduleStartTimeLabel.Layout.Row = 1;
            app.SimulationScheduleStartTimeLabel.Layout.Column = 1;
            app.SimulationScheduleStartTimeLabel.Text = '起始时间 / s';

            % Create SimulationScheduleStartTimeField
            app.SimulationScheduleStartTimeField = uieditfield(app.SimulationScheduleControlGrid, 'numeric');
            app.SimulationScheduleStartTimeField.Limits = [0 Inf];
            app.SimulationScheduleStartTimeField.ValueDisplayFormat = '%.3f';
            app.SimulationScheduleStartTimeField.ValueChangedFcn = createCallbackFcn(app, @SimulationScheduleWindowValueChanged, true);
            app.SimulationScheduleStartTimeField.Tooltip = '仿真完成时自动定位至全周期脉宽最多的控制周期';
            app.SimulationScheduleStartTimeField.Layout.Row = 1;
            app.SimulationScheduleStartTimeField.Layout.Column = 2;

            % Create SimulationScheduleCycleCountLabel
            app.SimulationScheduleCycleCountLabel = uilabel(app.SimulationScheduleControlGrid);
            app.SimulationScheduleCycleCountLabel.HorizontalAlignment = 'right';
            app.SimulationScheduleCycleCountLabel.Layout.Row = 1;
            app.SimulationScheduleCycleCountLabel.Layout.Column = 3;
            app.SimulationScheduleCycleCountLabel.Text = '显示周期数';

            % Create SimulationScheduleCycleCountField
            app.SimulationScheduleCycleCountField = uieditfield(app.SimulationScheduleControlGrid, 'numeric');
            app.SimulationScheduleCycleCountField.Limits = [1 Inf];
            app.SimulationScheduleCycleCountField.ValueDisplayFormat = '%.0f';
            app.SimulationScheduleCycleCountField.ValueChangedFcn = createCallbackFcn(app, @SimulationScheduleWindowValueChanged, true);
            app.SimulationScheduleCycleCountField.Tooltip = '显示的连续控制周期数量';
            app.SimulationScheduleCycleCountField.Layout.Row = 1;
            app.SimulationScheduleCycleCountField.Layout.Column = 4;
            app.SimulationScheduleCycleCountField.Value = 5;

            % Create SimulationPulseTab
            app.SimulationPulseTab = uitab(app.SimulationViewTabGroup);
            app.SimulationPulseTab.Title = '推力器实际脉宽对比';

            % Create SimulationPulseGrid
            app.SimulationPulseHostGrid = uigridlayout(app.SimulationPulseTab, [1 1]);
            app.SimulationPulseHostGrid.Padding = [0 0 0 0];
            app.SimulationPulseCaseTabGroup = uitabgroup(app.SimulationPulseHostGrid);
            app.SimulationNominalPulseTab = uitab(app.SimulationPulseCaseTabGroup);
            app.SimulationNominalPulseTab.Title = '标况（原布局）';
            app.SimulationPulseGrid = uigridlayout(app.SimulationNominalPulseTab);
            app.SimulationPulseGrid.ColumnWidth = {'1x'};
            app.SimulationPulseGrid.RowHeight = {'1x'};
            app.SimulationPulseGrid.Padding = [8 8 8 8];
            app.SimulationPulseGrid.Scrollable = 'on';

            % Create SimulationPulsePlaceholderAxes
            app.SimulationPulsePlaceholderAxes = uiaxes(app.SimulationPulseGrid);
            app.SimulationPulsePlaceholderAxes.Layout.Row = 1;
            app.SimulationPulsePlaceholderAxes.Layout.Column = 1;
            app.SimulationPulsePlaceholderAxes.Visible = 'off';

            % Create SimulationTrajectoryTab
            app.SimulationTrajectoryTab = uitab(app.SimulationViewTabGroup);
            app.SimulationTrajectoryTab.Title = '位置姿态视图对比';

            % Create SimulationTrajectoryGrid
            app.SimulationTrajectoryGrid = uigridlayout(app.SimulationTrajectoryTab);
            app.SimulationTrajectoryGrid.ColumnWidth = {'1x'};
            app.SimulationTrajectoryGrid.RowHeight = {'1x'};
            app.SimulationTrajectoryGrid.Padding = [0 0 0 0];

            % Create SimulationTrajectoryCaseTabGroup
            app.SimulationTrajectoryCaseTabGroup = uitabgroup(app.SimulationTrajectoryGrid);
            app.SimulationTrajectoryCaseTabGroup.Layout.Row = 1;
            app.SimulationTrajectoryCaseTabGroup.Layout.Column = 1;

            % Create SimulationNominalTrajectoryTab
            app.SimulationNominalTrajectoryTab = uitab(app.SimulationTrajectoryCaseTabGroup);
            app.SimulationNominalTrajectoryTab.Title = '标况（原布局）';

            % Create SimulationNominalTrajectoryGrid
            app.SimulationNominalTrajectoryGrid = uigridlayout(app.SimulationNominalTrajectoryTab);
            app.SimulationNominalTrajectoryGrid.Padding = [8 8 8 8];

            % Create SimulationNominalTrajectory3DAxes
            app.SimulationNominalTrajectory3DAxes = uiaxes(app.SimulationNominalTrajectoryGrid);
            app.SimulationNominalTrajectory3DAxes.Layout.Row = 1;
            app.SimulationNominalTrajectory3DAxes.Layout.Column = 1;

            % Create SimulationNominalTrajectoryXYAxes
            app.SimulationNominalTrajectoryXYAxes = uiaxes(app.SimulationNominalTrajectoryGrid);
            app.SimulationNominalTrajectoryXYAxes.Layout.Row = 1;
            app.SimulationNominalTrajectoryXYAxes.Layout.Column = 2;

            % Create SimulationNominalTrajectoryXZAxes
            app.SimulationNominalTrajectoryXZAxes = uiaxes(app.SimulationNominalTrajectoryGrid);
            app.SimulationNominalTrajectoryXZAxes.Layout.Row = 2;
            app.SimulationNominalTrajectoryXZAxes.Layout.Column = 1;

            % Create SimulationNominalTrajectoryYZAxes
            app.SimulationNominalTrajectoryYZAxes = uiaxes(app.SimulationNominalTrajectoryGrid);
            app.SimulationNominalTrajectoryYZAxes.Layout.Row = 2;
            app.SimulationNominalTrajectoryYZAxes.Layout.Column = 2;

            % Create SimulationCommandTab
            app.SimulationCommandTab = uitab(app.SimulationViewTabGroup);
            app.SimulationCommandTab.Title = '推力器实际输出对比';

            % Create SimulationCommandGrid
            app.SimulationCommandGrid = uigridlayout(app.SimulationCommandTab);
            app.SimulationCommandGrid.ColumnWidth = {'1x'};
            app.SimulationCommandGrid.RowHeight = {'1x'};
            app.SimulationCommandGrid.Padding = [0 0 0 0];

            % Create SimulationCommandCaseTabGroup
            app.SimulationCommandCaseTabGroup = uitabgroup(app.SimulationCommandGrid);
            app.SimulationCommandCaseTabGroup.Layout.Row = 1;
            app.SimulationCommandCaseTabGroup.Layout.Column = 1;

            % Create SimulationNominalCommandTab
            app.SimulationNominalCommandTab = uitab(app.SimulationCommandCaseTabGroup);
            app.SimulationNominalCommandTab.Title = '标况（原布局）';

            % Create SimulationNominalCommandGrid
            app.SimulationNominalCommandGrid = uigridlayout(app.SimulationNominalCommandTab);
            app.SimulationNominalCommandGrid.ColumnWidth = {'1x', '1x', '1x'};
            app.SimulationNominalCommandGrid.Padding = [8 8 8 8];

            % Create SimulationForceXAxes
            app.SimulationForceXAxes = uiaxes(app.SimulationNominalCommandGrid);
            app.SimulationForceXAxes.Layout.Row = 1;
            app.SimulationForceXAxes.Layout.Column = 1;

            % Create SimulationForceYAxes
            app.SimulationForceYAxes = uiaxes(app.SimulationNominalCommandGrid);
            app.SimulationForceYAxes.Layout.Row = 1;
            app.SimulationForceYAxes.Layout.Column = 2;

            % Create SimulationForceZAxes
            app.SimulationForceZAxes = uiaxes(app.SimulationNominalCommandGrid);
            app.SimulationForceZAxes.Layout.Row = 1;
            app.SimulationForceZAxes.Layout.Column = 3;

            % Create SimulationTorqueXAxes
            app.SimulationTorqueXAxes = uiaxes(app.SimulationNominalCommandGrid);
            app.SimulationTorqueXAxes.Layout.Row = 2;
            app.SimulationTorqueXAxes.Layout.Column = 1;

            % Create SimulationTorqueYAxes
            app.SimulationTorqueYAxes = uiaxes(app.SimulationNominalCommandGrid);
            app.SimulationTorqueYAxes.Layout.Row = 2;
            app.SimulationTorqueYAxes.Layout.Column = 2;

            % Create SimulationTorqueZAxes
            app.SimulationTorqueZAxes = uiaxes(app.SimulationNominalCommandGrid);
            app.SimulationTorqueZAxes.Layout.Row = 2;
            app.SimulationTorqueZAxes.Layout.Column = 3;

            % Create SimulationPositionTab
            app.SimulationPositionTab = uitab(app.SimulationViewTabGroup);
            app.SimulationPositionTab.Title = '状态响应对比';

            % Create SimulationPositionGrid
            app.SimulationPositionHostGrid = uigridlayout(app.SimulationPositionTab, [1 1]);
            app.SimulationPositionHostGrid.Padding = [0 0 0 0];
            app.SimulationPositionCaseTabGroup = uitabgroup(app.SimulationPositionHostGrid);
            app.SimulationNominalPositionTab = uitab(app.SimulationPositionCaseTabGroup);
            app.SimulationNominalPositionTab.Title = '标况（原布局）';
            app.SimulationPositionGrid = uigridlayout(app.SimulationNominalPositionTab);
            app.SimulationPositionGrid.Padding = [8 8 8 8];

            % Create SimulationAxes
            app.SimulationAxes = uiaxes(app.SimulationPositionGrid);
            app.SimulationAxes.Layout.Row = 1;
            app.SimulationAxes.Layout.Column = 1;

            % Create SimulationAttitudeAxes
            app.SimulationAttitudeAxes = uiaxes(app.SimulationPositionGrid);
            app.SimulationAttitudeAxes.Layout.Row = 1;
            app.SimulationAttitudeAxes.Layout.Column = 2;

            % Create SimulationPositionErrorAxes
            app.SimulationPositionErrorAxes = uiaxes(app.SimulationPositionGrid);
            app.SimulationPositionErrorAxes.Layout.Row = 2;
            app.SimulationPositionErrorAxes.Layout.Column = 1;

            % Create SimulationAttitudeErrorAxes
            app.SimulationAttitudeErrorAxes = uiaxes(app.SimulationPositionGrid);
            app.SimulationAttitudeErrorAxes.Layout.Row = 2;
            app.SimulationAttitudeErrorAxes.Layout.Column = 2;

            % Create GenerateAllocationButton_2
            app.GenerateAllocationButton_2 = uibutton(app.GridLayout10, 'push');
            app.GenerateAllocationButton_2.Layout.Row = 1;
            app.GenerateAllocationButton_2.Layout.Column = 4;
            app.GenerateAllocationButton_2.Text = '运行闭环仿真';

            % Create StartOptimizationButton_10
            app.StartOptimizationButton_10 = uibutton(app.GridLayout10, 'push');
            app.StartOptimizationButton_10.Layout.Row = 1;
            app.StartOptimizationButton_10.Layout.Column = 3;
            app.StartOptimizationButton_10.Text = '布局选择';

            % Create StartOptimizationButton_11
            app.StartOptimizationButton_11 = uibutton(app.GridLayout10, 'push');
            app.StartOptimizationButton_11.Layout.Row = 1;
            app.StartOptimizationButton_11.Layout.Column = 2;
            app.StartOptimizationButton_11.Text = '条件设置';

            % Create AllocationTab
            app.AllocationTab = uitab(app.TabGroup);
            app.AllocationTab.Title = '调用策略';

            % Create GridLayout9
            app.GridLayout9 = uigridlayout(app.AllocationTab);
            app.GridLayout9.ColumnWidth = {'1x', 104, 104, 200};
            app.GridLayout9.RowHeight = {55, '1x'};
            app.GridLayout9.Padding = [22 18 22 18];

            % Create Label9
            app.Label9 = uilabel(app.GridLayout9);
            app.Label9.FontSize = 22;
            app.Label9.FontWeight = 'bold';
            app.Label9.FontColor = [0.1 0.27 0.42];
            app.Label9.Layout.Row = 1;
            app.Label9.Layout.Column = 1;
            app.Label9.Text = '推力器调用策略';

            % Create AllocationViewTabGroup
            app.AllocationViewTabGroup = uitabgroup(app.GridLayout9);
            app.AllocationViewTabGroup.Layout.Row = 2;
            app.AllocationViewTabGroup.Layout.Column = [1 4];

            % Create AllocationStrategyTab
            app.AllocationStrategyTab = uitab(app.AllocationViewTabGroup);
            app.AllocationStrategyTab.Title = '六轴推力器分配';

            % Create AllocationStrategyGrid
            app.AllocationStrategyGrid = uigridlayout(app.AllocationStrategyTab);
            app.AllocationStrategyGrid.ColumnWidth = {'1x'};
            app.AllocationStrategyGrid.RowHeight = {'1x'};

            % Create AllocationStrategyTable
            app.AllocationStrategyTable = uitable(app.AllocationStrategyGrid);
            app.AllocationStrategyTable.ColumnName = {'控制类型'; '标况下调用（原布局）'; '故障下调用（原布局）'; '调用状态（原布局）'};
            app.AllocationStrategyTable.RowName = {};
            app.AllocationStrategyTable.Layout.Row = 1;
            app.AllocationStrategyTable.Layout.Column = 1;

            % Create GenerateAllocationButton
            app.GenerateAllocationButton = uibutton(app.GridLayout9, 'push');
            app.GenerateAllocationButton.ButtonPushedFcn = createCallbackFcn(app, @GenerateAllocationButtonPushed, true);
            app.GenerateAllocationButton.Layout.Row = 1;
            app.GenerateAllocationButton.Layout.Column = 4;
            app.GenerateAllocationButton.Text = '执行调用策略';

            % Create StartOptimizationButton_8
            app.StartOptimizationButton_8 = uibutton(app.GridLayout9, 'push');
            app.StartOptimizationButton_8.Layout.Row = 1;
            app.StartOptimizationButton_8.Layout.Column = 3;
            app.StartOptimizationButton_8.Text = '布局选择';

            % Create StartOptimizationButton_9
            app.StartOptimizationButton_9 = uibutton(app.GridLayout9, 'push');
            app.StartOptimizationButton_9.Layout.Row = 1;
            app.StartOptimizationButton_9.Layout.Column = 2;
            app.StartOptimizationButton_9.Text = '条件设置';

            % Create ReconfigDesignTab
            app.ReconfigDesignTab = uitab(app.TabGroup);
            app.ReconfigDesignTab.Title = '可重构评价';

            % Create GridLayout25
            app.GridLayout25 = uigridlayout(app.ReconfigDesignTab);
            app.GridLayout25.ColumnWidth = {'1.1x', 104, 104, 200};
            app.GridLayout25.RowHeight = {55, '1x'};
            app.GridLayout25.Padding = [22 18 22 18];

            % Create MetricsEvaluateButton
            app.MetricsEvaluateButton = uibutton(app.GridLayout25, 'push');
            app.MetricsEvaluateButton.ButtonPushedFcn = createCallbackFcn(app, @MetricsEvaluateButtonPushed, true);
            app.MetricsEvaluateButton.Layout.Row = 1;
            app.MetricsEvaluateButton.Layout.Column = 4;
            app.MetricsEvaluateButton.Text = '可重构性评价';

            % Create Label41
            app.Label41 = uilabel(app.GridLayout25);
            app.Label41.FontSize = 22;
            app.Label41.FontWeight = 'bold';
            app.Label41.FontColor = [0.1 0.27 0.42];
            app.Label41.Layout.Row = 1;
            app.Label41.Layout.Column = 1;
            app.Label41.Text = '可重构性评价指标';

            % Create MetricsViewTabGroup
            app.MetricsViewTabGroup = uitabgroup(app.GridLayout25);
            app.MetricsViewTabGroup.Layout.Row = 2;
            app.MetricsViewTabGroup.Layout.Column = [1 4];

            % Create MetricsSingleFaultTab
            app.MetricsSingleFaultTab = uitab(app.MetricsViewTabGroup);
            app.MetricsSingleFaultTab.Title = '综合评价';

            % Create MetricsSingleFaultGrid
            app.MetricsSingleFaultGrid = uigridlayout(app.MetricsSingleFaultTab);
            app.MetricsSingleFaultGrid.ColumnWidth = {'1x'};
            app.MetricsSingleFaultGrid.RowHeight = {44, '1x'};

            % Create SingleFaultInfoLabel
            app.SingleFaultInfoLabel = uilabel(app.MetricsSingleFaultGrid);
            app.SingleFaultInfoLabel.Layout.Row = 1;
            app.SingleFaultInfoLabel.Layout.Column = 1;
            app.SingleFaultInfoLabel.Text = '';
            app.SingleFaultInfoLabel.WordWrap = 'on';

            % Create SingleFaultTable
            app.SingleFaultTable = uitable(app.MetricsSingleFaultGrid);
            app.SingleFaultTable.ColumnName = {'故障推力器编号'};
            app.SingleFaultTable.RowName = {};
            app.SingleFaultTable.Layout.Row = 2;
            app.SingleFaultTable.Layout.Column = 1;

            % 策略明细单独展示，保留原综合评价表及全部图页。
            % 姿轨干扰组件停用，原实现保留供后续闭环仿真使用。
%             app.MetricsPolicyTab = uitab(app.MetricsViewTabGroup);
%             % 原页名保留：策略能力与干扰。
%             app.MetricsPolicyTab.Title = '姿轨干扰';
%             app.MetricsPolicyGrid = uigridlayout(app.MetricsPolicyTab,[2,1]);
%             % 原行高：app.MetricsPolicyGrid.RowHeight = {60,'1x'};
%             app.MetricsPolicyGrid.RowHeight = {76,'1x'};
%             app.PolicyInfoLabel = uilabel(app.MetricsPolicyGrid,'WordWrap','on');
%             app.PolicyInfoLabel.Layout.Row = 1;
%             app.PolicyDetailTable = uitable(app.MetricsPolicyGrid,'RowName',{});
%             app.PolicyDetailTable.Layout.Row = 2;
%

            % Create MetricsChartsTab
            app.MetricsChartsTab = uitab(app.MetricsViewTabGroup);
            app.MetricsChartsTab.Title = '归一化指标';

            % Create MetricsChartsGrid
            app.MetricsChartsGrid = uigridlayout(app.MetricsChartsTab);
            app.MetricsChartsGrid.Padding = [8 8 8 8];

            % Create MetricsJcAxes
            app.MetricsJcAxes = uiaxes(app.MetricsChartsGrid);
            app.MetricsJcAxes.Layout.Row = 1;
            app.MetricsJcAxes.Layout.Column = 1;

            % Create MetricsJoAxes
            app.MetricsJoAxes = uiaxes(app.MetricsChartsGrid);
            app.MetricsJoAxes.Layout.Row = 1;
            app.MetricsJoAxes.Layout.Column = 2;

            % Create MetricsJtAxes
            app.MetricsJtAxes = uiaxes(app.MetricsChartsGrid);
            app.MetricsJtAxes.Layout.Row = 2;
            app.MetricsJtAxes.Layout.Column = 1;

            % Create MetricsJfAxes
            app.MetricsJfAxes = uiaxes(app.MetricsChartsGrid);
            app.MetricsJfAxes.Layout.Row = 2;
            app.MetricsJfAxes.Layout.Column = 2;

            % Create MetricsAnglesTab
            app.MetricsAnglesTab = uitab(app.MetricsViewTabGroup);
            app.MetricsAnglesTab.Title = '可诊断性';

            % Create MetricsAnglesGrid
            app.MetricsAnglesGrid = uigridlayout(app.MetricsAnglesTab);
            app.MetricsAnglesGrid.ColumnWidth = {'1x'};
            app.MetricsAnglesGrid.RowHeight = {'1x'};
            app.MetricsAnglesGrid.Padding = [8 8 8 8];

            % Create MetricsAngleAxes1
            app.MetricsAngleAxes1 = uiaxes(app.MetricsAnglesGrid);
            app.MetricsAngleAxes1.Layout.Row = 1;
            app.MetricsAngleAxes1.Layout.Column = 1;

            % Create MetricsAngleAxes2
            app.MetricsAngleAxes2 = uiaxes(app.MetricsAnglesGrid);
            app.MetricsAngleAxes2.Layout.Row = 1;
            app.MetricsAngleAxes2.Layout.Column = 1;
            app.MetricsAngleAxes2.Visible = 'off';

            % Create MetricsAngleAxes3
            app.MetricsAngleAxes3 = uiaxes(app.MetricsAnglesGrid);
            app.MetricsAngleAxes3.Layout.Row = 1;
            app.MetricsAngleAxes3.Layout.Column = 1;
            app.MetricsAngleAxes3.Visible = 'off';

            % Create MetricsControlTab
            app.MetricsControlTab = uitab(app.MetricsViewTabGroup);
            app.MetricsControlTab.Title = '控制能力包络';

            % Create MetricsControlGrid
            app.MetricsControlGrid = uigridlayout(app.MetricsControlTab);
            app.MetricsControlGrid.RowHeight = {'1x'};
            app.MetricsControlGrid.Padding = [8 8 8 8];

            % Create MetricsTorqueAxes
            app.MetricsTorqueAxes = uiaxes(app.MetricsControlGrid);
            app.MetricsTorqueAxes.Layout.Row = 1;
            app.MetricsTorqueAxes.Layout.Column = 2;

            % Create MetricsForceAxes
            app.MetricsForceAxes = uiaxes(app.MetricsControlGrid);
            app.MetricsForceAxes.Layout.Row = 1;
            app.MetricsForceAxes.Layout.Column = 1;

            % Create StartOptimizationButton_6
            app.StartOptimizationButton_6 = uibutton(app.GridLayout25, 'push');
            app.StartOptimizationButton_6.Layout.Row = 1;
            app.StartOptimizationButton_6.Layout.Column = 3;
            app.StartOptimizationButton_6.Text = '布局选择';

            % Create StartOptimizationButton_7
            app.StartOptimizationButton_7 = uibutton(app.GridLayout25, 'push');
            app.StartOptimizationButton_7.Layout.Row = 1;
            app.StartOptimizationButton_7.Layout.Column = 2;
            app.StartOptimizationButton_7.Text = '条件设置';

            % Create EvaluationTab
            app.EvaluationTab = uitab(app.TabGroup);
            app.EvaluationTab.Title = '可重构判断';

            % Create GridLayout8
            app.GridLayout8 = uigridlayout(app.EvaluationTab);
            app.GridLayout8.ColumnWidth = {'1x', 104, 104, 200};
            app.GridLayout8.RowHeight = {55, '1x'};
            app.GridLayout8.Padding = [22 18 22 18];

            % Create JudgmentViewTabGroup
            app.JudgmentViewTabGroup = uitabgroup(app.GridLayout8);
            app.JudgmentViewTabGroup.Layout.Row = 2;
            app.JudgmentViewTabGroup.Layout.Column = [1 4];

            % Create JudgmentSummaryTab
            app.JudgmentSummaryTab = uitab(app.JudgmentViewTabGroup);
            app.JudgmentSummaryTab.Title = '不同布局对比';

            % Create JudgmentSummaryGrid
            app.JudgmentSummaryGrid = uigridlayout(app.JudgmentSummaryTab);
            app.JudgmentSummaryGrid.ColumnWidth = {'1x'};
            app.JudgmentSummaryGrid.RowHeight = {'1x'};

            % Create JudgmentSummaryTable
            app.JudgmentSummaryTable = uitable(app.JudgmentSummaryGrid);
            app.JudgmentSummaryTable.ColumnName = {'布局方案'; '故障数量'; '故障状态'; '可重构数'; '不可重构数'};
            app.JudgmentSummaryTable.RowName = {};
            app.JudgmentSummaryTable.Layout.Row = 1;
            app.JudgmentSummaryTable.Layout.Column = 1;

            % Create JudgmentDetailTab
            app.JudgmentDetailTab = uitab(app.JudgmentViewTabGroup);
            app.JudgmentDetailTab.Title = '可重构性判断';

            % Create JudgmentDetailGrid
            app.JudgmentDetailGrid = uigridlayout(app.JudgmentDetailTab);
            app.JudgmentDetailGrid.ColumnWidth = {'1x'};
            app.JudgmentDetailGrid.RowHeight = {'1x'};

            % Create EvaluationTable
            app.EvaluationTable = uitable(app.JudgmentDetailGrid);
%             app.EvaluationTable.ColumnName = {'故障编号'; '时序验证力/N'; '时序验证力矩/Nm'; '归一化最弱轴'; '理论单轴力/N'; '理论单轴力矩/Nm'; '时序验证方向/12'; '等脉宽方向/12'; '参考通过/%'; '判断结果'};
            app.EvaluationTable.ColumnName = {'故障编号'; '实际纯轴力/N'; '实际纯轴力矩/Nm'; '归一化最弱轴'; '理论纯轴力/N'; '理论纯轴力矩/Nm'; '连续方向/12'; '原点六维内点判断'};
            app.EvaluationTable.RowName = {};
            app.EvaluationTable.Layout.Row = 1;
            app.EvaluationTable.Layout.Column = 1;

            % Create Label8
            app.Label8 = uilabel(app.GridLayout8);
            app.Label8.FontSize = 22;
            app.Label8.FontWeight = 'bold';
            app.Label8.FontColor = [0.1 0.27 0.42];
            app.Label8.Layout.Row = 1;
            app.Label8.Layout.Column = 1;
            app.Label8.Text = '当前策略的12方向与参考指令验证';

            % Create StartOptimizationButton_2
            app.StartOptimizationButton_2 = uibutton(app.GridLayout8, 'push');
            app.StartOptimizationButton_2.ButtonPushedFcn = createCallbackFcn(app, @EvaluateButtonPushed, true);
            app.StartOptimizationButton_2.Layout.Row = 1;
            app.StartOptimizationButton_2.Layout.Column = 4;
            app.StartOptimizationButton_2.Text = '可重构性判断';

            % Create StartOptimizationButton_4
            app.StartOptimizationButton_4 = uibutton(app.GridLayout8, 'push');
            app.StartOptimizationButton_4.Layout.Row = 1;
            app.StartOptimizationButton_4.Layout.Column = 3;
            app.StartOptimizationButton_4.Text = '布局选择';

            % Create StartOptimizationButton_5
            app.StartOptimizationButton_5 = uibutton(app.GridLayout8, 'push');
            app.StartOptimizationButton_5.Layout.Row = 1;
            app.StartOptimizationButton_5.Layout.Column = 2;
            app.StartOptimizationButton_5.Text = '条件设置';

            % Create OptimizationTab
            app.OptimizationTab = uitab(app.TabGroup);
            app.OptimizationTab.Title = '优化计算';

            % Create GridLayout7
            app.GridLayout7 = uigridlayout(app.OptimizationTab);
            app.GridLayout7.ColumnWidth = {400, '1x', 104, 104};
            app.GridLayout7.RowHeight = {55, 260, 'fit', 55};
            app.GridLayout7.Padding = [22 18 22 18];

            % Create LayoutTemplateDropDown
            app.LayoutTemplateDropDown = uidropdown(app.GridLayout7);
            app.LayoutTemplateDropDown.Items = {'原布局'};
            app.LayoutTemplateDropDown.ValueChangedFcn = createCallbackFcn(app, @LayoutTemplateDropDownValueChanged, true);
            app.LayoutTemplateDropDown.Layout.Row = 1;
            app.LayoutTemplateDropDown.Layout.Column = [3 4];
            app.LayoutTemplateDropDown.Value = '原布局';

            % Create ApplyLayoutButton_2
            app.ApplyLayoutButton_2 = uibutton(app.GridLayout7, 'push');
            app.ApplyLayoutButton_2.ButtonPushedFcn = createCallbackFcn(app, @ImportLayoutButtonPushed, true);
            app.ApplyLayoutButton_2.Layout.Row = 4;
            app.ApplyLayoutButton_2.Layout.Column = 3;
            app.ApplyLayoutButton_2.Text = '导入布局';

            % Create LayoutViewTabGroup
            app.LayoutViewTabGroup = uitabgroup(app.GridLayout7);
            app.LayoutViewTabGroup.Layout.Row = [2 3];
            app.LayoutViewTabGroup.Layout.Column = [2 4];

            % Create LayoutParametersTab
            app.LayoutParametersTab = uitab(app.LayoutViewTabGroup);
            app.LayoutParametersTab.Title = '布局参数';

            % Create LayoutParametersGrid
            app.LayoutParametersGrid = uigridlayout(app.LayoutParametersTab);
            app.LayoutParametersGrid.ColumnWidth = {'1x'};
            app.LayoutParametersGrid.RowHeight = {'1x'};
            app.LayoutParametersGrid.Padding = [8 8 8 8];

            % Create LayoutTable
            app.LayoutTable = uitable(app.LayoutParametersGrid);
            app.LayoutTable.ColumnName = {'编号'; 'X'; 'Y'; 'Z'; 'Dx'; 'Dy'; 'Dz'};
            app.LayoutTable.ColumnWidth = {45, 'auto', 'auto', 'auto', 'auto', 'auto', 'auto'};
            app.LayoutTable.RowName = {};
            app.LayoutTable.ColumnEditable = [false true true true true true true];
            app.LayoutTable.Layout.Row = 1;
            app.LayoutTable.Layout.Column = 1;

            % Create LayoutViewsTab
            app.LayoutViewsTab = uitab(app.LayoutViewTabGroup);
            app.LayoutViewsTab.Title = '三视图';

            % Create LayoutViewsGrid
            app.LayoutViewsGrid = uigridlayout(app.LayoutViewsTab);
            app.LayoutViewsGrid.ColumnWidth = {'1x', '1x', '1x'};
            app.LayoutViewsGrid.RowHeight = {'1x'};
            app.LayoutViewsGrid.ColumnSpacing = 8;
            app.LayoutViewsGrid.Padding = [8 8 8 8];

            % Create LayoutXZAxes
            app.LayoutXZAxes = uiaxes(app.LayoutViewsGrid);
            app.LayoutXZAxes.Layout.Row = 1;
            app.LayoutXZAxes.Layout.Column = 3;

            % Create LayoutXYAxes
            app.LayoutXYAxes = uiaxes(app.LayoutViewsGrid);
            app.LayoutXYAxes.Layout.Row = 1;
            app.LayoutXYAxes.Layout.Column = 2;

            % Create LayoutYZAxes
            app.LayoutYZAxes = uiaxes(app.LayoutViewsGrid);
            app.LayoutYZAxes.Layout.Row = 1;
            app.LayoutYZAxes.Layout.Column = 1;

            % Create Layout3DTab
            app.Layout3DTab = uitab(app.LayoutViewTabGroup);
            app.Layout3DTab.Title = '当前布局';

            % Create Layout3DGrid
            app.Layout3DGrid = uigridlayout(app.Layout3DTab);
            app.Layout3DGrid.ColumnWidth = {'1x'};
            app.Layout3DGrid.RowHeight = {'1x'};

            % Create LayoutAxes
            app.LayoutAxes = uiaxes(app.Layout3DGrid);
            app.LayoutAxes.Layout.Row = 1;
            app.LayoutAxes.Layout.Column = 1;

            % Create Panel10_3
            app.Panel10_3 = uipanel(app.GridLayout7);
            app.Panel10_3.Title = '严格主备布局搜索设置';
            app.Panel10_3.Layout.Row = 3;
            app.Panel10_3.Layout.Column = 1;

            % Create GridLayout21_3
            app.GridLayout21_3 = uigridlayout(app.Panel10_3);
            app.GridLayout21_3.RowHeight = {26, 38, 26, 38};

            % Create DropDown1
            app.DropDown1 = uidropdown(app.GridLayout21_3);
            app.DropDown1.Items = {'115拓扑自动布局搜索','安装拓扑解析筛选','完整组选必要条件','安装几何与布局'};
            app.DropDown1.Layout.Row = 4;
            app.DropDown1.Layout.Column = 1;
            app.DropDown1.Value = '115拓扑自动布局搜索';

            % Create Label37
            app.Label37 = uilabel(app.GridLayout21_3);
            app.Label37.Layout.Row = 3;
            app.Label37.Layout.Column = 1;
            app.Label37.Text = '搜索阶段';

            % Create MinimumAxisCapacityLabel
            app.MinimumAxisCapacityLabel = uilabel(app.GridLayout21_3);
            app.MinimumAxisCapacityLabel.Layout.Row = 3;
            app.MinimumAxisCapacityLabel.Layout.Column = 2;
            app.MinimumAxisCapacityLabel.Text = '非零能力判据';

            % Create MinimumAxisCapacityField
            app.MinimumAxisCapacityField = uieditfield(app.GridLayout21_3, 'numeric');
            app.MinimumAxisCapacityField.Limits = [1e-5 12];
            app.MinimumAxisCapacityField.Layout.Row = 4;
            app.MinimumAxisCapacityField.Layout.Column = 2;
            app.MinimumAxisCapacityField.Value = 1e-5;
            app.MinimumAxisCapacityField.Enable = 'off';
            app.MinimumAxisCapacityField.Tooltip = '仅用于排除数值零，不优化控制能力大小。';

            % Create GenerationField_3
            app.GenerationField_3 = uieditfield(app.GridLayout21_3, 'numeric');
            app.GenerationField_3.Limits = [10 7200];
            app.GenerationField_3.HorizontalAlignment = 'left';
            app.GenerationField_3.Layout.Row = 2;
            app.GenerationField_3.Layout.Column = 2;
            app.GenerationField_3.Value = 600;

            % Create Label36_3
            app.Label36_3 = uilabel(app.GridLayout21_3);
            app.Label36_3.Layout.Row = 1;
            app.Label36_3.Layout.Column = 2;
            app.Label36_3.Text = '本轮搜索时限 / s';
            app.GenerationField_3.Tooltip = '每次点击的时间预算；保持不变即可分轮续跑，增加时限可减少启动开销。';

            % Create PopulationField
            app.PopulationField = uieditfield(app.GridLayout21_3, 'numeric');
            app.PopulationField.Limits = [1 1000];
            app.PopulationField.HorizontalAlignment = 'left';
            app.PopulationField.Layout.Row = 2;
            app.PopulationField.Layout.Column = 1;
            app.PopulationField.Value = 16;

            % Create Label35_3
            app.Label35_3 = uilabel(app.GridLayout21_3);
            app.Label35_3.Layout.Row = 1;
            app.Label35_3.Layout.Column = 1;
            app.Label35_3.Text = '几何阶段初值数量';

            % Create ThrusterPanel_2
            app.ThrusterPanel_2 = uipanel(app.GridLayout7);
            app.ThrusterPanel_2.Title = '安装约束';
            app.ThrusterPanel_2.Layout.Row = 2;
            app.ThrusterPanel_2.Layout.Column = 1;

            % Create ThrusterGrid_2
            app.ThrusterGrid_2 = uigridlayout(app.ThrusterPanel_2);
            app.ThrusterGrid_2.ColumnWidth = {67, 70, '1x', 67, 70, '1x'};
            app.ThrusterGrid_2.RowHeight = {34, 34, 34, 34, 34};

            % Create EditField1_5
            app.EditField1_5 = uieditfield(app.ThrusterGrid_2, 'text');
            app.EditField1_5.HorizontalAlignment = 'right';
            app.EditField1_5.Layout.Row = 5;
            app.EditField1_5.Layout.Column = 2;
            app.EditField1_5.Value = '[0,0.5π]';

            % Create EditField1_4
            app.EditField1_4 = uieditfield(app.ThrusterGrid_2, 'text');
            app.EditField1_4.HorizontalAlignment = 'right';
            app.EditField1_4.Layout.Row = 4;
            app.EditField1_4.Layout.Column = 2;
            app.EditField1_4.Value = '[0,2π]';

            % Create EditField1_3
            app.EditField1_3 = uieditfield(app.ThrusterGrid_2, 'text');
            app.EditField1_3.HorizontalAlignment = 'right';
            app.EditField1_3.Layout.Row = 3;
            app.EditField1_3.Layout.Column = 2;
            app.EditField1_3.Value = '[-0.6,0.6]';

            % Create EditField1_2
            app.EditField1_2 = uieditfield(app.ThrusterGrid_2, 'text');
            app.EditField1_2.HorizontalAlignment = 'right';
            app.EditField1_2.Layout.Row = 2;
            app.EditField1_2.Layout.Column = 2;
            app.EditField1_2.Value = '[0,0.6]';

            % Create Label24_37
            app.Label24_37 = uilabel(app.ThrusterGrid_2);
            app.Label24_37.Layout.Row = 5;
            app.Label24_37.Layout.Column = 3;
            app.Label24_37.Text = 'rad';

            % Create Label33_6
            app.Label33_6 = uilabel(app.ThrusterGrid_2);
            app.Label33_6.Layout.Row = 5;
            app.Label33_6.Layout.Column = 1;
            app.Label33_6.Text = '方向参数β';

            % Create Label24_36
            app.Label24_36 = uilabel(app.ThrusterGrid_2);
            app.Label24_36.Layout.Row = 4;
            app.Label24_36.Layout.Column = 3;
            app.Label24_36.Text = 'rad';

            % Create Label33_5
            app.Label33_5 = uilabel(app.ThrusterGrid_2);
            app.Label33_5.Layout.Row = 4;
            app.Label33_5.Layout.Column = 1;
            app.Label33_5.Text = '方向参数α';

            % Create Label24_35
            app.Label24_35 = uilabel(app.ThrusterGrid_2);
            app.Label24_35.Layout.Row = 3;
            app.Label24_35.Layout.Column = 3;
            app.Label24_35.Text = 'm';

            % Create Label33_4
            app.Label33_4 = uilabel(app.ThrusterGrid_2);
            app.Label33_4.Layout.Row = 3;
            app.Label33_4.Layout.Column = 1;
            app.Label33_4.Text = '位置参数b';

            % Create Label24_34
            app.Label24_34 = uilabel(app.ThrusterGrid_2);
            app.Label24_34.Layout.Row = 2;
            app.Label24_34.Layout.Column = 3;
            app.Label24_34.Text = 'm';

            % Create Label33_3
            app.Label33_3 = uilabel(app.ThrusterGrid_2);
            app.Label33_3.Layout.Row = 2;
            app.Label33_3.Layout.Column = 1;
            app.Label33_3.Text = '位置参数a';

            % Create Label24_33
            app.Label24_33 = uilabel(app.ThrusterGrid_2);
            app.Label24_33.Layout.Row = 1;
            app.Label24_33.Layout.Column = 6;
            app.Label24_33.Text = '面';

            % Create FmaxField_4
            app.FmaxField_4 = uieditfield(app.ThrusterGrid_2, 'numeric');
            app.FmaxField_4.Limits = [0 Inf];
            app.FmaxField_4.Layout.Row = 1;
            app.FmaxField_4.Layout.Column = 5;
            app.FmaxField_4.Value = 2;

            % Create Label49_4
            app.Label49_4 = uilabel(app.ThrusterGrid_2);
            app.Label49_4.Layout.Row = 1;
            app.Label49_4.Layout.Column = 4;
            app.Label49_4.Text = '安装面X';

            % Create Label24_25
            app.Label24_25 = uilabel(app.ThrusterGrid_2);
            app.Label24_25.Layout.Row = 5;
            app.Label24_25.Layout.Column = 6;
            app.Label24_25.Text = 'deg';

            % Create NumericEditField2_6
            app.NumericEditField2_6 = uieditfield(app.ThrusterGrid_2, 'numeric');
            app.NumericEditField2_6.Limits = [0 Inf];
            app.NumericEditField2_6.Layout.Row = 5;
            app.NumericEditField2_6.Layout.Column = 5;

            % Create Label34_6
            app.Label34_6 = uilabel(app.ThrusterGrid_2);
            app.Label34_6.Layout.Row = 5;
            app.Label34_6.Layout.Column = 4;
            app.Label34_6.Text = '最小夹角θ';

            % Create Label24_24
            app.Label24_24 = uilabel(app.ThrusterGrid_2);
            app.Label24_24.Layout.Row = 4;
            app.Label24_24.Layout.Column = 6;
            app.Label24_24.Text = 'm';

            % Create NumericEditField2_5
            app.NumericEditField2_5 = uieditfield(app.ThrusterGrid_2, 'numeric');
            app.NumericEditField2_5.Limits = [0 Inf];
            app.NumericEditField2_5.Layout.Row = 4;
            app.NumericEditField2_5.Layout.Column = 5;

            % Create Label34_5
            app.Label34_5 = uilabel(app.ThrusterGrid_2);
            app.Label34_5.Layout.Row = 4;
            app.Label34_5.Layout.Column = 4;
            app.Label34_5.Text = '最小间距d';

            % Create Label24_23
            app.Label24_23 = uilabel(app.ThrusterGrid_2);
            app.Label24_23.Layout.Row = 3;
            app.Label24_23.Layout.Column = 6;
            app.Label24_23.Text = 'm';

            % Create Label24_22
            app.Label24_22 = uilabel(app.ThrusterGrid_2);
            app.Label24_22.Layout.Row = 2;
            app.Label24_22.Layout.Column = 6;
            app.Label24_22.Text = 'm';

            % Create Label24_20
            app.Label24_20 = uilabel(app.ThrusterGrid_2);
            app.Label24_20.Layout.Row = 1;
            app.Label24_20.Layout.Column = 3;
            app.Label24_20.Text = '台';

            % Create Label24_19
            app.Label24_19 = uilabel(app.ThrusterGrid_2);
            app.Label24_19.Layout.Row = 1;
            app.Label24_19.Layout.Column = 3;
            app.Label24_19.Text = '';

            % Create Label49_2
            app.Label49_2 = uilabel(app.ThrusterGrid_2);
            app.Label49_2.Layout.Row = 1;
            app.Label49_2.Layout.Column = 1;
            app.Label49_2.Text = '总台数Num';

            % Create FmaxField_2
            app.FmaxField_2 = uieditfield(app.ThrusterGrid_2, 'numeric');
            app.FmaxField_2.Limits = [0 Inf];
            app.FmaxField_2.Layout.Row = 1;
            app.FmaxField_2.Layout.Column = 2;
            app.FmaxField_2.Value = 12;

            % Create NumericEditField1_2
            app.NumericEditField1_2 = uieditfield(app.ThrusterGrid_2, 'numeric');
            app.NumericEditField1_2.Limits = [0 Inf];
            app.NumericEditField1_2.ValueChangedFcn = createCallbackFcn(app, @LayoutBoundaryValueChanged, true);
            app.NumericEditField1_2.Layout.Row = 2;
            app.NumericEditField1_2.Layout.Column = 5;
            app.NumericEditField1_2.Value = 0.6;

            % Create Label33_2
            app.Label33_2 = uilabel(app.ThrusterGrid_2);
            app.Label33_2.Layout.Row = 2;
            app.Label33_2.Layout.Column = 4;
            app.Label33_2.Text = '安装边界L';

            % Create NumericEditField2_4
            app.NumericEditField2_4 = uieditfield(app.ThrusterGrid_2, 'numeric');
            app.NumericEditField2_4.Limits = [0 Inf];
            app.NumericEditField2_4.ValueChangedFcn = createCallbackFcn(app, @LayoutBoundaryValueChanged, true);
            app.NumericEditField2_4.Layout.Row = 3;
            app.NumericEditField2_4.Layout.Column = 5;
            app.NumericEditField2_4.Value = 0.6;

            % Create Label34_4
            app.Label34_4 = uilabel(app.ThrusterGrid_2);
            app.Label34_4.Layout.Row = 3;
            app.Label34_4.Layout.Column = 4;
            app.Label34_4.Text = '安装边界W';

            % Create Label7
            app.Label7 = uilabel(app.GridLayout7);
            app.Label7.FontSize = 22;
            app.Label7.FontWeight = 'bold';
            app.Label7.FontColor = [0.1 0.27 0.42];
            app.Label7.Layout.Row = 1;
            app.Label7.Layout.Column = 1;
            app.Label7.Text = '布局优化设计';

            % Create DeleteLayoutButton
            app.DeleteLayoutButton = uibutton(app.GridLayout7, 'push');
            app.DeleteLayoutButton.ButtonPushedFcn = createCallbackFcn(app, @DeleteLayoutButtonPushed, true);
            app.DeleteLayoutButton.Layout.Row = 4;
            app.DeleteLayoutButton.Layout.Column = 4;
            app.DeleteLayoutButton.Text = '删除布局';

            % Create ParameterFileGrid_3
            app.ParameterFileGrid_3 = uigridlayout(app.GridLayout7);
            app.ParameterFileGrid_3.ColumnWidth = {'1x', 132, 146};
            app.ParameterFileGrid_3.RowHeight = {55};
            app.ParameterFileGrid_3.Padding = [0 6 0 0];
            app.ParameterFileGrid_3.Layout.Row = 4;
            app.ParameterFileGrid_3.Layout.Column = 1;

            app.LoadSearchRecordButton = uibutton(app.ParameterFileGrid_3,'push');
            app.LoadSearchRecordButton.Layout.Row = 1;
            app.LoadSearchRecordButton.Layout.Column = 2;
            app.LoadSearchRecordButton.Text = '载入搜索记录';
            app.LoadSearchRecordButton.Tooltip = '通常自动接续上次记录；也可手动选择 diagnosis.json。';

            % Create StartOptimizationButton_3
            app.StartOptimizationButton_3 = uibutton(app.ParameterFileGrid_3, 'push');
            app.StartOptimizationButton_3.ButtonPushedFcn = createCallbackFcn(app, @StartOptimizationButtonPushed, true);
            app.StartOptimizationButton_3.Layout.Row = 1;
            app.StartOptimizationButton_3.Layout.Column = 3;
            app.StartOptimizationButton_3.Text = '查找必要条件';

            % Create ParameterTab
            app.ParameterTab = uitab(app.TabGroup);
            app.ParameterTab.Title = '输入条件';

            % Create GridLayout5
            app.GridLayout5 = uigridlayout(app.ParameterTab);
            app.GridLayout5.ColumnWidth = {'1x', '1x', '1x'};
            app.GridLayout5.RowHeight = {55, 'fit', 'fit', 'fit'};
            app.GridLayout5.ColumnSpacing = 16;
            app.GridLayout5.Padding = [24 18 24 18];

            % Create ParameterFileGrid_2
            app.ParameterFileGrid_2 = uigridlayout(app.GridLayout5);
            app.ParameterFileGrid_2.ColumnWidth = {'1x', 104, 104};
            app.ParameterFileGrid_2.RowHeight = {55};
            app.ParameterFileGrid_2.Padding = [0 6 0 0];
            app.ParameterFileGrid_2.Layout.Row = 4;
            app.ParameterFileGrid_2.Layout.Column = 3;

            % Create Button2_4
            app.Button2_4 = uibutton(app.ParameterFileGrid_2, 'push');
            app.Button2_4.ButtonPushedFcn = createCallbackFcn(app, @LoadParameterButtonPushed, true);
            app.Button2_4.Layout.Row = 1;
            app.Button2_4.Layout.Column = 3;
            app.Button2_4.Text = '加载参数文件';

            % Create Button2_2
            app.Button2_2 = uibutton(app.ParameterFileGrid_2, 'push');
            app.Button2_2.ButtonPushedFcn = createCallbackFcn(app, @ResetParameterButtonPushed, true);
            app.Button2_2.Layout.Row = 1;
            app.Button2_2.Layout.Column = 2;
            app.Button2_2.Text = '恢复默认设置';

            % Create ThrusterPanel_3
            app.ThrusterPanel_3 = uipanel(app.GridLayout5);
            app.ThrusterPanel_3.Title = '推力器';
            app.ThrusterPanel_3.Layout.Row = 3;
            app.ThrusterPanel_3.Layout.Column = 2;

            % Create ThrusterGrid_3
            app.ThrusterGrid_3 = uigridlayout(app.ThrusterPanel_3);
            app.ThrusterGrid_3.ColumnWidth = {67, 70, 55};
            app.ThrusterGrid_3.RowHeight = {34, 34};

            % Create Label24_28
            app.Label24_28 = uilabel(app.ThrusterGrid_3);
            app.Label24_28.Layout.Row = 2;
            app.Label24_28.Layout.Column = 3;
            app.Label24_28.Text = 's';

            % Create Label24_27
            app.Label24_27 = uilabel(app.ThrusterGrid_3);
            app.Label24_27.Layout.Row = 1;
            app.Label24_27.Layout.Column = 3;
            app.Label24_27.Text = 'N';

            % Create Label24_26
            app.Label24_26 = uilabel(app.ThrusterGrid_3);
            app.Label24_26.Layout.Row = 1;
            app.Label24_26.Layout.Column = 3;
            app.Label24_26.Text = '';

            % Create Label49_3
            app.Label49_3 = uilabel(app.ThrusterGrid_3);
            app.Label49_3.Layout.Row = 1;
            app.Label49_3.Layout.Column = 1;
            app.Label49_3.Text = '标称推力F';

            % Create FmaxField_3
            app.FmaxField_3 = uieditfield(app.ThrusterGrid_3, 'numeric');
            app.FmaxField_3.Limits = [0 Inf];
            app.FmaxField_3.Layout.Row = 1;
            app.FmaxField_3.Layout.Column = 2;
            app.FmaxField_3.Value = 10;

            % Create Label50_3
            app.Label50_3 = uilabel(app.ThrusterGrid_3);
            app.Label50_3.Layout.Row = 2;
            app.Label50_3.Layout.Column = 1;
            app.Label50_3.Text = '最小脉宽t';

            % Create MinPulseField_3
            app.MinPulseField_3 = uieditfield(app.ThrusterGrid_3, 'numeric');
            app.MinPulseField_3.Limits = [0 Inf];
            app.MinPulseField_3.Layout.Row = 2;
            app.MinPulseField_3.Layout.Column = 2;
            app.MinPulseField_3.Value = 0.02;

            % Create Panel8
            app.Panel8 = uipanel(app.GridLayout5);
            app.Panel8.Title = '任务场景';
            app.Panel8.Layout.Row = [2 3];
            app.Panel8.Layout.Column = 1;

            % Create GridLayout19
            app.GridLayout19 = uigridlayout(app.Panel8);
            app.GridLayout19.ColumnWidth = {'fit', 70, 55};
            app.GridLayout19.RowHeight = {34, 34, 34, 34, 34, 34, 34};

            % Create MinPulseField_4
            app.MinPulseField_4 = uieditfield(app.GridLayout19, 'numeric');
            app.MinPulseField_4.Limits = [0 Inf];
            app.MinPulseField_4.Layout.Row = 6;
            app.MinPulseField_4.Layout.Column = 2;
            app.MinPulseField_4.Value = 0.005;

            % Create Label24_32
            app.Label24_32 = uilabel(app.GridLayout19);
            app.Label24_32.Layout.Row = 6;
            app.Label24_32.Layout.Column = 3;
            app.Label24_32.Text = 's';

            % Create Label38_5
            app.Label38_5 = uilabel(app.GridLayout19);
            app.Label38_5.Layout.Row = 6;
            app.Label38_5.Layout.Column = 1;
            app.Label38_5.Text = '时间步长dt';

            % Create Label24_6
            app.Label24_6 = uilabel(app.GridLayout19);
            app.Label24_6.Layout.Row = 5;
            app.Label24_6.Layout.Column = 3;
            app.Label24_6.Text = 's';

            % Create Label24_5
            app.Label24_5 = uilabel(app.GridLayout19);
            app.Label24_5.Layout.Row = 4;
            app.Label24_5.Layout.Column = 3;
            app.Label24_5.Text = 'deg';

            % Create Label24_4
            app.Label24_4 = uilabel(app.GridLayout19);
            app.Label24_4.Layout.Row = 3;
            app.Label24_4.Layout.Column = 3;
            app.Label24_4.Text = 'deg';

            % Create Label24_3
            app.Label24_3 = uilabel(app.GridLayout19);
            app.Label24_3.Layout.Row = 2;
            app.Label24_3.Layout.Column = 3;
            app.Label24_3.Text = 'm';

            % Create Label24_2
            app.Label24_2 = uilabel(app.GridLayout19);
            app.Label24_2.Layout.Row = 1;
            app.Label24_2.Layout.Column = 3;
            app.Label24_2.Text = 'm';

            % Create EditField4
            app.EditField4 = uieditfield(app.GridLayout19, 'text');
            app.EditField4.HorizontalAlignment = 'right';
            app.EditField4.Layout.Row = 4;
            app.EditField4.Layout.Column = 2;
            app.EditField4.Value = '[5,25,35]';

            % Create Label27
            app.Label27 = uilabel(app.GridLayout19);
            app.Label27.Layout.Row = 4;
            app.Label27.Layout.Column = 1;
            app.Label27.Text = '目标姿态et';

            % Create EditField3
            app.EditField3 = uieditfield(app.GridLayout19, 'text');
            app.EditField3.HorizontalAlignment = 'right';
            app.EditField3.Layout.Row = 3;
            app.EditField3.Layout.Column = 2;
            app.EditField3.Value = '[0,15,55]';

            % Create Label26
            app.Label26 = uilabel(app.GridLayout19);
            app.Label26.Layout.Row = 3;
            app.Label26.Layout.Column = 1;
            app.Label26.Text = '初始姿态e0';

            % Create EditField2
            app.EditField2 = uieditfield(app.GridLayout19, 'text');
            app.EditField2.HorizontalAlignment = 'right';
            app.EditField2.Layout.Row = 2;
            app.EditField2.Layout.Column = 2;
            app.EditField2.Value = '[5,25,35]';

            % Create Label25
            app.Label25 = uilabel(app.GridLayout19);
            app.Label25.Layout.Row = 2;
            app.Label25.Layout.Column = 1;
            app.Label25.Text = '目标位置rt';

            % Create EditField1
            app.EditField1 = uieditfield(app.GridLayout19, 'text');
            app.EditField1.HorizontalAlignment = 'right';
            app.EditField1.Layout.Row = 1;
            app.EditField1.Layout.Column = 2;
            app.EditField1.Value = '[0,15,55]';

            % Create Label24
            app.Label24 = uilabel(app.GridLayout19);
            app.Label24.Layout.Row = 1;
            app.Label24.Layout.Column = 1;
            app.Label24.Text = '初始位置r0';

            % Create SimulationTimeField
            app.SimulationTimeField = uieditfield(app.GridLayout19, 'numeric');
            app.SimulationTimeField.Limits = [1 Inf];
            app.SimulationTimeField.Layout.Row = 5;
            app.SimulationTimeField.Layout.Column = 2;
            app.SimulationTimeField.Value = 2000;

            % Create Label38
            app.Label38 = uilabel(app.GridLayout19);
            app.Label38.Layout.Row = 5;
            app.Label38.Layout.Column = 1;
            app.Label38.Text = '仿真时间Ts';

            % Create Label50_6
            app.Label50_6 = uilabel(app.GridLayout19);
            app.Label50_6.Layout.Row = 7;
            app.Label50_6.Layout.Column = 1;
            app.Label50_6.Text = '调用策略';

            % Create AllocationModeDropDown_2
            app.AllocationModeDropDown_2 = uidropdown(app.GridLayout19);
            app.AllocationModeDropDown_2.Items = {'异步分时复用', '同步分时复用', '联合优化复用'};
            app.AllocationModeDropDown_2.Layout.Row = 7;
            app.AllocationModeDropDown_2.Layout.Column = [2 3];
            app.AllocationModeDropDown_2.Value = '同步分时复用';

            % Create SpacecraftPanel
            app.SpacecraftPanel = uipanel(app.GridLayout5);
            app.SpacecraftPanel.Title = '飞行器';
            app.SpacecraftPanel.Layout.Row = 2;
            app.SpacecraftPanel.Layout.Column = 2;

            % Create SpacecraftGrid
            app.SpacecraftGrid = uigridlayout(app.SpacecraftPanel);
            app.SpacecraftGrid.ColumnWidth = {67, 70, '1x'};
            app.SpacecraftGrid.RowHeight = {34, 34, 34, 34};

            % Create Label24_10
            app.Label24_10 = uilabel(app.SpacecraftGrid);
            app.Label24_10.Layout.Row = 4;
            app.Label24_10.Layout.Column = 3;
            app.Label24_10.Text = 'kg·m^2';

            % Create Label24_9
            app.Label24_9 = uilabel(app.SpacecraftGrid);
            app.Label24_9.Layout.Row = 3;
            app.Label24_9.Layout.Column = 3;
            app.Label24_9.Text = 'kg·m^2';

            % Create Label24_8
            app.Label24_8 = uilabel(app.SpacecraftGrid);
            app.Label24_8.Layout.Row = 2;
            app.Label24_8.Layout.Column = 3;
            app.Label24_8.Text = 'kg·m^2';

            % Create Label24_7
            app.Label24_7 = uilabel(app.SpacecraftGrid);
            app.Label24_7.Layout.Row = 1;
            app.Label24_7.Layout.Column = 3;
            app.Label24_7.Text = 'kg';

            % Create Label43
            app.Label43 = uilabel(app.SpacecraftGrid);
            app.Label43.Layout.Row = 1;
            app.Label43.Layout.Column = 1;
            app.Label43.Text = '质量m';

            % Create MassField
            app.MassField = uieditfield(app.SpacecraftGrid, 'numeric');
            app.MassField.Limits = [0 Inf];
            app.MassField.ValueDisplayFormat = '%.0f';
            app.MassField.Layout.Row = 1;
            app.MassField.Layout.Column = 2;
            app.MassField.Value = 3700;

            % Create Label44
            app.Label44 = uilabel(app.SpacecraftGrid);
            app.Label44.Layout.Row = 2;
            app.Label44.Layout.Column = 1;
            app.Label44.Text = '主惯量Ixx';

            % Create JxxField
            app.JxxField = uieditfield(app.SpacecraftGrid, 'numeric');
            app.JxxField.Limits = [0 Inf];
            app.JxxField.ValueDisplayFormat = '%.0f';
            app.JxxField.Layout.Row = 2;
            app.JxxField.Layout.Column = 2;
            app.JxxField.Value = 10000;

            % Create Label45
            app.Label45 = uilabel(app.SpacecraftGrid);
            app.Label45.Layout.Row = 3;
            app.Label45.Layout.Column = 1;
            app.Label45.Text = '主惯量Iyy';

            % Create JyyField
            app.JyyField = uieditfield(app.SpacecraftGrid, 'numeric');
            app.JyyField.Limits = [0 Inf];
            app.JyyField.ValueDisplayFormat = '%.0f';
            app.JyyField.Layout.Row = 3;
            app.JyyField.Layout.Column = 2;
            app.JyyField.Value = 6000;

            % Create Label46
            app.Label46 = uilabel(app.SpacecraftGrid);
            app.Label46.Layout.Row = 4;
            app.Label46.Layout.Column = 1;
            app.Label46.Text = '主惯量Izz';

            % Create JzzField
            app.JzzField = uieditfield(app.SpacecraftGrid, 'numeric');
            app.JzzField.Limits = [0 Inf];
            app.JzzField.ValueDisplayFormat = '%.0f';
            app.JzzField.Layout.Row = 4;
            app.JzzField.Layout.Column = 2;
            app.JzzField.Value = 13000;

            % Create ControllerPanel
            app.ControllerPanel = uipanel(app.GridLayout5);
            app.ControllerPanel.Title = '控制器';
            app.ControllerPanel.Layout.Row = [2 3];
            app.ControllerPanel.Layout.Column = 3;

            % Create ControllerGrid
            app.ControllerGrid = uigridlayout(app.ControllerPanel);
            app.ControllerGrid.ColumnWidth = {67, 70, '1x'};
            app.ControllerGrid.RowHeight = {34, 34, 34, 34, 34, 34, 34};

            % Create Label24_11
            app.Label24_11 = uilabel(app.ControllerGrid);
            app.Label24_11.Layout.Row = 1;
            app.Label24_11.Layout.Column = 3;
            app.Label24_11.Text = 's';

            % Create Label47
            app.Label47 = uilabel(app.ControllerGrid);
            app.Label47.Layout.Row = 1;
            app.Label47.Layout.Column = 1;
            app.Label47.Text = '控制周期T';

            % Create ControlPeriodField
            app.ControlPeriodField = uieditfield(app.ControllerGrid, 'numeric');
            app.ControlPeriodField.Limits = [0 Inf];
            app.ControlPeriodField.Layout.Row = 1;
            app.ControlPeriodField.Layout.Column = 2;
            app.ControlPeriodField.Value = 0.4;

            % Create Label47_7
            app.Label47_7 = uilabel(app.ControllerGrid);
            app.Label47_7.Layout.Row = 2;
            app.Label47_7.Layout.Column = 1;
            app.Label47_7.Text = '位置参数Kp';

            % Create Label47_8
            app.Label47_8 = uilabel(app.ControllerGrid);
            app.Label47_8.Layout.Row = 4;
            app.Label47_8.Layout.Column = 1;
            app.Label47_8.Text = '位置参数Kd';

            % Create Label47_9
            app.Label47_9 = uilabel(app.ControllerGrid);
            app.Label47_9.Layout.Row = 5;
            app.Label47_9.Layout.Column = 1;
            app.Label47_9.Text = '姿态参数Kp';

            % Create Label47_10
            app.Label47_10 = uilabel(app.ControllerGrid);
            app.Label47_10.Layout.Row = 6;
            app.Label47_10.Layout.Column = 1;
            app.Label47_10.Text = '姿态参数Ki';

            % Create Label47_11
            app.Label47_11 = uilabel(app.ControllerGrid);
            app.Label47_11.Layout.Row = 7;
            app.Label47_11.Layout.Column = 1;
            app.Label47_11.Text = '姿态参数Kd';

            % Create FmaxField_7
            app.FmaxField_7 = uieditfield(app.ControllerGrid, 'numeric');
            app.FmaxField_7.Limits = [0 Inf];
            app.FmaxField_7.Layout.Row = 2;
            app.FmaxField_7.Layout.Column = 2;
            app.FmaxField_7.Value = 10;

            % Create FmaxField_8
            app.FmaxField_8 = uieditfield(app.ControllerGrid, 'numeric');
            app.FmaxField_8.Limits = [0 Inf];
            app.FmaxField_8.Layout.Row = 4;
            app.FmaxField_8.Layout.Column = 2;
            app.FmaxField_8.Value = 300;

            % Create FmaxField_9
            app.FmaxField_9 = uieditfield(app.ControllerGrid, 'numeric');
            app.FmaxField_9.Limits = [0 Inf];
            app.FmaxField_9.Layout.Row = 5;
            app.FmaxField_9.Layout.Column = 2;
            app.FmaxField_9.Value = 400;

            % Create FmaxField_10
            app.FmaxField_10 = uieditfield(app.ControllerGrid, 'numeric');
            app.FmaxField_10.Limits = [0 Inf];
            app.FmaxField_10.Layout.Row = 6;
            app.FmaxField_10.Layout.Column = 2;
            app.FmaxField_10.Value = 20;

            % Create FmaxField_11
            app.FmaxField_11 = uieditfield(app.ControllerGrid, 'numeric');
            app.FmaxField_11.Limits = [0 Inf];
            app.FmaxField_11.Layout.Row = 7;
            app.FmaxField_11.Layout.Column = 2;
            app.FmaxField_11.Value = 3200;

            % Create FmaxField_12
            app.FmaxField_12 = uieditfield(app.ControllerGrid, 'numeric');
            app.FmaxField_12.Limits = [0 Inf];
            app.FmaxField_12.Layout.Row = 3;
            app.FmaxField_12.Layout.Column = 2;

            % Create Label47_12
            app.Label47_12 = uilabel(app.ControllerGrid);
            app.Label47_12.Layout.Row = 3;
            app.Label47_12.Layout.Column = 1;
            app.Label47_12.Text = '位置参数Ki';

            % Create Label4
            app.Label4 = uilabel(app.GridLayout5);
            app.Label4.FontSize = 22;
            app.Label4.FontWeight = 'bold';
            app.Label4.FontColor = [0.1 0.27 0.42];
            app.Label4.Layout.Row = 1;
            app.Label4.Layout.Column = 1;
            app.Label4.Text = '输入条件设置';

            % Create ThrusterPanel_4
            app.ThrusterPanel_4 = uipanel(app.GridLayout5);
            app.ThrusterPanel_4.Title = '故障状态';
            app.ThrusterPanel_4.Layout.Row = 4;
            app.ThrusterPanel_4.Layout.Column = 1;

            % Create ThrusterGrid_4
            app.ThrusterGrid_4 = uigridlayout(app.ThrusterPanel_4);
            app.ThrusterGrid_4.ColumnWidth = {67, 70, '1x'};
            app.ThrusterGrid_4.RowHeight = {34, 34, 34};

            % Create Label24_39
            app.Label24_39 = uilabel(app.ThrusterGrid_4);
            app.Label24_39.Layout.Row = 2;
            app.Label24_39.Layout.Column = 3;
            app.Label24_39.Text = 's';

            % Create Label24_40
            app.Label24_40 = uilabel(app.ThrusterGrid_4);
            app.Label24_40.Layout.Row = 1;
            app.Label24_40.Layout.Column = 3;
            app.Label24_40.Text = '台';

            % Create Label49_5
            app.Label49_5 = uilabel(app.ThrusterGrid_4);
            app.Label49_5.Layout.Row = 1;
            app.Label49_5.Layout.Column = 1;
            app.Label49_5.Text = '故障台数';

            % Create FmaxField_5
            app.FmaxField_5 = uieditfield(app.ThrusterGrid_4, 'numeric');
            app.FmaxField_5.Limits = [0 Inf];
            app.FmaxField_5.Layout.Row = 1;
            app.FmaxField_5.Layout.Column = 2;
            app.FmaxField_5.Value = 1;

            % Create Label50_4
            app.Label50_4 = uilabel(app.ThrusterGrid_4);
            app.Label50_4.Layout.Row = 2;
            app.Label50_4.Layout.Column = 1;
            app.Label50_4.Text = '故障时间';

            % Create MinPulseField_5
            app.MinPulseField_5 = uieditfield(app.ThrusterGrid_4, 'numeric');
            app.MinPulseField_5.Limits = [0 Inf];
            app.MinPulseField_5.Layout.Row = 2;
            app.MinPulseField_5.Layout.Column = 2;
            app.MinPulseField_5.Value = 1000;

            % Create Label50_5
            app.Label50_5 = uilabel(app.ThrusterGrid_4);
            app.Label50_5.Layout.Row = 3;
            app.Label50_5.Layout.Column = 1;
            app.Label50_5.Text = '故障编号';

            % Create FmaxField_6
            app.FmaxField_6 = uieditfield(app.ThrusterGrid_4, 'numeric');
            app.FmaxField_6.Limits = [0 Inf];
            app.FmaxField_6.Layout.Row = 3;
            app.FmaxField_6.Layout.Column = 2;
            app.FmaxField_6.Value = 1;

            % Create HomeTab
            app.HomeTab = uitab(app.TabGroup);
            app.HomeTab.Title = '项目首页';

            % Create GridLayout4
            app.GridLayout4 = uigridlayout(app.HomeTab);
            app.GridLayout4.ColumnWidth = {'1x', '1x', '1x'};
            app.GridLayout4.RowHeight = {80, 125, 125, 125};
            app.GridLayout4.ColumnSpacing = 16;
            app.GridLayout4.RowSpacing = 16;
            app.GridLayout4.Padding = [26 20 26 20];

            % Create HomeSimulationPanel
            app.HomeSimulationPanel = uipanel(app.GridLayout4);
            app.HomeSimulationPanel.BackgroundColor = [0.96 0.96 0.96];
            app.HomeSimulationPanel.Layout.Row = 3;
            app.HomeSimulationPanel.Layout.Column = 3;

            % Create HomeSimulationGrid
            app.HomeSimulationGrid = uigridlayout(app.HomeSimulationPanel);
            app.HomeSimulationGrid.ColumnWidth = {'1x'};
            app.HomeSimulationGrid.RowHeight = {38, '1x'};
            app.HomeSimulationGrid.Padding = [16 12 16 12];

            % Create HomeSimulationTitle
            app.HomeSimulationTitle = uilabel(app.HomeSimulationGrid);
            app.HomeSimulationTitle.FontSize = 17;
            app.HomeSimulationTitle.FontWeight = 'bold';
            app.HomeSimulationTitle.FontColor = [0.1 0.27 0.42];
            app.HomeSimulationTitle.Layout.Row = 1;
            app.HomeSimulationTitle.Layout.Column = 1;
            app.HomeSimulationTitle.Text = '闭环仿真';

            % Create HomeSimulationDesc
            app.HomeSimulationDesc = uilabel(app.HomeSimulationGrid);
            app.HomeSimulationDesc.FontSize = 13;
            app.HomeSimulationDesc.FontColor = [0.2 0.27 0.33];
            app.HomeSimulationDesc.Layout.Row = 2;
            app.HomeSimulationDesc.Layout.Column = 1;
            app.HomeSimulationDesc.Text = '位置响应｜姿态响应';

            % Create Panel7
            app.Panel7 = uipanel(app.GridLayout4);
            app.Panel7.BackgroundColor = [0.96 0.96 0.96];
            app.Panel7.Layout.Row = 3;
            app.Panel7.Layout.Column = 2;

            % Create GridLayout18
            app.GridLayout18 = uigridlayout(app.Panel7);
            app.GridLayout18.ColumnWidth = {'1x'};
            app.GridLayout18.RowHeight = {38, '1x'};
            app.GridLayout18.Padding = [16 12 16 12];

            % Create Label23
            app.Label23 = uilabel(app.GridLayout18);
            app.Label23.FontSize = 13;
            app.Label23.FontColor = [0.2 0.27 0.33];
            app.Label23.Layout.Row = 2;
            app.Label23.Layout.Column = 1;
            app.Label23.Text = '复用策略｜分配策略';

            % Create Label22
            app.Label22 = uilabel(app.GridLayout18);
            app.Label22.FontSize = 17;
            app.Label22.FontWeight = 'bold';
            app.Label22.FontColor = [0.1 0.27 0.42];
            app.Label22.Layout.Row = 1;
            app.Label22.Layout.Column = 1;
            app.Label22.Text = '调用策略';

            % Create Panel6
            app.Panel6 = uipanel(app.GridLayout4);
            app.Panel6.BackgroundColor = [0.96 0.96 0.96];
            app.Panel6.Layout.Row = 3;
            app.Panel6.Layout.Column = 1;

            % Create GridLayout17
            app.GridLayout17 = uigridlayout(app.Panel6);
            app.GridLayout17.ColumnWidth = {'1x'};
            app.GridLayout17.RowHeight = {38, '1x'};
            app.GridLayout17.Padding = [16 12 16 12];

            % Create Label21
            app.Label21 = uilabel(app.GridLayout17);
            app.Label21.WordWrap = 'on';
            app.Label21.FontSize = 13;
            app.Label21.FontColor = [0.2 0.27 0.33];
            app.Label21.Layout.Row = 2;
            app.Label21.Layout.Column = 1;
            app.Label21.Text = '实际控制能力｜理论控制能力｜综合评价';

            % Create Label20
            app.Label20 = uilabel(app.GridLayout17);
            app.Label20.FontSize = 17;
            app.Label20.FontWeight = 'bold';
            app.Label20.FontColor = [0.1 0.27 0.42];
            app.Label20.Layout.Row = 1;
            app.Label20.Layout.Column = 1;
            app.Label20.Text = '可重构评价';

            % Create Panel5
            app.Panel5 = uipanel(app.GridLayout4);
            app.Panel5.BackgroundColor = [0.96 0.96 0.96];
            app.Panel5.Layout.Row = 2;
            app.Panel5.Layout.Column = 3;

            % Create GridLayout16
            app.GridLayout16 = uigridlayout(app.Panel5);
            app.GridLayout16.ColumnWidth = {'1x'};
            app.GridLayout16.RowHeight = {38, '1x'};
            app.GridLayout16.Padding = [16 12 16 12];

            % Create Label19
            app.Label19 = uilabel(app.GridLayout16);
            app.Label19.FontSize = 13;
            app.Label19.FontColor = [0.2 0.27 0.33];
            app.Label19.Layout.Row = 2;
            app.Label19.Layout.Column = 1;
            app.Label19.Text = '标况｜单台故障｜多台故障';

            % Create Label18
            app.Label18 = uilabel(app.GridLayout16);
            app.Label18.FontSize = 17;
            app.Label18.FontWeight = 'bold';
            app.Label18.FontColor = [0.1 0.27 0.42];
            app.Label18.Layout.Row = 1;
            app.Label18.Layout.Column = 1;
            app.Label18.Text = '可重构判断';

            % Create Panel3
            app.Panel3 = uipanel(app.GridLayout4);
            app.Panel3.BackgroundColor = [0.96 0.96 0.96];
            app.Panel3.Layout.Row = 2;
            app.Panel3.Layout.Column = 2;

            % Create GridLayout14
            app.GridLayout14 = uigridlayout(app.Panel3);
            app.GridLayout14.ColumnWidth = {'1x'};
            app.GridLayout14.RowHeight = {38, '1x'};
            app.GridLayout14.Padding = [16 12 16 12];

            % Create Label15
            app.Label15 = uilabel(app.GridLayout14);
            app.Label15.FontSize = 13;
            app.Label15.FontColor = [0.2 0.27 0.33];
            app.Label15.Layout.Row = 2;
            app.Label15.Layout.Column = 1;
            app.Label15.Text = '安装约束｜优化算法｜布局显示';

            % Create Label14
            app.Label14 = uilabel(app.GridLayout14);
            app.Label14.FontSize = 17;
            app.Label14.FontWeight = 'bold';
            app.Label14.FontColor = [0.1 0.27 0.42];
            app.Label14.Layout.Row = 1;
            app.Label14.Layout.Column = 1;
            app.Label14.Text = '优化设计';

            % Create Panel2
            app.Panel2 = uipanel(app.GridLayout4);
            app.Panel2.BackgroundColor = [0.96 0.96 0.96];
            app.Panel2.Layout.Row = 2;
            app.Panel2.Layout.Column = 1;

            % Create GridLayout13
            app.GridLayout13 = uigridlayout(app.Panel2);
            app.GridLayout13.ColumnWidth = {'1x'};
            app.GridLayout13.RowHeight = {38, '1x'};
            app.GridLayout13.Padding = [16 12 16 12];

            % Create Label13
            app.Label13 = uilabel(app.GridLayout13);
            app.Label13.FontSize = 13;
            app.Label13.FontColor = [0.2 0.27 0.33];
            app.Label13.Layout.Row = 2;
            app.Label13.Layout.Column = 1;
            app.Label13.Text = '任务场景｜参数设置';

            % Create Label12
            app.Label12 = uilabel(app.GridLayout13);
            app.Label12.FontSize = 17;
            app.Label12.FontWeight = 'bold';
            app.Label12.FontColor = [0.1 0.27 0.42];
            app.Label12.Layout.Row = 1;
            app.Label12.Layout.Column = 1;
            app.Label12.Text = '输入条件';

            % Create Label3
            app.Label3 = uilabel(app.GridLayout4);
            app.Label3.HorizontalAlignment = 'center';
            app.Label3.FontSize = 24;
            app.Label3.FontWeight = 'bold';
            app.Label3.FontColor = [0.1 0.27 0.42];
            app.Label3.Layout.Row = 1;
            app.Label3.Layout.Column = [1 3];
            app.Label3.Text = {'机动飞行器姿轨耦合复用推力器'; '故障下的可重构性设计'};

            % Create NavigationPanel
            app.NavigationPanel = uipanel(app.BodyGrid);
            app.NavigationPanel.Title = '功能导航';
            app.NavigationPanel.BackgroundColor = [0.93 0.95 0.97];
            app.NavigationPanel.Layout.Row = 1;
            app.NavigationPanel.Layout.Column = 1;

            % Create GridLayout3
            app.GridLayout3 = uigridlayout(app.NavigationPanel);
            app.GridLayout3.ColumnWidth = {'1x'};
            app.GridLayout3.RowHeight = {'1x', 55};
            app.GridLayout3.Padding = [8 12 8 8];

            % Create Button1
            app.Button1 = uibutton(app.GridLayout3, 'push');
            app.Button1.ButtonPushedFcn = createCallbackFcn(app, @HelpButtonPushed, true);
            app.Button1.Layout.Row = 2;
            app.Button1.Layout.Column = 1;
            app.Button1.Text = '使用说明';

            % Create NavigationList
            app.NavigationList = uilistbox(app.GridLayout3);
            app.NavigationList.Items = {'项目首页', '输入条件', '优化设计', '可重构判断', '可重构评价', '调用策略', '闭环仿真'};
            app.NavigationList.ValueChangedFcn = createCallbackFcn(app, @NavigationListValueChanged, true);
            app.NavigationList.FontSize = 15;
            app.NavigationList.Layout.Row = 1;
            app.NavigationList.Layout.Column = 1;
            app.NavigationList.Value = '项目首页';

            % Create HeaderPanel
            app.HeaderPanel = uipanel(app.RootGrid);
            app.HeaderPanel.BorderType = 'none';
            app.HeaderPanel.Layout.Row = 1;
            app.HeaderPanel.Layout.Column = 1;

            % Create GridLayout1
            app.GridLayout1 = uigridlayout(app.HeaderPanel);
            app.GridLayout1.ColumnWidth = {'1x', 104, 104, 104, 15};
            app.GridLayout1.RowHeight = {55};
            app.GridLayout1.Padding = [20 10 12 10];
            app.GridLayout1.BackgroundColor = [0.075 0.2 0.32];

            % Create ImportButton
            app.ImportButton = uibutton(app.GridLayout1, 'push');
            app.ImportButton.ButtonPushedFcn = createCallbackFcn(app, @ImportButtonPushed, true);
            app.ImportButton.Layout.Row = 1;
            app.ImportButton.Layout.Column = 4;
            app.ImportButton.Text = '导出布局';

            % Create OpenButton
            app.OpenButton = uibutton(app.GridLayout1, 'push');
            app.OpenButton.ButtonPushedFcn = createCallbackFcn(app, @OpenButtonPushed, true);
            app.OpenButton.Layout.Row = 1;
            app.OpenButton.Layout.Column = 3;
            app.OpenButton.Text = '导出参数';

            % Create Label1
            app.Label1 = uilabel(app.GridLayout1);
            app.Label1.FontSize = 22;
            app.Label1.FontWeight = 'bold';
            app.Label1.FontColor = [1 1 1];
            app.Label1.Layout.Row = 1;
            app.Label1.Layout.Column = 1;
            app.Label1.Text = 'RC-Designer  可重构性设计软件';

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = RCDesignerPrototype

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end
