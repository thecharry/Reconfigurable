classdef RCDesignerPrototype < matlab.apps.AppBase

    % Properties that correspond to app components
    % Properties that correspond to App Designer components
    % Properties that correspond to App Designer components
    % Properties that correspond to App Designer components
    % Properties that correspond to App Designer components
    % Properties that correspond to App Designer components
    % Properties that correspond to App Designer components
    % Properties that correspond to App Designer components
    % Properties that correspond to App Designer components
    % Properties that correspond to App Designer components
    % Properties that correspond to App Designer components
    properties (Access = public)
        UIFigure                         matlab.ui.Figure
        RootGrid                         matlab.ui.container.GridLayout
        Panel1                           matlab.ui.container.Panel
        GridLayout2                      matlab.ui.container.GridLayout
        Label2                           matlab.ui.control.Label
        StatusLabel                      matlab.ui.control.Label
        ProgressGauge                    matlab.ui.control.LinearGauge
        ProgressLabel                    matlab.ui.control.Label
        BodyGrid                         matlab.ui.container.GridLayout
        PageHostPanel                    matlab.ui.container.Panel
        TabGroup                         matlab.ui.container.TabGroup
        HomeTab                          matlab.ui.container.Tab
        GridLayout4                      matlab.ui.container.GridLayout
        HomeSimulationPanel              matlab.ui.container.Panel
        HomeSimulationGrid               matlab.ui.container.GridLayout
        HomeSimulationTitle              matlab.ui.control.Label
        HomeSimulationDesc               matlab.ui.control.Label
        Panel7                           matlab.ui.container.Panel
        GridLayout18                     matlab.ui.container.GridLayout
        Label23                          matlab.ui.control.Label
        Label22                          matlab.ui.control.Label
        Panel6                           matlab.ui.container.Panel
        GridLayout17                     matlab.ui.container.GridLayout
        Label21                          matlab.ui.control.Label
        Label20                          matlab.ui.control.Label
        Panel5                           matlab.ui.container.Panel
        GridLayout16                     matlab.ui.container.GridLayout
        Label19                          matlab.ui.control.Label
        Label18                          matlab.ui.control.Label
        Panel3                           matlab.ui.container.Panel
        GridLayout14                     matlab.ui.container.GridLayout
        Label15                          matlab.ui.control.Label
        Label14                          matlab.ui.control.Label
        Panel2                           matlab.ui.container.Panel
        GridLayout13                     matlab.ui.container.GridLayout
        Label13                          matlab.ui.control.Label
        Label12                          matlab.ui.control.Label
        Label3                           matlab.ui.control.Label
        ParameterTab                     matlab.ui.container.Tab
        GridLayout5                      matlab.ui.container.GridLayout
        ParameterFileGrid_2              matlab.ui.container.GridLayout
        Button2_4                        matlab.ui.control.Button
        ThrusterPanel_3                  matlab.ui.container.Panel
        ThrusterGrid_3                   matlab.ui.container.GridLayout
        Label24_28                       matlab.ui.control.Label
        Label24_27                       matlab.ui.control.Label
        Label24_26                       matlab.ui.control.Label
        Label49_3                        matlab.ui.control.Label
        FmaxField_3                      matlab.ui.control.NumericEditField
        Label50_3                        matlab.ui.control.Label
        MinPulseField_3                  matlab.ui.control.NumericEditField
        TextArea                         matlab.ui.control.TextArea
        ParameterFileGrid                matlab.ui.container.GridLayout
        Button2_2                        matlab.ui.control.Button
        Panel8                           matlab.ui.container.Panel
        GridLayout19                     matlab.ui.container.GridLayout
        MinPulseField_4                  matlab.ui.control.NumericEditField
        Label24_32                       matlab.ui.control.Label
        Label38_5                        matlab.ui.control.Label
        Label24_31                       matlab.ui.control.Label
        SimulationTimeField_4            matlab.ui.control.NumericEditField
        Label38_4                        matlab.ui.control.Label
        Label24_30                       matlab.ui.control.Label
        SimulationTimeField_3            matlab.ui.control.NumericEditField
        Label38_3                        matlab.ui.control.Label
        Label24_29                       matlab.ui.control.Label
        SimulationTimeField_2            matlab.ui.control.NumericEditField
        Label38_2                        matlab.ui.control.Label
        Label24_6                        matlab.ui.control.Label
        Label24_5                        matlab.ui.control.Label
        Label24_4                        matlab.ui.control.Label
        Label24_3                        matlab.ui.control.Label
        Label24_2                        matlab.ui.control.Label
        EditField4                       matlab.ui.control.EditField
        Label27                          matlab.ui.control.Label
        EditField3                       matlab.ui.control.EditField
        Label26                          matlab.ui.control.Label
        EditField2                       matlab.ui.control.EditField
        Label25                          matlab.ui.control.Label
        EditField1                       matlab.ui.control.EditField
        Label24                          matlab.ui.control.Label
        SimulationTimeField              matlab.ui.control.NumericEditField
        Label38                          matlab.ui.control.Label
        SpacecraftPanel                  matlab.ui.container.Panel
        SpacecraftGrid                   matlab.ui.container.GridLayout
        Label24_10                       matlab.ui.control.Label
        Label24_9                        matlab.ui.control.Label
        Label24_8                        matlab.ui.control.Label
        Label24_7                        matlab.ui.control.Label
        Label43                          matlab.ui.control.Label
        MassField                        matlab.ui.control.NumericEditField
        Label44                          matlab.ui.control.Label
        JxxField                         matlab.ui.control.NumericEditField
        Label45                          matlab.ui.control.Label
        JyyField                         matlab.ui.control.NumericEditField
        Label46                          matlab.ui.control.Label
        JzzField                         matlab.ui.control.NumericEditField
        ControllerPanel                  matlab.ui.container.Panel
        ControllerGrid                   matlab.ui.container.GridLayout
        Label24_11                       matlab.ui.control.Label
        Label47                          matlab.ui.control.Label
        ControlPeriodField               matlab.ui.control.NumericEditField
        Label4                           matlab.ui.control.Label
        OptimizationTab                  matlab.ui.container.Tab
        GridLayout7                      matlab.ui.container.GridLayout
        LayoutTemplateDropDown           matlab.ui.control.DropDown
        DeleteLayoutButton               matlab.ui.control.Button
        ApplyLayoutButton_2              matlab.ui.control.Button
        LayoutViewTabGroup               matlab.ui.container.TabGroup
        Layout3DTab                      matlab.ui.container.Tab
        Layout3DGrid                     matlab.ui.container.GridLayout
        LayoutAxes                       matlab.ui.control.UIAxes
        LayoutViewsTab                   matlab.ui.container.Tab
        LayoutViewsGrid                  matlab.ui.container.GridLayout
        LayoutXZAxes                     matlab.ui.control.UIAxes
        LayoutXYAxes                     matlab.ui.control.UIAxes
        LayoutYZAxes                     matlab.ui.control.UIAxes
        LayoutParametersTab              matlab.ui.container.Tab
        LayoutParametersGrid             matlab.ui.container.GridLayout
        LayoutTable                      matlab.ui.control.Table
        Panel10_3                        matlab.ui.container.Panel
        GridLayout21_3                   matlab.ui.container.GridLayout
        StartOptimizationButton_3        matlab.ui.control.Button
        DropDown1                        matlab.ui.control.DropDown
        Label37                          matlab.ui.control.Label
        GenerationField_3                matlab.ui.control.NumericEditField
        Label36_3                        matlab.ui.control.Label
        PopulationField                  matlab.ui.control.NumericEditField
        Label35_3                        matlab.ui.control.Label
        ThrusterPanel_2                  matlab.ui.container.Panel
        ThrusterGrid_2                   matlab.ui.container.GridLayout
        EditField1_5                     matlab.ui.control.EditField
        EditField1_4                     matlab.ui.control.EditField
        EditField1_3                     matlab.ui.control.EditField
        EditField1_2                     matlab.ui.control.EditField
        Label24_37                       matlab.ui.control.Label
        Label33_6                        matlab.ui.control.Label
        Label24_36                       matlab.ui.control.Label
        Label33_5                        matlab.ui.control.Label
        Label24_35                       matlab.ui.control.Label
        Label33_4                        matlab.ui.control.Label
        Label24_34                       matlab.ui.control.Label
        Label33_3                        matlab.ui.control.Label
        Label24_33                       matlab.ui.control.Label
        FmaxField_4                      matlab.ui.control.NumericEditField
        Label49_4                        matlab.ui.control.Label
        Label24_25                       matlab.ui.control.Label
        NumericEditField2_6              matlab.ui.control.NumericEditField
        Label34_6                        matlab.ui.control.Label
        Label24_24                       matlab.ui.control.Label
        NumericEditField2_5              matlab.ui.control.NumericEditField
        Label34_5                        matlab.ui.control.Label
        Label24_23                       matlab.ui.control.Label
        Label24_22                       matlab.ui.control.Label
        Label24_20                       matlab.ui.control.Label
        Label24_19                       matlab.ui.control.Label
        Label49_2                        matlab.ui.control.Label
        FmaxField_2                      matlab.ui.control.NumericEditField
        NumericEditField1_2              matlab.ui.control.NumericEditField
        Label33_2                        matlab.ui.control.Label
        NumericEditField2_4              matlab.ui.control.NumericEditField
        Label34_4                        matlab.ui.control.Label
        Label7                           matlab.ui.control.Label
        EvaluationTab                    matlab.ui.container.Tab
        GridLayout8                      matlab.ui.container.GridLayout
        Panel10_2                        matlab.ui.container.Panel
        GridLayout21_2                   matlab.ui.container.GridLayout
        FaultCountDropDown               matlab.ui.control.DropDown
        StartOptimizationButton_2        matlab.ui.control.Button
        JudgmentLayoutDropDown           matlab.ui.control.DropDown
        GenerationField_2                matlab.ui.control.EditField
        JudgmentLayoutLabel              matlab.ui.control.Label
        Label36_2                        matlab.ui.control.Label
        Label35_2                        matlab.ui.control.Label
        JudgmentViewTabGroup             matlab.ui.container.TabGroup
        JudgmentDetailTab                matlab.ui.container.Tab
        JudgmentDetailGrid               matlab.ui.container.GridLayout
        EvaluationTable                  matlab.ui.control.Table
        JudgmentSummaryTab               matlab.ui.container.Tab
        JudgmentSummaryGrid              matlab.ui.container.GridLayout
        JudgmentSummaryTable             matlab.ui.control.Table
        Label8                           matlab.ui.control.Label
        ReconfigDesignTab                matlab.ui.container.Tab
        GridLayout25                     matlab.ui.container.GridLayout
        MetricsEvaluateButton            matlab.ui.control.Button
        Label41                          matlab.ui.control.Label
        MetricsViewTabGroup              matlab.ui.container.TabGroup
        MetricsControlTab                matlab.ui.container.Tab
        MetricsControlGrid               matlab.ui.container.GridLayout
        MetricsTorqueAxes                matlab.ui.control.UIAxes
        MetricsForceAxes                 matlab.ui.control.UIAxes
        MetricsAnglesTab                 matlab.ui.container.Tab
        MetricsAnglesGrid                matlab.ui.container.GridLayout
        MetricsAngleAxes1                matlab.ui.control.UIAxes
        MetricsAngleAxes2                matlab.ui.control.UIAxes
        MetricsAngleAxes3                matlab.ui.control.UIAxes
        MetricsChartsTab                 matlab.ui.container.Tab
        MetricsChartsGrid                matlab.ui.container.GridLayout
        MetricsJcAxes                    matlab.ui.control.UIAxes
        MetricsJoAxes                    matlab.ui.control.UIAxes
        MetricsJtAxes                    matlab.ui.control.UIAxes
        MetricsJfAxes                    matlab.ui.control.UIAxes
        MetricsSingleFaultTab            matlab.ui.container.Tab
        MetricsSingleFaultGrid           matlab.ui.container.GridLayout
        SingleFaultInfoLabel             matlab.ui.control.Label
        SingleFaultTable                 matlab.ui.control.Table
        AllocationTab                    matlab.ui.container.Tab
        GridLayout9                      matlab.ui.container.GridLayout
        Label9                           matlab.ui.control.Label
        AllocationViewTabGroup           matlab.ui.container.TabGroup
        AllocationStrategyTab            matlab.ui.container.Tab
        AllocationStrategyGrid           matlab.ui.container.GridLayout
        AllocationStrategyTable          matlab.ui.control.Table
        AllocationVerificationTab        matlab.ui.container.Tab
        AllocationVerificationGrid       matlab.ui.container.GridLayout
        AllocationVerificationSummaryLabel matlab.ui.control.Label
        AllocationVerificationTable      matlab.ui.control.Table
        Panel10_4                        matlab.ui.container.Panel
        GridLayout21_4                   matlab.ui.container.GridLayout
        JudgmentLayoutDropDown_2         matlab.ui.control.DropDown
        GenerationField_4                matlab.ui.control.EditField
        JudgmentLayoutLabel_2            matlab.ui.control.Label
        Label36_4                        matlab.ui.control.Label
        Label35_4                        matlab.ui.control.Label
        AllocationModeDropDown           matlab.ui.control.DropDown
        GenerateAllocationButton         matlab.ui.control.Button
        Label35_5                        matlab.ui.control.Label
        AllocationModeDropDown_2         matlab.ui.control.DropDown
        SimulationTab                    matlab.ui.container.Tab
        GridLayout10                     matlab.ui.container.GridLayout
        Label10                          matlab.ui.control.Label
        SimulationViewTabGroup           matlab.ui.container.TabGroup
        SimulationPositionTab            matlab.ui.container.Tab
        SimulationPositionGrid           matlab.ui.container.GridLayout
        SimulationAxes                   matlab.ui.control.UIAxes
        SimulationAttitudeAxes           matlab.ui.control.UIAxes
        SimulationPositionErrorAxes      matlab.ui.control.UIAxes
        SimulationAttitudeErrorAxes      matlab.ui.control.UIAxes
        SimulationFaultCompareTab        matlab.ui.container.Tab
        SimulationFaultGrid              matlab.ui.container.GridLayout
        SimulationFaultPosXAxes          matlab.ui.control.UIAxes
        SimulationFaultPosYAxes          matlab.ui.control.UIAxes
        SimulationFaultPosZAxes          matlab.ui.control.UIAxes
        SimulationFaultAttXAxes          matlab.ui.control.UIAxes
        SimulationFaultAttYAxes          matlab.ui.control.UIAxes
        SimulationFaultAttZAxes          matlab.ui.control.UIAxes
        SimulationCommandTab             matlab.ui.container.Tab
        SimulationCommandGrid            matlab.ui.container.GridLayout
        SimulationCommandCaseTabGroup    matlab.ui.container.TabGroup
        SimulationNominalCommandTab      matlab.ui.container.Tab
        SimulationNominalCommandGrid     matlab.ui.container.GridLayout
        SimulationForceXAxes             matlab.ui.control.UIAxes
        SimulationForceYAxes             matlab.ui.control.UIAxes
        SimulationForceZAxes             matlab.ui.control.UIAxes
        SimulationTorqueXAxes            matlab.ui.control.UIAxes
        SimulationTorqueYAxes            matlab.ui.control.UIAxes
        SimulationTorqueZAxes            matlab.ui.control.UIAxes
        SimulationFaultCommandTab        matlab.ui.container.Tab
        SimulationFaultCommandGrid       matlab.ui.container.GridLayout
        SimulationFaultForceXAxes        matlab.ui.control.UIAxes
        SimulationFaultForceYAxes        matlab.ui.control.UIAxes
        SimulationFaultForceZAxes        matlab.ui.control.UIAxes
        SimulationFaultTorqueXAxes       matlab.ui.control.UIAxes
        SimulationFaultTorqueYAxes       matlab.ui.control.UIAxes
        SimulationFaultTorqueZAxes       matlab.ui.control.UIAxes
        SimulationTrajectoryTab          matlab.ui.container.Tab
        SimulationTrajectoryGrid         matlab.ui.container.GridLayout
        SimulationTrajectoryCaseTabGroup matlab.ui.container.TabGroup
        SimulationNominalTrajectoryTab   matlab.ui.container.Tab
        SimulationNominalTrajectoryGrid  matlab.ui.container.GridLayout
        SimulationNominalTrajectory3DAxes matlab.ui.control.UIAxes
        SimulationNominalTrajectoryXYAxes matlab.ui.control.UIAxes
        SimulationNominalTrajectoryXZAxes matlab.ui.control.UIAxes
        SimulationNominalTrajectoryYZAxes matlab.ui.control.UIAxes
        SimulationFaultTrajectoryTab     matlab.ui.container.Tab
        SimulationFaultTrajectoryGrid    matlab.ui.container.GridLayout
        SimulationFaultTrajectory3DAxes  matlab.ui.control.UIAxes
        SimulationFaultTrajectoryXYAxes  matlab.ui.control.UIAxes
        SimulationFaultTrajectoryXZAxes  matlab.ui.control.UIAxes
        SimulationFaultTrajectoryYZAxes  matlab.ui.control.UIAxes
        SimulationPulseTab               matlab.ui.container.Tab
        SimulationPulseGrid              matlab.ui.container.GridLayout
        SimulationPulsePlaceholderAxes   matlab.ui.control.UIAxes
        SimulationScheduleTab            matlab.ui.container.Tab
        SimulationScheduleGrid           matlab.ui.container.GridLayout
        SimulationScheduleControlGrid    matlab.ui.container.GridLayout
        SimulationScheduleStartTimeLabel matlab.ui.control.Label
        SimulationScheduleStartTimeField matlab.ui.control.NumericEditField
        SimulationScheduleCycleCountLabel matlab.ui.control.Label
        SimulationScheduleCycleCountField matlab.ui.control.NumericEditField
        SimulationScheduleAxes           matlab.ui.control.UIAxes
        Panel10_5                        matlab.ui.container.Panel
        GridLayout21_5                   matlab.ui.container.GridLayout
        RunSimulationButton              matlab.ui.control.Button
        Label36_5                        matlab.ui.control.Label
        GenerationField_5                matlab.ui.control.EditField
        Label36_6                        matlab.ui.control.Label
        SimulationLayoutDropDown         matlab.ui.control.DropDown
        GenerationField_6                matlab.ui.control.EditField
        Label36_7                        matlab.ui.control.Label
        Label24_38                       matlab.ui.control.Label
        ControllerPanel_2                matlab.ui.container.Panel
        ControllerGrid_2                 matlab.ui.container.GridLayout
        Label47_2                        matlab.ui.control.Label
        ControlPeriodField_2             matlab.ui.control.NumericEditField
        Label47_3                        matlab.ui.control.Label
        ControlPeriodField_3             matlab.ui.control.NumericEditField
        Label47_4                        matlab.ui.control.Label
        ControlPeriodField_4             matlab.ui.control.NumericEditField
        Label47_5                        matlab.ui.control.Label
        ControlPeriodField_5             matlab.ui.control.NumericEditField
        Label47_6                        matlab.ui.control.Label
        ControlPeriodField_6             matlab.ui.control.NumericEditField
        NavigationPanel                  matlab.ui.container.Panel
        GridLayout3                      matlab.ui.container.GridLayout
        Button1                          matlab.ui.control.Button
        NavigationList                   matlab.ui.control.ListBox
        HeaderPanel                      matlab.ui.container.Panel
        GridLayout1                      matlab.ui.container.GridLayout
        ImportButton                     matlab.ui.control.Button
        OpenButton                       matlab.ui.control.Button
        Label1                           matlab.ui.control.Label
    end

    properties (Access = private)
        ProjectRoot
        CurrentParams
        ReferenceB
        ReferenceR
        CurrentB
        CurrentR
        OptimResult
        ReconfigJudgmentResult
        EvaluationResult
        SimulationLog
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
    end
    
    methods (Access = private)

        function initializeApp(app)
            app.ProjectRoot = app.resolveProjectRoot();
            addpath(app.ProjectRoot, '-begin');
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
            app.StartOptimizationButton_2.ButtonPushedFcn = createCallbackFcn(app, @EvaluateButtonPushed, true);
            app.MetricsEvaluateButton.ButtonPushedFcn = createCallbackFcn(app, @MetricsEvaluateButtonPushed, true);
            app.GenerateAllocationButton.ButtonPushedFcn = createCallbackFcn(app, @GenerateAllocationButtonPushed, true);
            app.AllocationModeDropDown.ValueChangedFcn = createCallbackFcn(app, @AllocationModeDropDownValueChanged, true);
            app.SimulationScheduleStartTimeField.ValueChangedFcn = createCallbackFcn(app, @SimulationScheduleWindowValueChanged, true);
            app.SimulationScheduleCycleCountField.ValueChangedFcn = createCallbackFcn(app, @SimulationScheduleWindowValueChanged, true);
            app.RunSimulationButton.ButtonPushedFcn = createCallbackFcn(app, @RunSimulationButtonPushed, true);
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
            app.syncAllocationStrategyControls();
            app.TextArea.Value = {''};
            app.refreshLayoutDropDown('原布局');
            app.prepareDiagnosticAxes(1);
            app.populateLayoutTable(app.CurrentB, app.CurrentR);
            app.drawLayout();
            app.updateAllocationTable([]);
            app.updateAllocationStrategyTable([]);
            app.updateAllocationVerification([]);
            app.ProgressStartTic = [];
            app.ProgressGauge.Value = 0;
            app.ProgressLabel.Text = '等待运行｜0%｜耗时 00:00';
            app.updateReport("已载入默认参数和原始推力器布局。");
            app.setStatus('就绪｜已载入默认参数');
        end

        function populateParameterControls(app)
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
            app.SimulationTimeField_2.Value = app.paramOrDefault(params, 'R_earth', 6371000);
            app.SimulationTimeField_3.Value = app.paramOrDefault(params, 'h_orbit', 400000);
            app.SimulationTimeField_4.Value = app.paramOrDefault(params, 'mu', 3.986e14);
            app.ControlPeriodField_2.Value = app.paramOrDefault(params, 'Kp_pos', 10);
            app.ControlPeriodField_3.Value = app.paramOrDefault(params, 'Kd_pos', 300);
            app.ControlPeriodField_4.Value = app.paramOrDefault(params, 'Kp_att', 400);
            app.ControlPeriodField_5.Value = app.paramOrDefault(params, 'Ki_att', 20);
            app.ControlPeriodField_6.Value = app.paramOrDefault(params, 'Kd_att', 3200);
            app.FmaxField_4.Value = 2;
            app.NumericEditField1_2.Value = 0.6;
            app.NumericEditField2_4.Value = 0.6;
            app.NumericEditField2_5.Value = 0;
            app.NumericEditField2_6.Value = 0;
            app.PopulationField.Value = 500;
            app.GenerationField_3.Value = 50;
            app.GenerationField_2.Value = '1';
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
            params.R_earth = app.SimulationTimeField_2.Value;
            params.h_orbit = app.SimulationTimeField_3.Value;
            params.mu = app.SimulationTimeField_4.Value;
            params.n = sqrt(params.mu / (params.R_earth + params.h_orbit)^3);
            params.Kp_pos = app.ControlPeriodField_2.Value;
            params.Kd_pos = app.ControlPeriodField_3.Value;
            params.Kp_att = app.ControlPeriodField_4.Value;
            params.Ki_att = app.ControlPeriodField_5.Value;
            params.Kd_att = app.ControlPeriodField_6.Value;

            if ~isempty(app.CurrentB) && size(app.CurrentB, 2) == params.Num
                params.B_all = app.CurrentB;
                params.r_all = app.CurrentR;
            elseif ~isempty(app.ReferenceB) && size(app.ReferenceB, 2) == params.Num
                params.B_all = app.ReferenceB;
                params.r_all = app.ReferenceR;
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
                app.LayoutTemplateDropDown.Items = {'原布局'};
                app.LayoutTemplateDropDown.Value = '原布局';
                app.refreshJudgmentLayoutDropDown();
                return;
            end
            names = {app.LayoutEntries.Name};
            app.LayoutTemplateDropDown.Items = names;
            if nargin < 2 || ~any(strcmp(names, selectedName))
                selectedName = names{1};
            end
            app.LayoutTemplateDropDown.Value = selectedName;
            app.refreshJudgmentLayoutDropDown();
        end

        function refreshJudgmentLayoutDropDown(app)
            if isempty(app.JudgmentLayoutDropDown) || ...
                    ~isvalid(app.JudgmentLayoutDropDown)
                return;
            end
            if isempty(app.LayoutEntries)
                names = {'原布局'};
            else
                names = {app.LayoutEntries.Name};
            end
            oldValue = app.JudgmentLayoutDropDown.Value;
            app.JudgmentLayoutDropDown.Items = names;
            if any(strcmp(names, oldValue))
                app.JudgmentLayoutDropDown.Value = oldValue;
            else
                app.JudgmentLayoutDropDown.Value = names{1};
            end

            app.refreshNamedLayoutDropDown(app.JudgmentLayoutDropDown_2, names);
            app.refreshNamedLayoutDropDown(app.SimulationLayoutDropDown, names);
        end

        function refreshNamedLayoutDropDown(app, dropDown, names)
            if isempty(dropDown) || ~isvalid(dropDown)
                return;
            end
            oldValue = dropDown.Value;
            dropDown.Items = names;
            if any(strcmp(names, oldValue))
                dropDown.Value = oldValue;
            else
                dropDown.Value = names{1};
            end
        end

        function layout = selectedJudgmentLayout(app)
            selectedName = app.JudgmentLayoutDropDown.Value;
            index = find(strcmp({app.LayoutEntries.Name}, selectedName), 1, 'first');
            if isempty(index)
                error('未找到判断页选择的布局“%s”。', selectedName);
            end
            layout = struct('name', app.LayoutEntries(index).Name, ...
                'B', app.LayoutEntries(index).B, 'r', app.LayoutEntries(index).r);
        end

        function layout = selectedAllocationLayout(app)
            selectedName = app.JudgmentLayoutDropDown_2.Value;
            index = find(strcmp({app.LayoutEntries.Name}, selectedName), 1, 'first');
            if isempty(index)
                error('未找到调用策略页选择的布局“%s”。', selectedName);
            end
            layout = struct('name', app.LayoutEntries(index).Name, ...
                'B', app.LayoutEntries(index).B, 'r', app.LayoutEntries(index).r);
        end

        function layout = selectedSimulationLayout(app)
            selectedName = app.SimulationLayoutDropDown.Value;
            index = find(strcmp({app.LayoutEntries.Name}, selectedName), 1, 'first');
            if isempty(index)
                error('未找到闭环仿真页选择的布局“%s”。', selectedName);
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

        function [B, r] = readLayoutFile(app, filePath)
            [~, ~, ext] = fileparts(filePath);
            if strcmpi(ext, '.mat')
                data = load(filePath);
                if isfield(data, 'B_opt') && isfield(data, 'r_opt')
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
                else
                    error('MAT 文件中未找到 B/r、B_opt/r_opt 或 B_all/r_all。');
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
            faultyIndices = [];
            if ~isempty(app.GenerationField_2)
                faultyIndices = app.parseIndexList(app.GenerationField_2.Value);
            end
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
            simCfg.Kp_pos = app.ControlPeriodField_2.Value;
            simCfg.Kd_pos = app.ControlPeriodField_3.Value;
            simCfg.Kp_att = app.ControlPeriodField_4.Value;
            simCfg.Ki_att = app.ControlPeriodField_5.Value;
            simCfg.Kd_att = app.ControlPeriodField_6.Value;
            simCfg.T_sim = app.SimulationTimeField.Value;
            simCfg.faulty_time = app.toScalar(app.GenerationField_5.Value, NaN);
            if ~isfinite(simCfg.faulty_time) || simCfg.faulty_time < 0 || ...
                    simCfg.faulty_time > simCfg.T_sim
                error('故障时刻应为0至仿真时间 %.3g s之间的数值。', simCfg.T_sim);
            end
            simCfg.dt = max(eps, app.MinPulseField_4.Value);
            simCfg.true_faults = app.parseIndexList(app.GenerationField_6.Value);
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
            app.TextArea.Value = cellstr(app.parameterTableToText(tableData, "新参数"));
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

        function faultSets = getFaultCases(app, params)
            typedFaults = app.parseIndexList(app.GenerationField_2.Value);
            if any(typedFaults > params.Num)
                error('故障编号不能大于当前布局的推力器总数 %d。', params.Num);
            end
            switch app.FaultCountDropDown.Value
                case '两台故障'
                    faultCount = 2;
                case '自定义故障组合'
                    faultCount = numel(typedFaults);
                    if faultCount == 0
                        error('自定义故障组合不能为空，请输入推力器编号，例如 [1 3]。');
                    end
                    faultSets = {typedFaults};
                    return;
                otherwise
                    faultCount = 1;
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

        function results = evaluateReconfigJudgment(app, layoutName, B, params, faultSets)
            layoutParams = params;
            layoutParams.Num = size(B, 2);
            template = struct('LayoutName', '', 'FaultSet', '', ...
                'JcForce', NaN, 'JcTorque', NaN, 'Jc6D', NaN, ...
                'Conclusion', '');
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
                    isReconfig = forceMargin > 1e-8 && torqueMargin > 1e-8 && jointMargin > 1e-8;
                    item = template;
                    item.LayoutName = char(layoutName);
                    item.FaultSet = app.faultSetText(faultSet);
                    item.JcForce = forceMargin;
                    item.JcTorque = torqueMargin;
                    item.Jc6D = jointMargin;
                    if isReconfig
                        item.Conclusion = '可重构';
                    else
                        item.Conclusion = '不可重构';
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

        function data = judgmentToCell(app, results)
            data = cell(numel(results), 5);
            for index = 1:numel(results)
                data(index, :) = {results(index).FaultSet, ...
                    results(index).JcForce, results(index).JcTorque, ...
                    results(index).Jc6D, results(index).Conclusion};
            end
        end

        function highlightUnreconfigurableRows(app, results)
            % 每次判断后重新生成样式，避免切换故障条件时旧行仍被标红。
            try
                removeStyle(app.EvaluationTable);
                if isempty(results)
                    return;
                end
                conclusions = string({results.Conclusion});
                rowNumbers = find(contains(conclusions, "不可重构"));
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

        function result = evaluateLayoutSummary(app, name, B, params, faultLevel)
            layoutParams = params;
            layoutParams.Num = size(B, 2);
            if faultLevel > layoutParams.Num
                error('故障数量不能大于布局“%s”的推力器总数。', char(name));
            end
            [~, Jc1, ~, Jc, Jo, Jt, Jf] = Reconfig_eval(layoutParams, B, faultLevel);
            result = struct();
            result.Name = char(name);
            result.Jc = min(Jc);
            result.JcForce = min(Jc1(:, 1));
            result.JcTorque = min(Jc1(:, 2));
            result.Jo = min(Jo);
            result.Jt = mean(Jt);
            result.Jf = mean(Jf);
            result.Score = result.Jc + 0.1 * result.Jo + result.Jt - 0.01 * result.Jf;
        end

        function data = summaryToCell(app, summaries)
            data = cell(numel(summaries), 6);
            for index = 1:numel(summaries)
                data(index, :) = {summaries(index).Name, summaries(index).Jc, ...
                    summaries(index).Jo, summaries(index).Jt, summaries(index).Jf, ...
                    summaries(index).Score};
            end
        end

        function updateAllocationTable(app, info)
            if ~isprop(app, 'AllocationTable')
                return;
            end
            if isempty(info)
                app.AllocationTable.Data = {
                    '轨控脉宽', '-', '等待生成';
                    '姿控脉宽', '-', '等待生成';
                    '最终复用脉宽', '-', '等待生成'};
                return;
            end
            if isfield(info, 'mode') && ...
                    strcmp(info.mode, 'joint_optimization')
                app.AllocationTable.Data = {
                    '联合六维脉宽', app.activeThrusterText(info.Prop_6D), '直接匹配六维目标控制量';
                    '轨控/姿控拆分', '-', '联合优化不预先分成两类纯控制组';
                    '最终调用脉宽', app.activeThrusterText(info.Prop_Final), '每台推力器在本周期的总喷气时间';
                    '分配残差', sprintf('%.3e', norm(info.residual)), '指令与实际控制效果的差值'};
            else
                app.AllocationTable.Data = {
                    '轨控脉宽', app.activeThrusterText(info.Prop_F_used), '由位置控制需求得到';
                    '姿控脉宽', app.activeThrusterText(info.Prop_T_used), '由姿态控制需求得到';
                    '最终复用脉宽', app.activeThrusterText(info.Prop_Final), '轨控与姿控复用后的实际指令';
                    '分配残差', sprintf('%.3e', norm(info.residual)), '指令与实际控制效果的差值'};
            end
        end

        function updateAllocationStrategyTable(app, strategy)
            if isempty(strategy)
                axesText = {'+X'; '-X'; '+Y'; '-Y'; '+Z'; '-Z'; ...
                    '+X'; '-X'; '+Y'; '-Y'; '+Z'; '-Z'};
                controlType = [repmat({'轨道控制'}, 6, 1); ...
                    repmat({'姿态控制'}, 6, 1)];
                app.AllocationStrategyTable.ColumnName = ...
                    {'控制类型', '轴向', '标况主份', '主份配对', ...
                     '故障下调用', '故障关停/状态'};
                app.AllocationStrategyTable.Data = [controlType, axesText, ...
                    repmat({'等待生成'}, 12, 4)];
                app.applyAllocationControlColors();
                return;
            end
            app.AllocationStrategyTable.ColumnName = strategy.ColumnNames;
            app.AllocationStrategyTable.Data = strategy.Data;
            app.applyAllocationControlColors();
        end

        function updateAllocationVerification(app, verification)
            % “解耦验证”表不展示某一时刻的轨道/姿态响应，而是直接
            % 展示调用器分离出的两部分脉冲在一个控制周期内的六维效果。
            if isempty(verification)
                app.AllocationVerificationSummaryLabel.Text = '等待生成调用策略';
                app.AllocationVerificationTable.ColumnName = { ...
                    '工况', '可重构', '姿控缩放', '轨控缩放', ...
                    '平均轨控残余力矩', '平均姿控残余力', ...
                    '瞬时轨控残余力矩峰值', '瞬时姿控残余力峰值', ...
                    '周期平均解耦', '最大脉宽', '执行状态'};
                app.AllocationVerificationTable.Data = { ...
                    '等待生成', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-'};
                return;
            end
            app.AllocationVerificationSummaryLabel.Text = verification.Summary;
            app.AllocationVerificationTable.ColumnName = verification.ColumnNames;
            app.AllocationVerificationTable.Data = verification.Data;
        end

        function verification = buildAllocationVerification(app, params, ...
                layout, faultCases)
            % 用固定的六轴联合负载展示策略的关键性质：姿控优先时的
            % 整体缩放，以及 B*u_F、B*u_T 中交叉分量是否为零。
            matrixConf = params.F_max * layout.B;
            command = app.allocationVerificationCommand(matrixConf);
            verificationCases = [{[]}, faultCases(:)'];
            rowCount = numel(verificationCases);
            data = cell(rowCount, 11);
            maxOrbitCrossTorque = 0;
            maxAttitudeCrossForce = 0;
            maxInstantOrbitCrossTorque = 0;
            maxInstantAttitudeCrossForce = 0;
            reconfigurableCount = 0;

            for caseIndex = 1:rowCount
                faults = verificationCases{caseIndex};
                [pulse, info] = Thruster_allocator( ...
                    command(1:3), command(4:6), matrixConf, faults, params);
                if isfinite(info.orbit_cross_torque)
                    maxOrbitCrossTorque = max(maxOrbitCrossTorque, ...
                        info.orbit_cross_torque);
                end
                if isfinite(info.attitude_cross_force)
                    maxAttitudeCrossForce = max(maxAttitudeCrossForce, ...
                        info.attitude_cross_force);
                end
                if isfield(info, 'scheduler')
                    instantOrbitCrossTorque = ...
                        info.scheduler.peak_orbit_torque;
                    instantAttitudeCrossForce = ...
                        info.scheduler.peak_attitude_force;
                else
                    [instantOrbitCrossTorque, instantAttitudeCrossForce] = ...
                        app.allocationInstantaneousCross( ...
                        matrixConf, info.Prop_F_used, ...
                        info.Prop_T_used, params.T);
                end
                if isfinite(instantOrbitCrossTorque)
                    maxInstantOrbitCrossTorque = max( ...
                        maxInstantOrbitCrossTorque, instantOrbitCrossTorque);
                end
                if isfinite(instantAttitudeCrossForce)
                    maxInstantAttitudeCrossForce = max( ...
                        maxInstantAttitudeCrossForce, instantAttitudeCrossForce);
                end
                reconfigurableCount = reconfigurableCount + ...
                    double(info.structurally_reconfigurable);

                if isempty(faults)
                    caseText = '标况';
                else
                    caseText = ['故障', app.faultSetText(faults)];
                end
                if info.structurally_reconfigurable
                    reconfigurableText = '是';
                else
                    reconfigurableText = '否';
                end
                if info.period_average_decoupled && ...
                        info.structurally_reconfigurable
                    decoupledText = '是';
                else
                    decoupledText = '否';
                end
                data(caseIndex, :) = {caseText, reconfigurableText, ...
                    app.allocationMetricText(info.attitude_scale, '%.3f'), ...
                    app.allocationMetricText(info.force_scale, '%.3f'), ...
                    app.allocationMetricText(info.orbit_cross_torque, '%.3e N m'), ...
                    app.allocationMetricText(info.attitude_cross_force, '%.3e N'), ...
                    app.allocationMetricText(instantOrbitCrossTorque, '%.3e N m'), ...
                    app.allocationMetricText(instantAttitudeCrossForce, '%.3e N'), ...
                    decoupledText, sprintf('%.4f s', max(pulse)), ...
                    app.allocationVerificationStatus(info, ...
                    instantOrbitCrossTorque, instantAttitudeCrossForce)};
            end

            verification.Data = data;
            verification.ColumnNames = {'工况', '可重构', ...
                '姿控缩放', '轨控缩放', '平均轨控残余力矩', ...
                '平均姿控残余力', '瞬时轨控残余力矩峰值', ...
                '瞬时姿控残余力峰值', '周期平均解耦', '最大脉宽', '执行状态'};
            verification.Summary = sprintf([ ...
                '六轴联合验证｜%d/%d 工况可重构｜轨控附带力矩最大 %.3e N m｜', ...
                '姿控附带力最大 %.3e N｜瞬时残余峰值 %.3e N m / %.3e N'], ...
                reconfigurableCount, rowCount, maxOrbitCrossTorque, ...
                maxAttitudeCrossForce, maxInstantOrbitCrossTorque, ...
                maxInstantAttitudeCrossForce);
        end

        function command = allocationVerificationCommand(~, matrixConf)
            % 每一维均有非零指令；0.70 的归一化幅值会使原布局出现
            % 轨控容量缩放，便于在界面中直观看到姿控优先机制。
            rowScale = max(abs(matrixConf), [], 2);
            rowScale(rowScale < 1e-12) = 1;
            direction = [1; -0.8; 0.6; 0.7; -0.6; 1];
            command = 0.70 * rowScale .* direction;
        end

        function [peakOrbitTorque, peakAttitudeForce] = ...
                allocationInstantaneousCross(~, matrixConf, ...
                orbitPulse, attitudePulse, controlPeriod)
            % 当前闭环仿真中每台推力器在周期起点开始喷气，在其脉宽
            % 结束时关闭。这里按全部开关时刻分段，计算每一段的瞬时
            % 交叉控制量。它与周期平均量不同，能暴露脉宽不等时的
            % 瞬时姿轨耦合。
            timeEdges = unique([0; orbitPulse(:); attitudePulse(:); ...
                controlPeriod]);
            peakOrbitTorque = 0;
            peakAttitudeForce = 0;
            for edgeIndex = 1:(numel(timeEdges) - 1)
                timeSample = 0.5 * (timeEdges(edgeIndex) + ...
                    timeEdges(edgeIndex + 1));
                orbitWrench = matrixConf * double(orbitPulse > timeSample);
                attitudeWrench = matrixConf * ...
                    double(attitudePulse > timeSample);
                peakOrbitTorque = max(peakOrbitTorque, ...
                    norm(orbitWrench(4:6)));
                peakAttitudeForce = max(peakAttitudeForce, ...
                    norm(attitudeWrench(1:3)));
            end
        end

        function text = allocationVerificationStatus(~, info, ...
                peakOrbitTorque, peakAttitudeForce)
            if isfield(info, 'mode') && strcmp(info.mode, 'joint_optimization')
                text = '联合六维优化（不拆分姿轨纯控制组）';
                return;
            end
            axisNames = {'Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz'};
            skippedAxes = {};
            if isfield(info, 'axis_strategy')
                for controlType = 1:2
                    for axis = 1:3
                        strategy = info.axis_strategy{controlType, axis};
                        if isstruct(strategy) && ...
                                strcmp(strategy.status, 'below_minimum_pulse')
                            skippedAxes{end + 1} = axisNames{ ...
                                axis + 3 * (controlType - 1)}; %#ok<AGROW>
                        end
                    end
                end
            end
            if isfield(info, 'scheduler') && ...
                    ~isempty(info.scheduler.suppressed_force_axes)
                axisText = strjoin(axisNames( ...
                    info.scheduler.suppressed_force_axes), ',');
                text = ['峰值约束停用 ', axisText];
            elseif isfield(info, 'impulse_margin_unavailable_axes') && ...
                    ~isempty(info.impulse_margin_unavailable_axes)
                axisText = strjoin(axisNames( ...
                    info.impulse_margin_unavailable_axes), ',');
                text = ['冲量裕度不足 ', axisText];
            elseif ~info.structurally_reconfigurable
                text = '无完整成对瞬时备份';
            elseif ~isempty(skippedAxes)
                text = ['最小脉宽跳过 ', strjoin(skippedAxes, ',')];
            elseif isfield(info, 'uses_impulse_margin_fallback') && ...
                    info.uses_impulse_margin_fallback
                text = '冲量裕度耦合备用';
            elseif info.attitude_scale < 1 - 1e-9
                text = '姿控同比例缩放，轨控暂停';
            elseif info.force_scale < 1 - 1e-9
                text = '轨控同比例缩放';
            else
                text = '完整执行';
            end
            if peakOrbitTorque > 1e-9 || peakAttitudeForce > 1e-9
                text = [text, '；存在瞬时残余'];
            end
        end

        function strategy = buildAllocationStrategyComparison(app, params, ...
                layout, faultCases)
            if ~iscell(faultCases)
                faultCases = {faultCases};
            end
            if numel(faultCases) == 1
                strategy = RCPlotter.allocationComparisonData( ...
                    params, layout, faultCases{1});
                return;
            end

            rowsPerCase = 12;
            allRows = {};
            baseColumnNames = {};
            for caseIndex = 1:numel(faultCases)
                app.StatusLabel.Text = sprintf( ...
                    '正在生成全部单故障调用策略｜%d/%d', ...
                    caseIndex, numel(faultCases));
                drawnow limitrate nocallbacks;
                caseResult = RCPlotter.allocationComparisonData( ...
                    params, layout, faultCases{caseIndex});
                if isempty(baseColumnNames)
                    baseColumnNames = caseResult.ColumnNames;
                    allRows = cell(rowsPerCase * numel(faultCases), ...
                        numel(baseColumnNames) + 1);
                end
                rowCount = size(caseResult.Data, 1);
                firstRow = (caseIndex - 1) * rowsPerCase + 1;
                lastRow = firstRow + rowCount - 1;
                conditionText = ['故障', app.faultSetText(faultCases{caseIndex})];
                allRows(firstRow:lastRow, :) = [ ...
                    repmat({conditionText}, rowCount, 1), caseResult.Data];
            end
            strategy = struct('Data', {allRows}, 'ColumnNames', ...
                {{'故障工况', baseColumnNames{:}}});
        end

        function applyAllocationControlColors(app)
            if isempty(app.AllocationStrategyTable) || ...
                    ~isvalid(app.AllocationStrategyTable)
                return;
            end
            try
                removeStyle(app.AllocationStrategyTable);
                tableData = app.AllocationStrategyTable.Data;
                if isempty(tableData)
                    return;
                end
                columnNames = string(app.AllocationStrategyTable.ColumnName);
                controlColumn = find(columnNames == "控制类型", 1, 'first');
                if isempty(controlColumn)
                    return;
                end
                if istable(tableData)
                    controlTypes = string(tableData{:, controlColumn});
                else
                    controlTypes = string(tableData(:, controlColumn));
                end
                orbitRows = find(controlTypes == "轨道控制");
                attitudeRows = find(controlTypes == "姿态控制");
                if ~isempty(orbitRows)
                    orbitStyle = uistyle('BackgroundColor', [1 1 1]);
                    addStyle(app.AllocationStrategyTable, orbitStyle, ...
                        'row', orbitRows);
                end
                if ~isempty(attitudeRows)
                    attitudeStyle = uistyle( ...
                        'BackgroundColor', [0.90 0.91 0.92]);
                    addStyle(app.AllocationStrategyTable, attitudeStyle, ...
                        'row', attitudeRows);
                end
            catch
                % 表格着色失败时仍保留完整的调用策略数据。
            end
        end

        function plotSimulationResult(app, stateLogs, stateLabels, ...
                faultLogs, faultLabels)
            commonOptions = struct('ShowLegend', true, 'ShowFaultTime', true);
            RCPlotter.plotResponseComponents(app.SimulationAxes, ...
                stateLogs, stateLabels, 'pos', commonOptions);
            RCPlotter.plotResponseComponents(app.SimulationAttitudeAxes, ...
                stateLogs, stateLabels, 'att', commonOptions);
            RCPlotter.plotErrorNorm(app.SimulationPositionErrorAxes, ...
                stateLogs, stateLabels, 'pos', commonOptions);
            RCPlotter.plotErrorNorm(app.SimulationAttitudeErrorAxes, ...
                stateLogs, stateLabels, 'att', commonOptions);

            firstOptions = struct('ShowLegend', true, 'ShowFaultTime', true);
            compactOptions = struct('ShowLegend', false, 'ShowFaultTime', true);
            RCPlotter.plotFaultCaseComponent(app.SimulationFaultPosXAxes, ...
                faultLogs, faultLabels, 'pos', 1, firstOptions);
            RCPlotter.plotFaultCaseComponent(app.SimulationFaultPosYAxes, ...
                faultLogs, faultLabels, 'pos', 2, compactOptions);
            RCPlotter.plotFaultCaseComponent(app.SimulationFaultPosZAxes, ...
                faultLogs, faultLabels, 'pos', 3, compactOptions);
            RCPlotter.plotFaultCaseComponent(app.SimulationFaultAttXAxes, ...
                faultLogs, faultLabels, 'att', 1, firstOptions);
            RCPlotter.plotFaultCaseComponent(app.SimulationFaultAttYAxes, ...
                faultLogs, faultLabels, 'att', 2, compactOptions);
            RCPlotter.plotFaultCaseComponent(app.SimulationFaultAttZAxes, ...
                faultLogs, faultLabels, 'att', 3, compactOptions);

            if numel(faultLogs) >= 2
                nominalLog = faultLogs{1};
                comparisonLog = faultLogs{2};
                comparisonLabels = {faultLabels{1}, faultLabels{2}};
            elseif ~isempty(faultLogs)
                nominalLog = faultLogs{1};
                comparisonLog = faultLogs{1};
                comparisonLabels = {faultLabels{1}, faultLabels{1}};
            else
                nominalLog = stateLogs{1};
                comparisonLog = stateLogs{1};
                comparisonLabels = {stateLabels{1}, stateLabels{1}};
            end

            app.SimulationNominalCommandTab.Title = comparisonLabels{1};
            app.SimulationFaultCommandTab.Title = comparisonLabels{2};
            nominalWrenchAxes = [app.SimulationForceXAxes, ...
                app.SimulationForceYAxes, app.SimulationForceZAxes, ...
                app.SimulationTorqueXAxes, app.SimulationTorqueYAxes, ...
                app.SimulationTorqueZAxes];
            nominalWrenchOptions = struct('ShowFaultTime', false, ...
                'LineColor', [0.10, 0.42, 0.76]);
            RCPlotter.plotBodyWrenchHistory(nominalWrenchAxes, nominalLog, ...
                comparisonLabels{1}, nominalWrenchOptions);
            faultWrenchAxes = [app.SimulationFaultForceXAxes, ...
                app.SimulationFaultForceYAxes, ...
                app.SimulationFaultForceZAxes, ...
                app.SimulationFaultTorqueXAxes, ...
                app.SimulationFaultTorqueYAxes, ...
                app.SimulationFaultTorqueZAxes];
            faultWrenchOptions = struct('ShowFaultTime', true, ...
                'LineColor', [0.90, 0.42, 0.16]);
            RCPlotter.plotBodyWrenchHistory(faultWrenchAxes, comparisonLog, ...
                comparisonLabels{2}, faultWrenchOptions);

            trajectoryOptions = struct('BodyHalfSize', ...
                app.currentBodyHalfSize(), 'SnapshotCount', 9, ...
                'AutoScaleBody', true);
            app.SimulationNominalTrajectoryTab.Title = comparisonLabels{1};
            app.SimulationFaultTrajectoryTab.Title = comparisonLabels{2};
            nominalTrajectoryAxes = [ ...
                app.SimulationNominalTrajectory3DAxes, ...
                app.SimulationNominalTrajectoryXYAxes, ...
                app.SimulationNominalTrajectoryXZAxes, ...
                app.SimulationNominalTrajectoryYZAxes];
            RCPlotter.plotTrajectoryViews(nominalTrajectoryAxes, ...
                nominalLog, comparisonLabels{1}, trajectoryOptions);
            faultTrajectoryAxes = [ ...
                app.SimulationFaultTrajectory3DAxes, ...
                app.SimulationFaultTrajectoryXYAxes, ...
                app.SimulationFaultTrajectoryXZAxes, ...
                app.SimulationFaultTrajectoryYZAxes];
            RCPlotter.plotTrajectoryViews(faultTrajectoryAxes, ...
                comparisonLog, comparisonLabels{2}, trajectoryOptions);

            if ~isempty(stateLogs)
                pulseLog = stateLogs{1};
                pulseAxes = app.prepareThrusterPulseAxes( ...
                    size(pulseLog.Pulse_Widths, 1));
                showPulseFaultTime = isfield(pulseLog, 'faulty_thrusters') && ...
                    ~isempty(pulseLog.faulty_thrusters);
                pulseOptions = struct('ShowFaultTime', showPulseFaultTime, ...
                    'UseControlHistory', true, ...
                    'LayoutName', stateLabels{1});
                RCPlotter.plotThrusterPulseWidths( ...
                    pulseAxes, pulseLog, pulseOptions);
                app.SimulationScheduleStartTimeField.Value = ...
                    RCPlotter.selectFullPeriodScheduleStart(pulseLog);
                scheduleOptions = struct('ShowFaultTime', true, ...
                    'ControlPeriod', pulseLog.Control_Period, ...
                    'StartTime', app.SimulationScheduleStartTimeField.Value, ...
                    'CycleCount', app.SimulationScheduleCycleCountField.Value, ...
                    'Title', [stateLabels{1}, '｜推力器调用时序']);
                RCPlotter.plotThrusterSchedule( ...
                    app.SimulationScheduleAxes, pulseLog, scheduleOptions);
            end
        end

        function refreshSimulationSchedulePlot(app)
            if isempty(app.SimulationLog) || ...
                    ~isstruct(app.SimulationLog) || ...
                    ~isfield(app.SimulationLog, 'StateLog')
                return;
            end
            pulseLog = app.SimulationLog.StateLog;
            if ~isstruct(pulseLog) || ...
                    ~isfield(pulseLog, 'Pulse_Schedule_History') || ...
                    isempty(pulseLog.Pulse_Schedule_History)
                return;
            end

            cycleCount = max(1, round( ...
                app.SimulationScheduleCycleCountField.Value));
            app.SimulationScheduleCycleCountField.Value = cycleCount;
            scheduleOptions = struct('ShowFaultTime', true, ...
                'ControlPeriod', pulseLog.Control_Period, ...
                'StartTime', app.SimulationScheduleStartTimeField.Value, ...
                'CycleCount', cycleCount, ...
                'Title', [app.SimulationLog.SelectedLayoutName, ...
                    '｜推力器调用时序']);
            RCPlotter.plotThrusterSchedule( ...
                app.SimulationScheduleAxes, pulseLog, scheduleOptions);
        end

        function logData = runClosedloopCase(app, params, B, simCfg, faultSet)
            layoutParams = params;
            layoutParams.Num = size(B, 2);
            layoutParams.B_all = B;
            simCfg.true_faults = faultSet;
            layoutParams.true_faults = faultSet;
            evalc('logData = Closedloop_sim(layoutParams, B, simCfg);');
        end

        function [faultSets, labels] = simulationFaultCases(app, params, selectedFaults)
            faultSets = {[]};
            labels = {'标况'};
            if isempty(selectedFaults)
                for index = 1:params.Num
                    faultSets{end + 1} = index; %#ok<AGROW>
                    labels{end + 1} = sprintf('故障%d', index); %#ok<AGROW>
                end
            else
                faultSets{end + 1} = selectedFaults;
                labels{end + 1} = ['故障', app.faultSetText(selectedFaults)];
            end
        end

        function faultNumbers = selectedSummaryFaultNumbers(app, maximumFaults)
            switch app.FaultCountDropDown.Value
                case '两台故障'
                    faultNumbers = min(2, maximumFaults);
                case '自定义故障组合'
                    faultNumbers = max(1, min(maximumFaults, ...
                        numel(app.parseIndexList(app.GenerationField_2.Value))));
                otherwise
                    faultNumbers = 1;
            end
        end

        function faultLevel = getFaultLevel(app)
            switch app.FaultCountDropDown.Value
                case '两台故障'
                    faultLevel = 2;
                case '自定义故障组合'
                    faultLevel = max(1, numel(app.parseIndexList(app.GenerationField_2.Value)));
                otherwise
                    faultLevel = 1;
            end
        end

        function faultyIndices = selectedMetricsFaultSet(app, maximumIndex)
            faultyIndices = app.parseIndexList(app.GenerationField_2.Value);
            if isempty(faultyIndices)
                error('请输入评价图使用的故障推力器编号，例如1或[1 3]。');
            end
            if any(faultyIndices > maximumIndex)
                error('故障编号不能大于当前布局的推力器总数 %d。', ...
                    maximumIndex);
            end
            switch app.FaultCountDropDown.Value
                case '两台故障'
                    expectedCount = 2;
                case '自定义故障组合'
                    expectedCount = numel(faultyIndices);
                otherwise
                    expectedCount = 1;
            end
            if numel(faultyIndices) ~= expectedCount
                error('“%s”需要输入%d个故障编号，当前输入了%d个。', ...
                    app.FaultCountDropDown.Value, expectedCount, ...
                    numel(faultyIndices));
            end
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

        function strategy = mapAllocationStrategy(~, displayStrategy)
            switch char(string(displayStrategy))
                case '联合优化分配'
                    strategy = 'joint_optimization';
                otherwise
                    strategy = 'primary_backup';
            end
        end

        function syncAllocationStrategyControls(app)
            % 联合优化直接求六维占空比，不再经过轴向主备分配。因此该
            % 复用模式下将分配下拉框固定为联合优化分配，避免无效组合。
            isJoint = strcmp(app.AllocationModeDropDown.Value, '联合优化复用');
            if isJoint
                app.AllocationModeDropDown_2.Items = {'联合优化分配'};
                app.AllocationModeDropDown_2.Value = '联合优化分配';
                app.AllocationModeDropDown_2.Enable = 'off';
            else
                app.AllocationModeDropDown_2.Items = {'最简主备分配'};
                app.AllocationModeDropDown_2.Value = '最简主备分配';
                app.AllocationModeDropDown_2.Enable = 'on';
            end
        end

        function text = activeThrusterText(app, pulse)
            active = find(pulse(:) > 1e-9);
            if isempty(active)
                text = '-';
                return;
            end
            parts = strings(1, numel(active));
            for index = 1:numel(active)
                parts(index) = sprintf('%d: %.4fs', active(index), pulse(active(index)));
            end
            text = char(strjoin(parts, ', '));
        end

        function text = allocationMetricText(~, value, formatText)
            if isfinite(value)
                text = sprintf(formatText, value);
            else
                text = '-';
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
            app.OptimizationMaxGenerations = max(1, round(maxGenerations));
            app.ProgressGauge.Value = 0;
            app.ProgressLabel.Text = sprintf('第 0/%d 代｜0%%｜耗时 00:00', ...
                app.OptimizationMaxGenerations);
            app.setStatus('正在进行布局优化｜正在初始化遗传算法...');
            drawnow;
        end

        function updateOptimizationProgress(app, info)
            if isempty(app.ProgressStartTic)
                app.ProgressStartTic = tic;
            end
            elapsedSeconds = toc(app.ProgressStartTic);
            evaluationCount = 0;
            if isfield(info, 'EvaluationCount') && isfinite(info.EvaluationCount)
                evaluationCount = max(0, round(info.EvaluationCount));
            end
            if isfield(info, 'Generation') && isfinite(info.Generation)
                app.OptimizationCurrentGeneration = max(0, ...
                    min(app.OptimizationMaxGenerations, round(info.Generation)));
            end
            if isfield(info, 'Progress') && isfinite(info.Progress)
                progressValue = 100 * max(0, min(1, info.Progress));
            else
                progressValue = 100 * app.OptimizationCurrentGeneration / ...
                    app.OptimizationMaxGenerations;
            end
            app.ProgressGauge.Value = progressValue;

            elapsedText = app.formatElapsedTime(elapsedSeconds);
            app.ProgressLabel.Text = sprintf('第 %d/%d 代｜%d%%｜耗时 %s', ...
                app.OptimizationCurrentGeneration, ...
                app.OptimizationMaxGenerations, floor(progressValue), elapsedText);
            if evaluationCount > 0
                app.StatusLabel.Text = sprintf( ...
                    '正在进行布局优化｜已评估 %d 个候选布局', evaluationCount);
            else
                app.StatusLabel.Text = sprintf( ...
                    '正在进行布局优化｜第 %d/%d 代', ...
                    app.OptimizationCurrentGeneration, ...
                    app.OptimizationMaxGenerations);
            end
            drawnow limitrate nocallbacks;
        end

        function finishOptimizationProgress(app, completed)
            if isempty(app.ProgressStartTic)
                elapsedSeconds = 0;
            else
                elapsedSeconds = toc(app.ProgressStartTic);
            end
            elapsedText = app.formatElapsedTime(elapsedSeconds);
            if completed
                app.ProgressGauge.Value = 100;
                app.ProgressLabel.Text = sprintf('已完成｜100%%｜耗时 %s', elapsedText);
            else
                app.ProgressLabel.Text = sprintf('运行中止｜耗时 %s', elapsedText);
            end
            app.ProgressStartTic = [];
            drawnow;
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

        % Value changed function: installation boundary L/W
        function LayoutBoundaryValueChanged(app, event)
            if ~isempty(app.CurrentB) && ~isempty(app.CurrentR)
                app.drawLayout();
                app.setStatus('安装边界已更新｜布局外形已刷新');
            end
        end

        % Size changed function: PageHostPanel
        function PageHostPanelSizeChanged(app, event)
            app.resizeMainTabGroup();
        end

        % Value changed function: AllocationModeDropDown
        function AllocationModeDropDownValueChanged(app, event)
            app.syncAllocationStrategyControls();
        end

        % Value changed function: schedule viewing window
        function SimulationScheduleWindowValueChanged(app, event)
            app.refreshSimulationSchedulePlot();
        end

        % Button pushed function: NewButton
        function NewButtonPushed(app, event)
            app.LoadParameterButtonPushed(event);
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

        % Button pushed function: ExportButton
        function ExportButtonPushed(app, event)
            app.GenerateReportButtonPushed(event);
        end

        % Button pushed function: Button1
        function HelpButtonPushed(app, event)
            message = sprintf(['使用说明：\n', ...
                '1. 在“输入条件”页设置参数；\n', ...
                '2. 推力器数量必须为4的整数倍；L/W分别对应Y/Z安装面尺寸，d和θ为可选约束；\n', ...
                '3. 在“优化设计”页运行控制能力优先的布局优化，并在右侧查看当前布局、三视图和布局参数；\n', ...
                '4. 可用右上角下拉框切换布局，导入布局后会自动加入方案列表；\n', ...
                '5. 故障编号可输入1或[1 3]等组合，再进行可重构判断；\n', ...
                '6. 在“可重构评价”页计算 Jc、Jo、Jt、Jf 和综合指标；\n', ...
                '7. 在“调用策略”和“闭环仿真”页生成策略和仿真曲线。']);
            uialert(app.UIFigure, message, 'RC-Designer 使用说明');
        end

        % Button pushed function: Button2
        function ResetParameterButtonPushed(app, event)
            app.loadDefaultState();
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

        % Cell edited function: LayoutTable
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
                [B, r] = app.readLayoutFile(filePath);
                name = app.addLayoutEntry('imported', B, r);
                app.updateReport("已导入" + string(name) + "：" + string(filePath));
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

        % Button pushed function: StartOptimizationButton
        function StartOptimizationButtonPushed(app, event)
            try
                params = app.readParamsFromUI();
                if ~strcmp(app.DropDown1.Value, '控制能力优先')
                    error('当前版本仅实现“控制能力优先”，请选择该优化目标。');
                end
                if params.Num < 4 || params.Num ~= round(params.Num) || mod(params.Num, 4) ~= 0
                    error('推力器数量必须为4的整数倍（4n），例如4、8、12或16。');
                end
                if app.FmaxField_4.Value ~= 2
                    error('当前对称布局算法使用 +X/-X 两个安装面，“安装面X”请设为2。');
                end
                optCfg = struct();
                optCfg.population_size = max(10, round(app.PopulationField.Value));
                optCfg.max_generations = max(1, round(app.GenerationField_3.Value));
                app.beginOptimizationProgress(optCfg.max_generations);
                optCfg.y_max = app.NumericEditField1_2.Value;
                optCfg.z_max = app.NumericEditField2_4.Value;
                optCfg.position_a_range = app.parseMathRange(app.EditField1_2.Value, '位置参数 a 范围');
                optCfg.position_b_range = app.parseMathRange(app.EditField1_3.Value, '位置参数 b 范围');
                optCfg.alpha_range = app.parseMathRange(app.EditField1_4.Value, '方位角 α 范围');
                optCfg.beta_range = app.parseMathRange(app.EditField1_5.Value, '俯仰角 β 范围');
                optCfg.min_install_distance = app.NumericEditField2_5.Value;
                optCfg.min_install_angle_deg = app.NumericEditField2_6.Value;
                optCfg.use_parallel = false;
                optCfg.display = 'iter';
                optCfg.save_result = true;
                optCfg.output_dir = app.ProjectRoot;
                optCfg.progress_callback = @(info) ...
                    app.updateOptimizationProgress(info);
                evalc('result = Optim_Algorithm(params, optCfg);');
                app.OptimResult = result;
                params.Num = size(result.B_opt, 2);
                params.B_all = result.B_opt;
                params.r_all = result.r_opt;
                app.CurrentParams = params;
                name = app.addLayoutEntry('optimized', result.B_opt, result.r_opt);
                app.LayoutViewTabGroup.SelectedTab = app.Layout3DTab;
                reportLine = "优化完成：" + string(name) + ...
                    "｜来源：" + string(result.source) + ...
                    "｜目标函数值：" + string(result.fval) + ...
                    sprintf("｜计算时间：%.3f s｜", result.elapsed_time) + ...
                    string(result.message);
                if isfield(result, 'output_file') && strlength(string(result.output_file)) > 0
                    reportLine = reportLine + "｜结果文件：" + string(result.output_file);
                end
                app.updateReport(reportLine);
                app.finishOptimizationProgress(true);
                app.setStatus(['优化完成｜', char(string(result.source))]);
            catch ME
                app.finishOptimizationProgress(false);
                app.showError('优化计算失败', ME);
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
        function MetricsEvaluateButtonPushed(app, event)
            try
                app.setBusy('正在计算可重构设计指标...');
                params = app.readParamsFromUI();
                layoutSet = app.buildAllCompatibleLayoutSet(size(app.CurrentB, 2));
                params.Num = size(layoutSet(1).B, 2);
                faultyIndices = app.selectedMetricsFaultSet(params.Num);
                faultLevel = numel(faultyIndices);
                summaries = repmat(struct('Name', '', 'Jc', NaN, ...
                    'JcForce', NaN, 'JcTorque', NaN, 'Jo', NaN, ...
                    'Jt', NaN, 'Jf', NaN, 'Score', NaN), 0, 1);
                for layoutIndex = 1:numel(layoutSet)
                    item = app.evaluateLayoutSummary(layoutSet(layoutIndex).name, ...
                        layoutSet(layoutIndex).B, params, faultLevel);
                    summaries(end + 1, 1) = item; %#ok<AGROW>
                end
                app.EvaluationResult = summaries;

                comparison = RCPlotter.metricComparisonData( ...
                    params, layoutSet, faultLevel);
                metricAxes = [app.MetricsJcAxes, app.MetricsJoAxes, ...
                    app.MetricsJtAxes, app.MetricsJfAxes];
                for metricIndex = 1:4
                    RCPlotter.plotMetricBars( ...
                        metricAxes(metricIndex), comparison, metricIndex);
                end

                RCPlotter.plotControlEnvelopeComparison( ...
                    app.MetricsForceAxes, app.MetricsTorqueAxes, ...
                    params, layoutSet, faultyIndices);

                angleAxes = app.prepareDiagnosticAxes(numel(layoutSet));
                for layoutIndex = 1:numel(layoutSet)
                    options = struct('Title', sprintf('%s｜故障%s：6维向量夹角', ...
                        layoutSet(layoutIndex).name, ...
                        app.faultSetText(faultyIndices)));
                    RCPlotter.plotDirectionAngleMatrix( ...
                        angleAxes(layoutIndex), layoutSet(layoutIndex).B, ...
                        faultyIndices, options);
                end

                singleFault = RCPlotter.singleFaultEvaluationData(params, layoutSet);
                app.SingleFaultTable.ColumnName = singleFault.ColumnNames;
                app.SingleFaultTable.Data = singleFault.Data;
                app.SingleFaultInfoLabel.Text = singleFault.WeightText;
                app.MetricsViewTabGroup.SelectedTab = app.MetricsSingleFaultTab;
                faultText = app.faultSetText(faultyIndices);
                app.updateReport("可重构设计指标计算完成：故障" + ...
                    string(faultText) + "，已对比" + string(numel(layoutSet)) + ...
                    "个布局的控制能力、可诊断性、归一化指标和综合评价。");
                app.setStatus(sprintf('可重构设计指标计算完成｜故障%s｜%d个布局', ...
                    faultText, numel(layoutSet)));
            catch ME
                app.showError('设计指标计算失败', ME);
            end
        end

        % Button pushed function: GenerateAllocationButton
        function GenerateAllocationButtonPushed(app, event)
            try
                app.setBusy('正在生成推力器调用策略...');
                params = app.readParamsFromUI();
                layout = app.selectedAllocationLayout();
                params.Num = size(layout.B, 2);
                params.alloc_mode = app.mapAllocationMode(app.AllocationModeDropDown.Value);
                params.allocation_strategy = app.mapAllocationStrategy( ...
                    app.AllocationModeDropDown_2.Value);
                matrixConf = params.F_max * layout.B;
                faultInputText = strtrim(char(string(app.GenerationField_4.Value)));
                faultyThrusters = app.parseIndexList(app.GenerationField_4.Value);
                if isempty(faultyThrusters)
                    if isempty(regexp(faultInputText, '\d', 'once'))
                        faultCases = arrayfun(@(index) index, ...
                            1:params.Num, 'UniformOutput', false);
                        invocationFaults = [];
                        faultDescription = sprintf('全部%d种单台故障', params.Num);
                    else
                        error('故障编号应为1～%d之间的整数，例如1或[1 3]；留空可生成全部单故障。', ...
                            params.Num);
                    end
                else
                    faultCases = {faultyThrusters};
                    invocationFaults = faultyThrusters;
                    faultDescription = app.faultSetText(faultyThrusters);
                end
                if any(faultyThrusters > params.Num)
                    error('故障编号不能大于布局“%s”的推力器总数 %d。', ...
                        layout.name, params.Num);
                end
                verificationCommand = app.allocationVerificationCommand( ...
                    matrixConf);
                forceCmd = verificationCommand(1:3);
                torqueCmd = verificationCommand(4:6);
                [propFinal, info] = Thruster_allocator(forceCmd, torqueCmd, ...
                    matrixConf, invocationFaults, params);
                info.Prop_Final = propFinal;
                app.updateAllocationTable(info);
                strategy = app.buildAllocationStrategyComparison( ...
                    params, layout, faultCases);
                app.updateAllocationStrategyTable(strategy);
                verification = app.buildAllocationVerification( ...
                    params, layout, faultCases);
                app.updateAllocationVerification(verification);
                app.AllocationViewTabGroup.SelectedTab = ...
                    app.AllocationVerificationTab;
                app.updateReport(sprintf(['调用策略生成完成：布局=%s，故障=%s，', ...
                    '复用策略=%s，分配策略=%s。'], layout.name, ...
                    faultDescription, ...
                    app.AllocationModeDropDown.Value, ...
                    app.AllocationModeDropDown_2.Value));
                if numel(faultCases) > 1
                    app.setStatus(sprintf('调用策略生成完成｜全部%d种单台故障', ...
                        numel(faultCases)));
                else
                    app.setStatus('调用策略生成完成');
                end
            catch ME
                app.showError('调用策略生成失败', ME);
            end
        end

        % Button pushed function: RunSimulationButton
        function RunSimulationButtonPushed(app, event)
            try
                app.setBusy('正在运行闭环仿真...');
                params = app.readParamsFromUI();
                params.alloc_mode = app.mapAllocationMode( ...
                    app.AllocationModeDropDown.Value);
                params.allocation_strategy = app.mapAllocationStrategy( ...
                    app.AllocationModeDropDown_2.Value);
                selectedLayout = app.selectedSimulationLayout();
                params.Num = size(selectedLayout.B, 2);
                simCfg = app.readSimulationConfig();
                selectedFaults = simCfg.true_faults;
                if any(selectedFaults > params.Num)
                    error('故障编号不能大于布局“%s”的推力器总数 %d。', ...
                        selectedLayout.name, params.Num);
                end

                [faultSets, faultLabels] = app.simulationFaultCases( ...
                    params, selectedFaults);
                faultLogs = cell(1, numel(faultSets));
                for caseIndex = 1:numel(faultSets)
                    app.setBusy(sprintf('正在仿真对比工况 %d/%d...', ...
                        caseIndex, numel(faultSets)));
                    faultLogs{caseIndex} = app.runClosedloopCase( ...
                        params, selectedLayout.B, simCfg, faultSets{caseIndex});
                end

                if isempty(selectedFaults)
                    stateLog = faultLogs{1};
                    selectedCondition = '标况';
                else
                    stateLog = faultLogs{2};
                    selectedCondition = ['故障', app.faultSetText(selectedFaults)];
                end
                stateLogs = {stateLog};
                stateLabels = {sprintf('%s·%s', ...
                    selectedLayout.name, selectedCondition)};

                app.SimulationLog = struct( ...
                    'LayoutSet', selectedLayout, ...
                    'SelectedLayoutName', selectedLayout.name, ...
                    'SelectedFaults', selectedFaults, ...
                    'FaultTime', simCfg.faulty_time, ...
                    'ReuseStrategy', app.AllocationModeDropDown.Value, ...
                    'AllocationStrategy', app.AllocationModeDropDown_2.Value, ...
                    'Controller', struct('Kp_pos', simCfg.Kp_pos, ...
                        'Kd_pos', simCfg.Kd_pos, 'Kp_att', simCfg.Kp_att, ...
                        'Ki_att', simCfg.Ki_att, 'Kd_att', simCfg.Kd_att), ...
                    'StateLog', stateLog, ...
                    'ComparisonLogs', {stateLogs}, ...
                    'ComparisonLabels', {stateLabels}, ...
                    'FaultLayoutName', selectedLayout.name, ...
                    'FaultSets', {faultSets}, 'FaultLogs', {faultLogs}, ...
                    'FaultLabels', {faultLabels});
                app.plotSimulationResult(stateLogs, stateLabels, ...
                    faultLogs, faultLabels);
                app.SimulationViewTabGroup.SelectedTab = app.SimulationPositionTab;
                if isempty(selectedFaults)
                    comparisonText = sprintf('标况和全部%d种单故障', params.Num);
                else
                    comparisonText = ['标况和', selectedCondition];
                end
                app.updateReport(sprintf(['闭环仿真完成：布局=%s，指定工况=%s，', ...
                    '故障时刻=%.3g s，复用策略=%s，分配策略=%s；', ...
                    '控制器参数=[位置Kp %.3g，位置Kd %.3g，姿态Kp %.3g，', ...
                    '姿态Ki %.3g，姿态Kd %.3g]；故障对比包含%s。'], ...
                    selectedLayout.name, selectedCondition, ...
                    simCfg.faulty_time, app.AllocationModeDropDown.Value, ...
                    app.AllocationModeDropDown_2.Value, simCfg.Kp_pos, ...
                    simCfg.Kd_pos, simCfg.Kp_att, simCfg.Ki_att, ...
                    simCfg.Kd_att, comparisonText));
                app.setStatus(sprintf('闭环仿真完成｜%s｜%s＋%s', ...
                    selectedLayout.name, app.AllocationModeDropDown.Value, ...
                    app.AllocationModeDropDown_2.Value));
            catch ME
                app.showError('闭环仿真失败', ME);
            end
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        % Create UIFigure and components
        % Create UIFigure and components
        % Create UIFigure and components
        % Create UIFigure and components
        % Create UIFigure and components
        % Create UIFigure and components
        % Create UIFigure and components
        % Create UIFigure and components
        % Create UIFigure and components
        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Color = [0.96 0.97 0.98];
            app.UIFigure.Position = [70 70 1280 800];
            app.UIFigure.Name = 'RC-Designer 可重构性设计软件';

            % Create RootGrid
            app.RootGrid = uigridlayout(app.UIFigure);
            app.RootGrid.ColumnWidth = {'1x'};
            app.RootGrid.RowHeight = {72, '1x', 38};
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
            app.GridLayout2.ColumnWidth = {'1x', 220, 245, 180};
            app.GridLayout2.RowHeight = {'1x'};
            app.GridLayout2.Padding = [18 2 12 2];

            % Create Label2
            app.Label2 = uilabel(app.GridLayout2);
            app.Label2.HorizontalAlignment = 'right';
            app.Label2.FontColor = [0.38 0.46 0.53];
            app.Label2.Layout.Row = 1;
            app.Label2.Layout.Column = 4;
            app.Label2.Text = '原型版本 0.1｜MATLAB R2023b';

            % Create StatusLabel
            app.StatusLabel = uilabel(app.GridLayout2);
            app.StatusLabel.Layout.Row = 1;
            app.StatusLabel.Layout.Column = 1;
            app.StatusLabel.Text = '就绪｜已载入默认参数';

            % Create ProgressGauge
            app.ProgressGauge = uigauge(app.GridLayout2, 'linear');
            app.ProgressGauge.Limits = [0 100];
            app.ProgressGauge.MajorTicks = [];
            app.ProgressGauge.Value = 0;
            app.ProgressGauge.Layout.Row = 1;
            app.ProgressGauge.Layout.Column = 2;

            % Create ProgressLabel
            app.ProgressLabel = uilabel(app.GridLayout2);
            app.ProgressLabel.HorizontalAlignment = 'center';
            app.ProgressLabel.FontColor = [0.25 0.36 0.45];
            app.ProgressLabel.Layout.Row = 1;
            app.ProgressLabel.Layout.Column = 3;
            app.ProgressLabel.Text = '等待运行｜0%｜耗时 00:00';

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
            app.TabGroup.Position = [0 -31 1059 707];

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
            app.Label21.Text = '评价指标｜综合评价';

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

            % Create ParameterTab
            app.ParameterTab = uitab(app.TabGroup);
            app.ParameterTab.Title = '输入条件';

            % Create GridLayout5
            app.GridLayout5 = uigridlayout(app.ParameterTab);
            app.GridLayout5.ColumnWidth = {'1x', '1x', '1x', '1x'};
            app.GridLayout5.RowHeight = {55, 220, 130, 90, 52};
            app.GridLayout5.ColumnSpacing = 16;
            app.GridLayout5.Padding = [24 18 24 18];

            % Create ParameterFileGrid_2
            app.ParameterFileGrid_2 = uigridlayout(app.GridLayout5);
            app.ParameterFileGrid_2.ColumnWidth = {'1x', 126, 126};
            app.ParameterFileGrid_2.RowHeight = {'1x'};
            app.ParameterFileGrid_2.Padding = [0 6 0 0];
            app.ParameterFileGrid_2.Layout.Row = 5;
            app.ParameterFileGrid_2.Layout.Column = [3 4];

            % Create Button2_4
            app.Button2_4 = uibutton(app.ParameterFileGrid_2, 'push');
            app.Button2_4.ButtonPushedFcn = createCallbackFcn(app, @LoadParameterButtonPushed, true);
            app.Button2_4.Layout.Row = 1;
            app.Button2_4.Layout.Column = 3;
            app.Button2_4.Text = '加载参数文件';

            % Create ThrusterPanel_3
            app.ThrusterPanel_3 = uipanel(app.GridLayout5);
            app.ThrusterPanel_3.Title = '推力器';
            app.ThrusterPanel_3.Layout.Row = 3;
            app.ThrusterPanel_3.Layout.Column = 2;

            % Create ThrusterGrid_3
            app.ThrusterGrid_3 = uigridlayout(app.ThrusterPanel_3);
            app.ThrusterGrid_3.ColumnWidth = {67, 70, '1x'};
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

            % Create TextArea
            app.TextArea = uitextarea(app.GridLayout5);
            app.TextArea.Layout.Row = [2 4];
            app.TextArea.Layout.Column = [3 4];

            % Create ParameterFileGrid
            app.ParameterFileGrid = uigridlayout(app.GridLayout5);
            app.ParameterFileGrid.ColumnWidth = {'1x', 126, 126};
            app.ParameterFileGrid.RowHeight = {'1x'};
            app.ParameterFileGrid.Padding = [0 6 0 0];
            app.ParameterFileGrid.Layout.Row = 5;
            app.ParameterFileGrid.Layout.Column = [1 2];

            % Create Button2_2
            app.Button2_2 = uibutton(app.ParameterFileGrid, 'push');
            app.Button2_2.ButtonPushedFcn = createCallbackFcn(app, @ResetParameterButtonPushed, true);
            app.Button2_2.Layout.Row = 1;
            app.Button2_2.Layout.Column = 3;
            app.Button2_2.Text = '恢复默认设置';

            % Create Panel8
            app.Panel8 = uipanel(app.GridLayout5);
            app.Panel8.Title = '任务场景';
            app.Panel8.Layout.Row = [2 4];
            app.Panel8.Layout.Column = 1;

            % Create GridLayout19
            app.GridLayout19 = uigridlayout(app.Panel8);
            app.GridLayout19.ColumnWidth = {'fit', 70, 'fit'};
            app.GridLayout19.RowHeight = {34, 34, 34, 34, 34, 34, 34, 34, 34};

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

            % Create Label24_31
            app.Label24_31 = uilabel(app.GridLayout19);
            app.Label24_31.Layout.Row = 9;
            app.Label24_31.Layout.Column = 3;
            app.Label24_31.Text = 'm^3/s^2';

            % Create SimulationTimeField_4
            app.SimulationTimeField_4 = uieditfield(app.GridLayout19, 'numeric');
            app.SimulationTimeField_4.Limits = [1 Inf];
            app.SimulationTimeField_4.Layout.Row = 9;
            app.SimulationTimeField_4.Layout.Column = 2;
            app.SimulationTimeField_4.Value = 398600000000000;

            % Create Label38_4
            app.Label38_4 = uilabel(app.GridLayout19);
            app.Label38_4.Layout.Row = 9;
            app.Label38_4.Layout.Column = 1;
            app.Label38_4.Text = '引力常数mu';

            % Create Label24_30
            app.Label24_30 = uilabel(app.GridLayout19);
            app.Label24_30.Layout.Row = 8;
            app.Label24_30.Layout.Column = 3;
            app.Label24_30.Text = 'm';

            % Create SimulationTimeField_3
            app.SimulationTimeField_3 = uieditfield(app.GridLayout19, 'numeric');
            app.SimulationTimeField_3.Limits = [1 Inf];
            app.SimulationTimeField_3.Layout.Row = 8;
            app.SimulationTimeField_3.Layout.Column = 2;
            app.SimulationTimeField_3.Value = 400000;

            % Create Label38_3
            app.Label38_3 = uilabel(app.GridLayout19);
            app.Label38_3.Layout.Row = 8;
            app.Label38_3.Layout.Column = 1;
            app.Label38_3.Text = '轨道高度h';

            % Create Label24_29
            app.Label24_29 = uilabel(app.GridLayout19);
            app.Label24_29.Layout.Row = 7;
            app.Label24_29.Layout.Column = 3;
            app.Label24_29.Text = 'm';

            % Create SimulationTimeField_2
            app.SimulationTimeField_2 = uieditfield(app.GridLayout19, 'numeric');
            app.SimulationTimeField_2.Limits = [1 Inf];
            app.SimulationTimeField_2.Layout.Row = 7;
            app.SimulationTimeField_2.Layout.Column = 2;
            app.SimulationTimeField_2.Value = 6371000;

            % Create Label38_2
            app.Label38_2 = uilabel(app.GridLayout19);
            app.Label38_2.Layout.Row = 7;
            app.Label38_2.Layout.Column = 1;
            app.Label38_2.Text = '地球半径R';

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
            app.ControllerPanel.Layout.Row = 4;
            app.ControllerPanel.Layout.Column = 2;

            % Create ControllerGrid
            app.ControllerGrid = uigridlayout(app.ControllerPanel);
            app.ControllerGrid.ColumnWidth = {67, 70, '1x'};
            app.ControllerGrid.RowHeight = {34};

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

            % Create Label4
            app.Label4 = uilabel(app.GridLayout5);
            app.Label4.FontSize = 22;
            app.Label4.FontWeight = 'bold';
            app.Label4.FontColor = [0.1 0.27 0.42];
            app.Label4.Layout.Row = 1;
            app.Label4.Layout.Column = 1;
            app.Label4.Text = '输入条件设置';

            % Create OptimizationTab
            app.OptimizationTab = uitab(app.TabGroup);
            app.OptimizationTab.Title = '优化计算';

            % Create GridLayout7
            app.GridLayout7 = uigridlayout(app.OptimizationTab);
            app.GridLayout7.ColumnWidth = {400, '1x', 104, 104, 104};
            app.GridLayout7.RowHeight = {55, 260, 'fit'};
            app.GridLayout7.Padding = [22 18 22 18];

            % Create LayoutTemplateDropDown
            app.LayoutTemplateDropDown = uidropdown(app.GridLayout7);
            app.LayoutTemplateDropDown.Items = {'原布局'};
            app.LayoutTemplateDropDown.ValueChangedFcn = createCallbackFcn(app, @LayoutTemplateDropDownValueChanged, true);
            app.LayoutTemplateDropDown.Layout.Row = 1;
            app.LayoutTemplateDropDown.Layout.Column = 5;
            app.LayoutTemplateDropDown.Value = '原布局';

            % Create DeleteLayoutButton
            app.DeleteLayoutButton = uibutton(app.GridLayout7, 'push');
            app.DeleteLayoutButton.ButtonPushedFcn = createCallbackFcn(app, @DeleteLayoutButtonPushed, true);
            app.DeleteLayoutButton.Layout.Row = 1;
            app.DeleteLayoutButton.Layout.Column = 4;
            app.DeleteLayoutButton.Text = '删除布局';

            % Create ApplyLayoutButton_2
            app.ApplyLayoutButton_2 = uibutton(app.GridLayout7, 'push');
            app.ApplyLayoutButton_2.ButtonPushedFcn = createCallbackFcn(app, @ImportLayoutButtonPushed, true);
            app.ApplyLayoutButton_2.Layout.Row = 1;
            app.ApplyLayoutButton_2.Layout.Column = 3;
            app.ApplyLayoutButton_2.Text = '导入布局';

            % Create LayoutViewTabGroup
            app.LayoutViewTabGroup = uitabgroup(app.GridLayout7);
            app.LayoutViewTabGroup.Layout.Row = [2 3];
            app.LayoutViewTabGroup.Layout.Column = [2 5];

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
            app.LayoutAxes.Visible = 'off';

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
            app.LayoutXZAxes.Visible = 'off';

            % Create LayoutXYAxes
            app.LayoutXYAxes = uiaxes(app.LayoutViewsGrid);
            app.LayoutXYAxes.Layout.Row = 1;
            app.LayoutXYAxes.Layout.Column = 2;
            app.LayoutXYAxes.Visible = 'off';

            % Create LayoutYZAxes
            app.LayoutYZAxes = uiaxes(app.LayoutViewsGrid);
            app.LayoutYZAxes.Layout.Row = 1;
            app.LayoutYZAxes.Layout.Column = 1;
            app.LayoutYZAxes.Visible = 'off';

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

            % Create Panel10_3
            app.Panel10_3 = uipanel(app.GridLayout7);
            app.Panel10_3.Title = '遗传算法设置';
            app.Panel10_3.Layout.Row = 3;
            app.Panel10_3.Layout.Column = 1;

            % Create GridLayout21_3
            app.GridLayout21_3 = uigridlayout(app.Panel10_3);
            app.GridLayout21_3.RowHeight = {26, 38, 26, 38};

            % Create StartOptimizationButton_3
            app.StartOptimizationButton_3 = uibutton(app.GridLayout21_3, 'push');
            app.StartOptimizationButton_3.ButtonPushedFcn = createCallbackFcn(app, @StartOptimizationButtonPushed, true);
            app.StartOptimizationButton_3.Layout.Row = 4;
            app.StartOptimizationButton_3.Layout.Column = 2;
            app.StartOptimizationButton_3.Text = '开始优化';

            % Create DropDown1
            app.DropDown1 = uidropdown(app.GridLayout21_3);
            app.DropDown1.Items = {'控制能力优先', '综合评价优先', '能耗与控制能力折中'};
            app.DropDown1.Layout.Row = 4;
            app.DropDown1.Layout.Column = 1;
            app.DropDown1.Value = '控制能力优先';

            % Create Label37
            app.Label37 = uilabel(app.GridLayout21_3);
            app.Label37.Layout.Row = 3;
            app.Label37.Layout.Column = 1;
            app.Label37.Text = '优化目标';

            % Create GenerationField_3
            app.GenerationField_3 = uieditfield(app.GridLayout21_3, 'numeric');
            app.GenerationField_3.Limits = [1 Inf];
            app.GenerationField_3.HorizontalAlignment = 'left';
            app.GenerationField_3.Layout.Row = 2;
            app.GenerationField_3.Layout.Column = 2;
            app.GenerationField_3.Value = 50;

            % Create Label36_3
            app.Label36_3 = uilabel(app.GridLayout21_3);
            app.Label36_3.Layout.Row = 1;
            app.Label36_3.Layout.Column = 2;
            app.Label36_3.Text = '最大迭代次数';

            % Create PopulationField
            app.PopulationField = uieditfield(app.GridLayout21_3, 'numeric');
            app.PopulationField.Limits = [10 Inf];
            app.PopulationField.HorizontalAlignment = 'left';
            app.PopulationField.Layout.Row = 2;
            app.PopulationField.Layout.Column = 1;
            app.PopulationField.Value = 500;

            % Create Label35_3
            app.Label35_3 = uilabel(app.GridLayout21_3);
            app.Label35_3.Layout.Row = 1;
            app.Label35_3.Layout.Column = 1;
            app.Label35_3.Text = '种群规模';

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

            % Create EvaluationTab
            app.EvaluationTab = uitab(app.TabGroup);
            app.EvaluationTab.Title = '可重构判断';

            % Create GridLayout8
            app.GridLayout8 = uigridlayout(app.EvaluationTab);
            app.GridLayout8.ColumnWidth = {200, '1x'};
            app.GridLayout8.RowHeight = {55, 'fit', '1x'};
            app.GridLayout8.Padding = [22 18 22 18];

            % Create Panel10_2
            app.Panel10_2 = uipanel(app.GridLayout8);
            app.Panel10_2.Title = '故障设置';
            app.Panel10_2.Layout.Row = 2;
            app.Panel10_2.Layout.Column = [1 2];

            % Create GridLayout21_2
            app.GridLayout21_2 = uigridlayout(app.Panel10_2);
            app.GridLayout21_2.ColumnWidth = {'1x', '1x', '1x', '1x'};
            app.GridLayout21_2.RowHeight = {'fit', 38};

            % Create FaultCountDropDown
            app.FaultCountDropDown = uidropdown(app.GridLayout21_2);
            app.FaultCountDropDown.Items = {'单台故障', '两台故障', '自定义故障组合'};
            app.FaultCountDropDown.Layout.Row = 2;
            app.FaultCountDropDown.Layout.Column = 1;
            app.FaultCountDropDown.Value = '单台故障';

            % Create StartOptimizationButton_2
            app.StartOptimizationButton_2 = uibutton(app.GridLayout21_2, 'push');
            app.StartOptimizationButton_2.ButtonPushedFcn = createCallbackFcn(app, @EvaluateButtonPushed, true);
            app.StartOptimizationButton_2.Layout.Row = 2;
            app.StartOptimizationButton_2.Layout.Column = 4;
            app.StartOptimizationButton_2.Text = '可重构性判断';

            % Create JudgmentLayoutDropDown
            app.JudgmentLayoutDropDown = uidropdown(app.GridLayout21_2);
            app.JudgmentLayoutDropDown.Items = {'原布局'};
            app.JudgmentLayoutDropDown.Layout.Row = 2;
            app.JudgmentLayoutDropDown.Layout.Column = 3;
            app.JudgmentLayoutDropDown.Value = '原布局';

            % Create GenerationField_2
            app.GenerationField_2 = uieditfield(app.GridLayout21_2, 'text');
            app.GenerationField_2.Layout.Row = 2;
            app.GenerationField_2.Layout.Column = 2;
            app.GenerationField_2.Value = '1';

            % Create JudgmentLayoutLabel
            app.JudgmentLayoutLabel = uilabel(app.GridLayout21_2);
            app.JudgmentLayoutLabel.Layout.Row = 1;
            app.JudgmentLayoutLabel.Layout.Column = 3;
            app.JudgmentLayoutLabel.Text = '布局方案';

            % Create Label36_2
            app.Label36_2 = uilabel(app.GridLayout21_2);
            app.Label36_2.Layout.Row = 1;
            app.Label36_2.Layout.Column = 2;
            app.Label36_2.Text = '故障编号';

            % Create Label35_2
            app.Label35_2 = uilabel(app.GridLayout21_2);
            app.Label35_2.Layout.Row = 1;
            app.Label35_2.Layout.Column = 1;
            app.Label35_2.Text = '故障情况';

            % Create JudgmentViewTabGroup
            app.JudgmentViewTabGroup = uitabgroup(app.GridLayout8);
            app.JudgmentViewTabGroup.Layout.Row = 3;
            app.JudgmentViewTabGroup.Layout.Column = [1 2];

            % Create JudgmentDetailTab
            app.JudgmentDetailTab = uitab(app.JudgmentViewTabGroup);
            app.JudgmentDetailTab.Title = '可重构性判断';

            % Create JudgmentDetailGrid
            app.JudgmentDetailGrid = uigridlayout(app.JudgmentDetailTab);
            app.JudgmentDetailGrid.ColumnWidth = {'1x'};
            app.JudgmentDetailGrid.RowHeight = {'1x'};

            % Create EvaluationTable
            app.EvaluationTable = uitable(app.JudgmentDetailGrid);
            app.EvaluationTable.ColumnName = {'故障编号'; '力裕度JcF'; '力矩裕度JcT'; '六维裕度Jc'; '判断结果'};
            app.EvaluationTable.RowName = {};
            app.EvaluationTable.Layout.Row = 1;
            app.EvaluationTable.Layout.Column = 1;

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

            % Create Label8
            app.Label8 = uilabel(app.GridLayout8);
            app.Label8.FontSize = 22;
            app.Label8.FontWeight = 'bold';
            app.Label8.FontColor = [0.1 0.27 0.42];
            app.Label8.Layout.Row = 1;
            app.Label8.Layout.Column = 1;
            app.Label8.Text = '可重构性判断';

            % Create ReconfigDesignTab
            app.ReconfigDesignTab = uitab(app.TabGroup);
            app.ReconfigDesignTab.Title = '可重构评价';

            % Create GridLayout25
            app.GridLayout25 = uigridlayout(app.ReconfigDesignTab);
            app.GridLayout25.ColumnWidth = {'1.1x', 200, 200};
            app.GridLayout25.RowHeight = {55, '1x'};
            app.GridLayout25.Padding = [22 18 22 18];

            % Create MetricsEvaluateButton
            app.MetricsEvaluateButton = uibutton(app.GridLayout25, 'push');
            app.MetricsEvaluateButton.ButtonPushedFcn = createCallbackFcn(app, @MetricsEvaluateButtonPushed, true);
            app.MetricsEvaluateButton.Layout.Row = 1;
            app.MetricsEvaluateButton.Layout.Column = 3;
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
            app.MetricsViewTabGroup.Layout.Column = [1 3];

            % Create MetricsControlTab
            app.MetricsControlTab = uitab(app.MetricsViewTabGroup);
            app.MetricsControlTab.Title = '控制能力';

            % Create MetricsControlGrid
            app.MetricsControlGrid = uigridlayout(app.MetricsControlTab);
            app.MetricsControlGrid.RowHeight = {'1x'};
            app.MetricsControlGrid.Padding = [8 8 8 8];

            % Create MetricsTorqueAxes
            app.MetricsTorqueAxes = uiaxes(app.MetricsControlGrid);
            app.MetricsTorqueAxes.Layout.Row = 1;
            app.MetricsTorqueAxes.Layout.Column = 2;
            app.MetricsTorqueAxes.Visible = 'off';

            % Create MetricsForceAxes
            app.MetricsForceAxes = uiaxes(app.MetricsControlGrid);
            app.MetricsForceAxes.Layout.Row = 1;
            app.MetricsForceAxes.Layout.Column = 1;
            app.MetricsForceAxes.Visible = 'off';

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
            app.MetricsAngleAxes1.Visible = 'off';

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
            app.MetricsJcAxes.Visible = 'off';

            % Create MetricsJoAxes
            app.MetricsJoAxes = uiaxes(app.MetricsChartsGrid);
            app.MetricsJoAxes.Layout.Row = 1;
            app.MetricsJoAxes.Layout.Column = 2;
            app.MetricsJoAxes.Visible = 'off';

            % Create MetricsJtAxes
            app.MetricsJtAxes = uiaxes(app.MetricsChartsGrid);
            app.MetricsJtAxes.Layout.Row = 2;
            app.MetricsJtAxes.Layout.Column = 1;
            app.MetricsJtAxes.Visible = 'off';

            % Create MetricsJfAxes
            app.MetricsJfAxes = uiaxes(app.MetricsChartsGrid);
            app.MetricsJfAxes.Layout.Row = 2;
            app.MetricsJfAxes.Layout.Column = 2;
            app.MetricsJfAxes.Visible = 'off';

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

            % Create SingleFaultTable
            app.SingleFaultTable = uitable(app.MetricsSingleFaultGrid);
            app.SingleFaultTable.ColumnName = {'故障推力器编号'};
            app.SingleFaultTable.RowName = {};
            app.SingleFaultTable.Layout.Row = 2;
            app.SingleFaultTable.Layout.Column = 1;

            % Create AllocationTab
            app.AllocationTab = uitab(app.TabGroup);
            app.AllocationTab.Title = '调用策略';

            % Create GridLayout9
            app.GridLayout9 = uigridlayout(app.AllocationTab);
            app.GridLayout9.RowHeight = {55, 'fit', '1x'};
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
            app.AllocationViewTabGroup.Layout.Row = 3;
            app.AllocationViewTabGroup.Layout.Column = [1 2];

            % Create AllocationStrategyTab
            app.AllocationStrategyTab = uitab(app.AllocationViewTabGroup);
            app.AllocationStrategyTab.Title = '六轴推力器分配';

            % Create AllocationStrategyGrid
            app.AllocationStrategyGrid = uigridlayout(app.AllocationStrategyTab);
            app.AllocationStrategyGrid.ColumnWidth = {'1x'};
            app.AllocationStrategyGrid.RowHeight = {'1x'};

            % Create AllocationStrategyTable
            app.AllocationStrategyTable = uitable(app.AllocationStrategyGrid);
            app.AllocationStrategyTable.ColumnName = {'控制类型'; '轴向'; '标况主份'; '主份配对'; '故障下调用'; '故障关停/状态'};
            app.AllocationStrategyTable.RowName = {};
            app.AllocationStrategyTable.Layout.Row = 1;
            app.AllocationStrategyTable.Layout.Column = 1;

            % Create AllocationVerificationTab
            app.AllocationVerificationTab = uitab(app.AllocationViewTabGroup);
            app.AllocationVerificationTab.Title = '解耦验证';

            % Create AllocationVerificationGrid
            app.AllocationVerificationGrid = uigridlayout( ...
                app.AllocationVerificationTab);
            app.AllocationVerificationGrid.ColumnWidth = {'1x'};
            app.AllocationVerificationGrid.RowHeight = {34, '1x'};
            app.AllocationVerificationGrid.Padding = [12 12 12 12];

            % Create AllocationVerificationSummaryLabel
            app.AllocationVerificationSummaryLabel = uilabel( ...
                app.AllocationVerificationGrid);
            app.AllocationVerificationSummaryLabel.FontWeight = 'bold';
            app.AllocationVerificationSummaryLabel.FontColor = [0.10 0.27 0.42];
            app.AllocationVerificationSummaryLabel.Layout.Row = 1;
            app.AllocationVerificationSummaryLabel.Layout.Column = 1;
            app.AllocationVerificationSummaryLabel.Text = '等待生成调用策略';

            % Create AllocationVerificationTable
            app.AllocationVerificationTable = uitable( ...
                app.AllocationVerificationGrid);
            app.AllocationVerificationTable.ColumnName = {'工况'; '可重构'; ...
                '姿控缩放'; '轨控缩放'; '平均轨控残余力矩'; ...
                '平均姿控残余力'; '瞬时轨控残余力矩峰值'; ...
                '瞬时姿控残余力峰值'; '周期平均解耦'; '最大脉宽'; '执行状态'};
            app.AllocationVerificationTable.ColumnWidth = {90, 70, 75, 75, ...
                125, 110, 150, 135, 100, 90, 220};
            app.AllocationVerificationTable.RowName = {};
            app.AllocationVerificationTable.Layout.Row = 2;
            app.AllocationVerificationTable.Layout.Column = 1;

            % Create Panel10_4
            app.Panel10_4 = uipanel(app.GridLayout9);
            app.Panel10_4.Title = '调用设置';
            app.Panel10_4.Layout.Row = 2;
            app.Panel10_4.Layout.Column = [1 2];

            % Create GridLayout21_4
            app.GridLayout21_4 = uigridlayout(app.Panel10_4);
            app.GridLayout21_4.ColumnWidth = {'1x', '1x', '1x', '1x', '1x'};
            app.GridLayout21_4.RowHeight = {'fit', 38};

            % Create JudgmentLayoutDropDown_2
            app.JudgmentLayoutDropDown_2 = uidropdown(app.GridLayout21_4);
            app.JudgmentLayoutDropDown_2.Items = {'原布局'};
            app.JudgmentLayoutDropDown_2.Layout.Row = 2;
            app.JudgmentLayoutDropDown_2.Layout.Column = 4;
            app.JudgmentLayoutDropDown_2.Value = '原布局';

            % Create GenerationField_4
            app.GenerationField_4 = uieditfield(app.GridLayout21_4, 'text');
            app.GenerationField_4.Layout.Row = 2;
            app.GenerationField_4.Layout.Column = 3;
            app.GenerationField_4.Value = '1';

            % Create JudgmentLayoutLabel_2
            app.JudgmentLayoutLabel_2 = uilabel(app.GridLayout21_4);
            app.JudgmentLayoutLabel_2.Layout.Row = 1;
            app.JudgmentLayoutLabel_2.Layout.Column = 4;
            app.JudgmentLayoutLabel_2.Text = '布局方案';

            % Create Label36_4
            app.Label36_4 = uilabel(app.GridLayout21_4);
            app.Label36_4.Layout.Row = 1;
            app.Label36_4.Layout.Column = 3;
            app.Label36_4.Text = '故障编号';

            % Create Label35_4
            app.Label35_4 = uilabel(app.GridLayout21_4);
            app.Label35_4.Layout.Row = 1;
            app.Label35_4.Layout.Column = 1;
            app.Label35_4.Text = '复用策略';

            % Create AllocationModeDropDown
            app.AllocationModeDropDown = uidropdown(app.GridLayout21_4);
            app.AllocationModeDropDown.Items = {'异步分时复用', '同步分时复用', '联合优化复用'};
            app.AllocationModeDropDown.Layout.Row = 2;
            app.AllocationModeDropDown.Layout.Column = 1;
            app.AllocationModeDropDown.Value = '同步分时复用';

            % Create GenerateAllocationButton
            app.GenerateAllocationButton = uibutton(app.GridLayout21_4, 'push');
            app.GenerateAllocationButton.ButtonPushedFcn = createCallbackFcn(app, @GenerateAllocationButtonPushed, true);
            app.GenerateAllocationButton.Layout.Row = 2;
            app.GenerateAllocationButton.Layout.Column = 5;
            app.GenerateAllocationButton.Text = '生成调用策略';

            % Create Label35_5
            app.Label35_5 = uilabel(app.GridLayout21_4);
            app.Label35_5.Layout.Row = 1;
            app.Label35_5.Layout.Column = 2;
            app.Label35_5.Text = '分配策略';

            % Create AllocationModeDropDown_2
            app.AllocationModeDropDown_2 = uidropdown(app.GridLayout21_4);
            app.AllocationModeDropDown_2.Items = {'最简主备分配'};
            app.AllocationModeDropDown_2.Layout.Row = 2;
            app.AllocationModeDropDown_2.Layout.Column = 2;
            app.AllocationModeDropDown_2.Value = '最简主备分配';

            % Create SimulationTab
            app.SimulationTab = uitab(app.TabGroup);
            app.SimulationTab.Title = '闭环仿真';

            % Create GridLayout10
            app.GridLayout10 = uigridlayout(app.SimulationTab);
            app.GridLayout10.ColumnWidth = {180, '1x', '1x'};
            app.GridLayout10.RowHeight = {55, 260, '1x'};
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
            app.SimulationViewTabGroup.Layout.Row = [2 3];
            app.SimulationViewTabGroup.Layout.Column = [2 3];

            % Create SimulationPositionTab
            app.SimulationPositionTab = uitab(app.SimulationViewTabGroup);
            app.SimulationPositionTab.Title = '状态响应';

            % Create SimulationPositionGrid
            app.SimulationPositionGrid = uigridlayout(app.SimulationPositionTab);
            app.SimulationPositionGrid.Padding = [8 8 8 8];

            % Create SimulationAxes
            app.SimulationAxes = uiaxes(app.SimulationPositionGrid);
            app.SimulationAxes.Layout.Row = 1;
            app.SimulationAxes.Layout.Column = 1;
            app.SimulationAxes.Visible = 'off';

            % Create SimulationAttitudeAxes
            app.SimulationAttitudeAxes = uiaxes(app.SimulationPositionGrid);
            app.SimulationAttitudeAxes.Layout.Row = 1;
            app.SimulationAttitudeAxes.Layout.Column = 2;
            app.SimulationAttitudeAxes.Visible = 'off';

            % Create SimulationPositionErrorAxes
            app.SimulationPositionErrorAxes = uiaxes(app.SimulationPositionGrid);
            app.SimulationPositionErrorAxes.Layout.Row = 2;
            app.SimulationPositionErrorAxes.Layout.Column = 1;
            app.SimulationPositionErrorAxes.Visible = 'off';

            % Create SimulationAttitudeErrorAxes
            app.SimulationAttitudeErrorAxes = uiaxes(app.SimulationPositionGrid);
            app.SimulationAttitudeErrorAxes.Layout.Row = 2;
            app.SimulationAttitudeErrorAxes.Layout.Column = 2;
            app.SimulationAttitudeErrorAxes.Visible = 'off';

            % Create SimulationFaultCompareTab
            app.SimulationFaultCompareTab = uitab(app.SimulationViewTabGroup);
            app.SimulationFaultCompareTab.Title = '故障工况对比';

            % Create SimulationFaultGrid
            app.SimulationFaultGrid = uigridlayout(app.SimulationFaultCompareTab);
            app.SimulationFaultGrid.ColumnWidth = {'1x', '1x', '1x'};
            app.SimulationFaultGrid.Padding = [8 8 8 8];

            % Create SimulationFaultPosXAxes
            app.SimulationFaultPosXAxes = uiaxes(app.SimulationFaultGrid);
            app.SimulationFaultPosXAxes.Layout.Row = 1;
            app.SimulationFaultPosXAxes.Layout.Column = 1;
            app.SimulationFaultPosXAxes.Visible = 'off';

            % Create SimulationFaultPosYAxes
            app.SimulationFaultPosYAxes = uiaxes(app.SimulationFaultGrid);
            app.SimulationFaultPosYAxes.Layout.Row = 1;
            app.SimulationFaultPosYAxes.Layout.Column = 2;
            app.SimulationFaultPosYAxes.Visible = 'off';

            % Create SimulationFaultPosZAxes
            app.SimulationFaultPosZAxes = uiaxes(app.SimulationFaultGrid);
            app.SimulationFaultPosZAxes.Layout.Row = 1;
            app.SimulationFaultPosZAxes.Layout.Column = 3;
            app.SimulationFaultPosZAxes.Visible = 'off';

            % Create SimulationFaultAttXAxes
            app.SimulationFaultAttXAxes = uiaxes(app.SimulationFaultGrid);
            app.SimulationFaultAttXAxes.Layout.Row = 2;
            app.SimulationFaultAttXAxes.Layout.Column = 1;
            app.SimulationFaultAttXAxes.Visible = 'off';

            % Create SimulationFaultAttYAxes
            app.SimulationFaultAttYAxes = uiaxes(app.SimulationFaultGrid);
            app.SimulationFaultAttYAxes.Layout.Row = 2;
            app.SimulationFaultAttYAxes.Layout.Column = 2;
            app.SimulationFaultAttYAxes.Visible = 'off';

            % Create SimulationFaultAttZAxes
            app.SimulationFaultAttZAxes = uiaxes(app.SimulationFaultGrid);
            app.SimulationFaultAttZAxes.Layout.Row = 2;
            app.SimulationFaultAttZAxes.Layout.Column = 3;
            app.SimulationFaultAttZAxes.Visible = 'off';

            % Create SimulationCommandTab
            app.SimulationCommandTab = uitab(app.SimulationViewTabGroup);
            app.SimulationCommandTab.Title = '六维推力器实际输出';

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
            app.SimulationNominalCommandTab.Title = '标况';

            % Create SimulationNominalCommandGrid
            app.SimulationNominalCommandGrid = uigridlayout(app.SimulationNominalCommandTab);
            app.SimulationNominalCommandGrid.ColumnWidth = {'1x', '1x', '1x'};
            app.SimulationNominalCommandGrid.RowHeight = {'1x', '1x'};
            app.SimulationNominalCommandGrid.Padding = [8 8 8 8];

            % Create SimulationForceXAxes
            app.SimulationForceXAxes = uiaxes(app.SimulationNominalCommandGrid);
            app.SimulationForceXAxes.Layout.Row = 1;
            app.SimulationForceXAxes.Layout.Column = 1;
            app.SimulationForceXAxes.Visible = 'off';

            % Create SimulationForceYAxes
            app.SimulationForceYAxes = uiaxes(app.SimulationNominalCommandGrid);
            app.SimulationForceYAxes.Layout.Row = 1;
            app.SimulationForceYAxes.Layout.Column = 2;
            app.SimulationForceYAxes.Visible = 'off';

            % Create SimulationForceZAxes
            app.SimulationForceZAxes = uiaxes(app.SimulationNominalCommandGrid);
            app.SimulationForceZAxes.Layout.Row = 1;
            app.SimulationForceZAxes.Layout.Column = 3;
            app.SimulationForceZAxes.Visible = 'off';

            % Create SimulationTorqueXAxes
            app.SimulationTorqueXAxes = uiaxes(app.SimulationNominalCommandGrid);
            app.SimulationTorqueXAxes.Layout.Row = 2;
            app.SimulationTorqueXAxes.Layout.Column = 1;
            app.SimulationTorqueXAxes.Visible = 'off';

            % Create SimulationTorqueYAxes
            app.SimulationTorqueYAxes = uiaxes(app.SimulationNominalCommandGrid);
            app.SimulationTorqueYAxes.Layout.Row = 2;
            app.SimulationTorqueYAxes.Layout.Column = 2;
            app.SimulationTorqueYAxes.Visible = 'off';

            % Create SimulationTorqueZAxes
            app.SimulationTorqueZAxes = uiaxes(app.SimulationNominalCommandGrid);
            app.SimulationTorqueZAxes.Layout.Row = 2;
            app.SimulationTorqueZAxes.Layout.Column = 3;
            app.SimulationTorqueZAxes.Visible = 'off';

            % Create SimulationFaultCommandTab
            app.SimulationFaultCommandTab = uitab(app.SimulationCommandCaseTabGroup);
            app.SimulationFaultCommandTab.Title = '单推力器故障';

            % Create SimulationFaultCommandGrid
            app.SimulationFaultCommandGrid = uigridlayout(app.SimulationFaultCommandTab);
            app.SimulationFaultCommandGrid.ColumnWidth = {'1x', '1x', '1x'};
            app.SimulationFaultCommandGrid.RowHeight = {'1x', '1x'};
            app.SimulationFaultCommandGrid.Padding = [8 8 8 8];

            % Create SimulationFaultForceXAxes
            app.SimulationFaultForceXAxes = uiaxes(app.SimulationFaultCommandGrid);
            app.SimulationFaultForceXAxes.Layout.Row = 1;
            app.SimulationFaultForceXAxes.Layout.Column = 1;
            app.SimulationFaultForceXAxes.Visible = 'off';

            % Create SimulationFaultForceYAxes
            app.SimulationFaultForceYAxes = uiaxes(app.SimulationFaultCommandGrid);
            app.SimulationFaultForceYAxes.Layout.Row = 1;
            app.SimulationFaultForceYAxes.Layout.Column = 2;
            app.SimulationFaultForceYAxes.Visible = 'off';

            % Create SimulationFaultForceZAxes
            app.SimulationFaultForceZAxes = uiaxes(app.SimulationFaultCommandGrid);
            app.SimulationFaultForceZAxes.Layout.Row = 1;
            app.SimulationFaultForceZAxes.Layout.Column = 3;
            app.SimulationFaultForceZAxes.Visible = 'off';

            % Create SimulationFaultTorqueXAxes
            app.SimulationFaultTorqueXAxes = uiaxes(app.SimulationFaultCommandGrid);
            app.SimulationFaultTorqueXAxes.Layout.Row = 2;
            app.SimulationFaultTorqueXAxes.Layout.Column = 1;
            app.SimulationFaultTorqueXAxes.Visible = 'off';

            % Create SimulationFaultTorqueYAxes
            app.SimulationFaultTorqueYAxes = uiaxes(app.SimulationFaultCommandGrid);
            app.SimulationFaultTorqueYAxes.Layout.Row = 2;
            app.SimulationFaultTorqueYAxes.Layout.Column = 2;
            app.SimulationFaultTorqueYAxes.Visible = 'off';

            % Create SimulationFaultTorqueZAxes
            app.SimulationFaultTorqueZAxes = uiaxes(app.SimulationFaultCommandGrid);
            app.SimulationFaultTorqueZAxes.Layout.Row = 2;
            app.SimulationFaultTorqueZAxes.Layout.Column = 3;
            app.SimulationFaultTorqueZAxes.Visible = 'off';

            % Create SimulationTrajectoryTab
            app.SimulationTrajectoryTab = uitab(app.SimulationViewTabGroup);
            app.SimulationTrajectoryTab.Title = '轨迹姿态视图';

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
            app.SimulationNominalTrajectoryTab.Title = '标况';

            % Create SimulationNominalTrajectoryGrid
            app.SimulationNominalTrajectoryGrid = uigridlayout(app.SimulationNominalTrajectoryTab);
            app.SimulationNominalTrajectoryGrid.ColumnWidth = {'1x', '1x'};
            app.SimulationNominalTrajectoryGrid.RowHeight = {'1x', '1x'};
            app.SimulationNominalTrajectoryGrid.Padding = [8 8 8 8];

            % Create SimulationNominalTrajectory3DAxes
            app.SimulationNominalTrajectory3DAxes = uiaxes(app.SimulationNominalTrajectoryGrid);
            app.SimulationNominalTrajectory3DAxes.Layout.Row = 1;
            app.SimulationNominalTrajectory3DAxes.Layout.Column = 1;
            app.SimulationNominalTrajectory3DAxes.Visible = 'off';

            % Create SimulationNominalTrajectoryXYAxes
            app.SimulationNominalTrajectoryXYAxes = uiaxes(app.SimulationNominalTrajectoryGrid);
            app.SimulationNominalTrajectoryXYAxes.Layout.Row = 1;
            app.SimulationNominalTrajectoryXYAxes.Layout.Column = 2;
            app.SimulationNominalTrajectoryXYAxes.Visible = 'off';

            % Create SimulationNominalTrajectoryXZAxes
            app.SimulationNominalTrajectoryXZAxes = uiaxes(app.SimulationNominalTrajectoryGrid);
            app.SimulationNominalTrajectoryXZAxes.Layout.Row = 2;
            app.SimulationNominalTrajectoryXZAxes.Layout.Column = 1;
            app.SimulationNominalTrajectoryXZAxes.Visible = 'off';

            % Create SimulationNominalTrajectoryYZAxes
            app.SimulationNominalTrajectoryYZAxes = uiaxes(app.SimulationNominalTrajectoryGrid);
            app.SimulationNominalTrajectoryYZAxes.Layout.Row = 2;
            app.SimulationNominalTrajectoryYZAxes.Layout.Column = 2;
            app.SimulationNominalTrajectoryYZAxes.Visible = 'off';

            % Create SimulationFaultTrajectoryTab
            app.SimulationFaultTrajectoryTab = uitab(app.SimulationTrajectoryCaseTabGroup);
            app.SimulationFaultTrajectoryTab.Title = '单推力器故障';

            % Create SimulationFaultTrajectoryGrid
            app.SimulationFaultTrajectoryGrid = uigridlayout(app.SimulationFaultTrajectoryTab);
            app.SimulationFaultTrajectoryGrid.ColumnWidth = {'1x', '1x'};
            app.SimulationFaultTrajectoryGrid.RowHeight = {'1x', '1x'};
            app.SimulationFaultTrajectoryGrid.Padding = [8 8 8 8];

            % Create SimulationFaultTrajectory3DAxes
            app.SimulationFaultTrajectory3DAxes = uiaxes(app.SimulationFaultTrajectoryGrid);
            app.SimulationFaultTrajectory3DAxes.Layout.Row = 1;
            app.SimulationFaultTrajectory3DAxes.Layout.Column = 1;
            app.SimulationFaultTrajectory3DAxes.Visible = 'off';

            % Create SimulationFaultTrajectoryXYAxes
            app.SimulationFaultTrajectoryXYAxes = uiaxes(app.SimulationFaultTrajectoryGrid);
            app.SimulationFaultTrajectoryXYAxes.Layout.Row = 1;
            app.SimulationFaultTrajectoryXYAxes.Layout.Column = 2;
            app.SimulationFaultTrajectoryXYAxes.Visible = 'off';

            % Create SimulationFaultTrajectoryXZAxes
            app.SimulationFaultTrajectoryXZAxes = uiaxes(app.SimulationFaultTrajectoryGrid);
            app.SimulationFaultTrajectoryXZAxes.Layout.Row = 2;
            app.SimulationFaultTrajectoryXZAxes.Layout.Column = 1;
            app.SimulationFaultTrajectoryXZAxes.Visible = 'off';

            % Create SimulationFaultTrajectoryYZAxes
            app.SimulationFaultTrajectoryYZAxes = uiaxes(app.SimulationFaultTrajectoryGrid);
            app.SimulationFaultTrajectoryYZAxes.Layout.Row = 2;
            app.SimulationFaultTrajectoryYZAxes.Layout.Column = 2;
            app.SimulationFaultTrajectoryYZAxes.Visible = 'off';

            % Create SimulationPulseTab
            app.SimulationPulseTab = uitab(app.SimulationViewTabGroup);
            app.SimulationPulseTab.Title = '各推力器脉宽';

            % Create SimulationPulseGrid
            app.SimulationPulseGrid = uigridlayout(app.SimulationPulseTab);
            app.SimulationPulseGrid.ColumnWidth = {'1x'};
            app.SimulationPulseGrid.RowHeight = {'1x'};
            app.SimulationPulseGrid.Padding = [8 8 8 8];
            app.SimulationPulseGrid.Scrollable = 'on';

            % Create SimulationPulsePlaceholderAxes
            app.SimulationPulsePlaceholderAxes = uiaxes(app.SimulationPulseGrid);
            app.SimulationPulsePlaceholderAxes.Layout.Row = 1;
            app.SimulationPulsePlaceholderAxes.Layout.Column = 1;
            app.SimulationPulsePlaceholderAxes.Visible = 'off';

            % Create SimulationScheduleTab
            app.SimulationScheduleTab = uitab(app.SimulationViewTabGroup);
            app.SimulationScheduleTab.Title = '调用时序';

            % Create SimulationScheduleGrid
            app.SimulationScheduleGrid = uigridlayout(app.SimulationScheduleTab);
            app.SimulationScheduleGrid.ColumnWidth = {'1x'};
            app.SimulationScheduleGrid.RowHeight = {34, '1x'};
            app.SimulationScheduleGrid.Padding = [8 8 8 8];

            % Create SimulationScheduleControlGrid
            app.SimulationScheduleControlGrid = uigridlayout(app.SimulationScheduleGrid);
            app.SimulationScheduleControlGrid.Layout.Row = 1;
            app.SimulationScheduleControlGrid.Layout.Column = 1;
            app.SimulationScheduleControlGrid.ColumnWidth = {'fit', 120, 'fit', 90, '1x'};
            app.SimulationScheduleControlGrid.RowHeight = {'1x'};
            app.SimulationScheduleControlGrid.Padding = [0 0 0 0];
            app.SimulationScheduleControlGrid.ColumnSpacing = 10;

            % Create SimulationScheduleStartTimeLabel
            app.SimulationScheduleStartTimeLabel = uilabel(app.SimulationScheduleControlGrid);
            app.SimulationScheduleStartTimeLabel.HorizontalAlignment = 'right';
            app.SimulationScheduleStartTimeLabel.Text = '起始时间 / s';
            app.SimulationScheduleStartTimeLabel.Layout.Row = 1;
            app.SimulationScheduleStartTimeLabel.Layout.Column = 1;

            % Create SimulationScheduleStartTimeField
            app.SimulationScheduleStartTimeField = uieditfield(app.SimulationScheduleControlGrid, 'numeric');
            app.SimulationScheduleStartTimeField.Limits = [0 Inf];
            app.SimulationScheduleStartTimeField.ValueDisplayFormat = '%.3f';
            app.SimulationScheduleStartTimeField.Tooltip = '仿真完成时自动定位至全周期脉宽最多的控制周期';
            app.SimulationScheduleStartTimeField.Value = 0;
            app.SimulationScheduleStartTimeField.Layout.Row = 1;
            app.SimulationScheduleStartTimeField.Layout.Column = 2;

            % Create SimulationScheduleCycleCountLabel
            app.SimulationScheduleCycleCountLabel = uilabel(app.SimulationScheduleControlGrid);
            app.SimulationScheduleCycleCountLabel.HorizontalAlignment = 'right';
            app.SimulationScheduleCycleCountLabel.Text = '显示周期数';
            app.SimulationScheduleCycleCountLabel.Layout.Row = 1;
            app.SimulationScheduleCycleCountLabel.Layout.Column = 3;

            % Create SimulationScheduleCycleCountField
            app.SimulationScheduleCycleCountField = uieditfield(app.SimulationScheduleControlGrid, 'numeric');
            app.SimulationScheduleCycleCountField.Limits = [1 Inf];
            app.SimulationScheduleCycleCountField.ValueDisplayFormat = '%.0f';
            app.SimulationScheduleCycleCountField.Tooltip = '显示的连续控制周期数量';
            app.SimulationScheduleCycleCountField.Value = 5;
            app.SimulationScheduleCycleCountField.Layout.Row = 1;
            app.SimulationScheduleCycleCountField.Layout.Column = 4;

            % Create SimulationScheduleAxes
            app.SimulationScheduleAxes = uiaxes(app.SimulationScheduleGrid);
            app.SimulationScheduleAxes.Layout.Row = 2;
            app.SimulationScheduleAxes.Layout.Column = 1;
            app.SimulationScheduleAxes.Visible = 'off';

            % Create Panel10_5
            app.Panel10_5 = uipanel(app.GridLayout10);
            app.Panel10_5.Title = '故障设置';
            app.Panel10_5.Layout.Row = 3;
            app.Panel10_5.Layout.Column = 1;

            % Create GridLayout21_5
            app.GridLayout21_5 = uigridlayout(app.Panel10_5);
            app.GridLayout21_5.ColumnWidth = {'1x', 'fit'};
            app.GridLayout21_5.RowHeight = {'fit', 38, 'fit', 38, 'fit', 38, 38};

            % Create RunSimulationButton
            app.RunSimulationButton = uibutton(app.GridLayout21_5, 'push');
            app.RunSimulationButton.ButtonPushedFcn = createCallbackFcn(app, @RunSimulationButtonPushed, true);
            app.RunSimulationButton.Layout.Row = 7;
            app.RunSimulationButton.Layout.Column = [1 2];
            app.RunSimulationButton.Text = '运行闭环仿真';

            % Create Label36_5
            app.Label36_5 = uilabel(app.GridLayout21_5);
            app.Label36_5.Layout.Row = 5;
            app.Label36_5.Layout.Column = 1;
            app.Label36_5.Text = '故障时刻';

            % Create GenerationField_5
            app.GenerationField_5 = uieditfield(app.GridLayout21_5, 'text');
            app.GenerationField_5.Layout.Row = 6;
            app.GenerationField_5.Layout.Column = 1;
            app.GenerationField_5.Value = '1000';

            % Create Label36_6
            app.Label36_6 = uilabel(app.GridLayout21_5);
            app.Label36_6.Layout.Row = 1;
            app.Label36_6.Layout.Column = 1;
            app.Label36_6.Text = '布局方案';

            % Create SimulationLayoutDropDown
            app.SimulationLayoutDropDown = uidropdown(app.GridLayout21_5);
            app.SimulationLayoutDropDown.Items = {'原布局'};
            app.SimulationLayoutDropDown.Layout.Row = 2;
            app.SimulationLayoutDropDown.Layout.Column = [1 2];
            app.SimulationLayoutDropDown.Value = '原布局';

            % Create GenerationField_6
            app.GenerationField_6 = uieditfield(app.GridLayout21_5, 'text');
            app.GenerationField_6.Layout.Row = 4;
            app.GenerationField_6.Layout.Column = [1 2];
            app.GenerationField_6.Value = '1';

            % Create Label36_7
            app.Label36_7 = uilabel(app.GridLayout21_5);
            app.Label36_7.Layout.Row = 3;
            app.Label36_7.Layout.Column = 1;
            app.Label36_7.Text = '故障编号';

            % Create Label24_38
            app.Label24_38 = uilabel(app.GridLayout21_5);
            app.Label24_38.Layout.Row = 6;
            app.Label24_38.Layout.Column = 2;
            app.Label24_38.Text = 's';

            % Create ControllerPanel_2
            app.ControllerPanel_2 = uipanel(app.GridLayout10);
            app.ControllerPanel_2.Title = '控制器参数';
            app.ControllerPanel_2.Layout.Row = 2;
            app.ControllerPanel_2.Layout.Column = 1;

            % Create ControllerGrid_2
            app.ControllerGrid_2 = uigridlayout(app.ControllerPanel_2);
            app.ControllerGrid_2.ColumnWidth = {67, 70};
            app.ControllerGrid_2.RowHeight = {34, 34, 34, 34, 34};

            % Create Label47_2
            app.Label47_2 = uilabel(app.ControllerGrid_2);
            app.Label47_2.Layout.Row = 1;
            app.Label47_2.Layout.Column = 1;
            app.Label47_2.Text = '位置Kp';

            % Create ControlPeriodField_2
            app.ControlPeriodField_2 = uieditfield(app.ControllerGrid_2, 'numeric');
            app.ControlPeriodField_2.Limits = [0 Inf];
            app.ControlPeriodField_2.Layout.Row = 1;
            app.ControlPeriodField_2.Layout.Column = 2;
            app.ControlPeriodField_2.Value = 10;

            % Create Label47_3
            app.Label47_3 = uilabel(app.ControllerGrid_2);
            app.Label47_3.Layout.Row = 2;
            app.Label47_3.Layout.Column = 1;
            app.Label47_3.Text = '位置Kd';

            % Create ControlPeriodField_3
            app.ControlPeriodField_3 = uieditfield(app.ControllerGrid_2, 'numeric');
            app.ControlPeriodField_3.Limits = [0 Inf];
            app.ControlPeriodField_3.Layout.Row = 2;
            app.ControlPeriodField_3.Layout.Column = 2;
            app.ControlPeriodField_3.Value = 300;

            % Create Label47_4
            app.Label47_4 = uilabel(app.ControllerGrid_2);
            app.Label47_4.Layout.Row = 3;
            app.Label47_4.Layout.Column = 1;
            app.Label47_4.Text = '姿态Kp';

            % Create ControlPeriodField_4
            app.ControlPeriodField_4 = uieditfield(app.ControllerGrid_2, 'numeric');
            app.ControlPeriodField_4.Limits = [0 Inf];
            app.ControlPeriodField_4.Layout.Row = 3;
            app.ControlPeriodField_4.Layout.Column = 2;
            app.ControlPeriodField_4.Value = 400;

            % Create Label47_5
            app.Label47_5 = uilabel(app.ControllerGrid_2);
            app.Label47_5.Layout.Row = 4;
            app.Label47_5.Layout.Column = 1;
            app.Label47_5.Text = '姿态Ki';

            % Create ControlPeriodField_5
            app.ControlPeriodField_5 = uieditfield(app.ControllerGrid_2, 'numeric');
            app.ControlPeriodField_5.Limits = [0 Inf];
            app.ControlPeriodField_5.Layout.Row = 4;
            app.ControlPeriodField_5.Layout.Column = 2;
            app.ControlPeriodField_5.Value = 20;

            % Create Label47_6
            app.Label47_6 = uilabel(app.ControllerGrid_2);
            app.Label47_6.Layout.Row = 5;
            app.Label47_6.Layout.Column = 1;
            app.Label47_6.Text = '姿态Kd';

            % Create ControlPeriodField_6
            app.ControlPeriodField_6 = uieditfield(app.ControllerGrid_2, 'numeric');
            app.ControlPeriodField_6.Limits = [0 Inf];
            app.ControlPeriodField_6.Layout.Row = 5;
            app.ControlPeriodField_6.Layout.Column = 2;
            app.ControlPeriodField_6.Value = 3200;

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
            app.GridLayout1.ColumnWidth = {'1x', 104, 104, 104, 24};
            app.GridLayout1.RowHeight = {'1x'};
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

            % Stabilize the hidden main tab group before the first frame.
            app.resizeMainTabGroup();

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
