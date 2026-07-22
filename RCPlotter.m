classdef RCPlotter
%RCPLOTTER Shared plotting utilities for scripts and App Designer.
%   All public methods accept an axes/UIAxes handle as their first input,
%   so the same plotting code can draw either in a normal MATLAB figure
%   or directly in an App Designer page.

    methods (Static)
        function plotLayout3D(ax, B, r, options)
        %PLOTLAYOUT3D Draw one thruster layout in three dimensions.
            if nargin < 4 || isempty(options)
                options = struct();
            end
            RCPlotter.validateLayoutInputs(ax, B, r);

            thrusterCount = size(r, 2);
            % faulty = RCPlotter.validIndices( ...
            %     RCPlotter.getOption(options, 'FaultyIndices', []), thrusterCount);
            faulty = 0;
            healthy = setdiff(1:thrusterCount, faulty);
            directions = B(1:3, :);

            cla(ax, 'reset');
            hold(ax, 'on');
            grid(ax, 'on');
            box(ax, 'on');

            bodyHalfSize = RCPlotter.getOption(options, ...
                'BodyHalfSize', [2, 0.6, 0.6]);
            RCPlotter.plotBodyCuboid3D(ax, bodyHalfSize, options);

            RCPlotter.plotThrusterGroup3D(ax, r, directions, healthy, ...
                [0.20, 0.70, 1.00], [0.85, 0.15, 0.15], '-');
            % RCPlotter.plotThrusterGroup3D(ax, r, directions, faulty, ...
            %     [0.78, 0.78, 0.78], [0.45, 0.45, 0.45], '--');

            if RCPlotter.getOption(options, 'ShowLabels', true)
                for index = 1:thrusterCount
                    labelColor = [0.1, 0.1, 0.1];
                    if ismember(index, faulty)
                        labelColor = [0.55, 0.10, 0.10];
                    end
                    text(ax, r(1, index), r(2, index), r(3, index), ...
                        sprintf(' %d', index), 'FontSize', 9, ...
                        'FontWeight', 'bold', 'Color', labelColor);
                end
            end

            xlabel(ax, RCPlotter.getOption(options, 'XLabel', 'X / m'));
            ylabel(ax, RCPlotter.getOption(options, 'YLabel', 'Y / m'));
            zlabel(ax, RCPlotter.getOption(options, 'ZLabel', 'Z / m'));
            title(ax, RCPlotter.getOption(options, 'Title', ...
                '推力器安装位置与推力方向'));
            axis(ax, 'equal');
            viewAngles = RCPlotter.getOption(options, 'View', [35, 25]);
            view(ax, viewAngles(1), viewAngles(2));
            hold(ax, 'off');
        end

        function plotLayoutProjection(ax, B, r, viewDefinition, options)
        %PLOTLAYOUTPROJECTION Draw a reusable 2-D projection of a layout.
        %   viewDefinition fields: axes, xlabel, ylabel, xlim, ylim,
        %   xdir, ydir and optional name.
            if nargin < 4 || isempty(viewDefinition)
                viewDefinition = struct('name', 'YZ正视图', ...
                    'axes', [2, 3], 'xlabel', 'Y', 'ylabel', 'Z', ...
                    'xlim', [-1, 1], 'ylim', [-1, 1], ...
                    'xdir', 'normal', 'ydir', 'reverse');
            end
            if nargin < 5 || isempty(options)
                options = struct();
            end
            RCPlotter.validateLayoutInputs(ax, B, r);

            projectionAxes = viewDefinition.axes;
            if numel(projectionAxes) ~= 2 || ...
                    any(~ismember(projectionAxes, 1:3))
                error('RCPlotter:InvalidProjection', ...
                    '投影坐标轴必须是 1至3 中的两个不同索引。');
            end

            thrusterCount = size(r, 2);
            % faulty = RCPlotter.validIndices( ...
            %     RCPlotter.getOption(options, 'FaultyIndices', []), thrusterCount);
            faulty = 0;
            healthy = setdiff(1:thrusterCount, faulty);
            directions = B(1:3, :);

            cla(ax, 'reset');
            view(ax, 2);
            hold(ax, 'on');
            grid(ax, 'on');
            box(ax, 'on');
            RCPlotter.plotBodyProjection(ax, projectionAxes, ...
                RCPlotter.getOption(options, 'BodyHalfSize', [2, 0.6, 0.6]));
            RCPlotter.plotThrusterGroup2D(ax, r, directions, healthy, ...
                projectionAxes, [0.20, 0.70, 1.00], [0.85, 0.15, 0.15], '-');
            % RCPlotter.plotThrusterGroup2D(ax, r, directions, faulty, ...
            %     projectionAxes, [0.80, 0.80, 0.80], [0.50, 0.50, 0.50], '--');

            if RCPlotter.getOption(options, 'ShowLabels', false)
                RCPlotter.plotProjectionLabels(ax, r, projectionAxes, ...
                    viewDefinition.xlim, viewDefinition.ylim);
            end

            xlabel(ax, viewDefinition.xlabel);
            ylabel(ax, viewDefinition.ylabel);
            axis(ax, 'equal');
            xlim(ax, viewDefinition.xlim);
            ylim(ax, viewDefinition.ylim);
            set(ax, 'XDir', viewDefinition.xdir, 'YDir', viewDefinition.ydir);

            layoutName = RCPlotter.getOption(options, 'LayoutName', '');
            showTitle = RCPlotter.getOption(options, 'ShowTitle', false);
            if showTitle
                viewName = RCPlotter.getStructField(viewDefinition, 'name', '');
                title(ax, strtrim([char(layoutName), ' ', char(viewName)]));
            end
            hold(ax, 'off');
        end

        function plotTrackingErrors(ax, logData, options)
        %PLOTTRACKINGERRORS Plot position and attitude error norms.
        %   logData may be one simulation-log struct or a cell array of
        %   logs.  Multiple logs are drawn with consistent colors.
            if nargin < 3 || isempty(options)
                options = struct();
            end
            if ~iscell(logData)
                logs = {logData};
            else
                logs = logData;
            end
            if isempty(logs)
                error('RCPlotter:EmptyLog', '闭环仿真结果不能为空。');
            end
            RCPlotter.validateAxes(ax);

            labels = RCPlotter.getOption(options, 'Labels', {});
            if isstring(labels)
                labels = cellstr(labels);
            elseif ischar(labels)
                labels = {labels};
            end
            if numel(labels) < numel(logs)
                labels = arrayfun(@(index)sprintf('结果%d', index), ...
                    1:numel(logs), 'UniformOutput', false);
            end

            colors = lines(max(1, numel(logs)));
            positionHandles = gobjects(1, numel(logs));
            attitudeHandles = gobjects(1, numel(logs));
            legendText = cell(1, 2 * numel(logs));

            cla(ax, 'reset');
            view(ax, 2);
            yyaxis(ax, 'left');
            hold(ax, 'on');
            for index = 1:numel(logs)
                logItem = logs{index};
                RCPlotter.validateSimulationLog(logItem);
                positionError = vecnorm(logItem.Y(1:3, :) - logItem.R, 2, 1);
                positionHandles(index) = plot(ax, logItem.Time, positionError, ...
                    '-', 'Color', colors(index, :), 'LineWidth', 1.25);
                legendText{index} = [labels{index}, '-位置误差'];
            end
            ylabel(ax, '位置误差 / m');

            yyaxis(ax, 'right');
            hold(ax, 'on');
            for index = 1:numel(logs)
                logItem = logs{index};
                attitudeDifference = mod( ...
                    (logItem.Y_euler - logItem.E) + pi, 2*pi) - pi;
                attitudeError = rad2deg(vecnorm(attitudeDifference, 2, 1));
                attitudeHandles(index) = plot(ax, logItem.Time, attitudeError, ...
                    '--', 'Color', colors(index, :), 'LineWidth', 1.25);
                legendText{numel(logs) + index} = ...
                    [labels{index}, '-姿态误差'];
            end
            ylabel(ax, '姿态误差 / deg');
            xlabel(ax, '时间 / s');
            title(ax, RCPlotter.getOption(options, 'Title', ...
                '闭环仿真跟踪误差'));
            grid(ax, 'on');

            showFaultTime = RCPlotter.getOption(options, 'ShowFaultTime', true);
            if showFaultTime
                faultTime = RCPlotter.getOption(options, 'FaultTime', []);
                if isempty(faultTime) && isfield(logs{1}, 'faulty_time')
                    faultTime = logs{1}.faulty_time;
                end
                if ~isempty(faultTime) && isfinite(faultTime)
                    xline(ax, faultTime, '--r', '故障时刻', ...
                        'HandleVisibility', 'off');
                end
            end

            if RCPlotter.getOption(options, 'ShowLegend', numel(logs) > 1)
                legend(ax, [positionHandles, attitudeHandles], legendText, ...
                    'Location', 'best');
            end
            hold(ax, 'off');
        end

        function plotOptimizationResult(ax, result, options)
        %PLOTOPTIMIZATIONRESULT Draw the optimization vector or a message.
            if nargin < 3 || isempty(options)
                options = struct();
            end
            RCPlotter.validateAxes(ax);
            cla(ax, 'reset');
            view(ax, 2);
            if isstruct(result) && isfield(result, 'x_opt') && ...
                    ~isempty(result.x_opt)
                bar(ax, result.x_opt);
                xlabel(ax, '优化变量序号');
                ylabel(ax, '变量值');
                objectiveValue = RCPlotter.getStructField(result, 'fval', NaN);
                if isfinite(objectiveValue)
                    defaultTitle = sprintf('优化结果：fval = %.4g', objectiveValue);
                else
                    defaultTitle = '优化结果';
                end
                title(ax, RCPlotter.getOption(options, 'Title', defaultTitle));
                grid(ax, 'on');
            else
                messageText = RCPlotter.getStructField(result, 'message', ...
                    '暂无可绘制的优化结果');
                text(ax, 0.5, 0.5, char(string(messageText)), ...
                    'HorizontalAlignment', 'center', ...
                    'VerticalAlignment', 'middle');
                axis(ax, 'off');
            end
        end

        function plotResponseComponents(ax, logData, labels, dataType, options)
        %PLOTRESPONSECOMPONENTS Plot x/y/z position or attitude responses.
            if nargin < 5 || isempty(options)
                options = struct();
            end
            logs = RCPlotter.normalizeLogs(logData);
            labels = RCPlotter.normalizeLabels(labels, numel(logs));
            RCPlotter.validateAxes(ax);

            cla(ax, 'reset');
            view(ax, 2);
            hold(ax, 'on');
            grid(ax, 'on');
            componentColors = lines(3);
            lineStyles = {'-', '--', ':', '-.'};
            lineWidths = [1.2, 1.3, 1.7, 1.3];
            if strcmpi(dataType, 'att')
                componentNames = {'phi', 'theta', 'psi'};
                defaultTitle = '姿态响应';
                defaultUnit = 'rad';
            else
                componentNames = {'x', 'y', 'z'};
                defaultTitle = '位置响应';
                defaultUnit = 'm';
            end

            handles = gobjects(1, numel(logs) * 3);
            legendText = cell(1, numel(handles));
            handleIndex = 0;
            for layoutIndex = 1:numel(logs)
                RCPlotter.validateSimulationLog(logs{layoutIndex});
                data = RCPlotter.responseData(logs{layoutIndex}, dataType);
                for componentIndex = 1:3
                    handleIndex = handleIndex + 1;
                    handles(handleIndex) = plot(ax, logs{layoutIndex}.Time, ...
                        data(componentIndex, :), ...
                        lineStyles{min(layoutIndex, numel(lineStyles))}, ...
                        'Color', componentColors(componentIndex, :), ...
                        'LineWidth', lineWidths(min(layoutIndex, numel(lineWidths))));
                    legendText{handleIndex} = ...
                        [labels{layoutIndex}, '-', componentNames{componentIndex}];
                end
            end
            RCPlotter.drawFaultLine(ax, logs, options);
            title(ax, RCPlotter.getOption(options, 'Title', defaultTitle));
            xlabel(ax, '时间 / s');
            ylabel(ax, RCPlotter.getOption(options, 'YLabel', defaultUnit));
            if RCPlotter.getOption(options, 'ShowLegend', true)
                legend(ax, handles, legendText, 'Location', 'best');
            end
            hold(ax, 'off');
        end

        function plotErrorNorm(ax, logData, labels, dataType, options)
        %PLOTERRORNORM Plot position or attitude error norms.
            if nargin < 5 || isempty(options)
                options = struct();
            end
            logs = RCPlotter.normalizeLogs(logData);
            labels = RCPlotter.normalizeLabels(labels, numel(logs));
            RCPlotter.validateAxes(ax);

            cla(ax, 'reset');
            view(ax, 2);
            hold(ax, 'on');
            grid(ax, 'on');
            lineStyles = {'-', '--', ':', '-.'};
            lineWidths = [1.2, 1.3, 1.7, 1.3];
            handles = gobjects(1, numel(logs));
            for layoutIndex = 1:numel(logs)
                errorData = RCPlotter.errorData(logs{layoutIndex}, dataType);
                handles(layoutIndex) = plot(ax, logs{layoutIndex}.Time, ...
                    vecnorm(errorData, 2, 1), ...
                    lineStyles{min(layoutIndex, numel(lineStyles))}, ...
                    'LineWidth', lineWidths(min(layoutIndex, numel(lineWidths))));
            end
            RCPlotter.drawFaultLine(ax, logs, options);
            if strcmpi(dataType, 'att')
                defaultTitle = '姿态误差范数';
                defaultYLabel = '||e_e|| / rad';
            else
                defaultTitle = '位置误差范数';
                defaultYLabel = '||e_r|| / m';
            end
            title(ax, RCPlotter.getOption(options, 'Title', defaultTitle));
            xlabel(ax, '时间 / s');
            ylabel(ax, RCPlotter.getOption(options, 'YLabel', defaultYLabel));
            if RCPlotter.getOption(options, 'ShowLegend', true)
                legend(ax, handles, labels, 'Location', 'best');
            end
            hold(ax, 'off');
        end

        function plotFaultCaseComponent(ax, logData, labels, dataType, componentIndex, options)
        %PLOTFAULTCASECOMPONENT Compare one response component across faults.
            if nargin < 6 || isempty(options)
                options = struct();
            end
            logs = RCPlotter.normalizeLogs(logData);
            labels = RCPlotter.normalizeLabels(labels, numel(logs));
            RCPlotter.validateAxes(ax);
            if componentIndex < 1 || componentIndex > 3
                error('RCPlotter:InvalidComponent', '响应分量索引必须为1至3。');
            end

            cla(ax, 'reset');
            view(ax, 2);
            hold(ax, 'on');
            grid(ax, 'on');
            colors = lines(max(1, numel(logs)));
            handles = gobjects(1, numel(logs));
            for caseIndex = 1:numel(logs)
                data = RCPlotter.responseData(logs{caseIndex}, dataType);
                lineWidth = 0.9;
                if caseIndex == 1
                    lineWidth = 1.8;
                end
                handles(caseIndex) = plot(ax, logs{caseIndex}.Time, ...
                    data(componentIndex, :), 'Color', colors(caseIndex, :), ...
                    'LineWidth', lineWidth);
            end
            RCPlotter.drawFaultLine(ax, logs, options);
            if strcmpi(dataType, 'att')
                names = {'phi', 'theta', 'psi'};
                unit = 'rad';
            else
                names = {'x', 'y', 'z'};
                unit = 'm';
            end
            xlabel(ax, '时间 / s');
            ylabel(ax, sprintf('%s / %s', names{componentIndex}, unit));
            title(ax, RCPlotter.getOption(options, 'Title', ...
                [names{componentIndex}, '响应']));
            if RCPlotter.getOption(options, 'ShowLegend', componentIndex == 1)
                legend(ax, handles, labels, 'Location', 'best');
            end
            hold(ax, 'off');
        end

        function plotThrusterPulseWidths(axesList, logData, options)
        %PLOTTHRUSTERPULSEWIDTHS Draw plot_4 pulse width for every thruster.
            if nargin < 3 || isempty(options)
                options = struct();
            end
            if ~isstruct(logData) || ...
                    ~isfield(logData, 'Pulse_Widths') || isempty(logData.Pulse_Widths)
                error('RCPlotter:InvalidPulseLog', ...
                    '闭环仿真结果中缺少 Pulse_Widths 数据。');
            end
            thrusterCount = size(logData.Pulse_Widths, 1);
            if numel(axesList) < thrusterCount
                error('RCPlotter:InsufficientPulseAxes', ...
                    '脉宽图需要%d个坐标轴，当前只有%d个。', ...
                    thrusterCount, numel(axesList));
            end

            useControlHistory = RCPlotter.getOption( ...
                options, 'UseControlHistory', true);
            hasControlHistory = useControlHistory && ...
                isfield(logData, 'Control_Time') && ...
                isfield(logData, 'Pulse_History') && ...
                ~isempty(logData.Control_Time) && ...
                size(logData.Pulse_History, 1) == thrusterCount;
            if hasControlHistory
                sampleCount = min(numel(logData.Control_Time), ...
                    size(logData.Pulse_History, 2));
                time = logData.Control_Time(1:sampleCount);
                pulse = logData.Pulse_History(:, 1:sampleCount);
            else
                sampleCount = min(numel(logData.Time), ...
                    size(logData.Pulse_Widths, 2));
                time = logData.Time(1:sampleCount);
                pulse = logData.Pulse_Widths(:, 1:sampleCount);
            end

            lineColor = RCPlotter.getOption(options, ...
                'LineColor', [0.12, 0.45, 0.78]);
            layoutName = char(string(RCPlotter.getOption( ...
                options, 'LayoutName', '')));
            for thrusterIndex = 1:thrusterCount
                ax = axesList(thrusterIndex);
                RCPlotter.validateAxes(ax);
                cla(ax, 'reset');
                view(ax, 2);
                hold(ax, 'on');
                grid(ax, 'on');
                box(ax, 'on');
                stairs(ax, time, pulse(thrusterIndex, :), ...
                    'Color', lineColor, 'LineWidth', 1.05);
                RCPlotter.drawFaultLine(ax, {logData}, options);
                if isempty(layoutName)
                    title(ax, sprintf('推力器 %d', thrusterIndex));
                else
                    title(ax, sprintf('%s｜推力器 %d', ...
                        layoutName, thrusterIndex));
                end
                xlabel(ax, '时间 / s');
                ylabel(ax, '脉宽 / s');
                pulseMaximum = max(pulse(thrusterIndex, :), [], 'omitnan');
                if isempty(pulseMaximum) || ~isfinite(pulseMaximum) || ...
                        pulseMaximum <= 0
                    pulseMaximum = 1;
                end
                ylim(ax, [0, 1.12 * pulseMaximum]);
                hold(ax, 'off');
            end
        end

        function plotDirectionAngleMatrix(ax, B, faultyIndices, options)
        %PLOTDIRECTIONANGLEMATRIX Plot the six-dimensional vector angles.
            if nargin < 3 || isempty(faultyIndices)
                faultyIndices = [];
            end
            if nargin < 4 || isempty(options)
                options = struct();
            end
            RCPlotter.validateAxes(ax);
            healthy = setdiff(1:size(B, 2), ...
                RCPlotter.validIndices(faultyIndices, size(B, 2)));
            matrix = B(:, healthy);
            matrix = matrix ./ max(vecnorm(matrix, 2, 1), 1e-12);
            cosine = min(max(matrix' * matrix, -1), 1);
            angleMatrix = acosd(cosine);

            cla(ax, 'reset');
            view(ax, 2);
            imagesc(ax, angleMatrix, [0, 180]);
            axis(ax, 'image');
            colormap(ax, jet);
            colorbar(ax);
            xticks(ax, 1:numel(healthy));
            yticks(ax, 1:numel(healthy));
            xticklabels(ax, string(healthy));
            yticklabels(ax, string(healthy));
            xlabel(ax, '推力器编号');
            ylabel(ax, '推力器编号');
            title(ax, RCPlotter.getOption(options, 'Title', ...
                '6维控制向量夹角 / deg'));
        end

        function plotMetricBars(ax, comparison, metricIndex, options)
        %PLOTMETRICBARS Plot one normalized reconfigurability metric.
            if nargin < 4 || isempty(options)
                options = struct();
            end
            RCPlotter.validateAxes(ax);
            metricNames = {'控制能力 J_c', '可诊断性 J_o', ...
                '控制指令 J_t', '能量消耗 J_f'};
            if metricIndex < 1 || metricIndex > numel(metricNames)
                error('RCPlotter:InvalidMetric', '评价指标索引必须为1至4。');
            end
            cla(ax, 'reset');
            view(ax, 2);
            values = comparison.MetricValues(:, :, metricIndex);
            handles = bar(ax, values, 'grouped');
            colors = [0.42, 0.72, 0.46; 0.92, 0.43, 0.39; ...
                0.36, 0.58, 0.86; 0.65, 0.48, 0.78];
            for index = 1:min(numel(handles), size(colors, 1))
                handles(index).FaceColor = colors(index, :);
                handles(index).EdgeColor = [0.35, 0.35, 0.35];
            end
            grid(ax, 'on');
            ylim(ax, [0, 1.05]);
            xticks(ax, 1:numel(comparison.FaultLabels));
            xticklabels(ax, comparison.FaultLabels);
            xtickangle(ax, 45);
            ylabel(ax, '归一化值');
            title(ax, RCPlotter.getOption(options, 'Title', ...
                metricNames{metricIndex}));
            legend(ax, comparison.LayoutNames, 'Location', 'best');
        end

        function plotControlEnvelopeComparison(forceAxes, torqueAxes, ...
                params, layoutSet, faultyIndices)
        %PLOTCONTROLENVELOPECOMPARISON Compare fault-specific force/torque spaces.
            if nargin < 5 || isempty(faultyIndices)
                faultyIndices = [];
            end
            if isempty(layoutSet)
                error('RCPlotter:EmptyLayoutSet', '至少需要一个布局方案。');
            end
            RCPlotter.validateAxes(forceAxes);
            RCPlotter.validateAxes(torqueAxes);
            comparisonSet = layoutSet;
            for layoutIndex = 1:numel(comparisonSet)
                thrusterCount = size(comparisonSet(layoutIndex).B, 2);
                faulty = RCPlotter.validIndices(faultyIndices, thrusterCount);
                healthy = setdiff(1:thrusterCount, faulty);
                if isempty(healthy)
                    error('RCPlotter:NoHealthyThruster', ...
                        '布局“%s”在所选故障下没有可用推力器。', ...
                        char(string(comparisonSet(layoutIndex).name)));
                end
                comparisonSet(layoutIndex).B = ...
                    comparisonSet(layoutIndex).B(:, healthy);
            end
            conditionText = RCPlotter.faultLabel(faultyIndices);
            colors = lines(numel(layoutSet));
            RCPlotter.drawControlEnvelopeAxis(forceAxes, params, comparisonSet, ...
                1:3, colors, [conditionText, '：力空间包络对比'], ...
                {'F_x / N', 'F_y / N', 'F_z / N'});
            RCPlotter.drawControlEnvelopeAxis(torqueAxes, params, comparisonSet, ...
                4:6, colors, [conditionText, '：力矩空间包络对比'], ...
                {'M_x / (N·m)', 'M_y / (N·m)', 'M_z / (N·m)'});
        end

        function plotControlCommandComparison(qualityAxes, errorAxes, ...
                comparison, faultyIndices)
        %PLOTCONTROLCOMMANDCOMPARISON Compare Jt and command tracking error.
            selected = RCPlotter.selectedFaultMetricCase( ...
                comparison, faultyIndices);
            conditionText = RCPlotter.faultLabel(faultyIndices);
            RCPlotter.drawLayoutMetricAxis(qualityAxes, ...
                selected.LayoutNames, selected.Original(:, 4), ...
                [conditionText, '：控制指令跟踪质量（越大越好）'], ...
                'J_t', true);
            if size(selected.Original, 2) >= 6
                errorValues = selected.Original(:, 6);
            else
                errorValues = max(0, 1 ./ max(selected.Original(:, 4), eps) - 1);
            end
            RCPlotter.drawLayoutMetricAxis(errorAxes, ...
                selected.LayoutNames, errorValues, ...
                [conditionText, '：平均相对指令误差（越小越好）'], ...
                '相对误差', false);
        end

        function plotEnergyConsumptionComparison(pulseAxes, scoreAxes, ...
                comparison, faultyIndices)
        %PLOTENERGYCONSUMPTIONCOMPARISON Compare raw pulse and normalized Jf.
            selected = RCPlotter.selectedFaultMetricCase( ...
                comparison, faultyIndices);
            conditionText = RCPlotter.faultLabel(faultyIndices);
            RCPlotter.drawLayoutMetricAxis(pulseAxes, ...
                selected.LayoutNames, selected.Original(:, 5), ...
                [conditionText, '：平均累计脉宽（越小越好）'], ...
                'J_f / s', false);
            RCPlotter.drawLayoutMetricAxis(scoreAxes, ...
                selected.LayoutNames, selected.Normalized(:, 4), ...
                [conditionText, '：归一化能量利用评价（越大越好）'], ...
                '归一化 J_f', true);
        end

        function comparison = metricComparisonData(params, layoutSet, faultNumber)
        %METRICCOMPARISONDATA Build normalized plot_2 metric data.
            faultNumber = max(1, min(round(faultNumber), params.Num));
            [evalGrid, ~] = RCPlotter.evaluateGrid(params, layoutSet, faultNumber);
            evalSet = evalGrid(:, 1);
            referenceFaults = evalSet{1}.FaultSets;
            caseCount = numel(referenceFaults);
            metricValues = zeros(caseCount, numel(layoutSet), 4);
            faultLabels = cell(1, caseCount);
            for caseIndex = 1:caseCount
                faultSet = RCPlotter.normalizeFaultSet( ...
                    referenceFaults{caseIndex}, params.Num);
                faultLabels{caseIndex} = RCPlotter.faultLabel(faultSet);
                for layoutIndex = 1:numel(layoutSet)
                    row = RCPlotter.findFaultRow( ...
                        evalSet{layoutIndex}.FaultSets, faultSet, params.Num);
                    if ~isempty(row)
                        metricValues(caseIndex, layoutIndex, :) = ...
                            evalSet{layoutIndex}.MetricRaw(row, :);
                    end
                end
            end
            comparison = struct();
            comparison.MetricValues = metricValues;
            comparison.FaultLabels = faultLabels;
            comparison.LayoutNames = arrayfun( ...
                @(item)char(string(item.name)), layoutSet, ...
                'UniformOutput', false);
            comparison.Evaluations = evalSet;
        end

        function result = allocationStrategyData(params, layoutSet, faultyThrusters)
        %ALLOCATIONSTRATEGYDATA Build the plot_5 six-axis strategy table.
            axisNames = {'X', 'Y', 'Z'};
            rows = cell(0, 5);
            for layoutIndex = 1:numel(layoutSet)
                matrix = params.F_max * layoutSet(layoutIndex).B;
                for isOrbit = [true, false]
                    if isOrbit
                        controlType = '轨道控制';
                    else
                        controlType = '姿态控制';
                    end
                    for axisIndex = 1:3
                        for direction = [1, -1]
                            indices = RCPlotter.axisStrategyIndices(params, matrix, ...
                                faultyThrusters, axisIndex, direction, isOrbit);
                            if direction > 0
                                directionText = '+';
                            else
                                directionText = '-';
                            end
                            rows(end + 1, :) = {char(string(layoutSet(layoutIndex).name)), ...
                                controlType, [directionText, axisNames{axisIndex}], ...
                                mat2str(indices), numel(indices)}; %#ok<AGROW>
                        end
                    end
                end
            end
            result = struct('Data', {rows}, 'ColumnNames', ...
                {{'布局方案', '控制类型', '轴向', '调用推力器', '台数'}});
        end

        function result = allocationComparisonData(params, layout, faultyThrusters)
        %ALLOCATIONCOMPARISONDATA Compare nominal and faulty six-axis calls.
            if numel(layout) ~= 1 || ~isfield(layout, 'B') || isempty(layout.B)
                error('RCPlotter:InvalidAllocationLayout', ...
                    '调用策略比较需要一个有效的布局方案。');
            end
            matrix = params.F_max * layout.B;
            thrusterCount = size(matrix, 2);
            faultyThrusters = RCPlotter.validIndices( ...
                faultyThrusters, thrusterCount);
            axisNames = {'X', 'Y', 'Z'};
            rows = cell(12, 4);
            rowIndex = 0;
            for isOrbit = [true, false]
                if isOrbit
                    controlType = '轨道控制';
                else
                    controlType = '姿态控制';
                end
                for axisIndex = 1:3
                    for direction = [1, -1]
                        rowIndex = rowIndex + 1;
                        nominal = RCPlotter.axisStrategyIndices(params, matrix, ...
                            [], axisIndex, direction, isOrbit);
                        faulty = RCPlotter.axisStrategyIndices(params, matrix, ...
                            faultyThrusters, axisIndex, direction, isOrbit);
                        if direction > 0
                            directionText = '+';
                        else
                            directionText = '-';
                        end
                        rows(rowIndex, :) = {controlType, ...
                            [directionText, axisNames{axisIndex}], ...
                            RCPlotter.indexListText(nominal), ...
                            RCPlotter.indexListText(faulty)};
                    end
                end
            end
            result = struct('Data', {rows}, 'ColumnNames', ...
                {{'控制类型', '轴向', '标况下调用', '故障下调用'}});
        end

        function result = reconfigSummaryData(params, layoutSet, faultNumbers)
        %RECONFIGSUMMARYDATA Build the plot_7 fault-count summary table.
            faultNumbers = unique(round(faultNumbers(:)'));
            faultNumbers = faultNumbers(faultNumbers >= 1 & faultNumbers <= params.Num);
            [evalGrid, ~] = RCPlotter.evaluateGrid(params, layoutSet, faultNumbers);
            columnNames = {'布局方案', '故障数量', '故障状态', ...
                '可重构数', '不可重构数'};
            data = cell(numel(layoutSet) * numel(faultNumbers), 5);
            rowIndex = 0;
            for layoutIndex = 1:numel(layoutSet)
                layoutName = char(string(layoutSet(layoutIndex).name));
                for faultIndex = 1:numel(faultNumbers)
                    evaluation = RCPlotter.applyReconfigStatus( ...
                        evalGrid{layoutIndex, faultIndex});
                    rows = 2:numel(evaluation.FaultSets);
                    reconfigurable = sum(evaluation.IsReconfig(rows));
                    nonreconfigurable = numel(rows) - reconfigurable;
                    if nonreconfigurable == 0
                        status = '完全可重构';
                    elseif reconfigurable == 0
                        status = '不可重构';
                    else
                        status = '部分可重构';
                    end
                    rowIndex = rowIndex + 1;
                    data(rowIndex, :) = {layoutName, faultNumbers(faultIndex), ...
                        status, reconfigurable, nonreconfigurable};
                end
            end
            result = struct('Data', {data}, 'ColumnNames', {columnNames}, ...
                'Evaluations', {evalGrid});
        end

        function result = singleFaultEvaluationData(params, layoutSet)
        %SINGLEFAULTEVALUATIONDATA Build the plot_8 score table.
            [evalGrid, raw] = RCPlotter.evaluateGrid(params, layoutSet, 1);
            weights = RCPlotter.combinedWeight(raw, ...
                RCPlotter.ahpWeight(params, 4), RCPlotter.entropyWeight(raw));
            evalSet = evalGrid(:, 1);
            for layoutIndex = 1:numel(evalSet)
                evaluation = evalSet{layoutIndex};
                evaluation.Score = evaluation.MetricRaw * weights;
                evaluation.Weight = weights;
                evaluation = RCPlotter.applyReconfigStatus(evaluation);
                evalSet{layoutIndex} = evaluation;
            end

            columnNames = {'故障推力器编号'};
            for layoutIndex = 1:numel(layoutSet)
                columnNames{end + 1} = ...
                    [char(string(layoutSet(layoutIndex).name)), '-综合评价']; %#ok<AGROW>
            end
            faultLabels = 0:params.Num;
            data = cell(numel(faultLabels) + 1, numel(columnNames));
            averages = zeros(1, numel(layoutSet));
            for layoutIndex = 1:numel(layoutSet)
                averages(layoutIndex) = mean(evalSet{layoutIndex}.Score, 'omitnan');
            end
            for rowIndex = 1:numel(faultLabels)
                fault = faultLabels(rowIndex);
                data{rowIndex, 1} = uint32(fault);
                target = [];
                if fault > 0
                    target = fault;
                end
                for layoutIndex = 1:numel(layoutSet)
                    row = RCPlotter.findFaultRow( ...
                        evalSet{layoutIndex}.FaultSets, target, params.Num);
                    if ~isempty(row)
                        data{rowIndex, layoutIndex + 1} = ...
                            evalSet{layoutIndex}.Score(row);
                    end
                end
            end
            averageRow = numel(faultLabels) + 1;
            data{averageRow, 1} = '平均值';
            for layoutIndex = 1:numel(layoutSet)
                data{averageRow, layoutIndex + 1} = averages(layoutIndex);
            end
            weightText = sprintf( ...
                '组合权重：控制 %.3f，诊断 %.3f，跟踪 %.3f，能耗 %.3f', ...
                weights(1), weights(2), weights(3), weights(4));
            averageItems = arrayfun(@(index)sprintf('%s %.4f', ...
                char(string(layoutSet(index).name)), averages(index)), ...
                1:numel(layoutSet), 'UniformOutput', false);
            result = struct('Data', {data}, 'ColumnNames', {columnNames}, ...
                'Weight', weights, 'WeightText', weightText, ...
                'AverageText', strjoin(averageItems, ' | '), ...
                'Evaluations', {evalSet});
        end
    end

    methods (Static, Access = private)
        function selected = selectedFaultMetricCase(comparison, faultyIndices)
            if ~isstruct(comparison) || ...
                    ~isfield(comparison, 'Evaluations') || ...
                    isempty(comparison.Evaluations)
                error('RCPlotter:InvalidMetricComparison', ...
                    '评价结果中缺少不同布局的指标数据。');
            end
            targetFaults = unique(round(double(faultyIndices(:)')));
            targetFaults = targetFaults(targetFaults >= 1);
            layoutCount = numel(comparison.Evaluations);
            original = zeros(layoutCount, 6);
            normalized = zeros(layoutCount, 4);
            for layoutIndex = 1:layoutCount
                evaluation = comparison.Evaluations{layoutIndex};
                targetRow = [];
                for rowIndex = 1:numel(evaluation.FaultSets)
                    candidate = unique(round(double( ...
                        evaluation.FaultSets{rowIndex}(:)')));
                    candidate = candidate(candidate >= 1);
                    if isequal(candidate, targetFaults)
                        targetRow = rowIndex;
                        break;
                    end
                end
                if isempty(targetRow)
                    error('RCPlotter:MetricCaseMissing', ...
                        '评价结果中未找到故障工况“%s”。', ...
                        RCPlotter.faultLabel(targetFaults));
                end
                columnCount = min(size(evaluation.MetricOriginal, 2), 6);
                original(layoutIndex, 1:columnCount) = ...
                    evaluation.MetricOriginal(targetRow, 1:columnCount);
                normalized(layoutIndex, :) = ...
                    evaluation.MetricRaw(targetRow, 1:4);
            end
            selected = struct('LayoutNames', {comparison.LayoutNames}, ...
                'Original', original, 'Normalized', normalized);
        end

        function drawLayoutMetricAxis(ax, layoutNames, values, ...
                titleText, yLabelText, unitInterval)
            RCPlotter.validateAxes(ax);
            values = double(values(:));
            layoutCount = numel(values);
            layoutNames = RCPlotter.normalizeLabels(layoutNames, layoutCount);
            cla(ax, 'reset');
            view(ax, 2);
            grid(ax, 'on');
            box(ax, 'on');
            colors = lines(max(1, layoutCount));
            barHandle = bar(ax, 1:layoutCount, values, 0.62, ...
                'FaceColor', 'flat', 'EdgeColor', [0.35, 0.35, 0.35]);
            barHandle.CData = colors(1:layoutCount, :);
            xticks(ax, 1:layoutCount);
            xticklabels(ax, layoutNames);
            xtickangle(ax, 15);
            xlim(ax, [0.35, max(1.65, layoutCount + 0.65)]);
            ylabel(ax, yLabelText);
            title(ax, titleText);
            finiteValues = values(isfinite(values) & values >= 0);
            if unitInterval
                ylim(ax, [0, 1.05]);
                labelOffset = 0.025;
            elseif isempty(finiteValues) || max(finiteValues) <= 0
                ylim(ax, [0, 1]);
                labelOffset = 0.025;
            else
                upperLimit = 1.18 * max(finiteValues);
                ylim(ax, [0, upperLimit]);
                labelOffset = 0.025 * upperLimit;
            end
            for index = 1:layoutCount
                if isfinite(values(index))
                    text(ax, index, values(index) + labelOffset, ...
                        sprintf('%.4g', values(index)), ...
                        'HorizontalAlignment', 'center', 'FontSize', 9);
                end
            end
        end

        function drawControlEnvelopeAxis(ax, params, layoutSet, ...
                componentRows, colors, titleText, axisLabels)
            cla(ax, 'reset');
            hold(ax, 'on');
            grid(ax, 'on');
            box(ax, 'on');
            plotHandles = gobjects(1, numel(layoutSet));
            layoutNames = cell(1, numel(layoutSet));
            for layoutIndex = 1:numel(layoutSet)
                matrix = params.F_max * layoutSet(layoutIndex).B(componentRows, :);
                points = RCPlotter.controlEnvelopePoints(matrix);
                color = colors(layoutIndex, :);
                layoutNames{layoutIndex} = char(string(layoutSet(layoutIndex).name));
                centered = points - mean(points, 1);
                if size(points, 1) >= 4 && rank(centered, 1e-9) >= 3
                    try
                        faces = convhulln(points);
                    catch
                        faces = convhulln(points, {'QJ'});
                    end
                    plotHandles(layoutIndex) = patch(ax, ...
                        'Vertices', points, 'Faces', faces, ...
                        'FaceColor', color, 'FaceAlpha', 0.14, ...
                        'EdgeColor', color, 'EdgeAlpha', 0.45, ...
                        'LineWidth', 0.8, 'DisplayName', layoutNames{layoutIndex});
                else
                    plotHandles(layoutIndex) = scatter3(ax, ...
                        points(:, 1), points(:, 2), points(:, 3), 18, color, ...
                        'filled', 'DisplayName', layoutNames{layoutIndex});
                end
            end
            plot3(ax, 0, 0, 0, 'ko', 'MarkerSize', 4, ...
                'MarkerFaceColor', 'k', 'HandleVisibility', 'off');
            xlabel(ax, axisLabels{1});
            ylabel(ax, axisLabels{2});
            zlabel(ax, axisLabels{3});
            title(ax, titleText);
            axis(ax, 'equal');
            view(ax, 3);
            legend(ax, plotHandles, layoutNames, 'Location', 'best');
            hold(ax, 'off');
        end

        function points = controlEnvelopePoints(matrix)
            thrusterCount = size(matrix, 2);
            if thrusterCount <= 16
                states = dec2bin(0:(2^thrusterCount - 1)) - '0';
                points = (matrix * states')';
            else
                sampleCount = max(1800, 120 * thrusterCount);
                sampleIndex = (0:(sampleCount - 1))';
                z = 1 - 2 * (sampleIndex + 0.5) / sampleCount;
                azimuth = pi * (3 - sqrt(5)) * sampleIndex;
                radius = sqrt(max(0, 1 - z.^2));
                directions = [radius .* cos(azimuth), ...
                    radius .* sin(azimuth), z];
                active = directions * matrix >= 0;
                points = (matrix * double(active)')';
                points = [points; zeros(1, 3); sum(matrix, 2)']; %#ok<AGROW>
            end
            tolerance = max(1, max(abs(points), [], 'all')) * 1e-10;
            points = unique(round(points / tolerance) * tolerance, 'rows');
        end

        function logs = normalizeLogs(logData)
            if iscell(logData)
                logs = logData(:)';
            elseif isstruct(logData) && numel(logData) > 1
                logs = num2cell(logData(:)');
            else
                logs = {logData};
            end
            if isempty(logs)
                error('RCPlotter:EmptyLog', '闭环仿真结果不能为空。');
            end
            for index = 1:numel(logs)
                RCPlotter.validateSimulationLog(logs{index});
            end
        end

        function labels = normalizeLabels(labels, count)
            if nargin < 1 || isempty(labels)
                labels = {};
            elseif isstring(labels)
                labels = cellstr(labels(:)');
            elseif ischar(labels)
                labels = {labels};
            else
                labels = labels(:)';
            end
            for index = (numel(labels) + 1):count
                labels{index} = sprintf('结果%d', index); %#ok<AGROW>
            end
            labels = labels(1:count);
            labels = cellfun(@(value)char(string(value)), labels, ...
                'UniformOutput', false);
        end

        function data = responseData(logData, dataType)
            if strcmpi(dataType, 'att') || strcmpi(dataType, 'attitude')
                data = logData.Y_euler;
            else
                data = logData.Y(1:3, :);
            end
            if size(data, 1) < 3
                error('RCPlotter:InvalidResponse', '闭环响应数据至少需要三个分量。');
            end
            data = data(1:3, :);
        end

        function data = errorData(logData, dataType)
            RCPlotter.validateSimulationLog(logData);
            if strcmpi(dataType, 'att') || strcmpi(dataType, 'attitude')
                data = mod((logData.Y_euler - logData.E) + pi, 2*pi) - pi;
            else
                data = logData.Y(1:3, :) - logData.R;
            end
        end

        function drawFaultLine(ax, logs, options)
            if ~RCPlotter.getOption(options, 'ShowFaultTime', true)
                return;
            end
            faultTime = RCPlotter.getOption(options, 'FaultTime', []);
            if isempty(faultTime) && isfield(logs{1}, 'faulty_time')
                faultTime = logs{1}.faulty_time;
            end
            if ~isempty(faultTime) && isnumeric(faultTime) && ...
                    isscalar(faultTime) && isfinite(faultTime)
                xline(ax, faultTime, '--r', '故障时刻', ...
                    'HandleVisibility', 'off');
            end
        end

        function [evalGrid, raw] = evaluateGrid(params, layoutSet, faultNumbers)
            if isempty(layoutSet)
                error('RCPlotter:EmptyLayoutSet', '至少需要一个布局方案。');
            end
            faultNumbers = unique(round(faultNumbers(:)'));
            faultNumbers = faultNumbers(faultNumbers >= 1 & faultNumbers <= params.Num);
            if isempty(faultNumbers)
                error('RCPlotter:InvalidFaultNumber', ...
                    '故障数量必须在1至推力器总数之间。');
            end

            evalGrid = cell(numel(layoutSet), numel(faultNumbers));
            raw = zeros(0, 4);
            for faultIndex = 1:numel(faultNumbers)
                faultNumber = faultNumbers(faultIndex);
                evalSet = cell(numel(layoutSet), 1);
                for layoutIndex = 1:numel(layoutSet)
                    matrix = layoutSet(layoutIndex).B;
                    layoutParams = params;
                    layoutParams.Num = size(matrix, 2);
                    if faultNumber > layoutParams.Num
                        error('RCPlotter:FaultNumberTooLarge', ...
                            '故障数量不能大于布局“%s”的推力器总数。', ...
                            char(string(layoutSet(layoutIndex).name)));
                    elseif faultNumber == layoutParams.Num
                        evaluation = struct('FaultSets', {{[], 1:layoutParams.Num}}, ...
                            'Jc', zeros(2, 2), 'Jo', zeros(2, 1), ...
                            'Jt', zeros(2, 1), 'Jf', zeros(2, 1), ...
                            'Jc6', zeros(2, 1));
                    else
                        evaluation = Reconfig_eval(layoutParams, matrix, faultNumber);
                    end
                    evalSet{layoutIndex} = evaluation;
                end
                [evalSet, rawNormalized] = RCPlotter.normalizeEvalSet(evalSet, params.Num);
                for layoutIndex = 1:numel(layoutSet)
                    evaluation = evalSet{layoutIndex};
                    rowCount = size(evaluation.MetricRaw, 1);
                    evaluation.Score = nan(rowCount, 1);
                    evaluation.Weight = nan(4, 1);
                    evaluation.IsReconfig = false(rowCount, 1);
                    evaluation.Status = strings(rowCount, 1);
                    evalGrid{layoutIndex, faultIndex} = evaluation;
                end
                raw = [raw; rawNormalized]; %#ok<AGROW>
            end
        end

        function [evalSet, raw] = normalizeEvalSet(evalSet, maximumIndex)
            originalSize = size(evalSet);
            evalList = evalSet(:);
            for layoutIndex = 1:numel(evalList)
                evaluation = evalList{layoutIndex};
                evaluation.MetricOriginal = RCPlotter.extractRawMetrics(evaluation);
                evaluation.MetricRaw = zeros(size(evaluation.MetricOriginal, 1), 4);
                evalList{layoutIndex} = evaluation;
            end
            if isempty(evalList)
                raw = zeros(0, 4);
                evalSet = reshape(evalList, originalSize);
                return;
            end

            allOriginal = zeros(0, 6);
            for layoutIndex = 1:numel(evalList)
                allOriginal = [allOriginal; evalList{layoutIndex}.MetricOriginal]; %#ok<AGROW>
            end
            forceRef = RCPlotter.bestBenefitRef(allOriginal(:, 1));
            torqueRef = RCPlotter.bestBenefitRef(allOriginal(:, 2));
            diagnosticRef = RCPlotter.bestBenefitRef(allOriginal(:, 3));
            trackingRef = RCPlotter.bestBenefitRef(allOriginal(:, 4));
            energyRef = RCPlotter.bestCostRef(allOriginal(:, 5));

            referenceFaultSets = evalList{1}.FaultSets;
            for caseIndex = 1:numel(referenceFaultSets)
                target = RCPlotter.normalizeFaultSet( ...
                    referenceFaultSets{caseIndex}, maximumIndex);
                for layoutIndex = 1:numel(evalList)
                    evaluation = evalList{layoutIndex};
                    row = RCPlotter.findFaultRow( ...
                        evaluation.FaultSets, target, maximumIndex);
                    if isempty(row)
                        continue;
                    end
                    original = evaluation.MetricOriginal(row, :);
                    evaluation.MetricRaw(row, :) = [ ...
                        min(RCPlotter.normalizeBenefit(original(1), forceRef), ...
                            RCPlotter.normalizeBenefit(original(2), torqueRef)), ...
                        RCPlotter.normalizeBenefit(original(3), diagnosticRef), ...
                        RCPlotter.normalizeBenefit(original(4), trackingRef), ...
                        RCPlotter.normalizeCost(original(5), energyRef)];
                    evalList{layoutIndex} = evaluation;
                end
            end

            raw = zeros(0, 4);
            for layoutIndex = 1:numel(evalList)
                if size(evalList{layoutIndex}.MetricRaw, 1) > 1
                    raw = [raw; evalList{layoutIndex}.MetricRaw(2:end, :)]; %#ok<AGROW>
                end
            end
            evalSet = reshape(evalList, originalSize);
        end

        function raw = extractRawMetrics(evaluation)
            rowCount = numel(evaluation.FaultSets);
            raw = zeros(rowCount, 6);
            if isfield(evaluation, 'Raw') && isfield(evaluation.Raw, 'JcForce')
                raw(:, 1) = RCPlotter.vectorOrZero(evaluation.Raw.JcForce, rowCount);
            elseif isfield(evaluation, 'Jc') && size(evaluation.Jc, 2) >= 1
                raw(:, 1) = RCPlotter.vectorOrZero(evaluation.Jc(:, 1), rowCount);
            end
            if isfield(evaluation, 'Raw') && isfield(evaluation.Raw, 'JcTorque')
                raw(:, 2) = RCPlotter.vectorOrZero(evaluation.Raw.JcTorque, rowCount);
            elseif isfield(evaluation, 'Jc') && size(evaluation.Jc, 2) >= 2
                raw(:, 2) = RCPlotter.vectorOrZero(evaluation.Jc(:, 2), rowCount);
            else
                raw(:, 2) = raw(:, 1);
            end
            if isfield(evaluation, 'Raw') && isfield(evaluation.Raw, 'JoAngle')
                raw(:, 3) = RCPlotter.vectorOrZero(evaluation.Raw.JoAngle, rowCount);
            elseif isfield(evaluation, 'Jo')
                raw(:, 3) = RCPlotter.vectorOrZero(evaluation.Jo, rowCount);
            end
            if isfield(evaluation, 'Raw') && isfield(evaluation.Raw, 'JtQuality')
                raw(:, 4) = RCPlotter.vectorOrZero(evaluation.Raw.JtQuality, rowCount);
            elseif isfield(evaluation, 'Jt')
                raw(:, 4) = RCPlotter.vectorOrZero(evaluation.Jt, rowCount);
            end
            if isfield(evaluation, 'Raw') && isfield(evaluation.Raw, 'JfPulse')
                raw(:, 5) = RCPlotter.vectorOrZero(evaluation.Raw.JfPulse, rowCount);
            elseif isfield(evaluation, 'Jf')
                raw(:, 5) = RCPlotter.vectorOrZero(evaluation.Jf, rowCount);
            end
            if isfield(evaluation, 'Raw') && isfield(evaluation.Raw, 'JtError')
                raw(:, 6) = RCPlotter.vectorOrZero(evaluation.Raw.JtError, rowCount);
            end
        end

        function value = vectorOrZero(value, count)
            value = value(:);
            result = zeros(count, 1);
            itemCount = min(count, numel(value));
            result(1:itemCount) = value(1:itemCount);
            result(~isfinite(result)) = 0;
            value = result;
        end

        function value = bestBenefitRef(values)
            values = values(isfinite(values) & values > 0);
            if isempty(values)
                value = 0;
            else
                value = max(values);
            end
        end

        function value = bestCostRef(values)
            values = values(isfinite(values) & values >= 0);
            if isempty(values)
                value = NaN;
            else
                value = min(values);
            end
        end

        function value = normalizeBenefit(value, reference)
            if ~isfinite(value) || value <= 0 || ...
                    ~isfinite(reference) || reference <= 1e-12
                value = 0;
            else
                value = max(0, min(1, value / reference));
            end
        end

        function value = normalizeCost(value, reference)
            if ~isfinite(value) || value < 0 || ~isfinite(reference)
                value = 0;
            elseif reference <= 1e-12
                value = double(value <= 1e-12);
            else
                value = max(0, min(1, reference / max(value, 1e-12)));
            end
        end

        function faultSet = normalizeFaultSet(faultSet, maximumIndex)
            faultSet = unique(round(double(faultSet(:)')));
            faultSet = faultSet(faultSet >= 1 & faultSet <= maximumIndex);
        end

        function label = faultLabel(faultSet)
            if isempty(faultSet)
                label = '标况';
            elseif numel(faultSet) == 1
                label = sprintf('故障%d', faultSet);
            else
                label = ['故障[', strjoin(cellstr(string(faultSet)), ','), ']'];
            end
        end

        function row = findFaultRow(faultSets, targetSet, maximumIndex)
            targetSet = RCPlotter.normalizeFaultSet(targetSet, maximumIndex);
            row = [];
            for index = 1:numel(faultSets)
                candidate = RCPlotter.normalizeFaultSet(faultSets{index}, maximumIndex);
                if isequal(candidate, targetSet)
                    row = index;
                    return;
                end
            end
        end

        function indices = axisStrategyIndices(params, matrix, faultyThrusters, ...
                axisIndex, direction, isOrbit)
            forceCommand = zeros(3, 1);
            torqueCommand = zeros(3, 1);
            if isOrbit
                level = 0.5 * max(abs(matrix(axisIndex, :)));
                forceCommand(axisIndex) = direction * level;
            else
                level = 0.5 * max(abs(matrix(axisIndex + 3, :)));
                torqueCommand(axisIndex) = direction * level;
            end
            if level <= 1e-12
                indices = [];
                return;
            end
            layoutParams = params;
            layoutParams.Num = size(matrix, 2);
            faultyThrusters = RCPlotter.validIndices( ...
                faultyThrusters, layoutParams.Num);
            % 主备选择属于轴向分配层。即使上层选择六维联合复用，
            % 六轴策略表仍按主用组合及故障后的备份组合进行展示。
            if isfield(layoutParams, 'allocation_strategy') && ...
                    strcmpi(layoutParams.allocation_strategy, 'primary_backup')
                layoutParams.alloc_mode = 'task_book';
            end
            [propFinal, info] = Thruster_invocation(forceCommand, torqueCommand, ...
                matrix, faultyThrusters, layoutParams);
            if isfield(info, 'mode') && strcmpi(info.mode, 'joint_6d')
                indices = find(propFinal > 1e-12)';
            elseif isOrbit
                indices = find(info.Prop_F > 1e-12)';
            else
                indices = find(info.Prop_T > 1e-12)';
            end
        end

        function text = indexListText(indices)
            indices = unique(round(double(indices(:)')));
            if isempty(indices)
                text = '-';
            else
                text = strjoin(cellstr(string(indices)), ', ');
            end
        end

        function evaluation = applyReconfigStatus(evaluation)
            rowCount = size(evaluation.MetricRaw, 1);
            evaluation.IsReconfig = false(rowCount, 1);
            if rowCount >= 2
                rows = 2:rowCount;
                evaluation.IsReconfig(rows) = ...
                    all(evaluation.MetricRaw(rows, :) > eps, 2);
            end
            evaluation.Status = repmat("不可重构", rowCount, 1);
            evaluation.Status(evaluation.IsReconfig) = "可重构";
        end

        function weights = ahpWeight(params, count)
            if isfield(params, 'reconfig_ahp_weight') && ...
                    ~isempty(params.reconfig_ahp_weight)
                weights = params.reconfig_ahp_weight(:);
                if numel(weights) == count && all(isfinite(weights)) && ...
                        sum(max(weights, 0)) > 1e-12
                    weights = max(weights, 0);
                    weights = weights / sum(weights);
                    return;
                end
            end
            if isfield(params, 'reconfig_ahp_matrix') && ...
                    ~isempty(params.reconfig_ahp_matrix)
                matrix = params.reconfig_ahp_matrix;
            else
                matrix = [1, 2, 2, 3; 1/2, 1, 2, 2; ...
                    1/2, 1/2, 1, 2; 1/3, 1/2, 1/2, 1];
            end
            if ~isequal(size(matrix), [count, count]) || ...
                    any(~isfinite(matrix(:))) || any(matrix(:) <= 0)
                weights = ones(count, 1) / count;
                return;
            end
            weights = prod(matrix, 2).^(1 / count);
            weights = weights / sum(weights);
        end

        function weights = entropyWeight(data)
            [rowCount, columnCount] = size(data);
            if columnCount == 0
                weights = zeros(0, 1);
                return;
            elseif rowCount <= 1
                weights = ones(columnCount, 1) / columnCount;
                return;
            end
            weights = zeros(columnCount, 1);
            data(~isfinite(data) | data < 0) = 0;
            for column = 1:columnCount
                total = sum(data(:, column));
                if total > 1e-12
                    probability = data(:, column) / total;
                else
                    probability = ones(rowCount, 1) / rowCount;
                end
                probability = probability(probability > 1e-12);
                weights(column) = 1 + ...
                    sum(probability .* log(probability)) / log(rowCount);
            end
            if sum(weights) > 1e-12
                weights = weights / sum(weights);
            else
                weights = ones(columnCount, 1) / columnCount;
            end
        end

        function weights = combinedWeight(data, subjective, objective)
            columnCount = size(data, 2);
            if columnCount == 0
                weights = zeros(0, 1);
                return;
            end
            subjective = subjective(:);
            objective = objective(:);
            if numel(subjective) ~= columnCount || numel(objective) ~= columnCount
                weights = ones(columnCount, 1) / columnCount;
                return;
            end
            squareSums = sum(data.^2, 1)';
            if sum(squareSums) < 1e-12
                weights = 0.5 * (subjective + objective);
            else
                matrix = diag(max(squareSums, 1e-12));
                rightSide = 0.5 * (subjective + objective) .* squareSums;
                unit = ones(columnCount, 1);
                solution = [matrix, unit; unit', 0] \ [rightSide; 1];
                weights = solution(1:columnCount);
            end
            weights(~isfinite(weights) | weights < 0) = 0;
            if sum(weights) < 1e-12
                weights = ones(columnCount, 1) / columnCount;
            else
                weights = weights / sum(weights);
            end
        end

        function validateAxes(ax)
            if isempty(ax) || ~isscalar(ax) || ~isgraphics(ax)
                error('RCPlotter:InvalidAxes', ...
                    '第一个输入必须是有效的 MATLAB Axes 或 UIAxes。');
            end
        end

        function validateLayoutInputs(ax, B, r)
            RCPlotter.validateAxes(ax);
            if ~isnumeric(B) || ~isnumeric(r) || ...
                    size(B, 1) < 3 || size(r, 1) ~= 3 || ...
                    size(B, 2) ~= size(r, 2)
                error('RCPlotter:InvalidLayout', ...
                    'B 应至少为 3×N，r 应为 3×N，且推力器数量一致。');
            end
        end

        function validateSimulationLog(logData)
            requiredFields = {'Time', 'Y', 'R', 'Y_euler', 'E'};
            if ~isstruct(logData) || ...
                    ~all(cellfun(@(name)isfield(logData, name), requiredFields))
                error('RCPlotter:InvalidLog', ...
                    '仿真结果缺少 Time、Y、R、Y_euler 或 E 字段。');
            end
        end

        function plotThrusterGroup3D(ax, r, directions, indices, ...
                markerColor, arrowColor, lineStyle)
            if isempty(indices)
                return;
            end
            plot3(ax, r(1, indices), r(2, indices), r(3, indices), 'o', ...
                'MarkerSize', 6, 'MarkerFaceColor', markerColor, ...
                'MarkerEdgeColor', 'k');
            quiver3(ax, r(1, indices), r(2, indices), r(3, indices), ...
                directions(1, indices), directions(2, indices), ...
                directions(3, indices), 0.35, 'Color', arrowColor, ...
                'LineWidth', 1.1, 'LineStyle', lineStyle, ...
                'MaxHeadSize', 0.8);
        end

        function plotBodyCuboid3D(ax, halfSize, options)
            halfSize = double(halfSize(:)');
            if numel(halfSize) ~= 3 || any(~isfinite(halfSize)) || ...
                    any(halfSize <= 0)
                error('RCPlotter:InvalidBodySize', ...
                    '立方体半尺寸必须是3个大于0的有限数值。');
            end
            x = halfSize(1);
            y = halfSize(2);
            z = halfSize(3);
            vertices = [ ...
                -x, -y, -z;  x, -y, -z;  x,  y, -z; -x,  y, -z; ...
                -x, -y,  z;  x, -y,  z;  x,  y,  z; -x,  y,  z];
            faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; ...
                2 3 7 6; 3 4 8 7; 4 1 5 8];
            faceColor = RCPlotter.getOption(options, ...
                'BodyFaceColor', [0.8, 0.8, 0.8]);
            faceAlpha = RCPlotter.getOption(options, 'BodyFaceAlpha', 0.18);
            edgeColor = RCPlotter.getOption(options, ...
                'BodyEdgeColor', [0.10, 0.18, 0.25]);
            lineWidth = RCPlotter.getOption(options, 'BodyLineWidth', 1.6);

            patch(ax, 'Vertices', vertices, 'Faces', faces, ...
                'FaceColor', faceColor, 'FaceAlpha', faceAlpha, ...
                'EdgeColor', 'none', 'HandleVisibility', 'off');

            edges = [1 2; 2 3; 3 4; 4 1; 5 6; 6 7; ...
                7 8; 8 5; 1 5; 2 6; 3 7; 4 8];
            for edgeIndex = 1:size(edges, 1)
                points = vertices(edges(edgeIndex, :), :);
                plot3(ax, points(:, 1), points(:, 2), points(:, 3), '-', ...
                    'Color', edgeColor, 'LineWidth', lineWidth, ...
                    'HandleVisibility', 'off');
            end
        end

        function plotBodyProjection(ax, projectionAxes, halfSize)
            xHalf = halfSize(projectionAxes(1));
            yHalf = halfSize(projectionAxes(2));
            bodyX = [-xHalf, xHalf, xHalf, -xHalf];
            bodyY = [-yHalf, -yHalf, yHalf, yHalf];
            patch(ax, bodyX, bodyY, [0.8, 0.8, 0.8], ...
                'FaceAlpha', 0.18, 'EdgeColor', [0.6, 0.6, 0.6], ...
                'LineWidth', 1.0);
        end

        function plotThrusterGroup2D(ax, r, directions, indices, ...
                projectionAxes, markerColor, arrowColor, lineStyle)
            if isempty(indices)
                return;
            end
            plot(ax, r(projectionAxes(1), indices), ...
                r(projectionAxes(2), indices), 'o', 'MarkerSize', 5, ...
                'MarkerFaceColor', markerColor, 'MarkerEdgeColor', 'k');
            quiver(ax, r(projectionAxes(1), indices), ...
                r(projectionAxes(2), indices), ...
                directions(projectionAxes(1), indices), ...
                directions(projectionAxes(2), indices), 0.35, ...
                'Color', arrowColor, 'LineWidth', 1.2, ...
                'LineStyle', lineStyle, 'MaxHeadSize', 0.8);
        end

        function plotProjectionLabels(ax, r, projectionAxes, xLimits, yLimits)
            points = r(projectionAxes, :);
            sideFlag = ones(1, size(points, 2));
            sideFlag(points(1, :) < 0) = -1;
            sideFlag(abs(points(1, :)) < 1e-9 & points(2, :) < 0) = -1;
            sideGap = max(0.32, 0.18 * diff(xLimits));
            labelX = [xLimits(1) + sideGap, xLimits(2) - sideGap];

            for sideValue = [-1, 1]
                indices = find(sideFlag == sideValue);
                if isempty(indices)
                    continue;
                end
                labelY = RCPlotter.spreadLabelY(points(2, indices), yLimits, 0.18);
                xPosition = labelX(1 + (sideValue > 0));
                for localIndex = 1:numel(indices)
                    text(ax, xPosition, labelY(localIndex), ...
                        sprintf('%d', indices(localIndex)), ...
                        'HorizontalAlignment', 'center', ...
                        'VerticalAlignment', 'middle', 'FontSize', 9, ...
                        'FontWeight', 'bold', 'Color', 'k');
                end
            end
        end

        function labelY = spreadLabelY(desiredY, limits, minimumGap)
            [sortedY, order] = sort(desiredY(:));
            if isempty(sortedY)
                labelY = sortedY;
                return;
            end
            lowerLimit = limits(1) + 0.12;
            upperLimit = limits(2) - 0.12;
            sortedY = max(lowerLimit, min(upperLimit, sortedY));
            for index = 2:numel(sortedY)
                sortedY(index) = max(sortedY(index), ...
                    sortedY(index - 1) + minimumGap);
            end
            overflow = sortedY(end) - upperLimit;
            if overflow > 0
                sortedY = sortedY - overflow;
            end
            for index = numel(sortedY)-1:-1:1
                sortedY(index) = min(sortedY(index), ...
                    sortedY(index + 1) - minimumGap);
            end
            underflow = lowerLimit - sortedY(1);
            if underflow > 0
                sortedY = sortedY + underflow;
            end
            labelY = zeros(size(sortedY));
            labelY(order) = sortedY;
        end

        function indices = validIndices(indices, maximumIndex)
            if isempty(indices)
                indices = [];
                return;
            end
            indices = unique(round(double(indices(:)')));
            indices = indices(indices >= 1 & indices <= maximumIndex);
        end

        function value = getOption(options, name, defaultValue)
            if isstruct(options) && isfield(options, name) && ...
                    ~isempty(options.(name))
                value = options.(name);
            else
                value = defaultValue;
            end
        end

        function value = getStructField(data, name, defaultValue)
            if isstruct(data) && isfield(data, name) && ~isempty(data.(name))
                value = data.(name);
            else
                value = defaultValue;
            end
        end
    end
end
