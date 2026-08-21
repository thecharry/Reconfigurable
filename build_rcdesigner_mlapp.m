function build_rcdesigner_mlapp
%BUILD_RCDESIGNER_MLAPP Build a fully editable App Designer application.

    rootDir = fileparts(mfilename('fullpath'));
    sourceDir = fullfile(rootDir, 'source');
    sourceFile = fullfile(sourceDir, 'RCDesignerPrototype.m');
    outputDir = fullfile(rootDir, 'app');
    outputFile = fullfile(outputDir, 'RCDesignerPrototype.mlapp');
    previewFile = fullfile(rootDir, 'artifacts', 'RCDesignerPrototype_preview.png');

    if ~isfile(sourceFile)
        error('RCDesigner:MissingSource', '找不到源码文件：%s', sourceFile);
    end
    if ~isfolder(outputDir)
        mkdir(outputDir);
    end

    originalPath = path;
    pathCleanup = onCleanup(@()path(originalPath)); %#ok<NASGU>
    addpath(sourceDir, '-begin');
    clear RCDesignerPrototype
    rehash;

    app = RCDesignerPrototype;
    appCleanup = onCleanup(@()deleteIfValid(app)); %#ok<NASGU>
    drawnow;
    app.UIFigure.Visible = 'off';
    drawnow;

    % The App Designer design model does not execute SizeChangedFcn while
    % it is being opened.  Persist the already calculated full-page tab
    % position so the design canvas is not shown at its 260-by-252 default.
    stabilizeMainTabGroup(app);

    % Plotting creates a hidden ScribeLayer inside UIAxes. cla() removes
    % curves but leaves that internal object behind, and App Designer then
    % fails while deserializing the axes. Replace plotted axes with clean
    % axes in the same grid cells before serializing the design model.
    replaceRuntimeAxes(app);
    stabilizeMainTabGroup(app);

    callbackNames = registeredCallbackNames();
    adapterMap = appdesigner.internal.appmetadata.getAllComponentAdapterMap();
    components = findDesignComponents(app.UIFigure, adapterMap);
    assignDesignTimeData(app, components, adapterMap, callbackNames);

    [callbacks, startupCallback, editableSectionCode] = ...
        buildCodeMetadata(sourceFile, callbackNames);

    serializer = appdesigner.internal.serialization.MLAPPSerializer( ...
        outputFile, app.UIFigure);
    serializer.OverwriteTargetFile = true;
    serializer.MatlabCodeText = fileread(sourceFile);
    serializer.Callbacks = callbacks;
    serializer.StartupCallback = startupCallback;
    serializer.EditableSectionCode = editableSectionCode;
    serializer.RunConfigurations = {''};
    serializer.SingletonMode = 'multi';

    metadata = appdesigner.internal.model.MetadataModel;
    metadata.Name = 'RC-Designer 可重构性设计软件';
    metadata.Author = '卫星项目组';
    metadata.Version = '0.3';
    metadata.Summary = '机动飞行器姿轨耦合复用推力器可重构性设计软件原型';
    metadata.Description = ['包含输入条件、优化设计与布局显示、可重构判断、', ...
        '可重构设计、调用策略、闭环仿真和结果报告页面。'];
    serializer.Metadata = metadata;

    if isfile(previewFile)
        serializer.ScreenshotPath = previewFile;
    end

    serializer.save();
    fprintf('MLAPP_CREATED=%s\n', outputFile);

    deleteIfValid(app);
    clear appCleanup app
    clear pathCleanup
end

function stabilizeMainTabGroup(app)
    if ~isprop(app, 'PageHostPanel') || ~isprop(app, 'TabGroup') || ...
            isempty(app.PageHostPanel) || isempty(app.TabGroup) || ...
            ~isvalid(app.PageHostPanel) || ~isvalid(app.TabGroup)
        return;
    end
    figurePosition = app.UIFigure.InnerPosition;
    rootPadding = app.RootGrid.Padding;
    rootRows = app.RootGrid.RowHeight;
    fixedRootHeight = 0;
    for index = 1:numel(rootRows)
        if isnumeric(rootRows{index})
            fixedRootHeight = fixedRootHeight + rootRows{index};
        end
    end
    bodyHeight = figurePosition(4) - rootPadding(2) - rootPadding(4) - ...
        fixedRootHeight - app.RootGrid.RowSpacing * (numel(rootRows) - 1);

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
    app.TabGroup.Position = [0, -tabBarHeight, max(1, hostWidth), ...
        max(1, hostHeight) + tabBarHeight];
end

function replaceRuntimeAxes(app)
    propertyNames = properties(app);
    axesNames = {};
    for index = 1:numel(propertyNames)
        propertyName = propertyNames{index};
        if isprop(app, propertyName) && ...
                isa(app.(propertyName), 'matlab.ui.control.UIAxes')
            axesNames{end + 1} = propertyName; %#ok<AGROW>
        end
    end
    for index = 1:numel(axesNames)
        propertyName = axesNames{index};
        if ~isprop(app, propertyName) || isempty(app.(propertyName))
            continue;
        end
        oldAxes = app.(propertyName);
        parent = oldAxes.Parent;
        row = oldAxes.Layout.Row;
        column = oldAxes.Layout.Column;
        visible = oldAxes.Visible;
        newAxes = uiaxes(parent);
        newAxes.Layout.Row = row;
        newAxes.Layout.Column = column;
        newAxes.Visible = visible;
        app.(propertyName) = newAxes;
        delete(oldAxes);
    end
end

function components = findDesignComponents(root, adapterMap)
    allObjects = findall(root);
    keep = false(size(allObjects));
    for index = 1:numel(allObjects)
        keep(index) = isKey(adapterMap, class(allObjects(index)));
    end
    components = allObjects(keep);
end

function assignDesignTimeData(app, components, adapterMap, callbackNames)
    preferredNames = getPreferredNames(app, components);
    codeNames = makeUniqueCodeNames(components, preferredNames);

    % Add the dynamic property used by native App Designer files.
    for index = 1:numel(components)
        component = components(index);
        if ~isprop(component, 'DesignTimeProperties')
            addprop(component, 'DesignTimeProperties');
        end
        designData = appdesigner.internal.model.DesignTimeProperties;
        designData.CodeName = codeNames{index};
        component.DesignTimeProperties = designData;
    end

    setSupportedCallbacks(components, callbackNames);

    generator = appdesigner.internal.codegeneration.ComponentCodeGenerator;
    appType = appdesigner.internal.serialization.app.AppTypes.StandardApp;
    for index = 1:numel(components)
        component = components(index);
        adapterClass = adapterMap(class(component));
        adapter = feval(adapterClass);
        component.DesignTimeProperties.ComponentCode = ...
            generator.getComponentGenerationCode(component, adapter, appType);
    end
end

function preferredNames = getPreferredNames(app, components)
    preferredNames = repmat({''}, size(components));
    propertyNames = properties(app);
    for propertyIndex = 1:numel(propertyNames)
        try
            value = app.(propertyNames{propertyIndex});
        catch
            continue;
        end
        if ~isscalar(value) || ~isvalidHandle(value)
            continue;
        end
        for componentIndex = 1:numel(components)
            if isequal(value, components(componentIndex))
                preferredNames{componentIndex} = propertyNames{propertyIndex};
                break;
            end
        end
    end
end

function tf = isvalidHandle(value)
    tf = isa(value, 'handle');
    if tf
        try
            tf = isvalid(value);
        catch
            tf = false;
        end
    end
end

function codeNames = makeUniqueCodeNames(components, preferredNames)
    codeNames = repmat({''}, size(components));
    usedNames = {};
    counters = containers.Map('KeyType', 'char', 'ValueType', 'double');

    % Reserve meaningful property names first.
    for index = 1:numel(components)
        candidate = preferredNames{index};
        if ~isempty(candidate) && ~ismember(candidate, usedNames)
            codeNames{index} = candidate;
            usedNames{end + 1} = candidate; %#ok<AGROW>
        end
    end

    % Give helper grids, labels, panels, and buttons stable generated names.
    for index = 1:numel(components)
        if ~isempty(codeNames{index})
            continue;
        end
        classParts = strsplit(class(components(index)), '.');
        baseName = matlab.lang.makeValidName(classParts{end});
        if isKey(counters, baseName)
            counters(baseName) = counters(baseName) + 1;
        else
            counters(baseName) = 1;
        end
        candidate = sprintf('%s%d', baseName, counters(baseName));
        while ismember(candidate, usedNames)
            counters(baseName) = counters(baseName) + 1;
            candidate = sprintf('%s%d', baseName, counters(baseName));
        end
        codeNames{index} = candidate;
        usedNames{end + 1} = candidate; %#ok<AGROW>
    end
end

function setSupportedCallbacks(components, supported)
    for componentIndex = 1:numel(components)
        component = components(componentIndex);
        propertyNames = properties(component);
        callbackNames = propertyNames(endsWith(propertyNames, 'Fcn'));
        for callbackIndex = 1:numel(callbackNames)
            propertyName = callbackNames{callbackIndex};
            try
                callbackValue = component.(propertyName);
                methodName = callbackMethodName(callbackValue, supported);
                if isempty(methodName)
                    component.(propertyName) = [];
                else
                    component.(propertyName) = methodName;
                end
            catch
            end
        end
    end
end

function methodName = callbackMethodName(callbackValue, supported)
    methodName = '';
    callbackText = '';
    if ischar(callbackValue) || (isstring(callbackValue) && isscalar(callbackValue))
        callbackText = char(callbackValue);
    elseif isa(callbackValue, 'function_handle')
        callbackText = func2str(callbackValue);

        % createCallbackFcn wraps the original method handle in an
        % anonymous function.  func2str() therefore reports only the
        % wrapper.  Recover the real callback from the captured workspace.
        try
            details = functions(callbackValue);
            if isfield(details, 'workspace') && ~isempty(details.workspace)
                for workspaceIndex = 1:numel(details.workspace)
                    workspaceData = details.workspace{workspaceIndex};
                    if isstruct(workspaceData) && ...
                            isfield(workspaceData, 'callback') && ...
                            isa(workspaceData.callback, 'function_handle')
                        callbackText = func2str(workspaceData.callback);
                        break;
                    end
                end
            end
        catch
        end
    else
        return;
    end

    callbackText = erase(callbackText, '@');
    for index = 1:numel(supported)
        if strcmp(callbackText, supported{index}) || ...
                contains(callbackText, ['app.', supported{index}, '('])
            methodName = supported{index};
            return;
        end
    end
end

function names = registeredCallbackNames()
    % Only callbacks directly associated with components belong in the
    % App Designer callback collection.  Other callable methods are kept
    % in the editable helper-method section.
    names = {'NavigationListValueChanged', ...
        'PageHostPanelSizeChanged', ...
        'LoadParameterButtonPushed', 'OpenButtonPushed', ...
        'ImportButtonPushed', ...
        'HelpButtonPushed', 'ResetParameterButtonPushed', ...
        'LayoutBoundaryValueChanged', ...
        'LayoutTemplateDropDownValueChanged', ...
        'AllocationModeDropDownValueChanged', ...
        'SimulationScheduleWindowValueChanged', ...
        'ApplyLayoutButtonPushed', 'ImportLayoutButtonPushed', ...
        'DeleteLayoutButtonPushed', ...
        'StartOptimizationButtonPushed', ...
        'EvaluateButtonPushed', 'MetricsEvaluateButtonPushed', ...
        'GenerateAllocationButtonPushed', 'RunSimulationButtonPushed'};
end

function [callbacks, startupCallback, editableSectionCode] = ...
        buildCodeMetadata(sourceFile, callbackNames)
    sourceText = fileread(sourceFile);
    sourceLines = regexp(sourceText, '\r\n|\n|\r', 'split');
    functionsInSource = parseTopLevelFunctions(sourceText, sourceLines);

    callbacks = struct('Name', {}, 'Code', {});
    for index = 1:numel(callbackNames)
        entry = findFunction(functionsInSource, callbackNames{index});
        callbacks(index).Name = callbackNames{index}; %#ok<AGROW>
        callbacks(index).Code = entry.Body; %#ok<AGROW>
    end

    startupEntry = findFunction(functionsInSource, 'startupFcn');
    startupCallback = struct( ...
        'Name', 'startupFcn', ...
        'Code', {startupEntry.Body});

    propertyToken = regexp(sourceText, ...
        ['(?s)\n(    properties \(Access = private\).*?', ...
         '\n    end)\n'], 'tokens', 'once');
    if isempty(propertyToken)
        error('RCDesigner:MissingPrivateProperties', ...
            '无法定位 App 的私有属性块。');
    end
    propertyLines = regexp(propertyToken{1}, '\r\n|\n|\r', 'split');

    generatedFunctions = [callbackNames, ...
        {'startupFcn', 'createComponents', 'RCDesignerPrototype', 'delete'}];
    helperMask = ~ismember({functionsInSource.Name}, generatedFunctions);
    helpers = functionsInSource(helperMask);

    editableSectionCode = {'    '};
    editableSectionCode = [editableSectionCode, propertyLines, {'    '}]; %#ok<AGROW>
    if ~isempty(helpers)
        editableSectionCode{end + 1} = '    methods (Access = private)';
        editableSectionCode{end + 1} = '';
        for index = 1:numel(helpers)
            editableSectionCode = [editableSectionCode, helpers(index).Full]; %#ok<AGROW>
            editableSectionCode{end + 1} = '';
        end
        editableSectionCode{end + 1} = '    end';
        editableSectionCode{end + 1} = '    ';
    end
end

function entries = parseTopLevelFunctions(sourceText, sourceLines)
    syntaxTree = mtree(sourceText);
    if syntaxTree.count == 1 && strcmp(syntaxTree.kind(), 'ERR')
        error('RCDesigner:InvalidSource', ...
            'RCDesignerPrototype.m 存在语法错误，无法生成 MLAPP。');
    end

    functionNodes = mtfind(syntaxTree, 'Kind', 'FUNCTION');
    nodeIndices = indices(functionNodes);
    entries = struct('Name', {}, 'StartLine', {}, 'EndLine', {}, ...
        'Body', {}, 'Full', {});
    for nodeIndex = nodeIndices
        functionNode = functionNodes.select(nodeIndex);
        parentNode = functionNode.trueparent;
        if isempty(parentNode) || ~strcmp(kind(parentNode), 'METHODS')
            continue;
        end

        startLine = functionNode.lineno;
        [endLine, ~] = functionNode.lastone;
        name = char(functionNode.Fname.string);
        if startLine < 1 || endLine > numel(sourceLines) || endLine <= startLine
            error('RCDesigner:FunctionParse', ...
                '无法解析函数 %s 的代码范围。', name);
        end

        entry.Name = name;
        entry.StartLine = startLine;
        entry.EndLine = endLine;
        entry.Body = sourceLines(startLine + 1:endLine - 1);
        entry.Full = sourceLines(startLine:endLine);
        entries(end + 1) = entry; %#ok<AGROW>
    end

    if ~isempty(entries)
        [~, order] = sort([entries.StartLine]);
        entries = entries(order);
    end
end

function entry = findFunction(entries, name)
    index = find(strcmp({entries.Name}, name), 1, 'first');
    if isempty(index)
        error('RCDesigner:MissingFunction', ...
            '生成 MLAPP 时找不到函数：%s。', name);
    end
    entry = entries(index);
end

function deleteIfValid(component)
    try
        if isvalid(component)
            delete(component);
        end
    catch
    end
end
