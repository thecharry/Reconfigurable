function sync_rcdesigner_source_from_model
%SYNC_RCDESIGNER_SOURCE_FROM_MODEL Align source with the MLAPP design model.

    rootDir = fileparts(mfilename('fullpath'));
    appFile = fullfile(rootDir, 'app', 'RCDesignerPrototype.mlapp');
    sourceFile = fullfile(rootDir, 'source', 'RCDesignerPrototype.m');
    extractDir = tempname;
    mkdir(extractDir);
    cleanupDir = onCleanup(@()removeFolder(extractDir)); %#ok<NASGU>
    unzip(appFile, extractDir);

    modelData = load(fullfile(extractDir, 'appdesigner', 'appModel.mat'));
    uiFigure = modelData.components.UIFigure;
    cleanupFigure = onCleanup(@()deleteIfValid(uiFigure)); %#ok<NASGU>
    components = designPreorder(uiFigure);

    publicBlock = createPublicPropertiesBlock(components);
    componentBlock = createComponentsBlock(components);

    sourceText = fileread(sourceFile);
    publicPattern = ['(?s)    properties \(Access = public\).*?', ...
        '    end\s*\n    properties \(Access = private\)'];
    publicReplacement = [publicBlock, newline, newline, ...
        '    properties (Access = private)'];
    originalText = sourceText;
    sourceText = regexprep(sourceText, publicPattern, ...
        publicReplacement, 'once');
    if strcmp(sourceText, originalText)
        error('RCDesigner:PropertySync', '无法定位公共组件属性块。');
    end

    componentPattern = ['(?s)        function createComponents\(app\).*?', ...
        '        end\s*\n    end\s*\n    % App creation and deletion\s*\n', ...
        '    methods \(Access = public\)'];
    componentReplacement = [componentBlock, newline, ...
        '    end', newline, newline, ...
        '    % App creation and deletion', newline, ...
        '    methods (Access = public)'];
    beforeComponentSync = sourceText;
    sourceText = regexprep(sourceText, componentPattern, ...
        componentReplacement, 'once');
    if strcmp(sourceText, beforeComponentSync)
        error('RCDesigner:ComponentSync', '无法定位 createComponents 方法。');
    end

    fileId = fopen(sourceFile, 'w', 'n', 'UTF-8');
    if fileId < 0
        error('RCDesigner:SourceWrite', '无法写入源码：%s', sourceFile);
    end
    cleanupFile = onCleanup(@()fclose(fileId)); %#ok<NASGU>
    fwrite(fileId, sourceText, 'char');
    fprintf('SOURCE_SYNCED=%s\n', sourceFile);
    fprintf('DESIGN_COMPONENTS=%d\n', numel(components));
end

function components = designPreorder(component)
    components = {component};
    if ~isprop(component, 'Children')
        return;
    end
    children = component.Children;
    children = flip(children);
    for index = 1:numel(children)
        if isprop(children(index), 'DesignTimeProperties')
            descendants = designPreorder(children(index));
            components = [components, descendants]; %#ok<AGROW>
        end
    end
end

function block = createPublicPropertiesBlock(components)
    lines = {'    % Properties that correspond to App Designer components'; ...
        '    properties (Access = public)'};
    for index = 1:numel(components)
        component = components{index};
        codeName = component.DesignTimeProperties.CodeName;
        lines{end + 1} = sprintf('        %-32s %s', ...
            codeName, class(component)); %#ok<AGROW>
    end
    lines{end + 1} = '    end';
    block = strjoin(lines, newline);
end

function block = createComponentsBlock(components)
    lines = {'        % Create UIFigure and components'; ...
        '        function createComponents(app)'; ...
        ''};

    for index = 1:numel(components)
        component = components{index};
        designData = component.DesignTimeProperties;
        codeName = designData.CodeName;
        parentCodeName = '';
        if ~isempty(component.Parent) && ...
                isprop(component.Parent, 'DesignTimeProperties')
            parentCodeName = component.Parent.DesignTimeProperties.CodeName;
        end

        lines{end + 1} = sprintf('            %% Create %s', codeName); %#ok<AGROW>
        componentCode = designData.ComponentCode;
        for codeIndex = 1:numel(componentCode)
            codeLine = componentCode{codeIndex};
            codeLine = strrep(codeLine, 'ad_CODENAME_ad', codeName);
            codeLine = strrep(codeLine, 'ad_PARENTCODENAME_ad', parentCodeName);
            codeLine = strrep(codeLine, 'ad_OBJECTNAME_ad', 'app');
            codeLine = convertCallbackLine(codeLine);
            lines{end + 1} = ['            ', codeLine]; %#ok<AGROW>
        end
        lines{end + 1} = ''; %#ok<AGROW>
    end

    lines{end + 1} = '            % Stabilize the hidden main tab group before the first frame.';
    lines{end + 1} = '            app.resizeMainTabGroup();';
    lines{end + 1} = '';
    lines{end + 1} = '            % Show the figure after all components are created';
    lines{end + 1} = '            app.UIFigure.Visible = ''on'';';
    lines{end + 1} = '        end';
    block = strjoin(lines, newline);
end

function codeLine = convertCallbackLine(codeLine)
    expression = ['app\.(\w+)\.(\w+Fcn) = ', ...
        '''([A-Za-z]\w*)'';'];
    replacement = 'app.$1.$2 = createCallbackFcn(app, @$3, true);';
    codeLine = regexprep(codeLine, expression, replacement);
end

function deleteIfValid(component)
    try
        if isvalid(component)
            delete(component);
        end
    catch
    end
end

function removeFolder(folderPath)
    try
        if isfolder(folderPath)
            rmdir(folderPath, 's');
        end
    catch
    end
end
