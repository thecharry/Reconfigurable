function Run_RCDesigner
%RUN_RCDESIGNER Start the packaged RC-Designer application reliably.

    rootDir = fileparts(mfilename('fullpath'));
    sourceDir = fullfile(rootDir, 'source');
    sourceFile = fullfile(sourceDir, 'RCDesignerPrototype.m');
    appFile = fullfile(rootDir, 'app', 'RCDesignerPrototype.mlapp');

    configureFunctionPriority();

    if ~isfile(sourceFile) || ~isfile(appFile)
        error('RCDesigner:MissingFiles', ...
            '软件文件不完整。请确认 source 和 app 文件夹均未被移动。');
    end

    % Treat the App Designer file as the editable master. Whenever it has
    % been saved, copy its current MATLAB code into the runtime source so
    % command-line launches use the same version the user edited.
    mlappIsFunctional = syncSourceFromMlapp(appFile, sourceFile);
    if ~mlappIsFunctional
        warning('RCDesigner:RecoveredFromDesignOnlyMlapp', ...
            ['检测到 MLAPP 只包含界面代码，已保留完整运行源码。', ...
             '请重新运行 build_rcdesigner_mlapp 修复 App Designer 文件。']);
    end

    % Keep the class source available for the lifetime of the app. Running
    % an unpacked MLAPP with run() can remove its temporary class path too
    % early, which causes warnings when the app window is later closed.
    if contains([path, pathsep], [sourceDir, pathsep])
        rmpath(sourceDir);
    end
    addpath(sourceDir, '-begin');
    clear RCDesignerPrototype
    rehash;
    RCDesignerPrototype;
end

function isFunctional = syncSourceFromMlapp(appFile, sourceFile)
    extractDir = tempname;
    mkdir(extractDir);
    cleanup = onCleanup(@()removeFolder(extractDir)); %#ok<NASGU>
    unzip(appFile, extractDir);
    documentFile = fullfile(extractDir, 'matlab', 'document.xml');
    documentText = fileread(documentFile);
    match = regexp(documentText, ...
        '(?s)<w:t><!\[CDATA\[(.*)\]\]></w:t>', 'tokens', 'once');
    if isempty(match)
        error('RCDesigner:InvalidMlappCode', ...
            '无法从 MLAPP 中读取 MATLAB 代码。');
    end
    mlappCode = match{1};
    requiredMarkers = {'function startupFcn(app)', ...
        'function wireCallbacks(app)', ...
        'function StartOptimizationButtonPushed(app, event)', ...
        'runStartupFcn(app, @startupFcn)'};
    isFunctional = all(cellfun(@(marker)contains(mlappCode, marker), requiredMarkers));
    if ~isFunctional
        return;
    end
    if strcmp(fileread(sourceFile), mlappCode)
        return;
    end
    fileId = fopen(sourceFile, 'w', 'n', 'UTF-8');
    if fileId < 0
        error('RCDesigner:SourceWrite', '无法同步运行源码。');
    end
    fileCleanup = onCleanup(@()fclose(fileId)); %#ok<NASGU>
    fwrite(fileId, mlappCode, 'char');
end

function removeFolder(folderPath)
    try
        if isfolder(folderPath)
            rmdir(folderPath, 's');
        end
    catch
    end
end

function configureFunctionPriority
% Keep the CORA/GPTIPS toolbox available, but let MATLAB built-ins win when
% function names conflict (notably extract, used by App Designer).
    gptipsPath = '/Users/charry/CORA/global/thirdparty/gptips2';
    if isfolder(gptipsPath) && contains([path, pathsep], [gptipsPath, pathsep])
        rmpath(gptipsPath);
        addpath(gptipsPath, '-end');
        rehash;
    end
end
