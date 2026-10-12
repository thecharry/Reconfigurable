function verify_persistent_ui_run_20260911
% 调用实际应用按钮，运行两轮真实求解并检查自动续跑和重新打开。
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'app'),'-begin');
    app=RCDesignerPrototype;
    cleanup=onCleanup(@()delete(app));
    app.TabGroup.SelectedTab=app.OptimizationTab;
    app.NavigationList.Value='优化设计';
    app.GenerationField_3.Value=30;
    snapshots=cell(1,2);
    for j=1:2
        if strcmp(app.StartOptimizationButton_3.Enable,'off'), break; end
        callback=app.StartOptimizationButton_3.ButtonPushedFcn;
        callback(app.StartOptimizationButton_3,[]);
        drawnow;
        latest=jsondecode(fileread(fullfile(root,'Optim_diagnosis_latest.json')));
        report=jsondecode(fileread(latest.diagnostic_report_file));
        assert(isfield(report,'branch_progress'),'缺少持久化分支统计。');
        snapshots{j}=struct('status',report.status,'progress',report.branch_progress, ...
            'report_file',latest.diagnostic_report_file,'ui_status',app.StatusLabel.Text, ...
            'button_text',app.StartOptimizationButton_3.Text,'button_enabled',char(app.StartOptimizationButton_3.Enable));
        if j==2
            assert(snapshots{2}.progress.solver_calls>snapshots{1}.progress.solver_calls,'第二轮没有接续累计求解计数。');
        end
    end
    exportapp(app.UIFigure,fullfile(root,'analysis','persistent_ui_after_resume_20260911.png'));
    clear cleanup;
    clear app;
    app=RCDesignerPrototype;cleanup=onCleanup(@()delete(app));
    assert(contains(app.StatusLabel.Text,'已载入上次搜索记录'),'重新打开未恢复搜索记录。');
    fid=fopen(fullfile(root,'analysis','persistent_ui_run_20260911.json'),'w','n','UTF-8');
    fileCleanup=onCleanup(@()fclose(fid));fprintf(fid,'%s',jsonencode(snapshots,PrettyPrint=true));
    disp(snapshots);
end
