function finalInfo = autosimFinalize(cfg, results, traceStore, learningHistory, model, plotState, runStatus)
    ts = autosimTimestamp();
    tag = lower(char(runStatus));

    summaryTbl = autosimSummaryTable(results);

    datasetMat = fullfile(cfg.paths.data_dir, sprintf('autosim_dataset_%s_%s.mat', ts, tag));
    datasetCsv = fullfile(cfg.paths.data_dir, sprintf('autosim_dataset_%s_%s.csv', ts, tag));
    traceCsv = fullfile(cfg.paths.data_dir, sprintf('autosim_trace_%s_%s.csv', ts, tag));
    historyCsv = fullfile(cfg.paths.data_dir, sprintf('autosim_learning_%s_%s.csv', ts, tag));
    modelPath = fullfile(cfg.paths.model_dir, sprintf('autosim_model_final_%s_%s.mat', ts, tag));
    plotPng = fullfile(cfg.paths.plot_dir, sprintf('autosim_result_%s_%s.png', ts, tag));
    perfCsv = fullfile(cfg.paths.data_dir, sprintf('autosim_performance_%s_%s.csv', ts, tag));
    decisionCsv = fullfile(cfg.paths.data_dir, sprintf('autosim_decision_metrics_%s_%s.csv', ts, tag));
    perfPng = fullfile(cfg.paths.plot_dir, sprintf('autosim_performance_%s_%s.png', ts, tag));
    gtPredPng = fullfile(cfg.paths.plot_dir, sprintf('autosim_gt_vs_pred_%s_%s.png', ts, tag));
    finalDatasetRoot = fullfile(cfg.paths.root, 'FinalDataset');
    finalDatasetDir = fullfile(finalDatasetRoot, char(cfg.paths.run_id));
    finalDatasetCsv = fullfile(finalDatasetDir, sprintf('autosim_final_dataset_%s_%s.csv', ts, tag));

    save(datasetMat, 'results', 'summaryTbl', 'traceStore', 'learningHistory', 'model', 'runStatus');
    writetable(summaryTbl, datasetCsv);
    if ~isempty(traceStore)
        writetable(traceStore, traceCsv);
    end

    try
        if ~exist(finalDatasetDir, 'dir')
            mkdir(finalDatasetDir);
        end
        finalTbl = autosimBuildFinalDatasetTable(summaryTbl);
        writetable(finalTbl, finalDatasetCsv);
    catch ME
        warning('[AUTOSIM] FinalDataset export failed: %s', ME.message);
    end
    if ~isempty(learningHistory)
        writetable(learningHistory, historyCsv);
    end
    save(modelPath, 'model');

    try
        if ~isempty(plotState) && isfield(plotState, 'fig') && isgraphics(plotState.fig)
            exportgraphics(plotState.fig, plotPng, 'Resolution', 150);
        end
    catch ME
        warning('[AUTOSIM] Plot save failed: %s', ME.message);
    end

    try
        autosimSaveScenarioPerformanceReport(summaryTbl, traceStore, perfCsv, decisionCsv, perfPng);
    catch ME
        warning('[AUTOSIM] Performance report save failed: %s', ME.message);
    end

    try
        autosimPlotGtVsPrediction(summaryTbl, traceStore, model, cfg, gtPredPng);
    catch ME
        warning('[AUTOSIM] GT-vs-pred plot failed: %s', ME.message);
    end

    valid = false(height(summaryTbl), 1);
    if ismember('label', summaryTbl.Properties.VariableNames)
        valid = (summaryTbl.label == "stable") | (summaryTbl.label == "unstable");
    end
    nValid = sum(valid);
    nStable = 0;
    stableRatio = 0.0;
    if nValid > 0
        nStable = sum(summaryTbl.label(valid) == "stable");
        stableRatio = nStable / nValid;
    end

    fprintf('[AUTOSIM] Saved dataset: %s\n', datasetCsv);
    fprintf('[AUTOSIM] Saved trace:   %s\n', traceCsv);
    fprintf('[AUTOSIM] Saved model:   %s\n', modelPath);
    fprintf('[AUTOSIM] Saved plot:    %s\n', plotPng);
    fprintf('[AUTOSIM] Saved perf:    %s\n', perfCsv);
    fprintf('[AUTOSIM] Saved dmetric: %s\n', decisionCsv);
    fprintf('[AUTOSIM] Saved perfpng: %s\n', perfPng);
    fprintf('[AUTOSIM] Saved gtpred:  %s\n', gtPredPng);
    fprintf('[AUTOSIM] Saved final:   %s\n', finalDatasetCsv);

    dEval = autosimEvaluateDecisionMetrics(summaryTbl);
    if dEval.n_valid > 0
        fprintf('[AUTOSIM] Decision metrics | Acc=%.3f Prec=%.3f SafeRec=%.3f UnsafeReject=%.3f UnsafeLand=%.3f F1=%.3f (excluded: intervention=%d, hover=%d)\n', ...
            dEval.accuracy, dEval.precision, dEval.recall, dEval.specificity, dEval.unsafe_landing_rate, dEval.f1, ...
            dEval.n_excluded_intervention, dEval.n_excluded_hover);
    end

    finalInfo = struct();
    finalInfo.hasValidLabel = (nValid > 0);
    finalInfo.nValid = nValid;
    finalInfo.nStable = nStable;
    finalInfo.stableRatio = stableRatio;
end

function finalTbl = autosimBuildFinalDatasetTable(summaryTbl)
    if isempty(summaryTbl)
        finalTbl = table();
        return;
    end

    wanted = { ...
        'scenario_id', ...
        'mean_wind_speed','max_wind_speed','wind_velocity_x','wind_velocity_y','wind_velocity', ...
        'wind_acceleration_x','wind_acceleration_y','wind_acceleration', ...
        'mean_abs_roll_deg','mean_abs_pitch_deg','mean_abs_vz','max_abs_vz', ...
        'mean_tag_error','max_tag_error','stability_std_z','stability_std_vz', ...
        'mean_imu_ang_vel','max_imu_ang_vel','mean_imu_lin_acc','max_imu_lin_acc', ...
        'wind_speed_cmd','wind_dir_cmd','hover_height_cmd', ...
        'gt_safe_to_land','label','success', ...
        'wind_risk_enc','alignment_enc','visual_enc', ...
        'onto_wind_condition','onto_gust','onto_temporal_pattern','onto_drone_state','onto_tag_observation' ...
    };

    cols = intersect(wanted, summaryTbl.Properties.VariableNames, 'stable');
    finalTbl = summaryTbl(:, cols);
end


