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

    save(datasetMat, 'results', 'summaryTbl', 'traceStore', 'learningHistory', 'model', 'runStatus');
    writetable(summaryTbl, datasetCsv);
    if ~isempty(traceStore)
        writetable(traceStore, traceCsv);
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


