function autosimSaveScenarioPerformanceReport(summaryTbl, traceStore, perfCsvPath, decisionCsvPath, perfPngPath)
    if isempty(summaryTbl) || ~ismember('scenario_id', summaryTbl.Properties.VariableNames)
        return;
    end

    dTbl = autosimBuildDecisionTable(summaryTbl);
    n = height(dTbl);
    if n == 0
        return;
    end

    perfTbl = dTbl;
    perfTbl.cum_accuracy = nan(n, 1);
    perfTbl.cum_precision = nan(n, 1);
    perfTbl.cum_recall = nan(n, 1);
    perfTbl.cum_unsafe_landing_rate = nan(n, 1);
    perfTbl.cum_f1 = nan(n, 1);

    for i = 1:n
        de = autosimEvaluateDecisionMetrics(dTbl(1:i, :));
        perfTbl.cum_accuracy(i) = de.accuracy;
        perfTbl.cum_precision(i) = de.precision;
        perfTbl.cum_recall(i) = de.recall;
        perfTbl.cum_unsafe_landing_rate(i) = de.unsafe_landing_rate;
        perfTbl.cum_f1(i) = de.f1;
    end

    % Keep per-scenario confidence summary when trace data is available.
    perfTbl.mean_confidence = nan(n,1);
    if ~isempty(traceStore) && ismember('scenario_id', traceStore.Properties.VariableNames) && ismember('pred_stable_prob', traceStore.Properties.VariableNames)
        for i = 1:n
            sid = perfTbl.scenario_id(i);
            v = traceStore.pred_stable_prob(traceStore.scenario_id == sid);
            v = v(isfinite(v));
            if ~isempty(v)
                perfTbl.mean_confidence(i) = mean(v);
            end
        end
    end

    writetable(perfTbl, perfCsvPath);

    dOverall = autosimEvaluateDecisionMetrics(dTbl);
    dOverallTbl = struct2table(dOverall);
    writetable(dOverallTbl, decisionCsvPath);

    fig = figure('Name', 'AutoSim Decision Performance', 'NumberTitle', 'off');
    tl = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax1 = nexttile(tl, 1);
    plot(ax1, perfTbl.scenario_id, perfTbl.cum_accuracy, '-', 'LineWidth', 1.6, 'Color', [0.00 0.45 0.74]);
    hold(ax1, 'on');
    plot(ax1, perfTbl.scenario_id, perfTbl.cum_precision, '-', 'LineWidth', 1.4, 'Color', [0.20 0.60 0.20]);
    plot(ax1, perfTbl.scenario_id, perfTbl.cum_recall, '-', 'LineWidth', 1.4, 'Color', [0.85 0.33 0.10]);
    plot(ax1, perfTbl.scenario_id, perfTbl.cum_unsafe_landing_rate, '-', 'LineWidth', 1.6, 'Color', [0.75 0.20 0.20]);
    ylim(ax1, [0 1]);
    xlabel(ax1, 'scenario');
    ylabel(ax1, 'score');
    title(ax1, 'Cumulative Decision Metrics');
    legend(ax1, {'accuracy', 'precision', 'safe recall', 'unsafe landing rate'}, 'Location', 'best');
    grid(ax1, 'on');

    ax2 = nexttile(tl, 2);
    metricVals = [dOverall.accuracy, dOverall.precision, dOverall.recall, dOverall.specificity, dOverall.unsafe_landing_rate];
    b = bar(ax2, metricVals, 0.70);
    b.FaceColor = 'flat';
    b.CData = [ ...
        0.00 0.45 0.74; ...
        0.20 0.60 0.20; ...
        0.85 0.33 0.10; ...
        0.49 0.18 0.56; ...
        0.75 0.20 0.20 ...
    ];
    ylim(ax2, [0 1]);
    xticks(ax2, 1:5);
    xticklabels(ax2, {'Accuracy','Precision','SafeRecall','UnsafeReject','UnsafeLand'});
    ylabel(ax2, 'score');
    title(ax2, sprintf('Overall Decision Metrics (valid=%d, unsafe=%d)', dOverall.n_valid, dOverall.n_unsafe));
    grid(ax2, 'on');

    exportgraphics(fig, perfPngPath, 'Resolution', 150);
end


