function out = AutoSimPaperPlots(runDir, outputDir)
% AutoSimPaperPlots
% Build paper-ready figures/tables from AutoSim final CSV outputs.
%
% Usage:
%   AutoSimPaperPlots
%   AutoSimPaperPlots('/home/j/INCSL/IICC26_ws/src/sjtu_drone-ros2/matlab/data/20260312_195110')
%   AutoSimPaperPlots(runDir, '/home/j/INCSL/IICC26_ws/src/sjtu_drone-ros2/matlab/plots/paper_custom')

    close all;

    rootDir = fileparts(mfilename('fullpath'));
    dataRoot = fullfile(rootDir, 'data');
    plotRoot = fullfile(rootDir, 'plots');

    if nargin < 1 || strlength(string(runDir)) == 0
        runDir = findLatestRunDir(dataRoot);
    end
    runDir = char(string(runDir));

    if nargin < 2 || strlength(string(outputDir)) == 0
        [~, runName] = fileparts(runDir);
        outputDir = fullfile(plotRoot, ['paper_' runName '_' datestr(now, 'yyyymmdd_HHMMSS')]);
    end
    outputDir = char(string(outputDir));
    ensureDir(outputDir);

    datasetPath = pickFile(runDir, {'autosim_dataset_latest.csv', 'autosim_dataset_*_completed.csv'});
    tracePath = pickFile(runDir, {'autosim_trace_latest.csv', 'autosim_trace_*_completed.csv'});
    perfPath = pickFile(runDir, {'autosim_performance_*_completed.csv'});
    dmetPath = pickFile(runDir, {'autosim_decision_metrics_*_completed.csv'});

    if isempty(datasetPath)
        error('AutoSimPaperPlots:NoDataset', 'Dataset CSV not found in %s', runDir);
    end

    datasetTbl = readtable(datasetPath);
    traceTbl = table();
    perfTblRaw = table();
    dmetTblRaw = table();

    if ~isempty(tracePath)
        traceTbl = readtable(tracePath);
    end
    if ~isempty(perfPath)
        perfTblRaw = readtable(perfPath);
    end
    if ~isempty(dmetPath)
        dmetTblRaw = readtable(dmetPath);
    end

    gtSafe = buildGtSafe(datasetTbl);
    predProposed = buildDecision(datasetTbl, 'pred_decision', 'landing_cmd_time');
    predExecuted = buildDecision(datasetTbl, 'executed_action', 'landing_cmd_time');

    baseline = buildThresholdBaseline(datasetTbl);
    predBaseline = baseline.predLand;

    mProposed = evalDecision(gtSafe, predProposed);
    mExecuted = evalDecision(gtSafe, predExecuted);
    mBaseline = evalDecision(gtSafe, predBaseline);

    cmpTbl = table( ...
        string({'Ontology+AI (policy)'; 'Ontology+AI (executed)'; 'Threshold baseline'}), ...
        [mProposed.nValid; mExecuted.nValid; mBaseline.nValid], ...
        [mProposed.accuracy; mExecuted.accuracy; mBaseline.accuracy], ...
        [mProposed.precision; mExecuted.precision; mBaseline.precision], ...
        [mProposed.recall; mExecuted.recall; mBaseline.recall], ...
        [mProposed.specificity; mExecuted.specificity; mBaseline.specificity], ...
        [mProposed.f1; mExecuted.f1; mBaseline.f1], ...
        [mProposed.unsafeLandingRate; mExecuted.unsafeLandingRate; mBaseline.unsafeLandingRate], ...
        [mProposed.tp; mExecuted.tp; mBaseline.tp], ...
        [mProposed.fp; mExecuted.fp; mBaseline.fp], ...
        [mProposed.fn; mExecuted.fn; mBaseline.fn], ...
        [mProposed.tn; mExecuted.tn; mBaseline.tn], ...
        'VariableNames', {'method','n_valid','accuracy','precision','safe_recall','unsafe_reject','f1','unsafe_landing_rate','TP','FP','FN','TN'});

    if ~isempty(dmetTblRaw)
        dmetTblRaw.source = repmat("autosim_saved", height(dmetTblRaw), 1);
    end

    writetable(cmpTbl, fullfile(outputDir, 'paper_table_method_comparison.csv'));
    writetable(struct2table(baseline.thresholds), fullfile(outputDir, 'paper_table_thresholds.csv'));

    fig1 = figure('Name', 'MethodComparison', 'Color', 'w', 'Position', [100 100 1180 460]);
    tl = tiledlayout(fig1, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax1 = nexttile(tl, 1);
    barVals = [mProposed.accuracy, mProposed.f1, mProposed.unsafeLandingRate; ...
               mBaseline.accuracy, mBaseline.f1, mBaseline.unsafeLandingRate; ...
               mExecuted.accuracy, mExecuted.f1, mExecuted.unsafeLandingRate];
    bh = bar(ax1, barVals, 0.88);
    bh(1).FaceColor = [0.10 0.45 0.78];
    bh(2).FaceColor = [0.13 0.60 0.33];
    bh(3).FaceColor = [0.78 0.22 0.22];
    xticks(ax1, 1:3);
    xticklabels(ax1, {'Ontology+AI (policy)', 'Threshold', 'Ontology+AI (executed)'});
    ylim(ax1, [0 1]);
    ylabel(ax1, 'score');
    title(ax1, 'Overall Metrics Comparison');
    legend(ax1, {'Accuracy', 'F1', 'Unsafe landing rate'}, 'Location', 'northoutside', 'Orientation', 'horizontal');
    grid(ax1, 'on');

    ax2 = nexttile(tl, 2);
    confVals = [mProposed.tp mProposed.fp mProposed.fn mProposed.tn; ...
                mBaseline.tp mBaseline.fp mBaseline.fn mBaseline.tn; ...
                mExecuted.tp mExecuted.fp mExecuted.fn mExecuted.tn];
    b2 = bar(ax2, confVals, 'stacked', 'BarWidth', 0.8);
    b2(1).FaceColor = [0.20 0.65 0.25];
    b2(2).FaceColor = [0.85 0.30 0.20];
    b2(3).FaceColor = [0.95 0.65 0.15];
    b2(4).FaceColor = [0.25 0.50 0.90];
    xticks(ax2, 1:3);
    xticklabels(ax2, {'Ontology+AI (policy)', 'Threshold', 'Ontology+AI (executed)'});
    ylabel(ax2, 'scenario count');
    title(ax2, 'Decision Outcome Composition');
    legend(ax2, {'TP','FP','FN','TN'}, 'Location', 'northoutside', 'Orientation', 'horizontal');
    grid(ax2, 'on');

    exportgraphics(fig1, fullfile(outputDir, 'paper_fig1_method_comparison.png'), 'Resolution', 220);

    n = height(datasetTbl);
    sid = getScenarioId(datasetTbl);
    if isempty(sid)
        sid = (1:n)';
    end
    trendP = cumulativeTrend(gtSafe, predProposed);
    trendB = cumulativeTrend(gtSafe, predBaseline);
    trendE = cumulativeTrend(gtSafe, predExecuted);

    fig2 = figure('Name', 'CumulativeTrends', 'Color', 'w', 'Position', [120 120 1180 520]);
    tl2 = tiledlayout(fig2, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax21 = nexttile(tl2, 1);
    plot(ax21, sid, smoothAdaptive(trendP.accuracy), '-', 'LineWidth', 2.0, 'Color', [0.10 0.45 0.78]);
    hold(ax21, 'on');
    plot(ax21, sid, smoothAdaptive(trendB.accuracy), '-', 'LineWidth', 1.8, 'Color', [0.30 0.30 0.30]);
    plot(ax21, sid, smoothAdaptive(trendE.accuracy), '--', 'LineWidth', 1.8, 'Color', [0.13 0.60 0.33]);
    ylim(ax21, [0 1]);
    ylabel(ax21, 'accuracy');
    title(ax21, 'Cumulative Accuracy Trend');
    legend(ax21, {'Ontology+AI (policy)','Threshold','Ontology+AI (executed)'}, 'Location', 'southoutside', 'Orientation', 'horizontal');
    grid(ax21, 'on');

    ax22 = nexttile(tl2, 2);
    plot(ax22, sid, smoothAdaptive(trendP.unsafeLandingRate), '-', 'LineWidth', 2.0, 'Color', [0.78 0.22 0.22]);
    hold(ax22, 'on');
    plot(ax22, sid, smoothAdaptive(trendB.unsafeLandingRate), '-', 'LineWidth', 1.8, 'Color', [0.35 0.35 0.35]);
    plot(ax22, sid, smoothAdaptive(trendE.unsafeLandingRate), '--', 'LineWidth', 1.8, 'Color', [0.55 0.10 0.65]);
    ylim(ax22, [0 1]);
    xlabel(ax22, 'scenario');
    ylabel(ax22, 'unsafe landing rate');
    title(ax22, 'Cumulative Unsafe Landing Rate');
    legend(ax22, {'Ontology+AI (policy)','Threshold','Ontology+AI (executed)'}, 'Location', 'southoutside', 'Orientation', 'horizontal');
    grid(ax22, 'on');

    exportgraphics(fig2, fullfile(outputDir, 'paper_fig2_cumulative_trends.png'), 'Resolution', 220);

    fig3 = figure('Name', 'RiskMap', 'Color', 'w', 'Position', [140 140 1180 500]);
    ax3 = axes(fig3);
    [xRisk, yRisk] = pickRiskAxes(datasetTbl);
    cls = classifyOutcome(gtSafe, predProposed);

    drawClass(ax3, xRisk, yRisk, cls == "TP", [0.15 0.65 0.20], 'TP');
    drawClass(ax3, xRisk, yRisk, cls == "FP", [0.85 0.25 0.20], 'FP');
    drawClass(ax3, xRisk, yRisk, cls == "FN", [0.95 0.70 0.15], 'FN');
    drawClass(ax3, xRisk, yRisk, cls == "TN", [0.20 0.45 0.90], 'TN');
    xlabel(ax3, 'wind severity (selected feature)');
    ylabel(ax3, 'visual/alignment severity (selected feature)');
    title(ax3, 'Scenario Risk Map (Ontology+AI policy decision)');
    grid(ax3, 'on');
    legend(ax3, 'Location', 'eastoutside');

    if isfield(baseline.thresholds, 'wind_threshold')
        xline(ax3, baseline.thresholds.wind_threshold, '--', 'Threshold wind', 'Color', [0.35 0.35 0.35], 'LineWidth', 1.1);
    end
    if isfield(baseline.thresholds, 'tag_error_threshold')
        yline(ax3, baseline.thresholds.tag_error_threshold, '--', 'Threshold visual', 'Color', [0.35 0.35 0.35], 'LineWidth', 1.1);
    end

    exportgraphics(fig3, fullfile(outputDir, 'paper_fig3_risk_map.png'), 'Resolution', 220);

    fig4 = figure('Name', 'ConfusionMatrices', 'Color', 'w', 'Position', [160 160 980 440]);
    tl4 = tiledlayout(fig4, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    plotConfusion(nexttile(tl4, 1), mProposed, 'Ontology+AI (policy)');
    plotConfusion(nexttile(tl4, 2), mBaseline, 'Threshold baseline');
    exportgraphics(fig4, fullfile(outputDir, 'paper_fig4_confusion_matrices.png'), 'Resolution', 220);

    if ~isempty(traceTbl) && ismember('pred_stable_prob', traceTbl.Properties.VariableNames)
        fig5 = figure('Name', 'ConfidenceHistogram', 'Color', 'w', 'Position', [180 180 980 420]);
        c = traceTbl.pred_stable_prob;
        c = c(isfinite(c));
        histogram(c, 25, 'FaceColor', [0.10 0.45 0.78], 'EdgeColor', [0.1 0.1 0.1]);
        xlabel('pred\_stable\_prob');
        ylabel('count');
        title('Prediction Confidence Distribution (trace-level)');
        grid on;
        exportgraphics(fig5, fullfile(outputDir, 'paper_fig5_confidence_hist.png'), 'Resolution', 220);
    end

    save(fullfile(outputDir, 'paper_metrics_struct.mat'), ...
        'mProposed', 'mExecuted', 'mBaseline', 'baseline', 'datasetPath', 'tracePath', 'perfPath', 'dmetPath');

    infoTxt = fullfile(outputDir, 'paper_summary.txt');
    fid = fopen(infoTxt, 'w');
    if fid > 0
        fprintf(fid, 'run_dir: %s\n', runDir);
        fprintf(fid, 'dataset: %s\n', datasetPath);
        fprintf(fid, 'trace: %s\n', tracePath);
        fprintf(fid, 'performance: %s\n', perfPath);
        fprintf(fid, 'decision_metrics: %s\n', dmetPath);
        fprintf(fid, '\n[Ontology+AI policy]\n');
        dumpMetric(fid, mProposed);
        fprintf(fid, '\n[Ontology+AI executed]\n');
        dumpMetric(fid, mExecuted);
        fprintf(fid, '\n[Threshold baseline]\n');
        dumpMetric(fid, mBaseline);
        fclose(fid);
    end

    out = struct();
    out.runDir = runDir;
    out.outputDir = outputDir;
    out.datasetPath = datasetPath;
    out.tracePath = tracePath;
    out.performancePath = perfPath;
    out.decisionMetricsPath = dmetPath;
    out.methodTablePath = fullfile(outputDir, 'paper_table_method_comparison.csv');
    out.thresholdTablePath = fullfile(outputDir, 'paper_table_thresholds.csv');
end


function runDir = findLatestRunDir(dataRoot)
    d = dir(dataRoot);
    isSub = [d.isdir] & ~startsWith({d.name}, '.');
    names = string({d(isSub).name});
    mask = strlength(names) == 15 & contains(names, '_');
    names = names(mask);
    if isempty(names)
        error('AutoSimPaperPlots:NoRunDir', 'No run directory found under %s', dataRoot);
    end
    names = sort(names);
    runDir = fullfile(dataRoot, char(names(end)));
end


function p = pickFile(runDir, candidates)
    p = '';
    for i = 1:numel(candidates)
        q = dir(fullfile(runDir, candidates{i}));
        if ~isempty(q)
            [~, idx] = max([q.datenum]);
            p = fullfile(runDir, q(idx).name);
            return;
        end
    end
end


function ensureDir(p)
    if ~exist(p, 'dir')
        mkdir(p);
    end
end


function sid = getScenarioId(tbl)
    sid = [];
    if ismember('scenario_id', tbl.Properties.VariableNames)
        sid = double(tbl.scenario_id);
    end
    if isrow(sid)
        sid = sid';
    end
end


function gtSafe = buildGtSafe(tbl)
    n = height(tbl);
    gtSafe = false(n, 1);
    if ismember('gt_safe_to_land', tbl.Properties.VariableNames)
        gt = string(tbl.gt_safe_to_land);
        gtSafe = (gt == "stable") | (gt == "safe");
        return;
    end
    if ismember('label', tbl.Properties.VariableNames)
        lb = string(tbl.label);
        gtSafe = (lb == "stable");
        return;
    end
    if ismember('success', tbl.Properties.VariableNames)
        gtSafe = logical(tbl.success);
    end
end


function predLand = buildDecision(tbl, decisionField, fallbackNumericField)
    n = height(tbl);
    predLand = false(n, 1);

    if ismember(decisionField, tbl.Properties.VariableNames)
        p = string(tbl.(decisionField));
        predLand = (p == "land");
        return;
    end

    if nargin >= 3 && ismember(fallbackNumericField, tbl.Properties.VariableNames)
        v = tbl.(fallbackNumericField);
        predLand = isfinite(v);
    end
end


function b = buildThresholdBaseline(tbl)
    n = height(tbl);
    gtSafe = buildGtSafe(tbl);

    wind = pickNumeric(tbl, {'mean_wind_speed','wind_speed_cmd','max_wind_speed'}, nan(n,1));
    tagErr = pickNumeric(tbl, {'max_tag_error','mean_tag_error','final_tag_error'}, nan(n,1));
    rollAbs = pickNumeric(tbl, {'mean_abs_roll_deg','final_abs_roll_deg'}, nan(n,1));
    pitchAbs = pickNumeric(tbl, {'mean_abs_pitch_deg','final_abs_pitch_deg'}, nan(n,1));

    safeMask = gtSafe & isfinite(wind) & isfinite(tagErr);
    if sum(safeMask) >= 20
        windThr = prctile(wind(safeMask), 90);
        tagThr = prctile(tagErr(safeMask), 90);
    else
        windThr = 1.8;
        tagThr = 0.20;
    end

    if sum(gtSafe & isfinite(rollAbs)) >= 20
        rollThr = prctile(rollAbs(gtSafe & isfinite(rollAbs)), 92);
    else
        rollThr = 8.0;
    end

    if sum(gtSafe & isfinite(pitchAbs)) >= 20
        pitchThr = prctile(pitchAbs(gtSafe & isfinite(pitchAbs)), 92);
    else
        pitchThr = 8.0;
    end

    pred = isfinite(wind) & isfinite(tagErr) & isfinite(rollAbs) & isfinite(pitchAbs) & ...
           (wind <= windThr) & (tagErr <= tagThr) & (rollAbs <= rollThr) & (pitchAbs <= pitchThr);

    b = struct();
    b.predLand = pred;
    b.thresholds = struct( ...
        'wind_threshold', windThr, ...
        'tag_error_threshold', tagThr, ...
        'roll_threshold_deg', rollThr, ...
        'pitch_threshold_deg', pitchThr);
end


function v = pickNumeric(tbl, candidates, fallback)
    v = fallback;
    for i = 1:numel(candidates)
        fn = candidates{i};
        if ismember(fn, tbl.Properties.VariableNames)
            vv = tbl.(fn);
            if isnumeric(vv)
                v = double(vv);
                return;
            end
        end
    end
end


function m = evalDecision(gtSafe, predLand)
    gt = logical(gtSafe(:));
    pr = logical(predLand(:));

    m.tp = sum(pr & gt);
    m.fp = sum(pr & ~gt);
    m.fn = sum(~pr & gt);
    m.tn = sum(~pr & ~gt);
    m.nValid = m.tp + m.fp + m.fn + m.tn;

    m.accuracy = safeDiv(m.tp + m.tn, m.nValid);
    m.precision = safeDiv(m.tp, m.tp + m.fp);
    m.recall = safeDiv(m.tp, m.tp + m.fn);
    m.specificity = safeDiv(m.tn, m.tn + m.fp);
    if isfinite(m.precision) && isfinite(m.recall) && (m.precision + m.recall) > 0
        m.f1 = 2 * m.precision * m.recall / (m.precision + m.recall);
    else
        m.f1 = nan;
    end
    m.unsafeLandingRate = safeDiv(m.fp, m.fp + m.tn);
end


function t = cumulativeTrend(gtSafe, predLand)
    n = numel(gtSafe);
    t.accuracy = nan(n,1);
    t.unsafeLandingRate = nan(n,1);

    gt = logical(gtSafe(:));
    pr = logical(predLand(:));

    for i = 1:n
        m = evalDecision(gt(1:i), pr(1:i));
        t.accuracy(i) = m.accuracy;
        t.unsafeLandingRate(i) = m.unsafeLandingRate;
    end
end


function y = smoothAdaptive(x)
    n = numel(x);
    w = max(5, floor(n / 35));
    if mod(w,2) == 0
        w = w + 1;
    end
    if w > n
        w = max(1, 2 * floor(n/2) + 1);
    end
    if w <= 1
        y = x;
    else
        y = movmean(x, w, 'omitnan');
    end
end


function [xRisk, yRisk] = pickRiskAxes(tbl)
    n = height(tbl);
    xRisk = pickNumeric(tbl, {'mean_wind_speed','max_wind_speed','wind_speed_cmd'}, zeros(n,1));
    yRisk = pickNumeric(tbl, {'max_tag_error','mean_tag_error','final_tag_error'}, zeros(n,1));

    if all(~isfinite(xRisk))
        xRisk = (1:n)';
    end
    if all(~isfinite(yRisk))
        yRisk = zeros(n,1);
    end

    xRisk(~isfinite(xRisk)) = medianOmitNan(xRisk);
    yRisk(~isfinite(yRisk)) = medianOmitNan(yRisk);
end


function cls = classifyOutcome(gtSafe, predLand)
    n = numel(gtSafe);
    cls = strings(n,1);
    gt = logical(gtSafe(:));
    pr = logical(predLand(:));
    cls(pr & gt) = "TP";
    cls(pr & ~gt) = "FP";
    cls(~pr & gt) = "FN";
    cls(~pr & ~gt) = "TN";
end


function drawClass(ax, x, y, mask, colorVal, label)
    if ~any(mask)
        return;
    end
    scatter(ax, x(mask), y(mask), 30, 'filled', ...
        'MarkerFaceColor', colorVal, 'MarkerEdgeColor', [0.08 0.08 0.08], ...
        'MarkerFaceAlpha', 0.72, 'MarkerEdgeAlpha', 0.45, 'DisplayName', label);
    hold(ax, 'on');
end


function plotConfusion(ax, m, ttl)
    cm = [m.tp m.fn; m.fp m.tn];
    imagesc(ax, cm);
    colormap(ax, parula);
    colorbar(ax);
    axis(ax, 'equal');
    axis(ax, 'tight');
    xticks(ax, 1:2);
    yticks(ax, 1:2);
    xticklabels(ax, {'Pred Land', 'Pred Abort'});
    yticklabels(ax, {'GT Safe', 'GT Unsafe'});
    title(ax, ttl);

    for r = 1:2
        for c = 1:2
            text(ax, c, r, num2str(cm(r,c)), 'HorizontalAlignment', 'center', ...
                'VerticalAlignment', 'middle', 'Color', 'w', 'FontWeight', 'bold', 'FontSize', 11);
        end
    end
end


function v = safeDiv(a, b)
    if b <= 0
        v = nan;
    else
        v = a / b;
    end
end


function dumpMetric(fid, m)
    fprintf(fid, 'n_valid: %d\n', m.nValid);
    fprintf(fid, 'accuracy: %.4f\n', m.accuracy);
    fprintf(fid, 'precision: %.4f\n', m.precision);
    fprintf(fid, 'safe_recall: %.4f\n', m.recall);
    fprintf(fid, 'unsafe_reject: %.4f\n', m.specificity);
    fprintf(fid, 'f1: %.4f\n', m.f1);
    fprintf(fid, 'unsafe_landing_rate: %.4f\n', m.unsafeLandingRate);
    fprintf(fid, 'TP=%d FP=%d FN=%d TN=%d\n', m.tp, m.fp, m.fn, m.tn);
end


function m = medianOmitNan(x)
    x = x(isfinite(x));
    if isempty(x)
        m = 0;
    else
        m = median(x);
    end
end