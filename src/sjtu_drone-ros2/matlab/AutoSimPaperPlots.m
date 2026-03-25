% AutoSimPaperPlots.m
% Standalone script for paper-ready figures/tables from AutoSim outputs.
%
% Usage:
% 1) Open this file in MATLAB and Run.
% 2) Optional: set runDir/outputDir below before running.
close all;
defaultRunDir = "";
% Optional overrides.
% Accept both runDir and run_dir to avoid name-mismatch issues.
if ~exist('runDir', 'var') || strlength(string(runDir)) == 0
    if exist('run_dir', 'var') && strlength(string(run_dir)) > 0
        runDir = run_dir;
    elseif strlength(defaultRunDir) > 0
        runDir = defaultRunDir;
    else
        runDir = "";
    end
end

% Accept both outputDir and output_dir.
if ~exist('outputDir', 'var') || strlength(string(outputDir)) == 0
    if exist('output_dir', 'var') && strlength(string(output_dir)) > 0
        outputDir = output_dir;
    else
        outputDir = "";
    end
end

rootDir = fileparts(mfilename('fullpath'));
dataRoot = fullfile(rootDir, 'data');
plotRoot = fullfile(rootDir, 'plots');
dataRoots = autosimPaperDiscoverDataRoots(rootDir, dataRoot);

if strlength(string(runDir)) == 0
    runDir = findLatestRunDir(dataRoots);
end
runDir = char(string(runDir));

if strlength(string(outputDir)) == 0
    [~, runName] = fileparts(runDir);
    outputDir = fullfile(plotRoot, ['paper_' runName '_' datestr(now, 'yyyymmdd_HHMMSS')]);
end

% Script variables persist in MATLAB base workspace.
% If outputDir is unchanged from the previous run, force a fresh timestamped folder.
if exist('paperPlotResult', 'var') && isstruct(paperPlotResult) && isfield(paperPlotResult, 'outputDir')
    prevOut = string(paperPlotResult.outputDir);
    if strlength(prevOut) > 0 && string(outputDir) == prevOut
        [~, runName] = fileparts(runDir);
        outputDir = fullfile(plotRoot, ['paper_' runName '_' datestr(now, 'yyyymmdd_HHMMSS')]);
    end
end

outputDir = char(string(outputDir));
ensureDir(outputDir);

fprintf('[AutoSimPaperPlots] runDir=%s\n', runDir);
fprintf('[AutoSimPaperPlots] outputDir=%s\n', outputDir);

FONT_AX = 24;
FONT_LABEL = 24;
FONT_TITLE = 36;
FONT_LEGEND = 24;

datasetPath = pickFile(runDir, {'autosim_dataset_latest.csv', 'autosim_dataset_*_completed.csv'});
tracePath = pickFile(runDir, {'autosim_trace_latest.csv', 'autosim_trace_*_completed.csv'});
perfPath = pickFile(runDir, {'autosim_performance_*_completed.csv'});
dmetPath = pickFile(runDir, {'autosim_decision_metrics_*_completed.csv'});

if isempty(datasetPath)
    error('AutoSimPaperPlots:NoDataset', 'Dataset CSV not found in %s', runDir);
end

datasetTbl = readtable(datasetPath);
datasetRawCount = height(datasetTbl);
[datasetTbl, recentNUsed] = autosimPaperApplyRecentWindow(datasetTbl);
nTotalScenario = height(datasetTbl);
traceTbl = table();
dmetTblRaw = table(); %#ok<NASGU>

if ~isempty(tracePath)
    traceTbl = readtable(tracePath);
    if isfinite(recentNUsed) && recentNUsed > 0 && height(traceTbl) > round(recentNUsed)
        traceTbl = traceTbl(end - round(recentNUsed) + 1:end, :);
    end
end
if ~isempty(dmetPath)
    dmetTblRaw = readtable(dmetPath); %#ok<NASGU>
end

if isfinite(recentNUsed) && recentNUsed > 0
    fprintf('[AutoSimPaperPlots] recent window: last %d rows (raw=%d, used=%d)\n', ...
        round(recentNUsed), datasetRawCount, nTotalScenario);
end

gtSafe = buildGtSafe(datasetTbl);
predProposed = buildDecision(datasetTbl, 'pred_decision', 'landing_cmd_time');

baseline = buildThresholdBaseline(datasetTbl);
predBaseline = baseline.predLand;

mProposed = evalDecision(gtSafe, predProposed);
mBaseline = evalDecision(gtSafe, predBaseline);

cmpTbl = table( ...
    string({'Ontology+AI (policy)'; 'Threshold baseline'}), ...
    [mProposed.nValid; mBaseline.nValid], ...
    [mProposed.accuracy; mBaseline.accuracy], ...
    [mProposed.precision; mBaseline.precision], ...
    [mProposed.recall; mBaseline.recall], ...
    [mProposed.specificity; mBaseline.specificity], ...
    [mProposed.balancedAccuracy; mBaseline.balancedAccuracy], ...
    [mProposed.f1; mBaseline.f1], ...
    [mProposed.unsafeLandingRate; mBaseline.unsafeLandingRate], ...
    [mProposed.tp; mBaseline.tp], ...
    [mProposed.fp; mBaseline.fp], ...
    [mProposed.fn; mBaseline.fn], ...
    [mProposed.tn; mBaseline.tn], ...
    'VariableNames', {'method','n_valid','accuracy','precision','safe_recall','unsafe_reject','balanced_accuracy','f1','unsafe_landing_rate','TP','FP','FN','TN'});

writetable(cmpTbl, fullfile(outputDir, 'paper_table_method_comparison.csv'));
writetable(struct2table(baseline.thresholds), fullfile(outputDir, 'paper_table_thresholds.csv'));

fig1 = figure('Name', 'MethodComparison', 'Color', 'w', 'Position', [100 100 1180 460]);
tl = tiledlayout(fig1, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

ax1 = nexttile(tl, 1);
barVals = [mProposed.accuracy, mProposed.f1, mProposed.unsafeLandingRate; ...
           mBaseline.accuracy, mBaseline.f1, mBaseline.unsafeLandingRate];
bh = bar(ax1, barVals, 0.88);
bh(1).FaceColor = [0.10 0.45 0.78];
bh(2).FaceColor = [0.13 0.60 0.33];
bh(3).FaceColor = [0.78 0.22 0.22];
xticks(ax1, 1:2);
xticklabels(ax1, {'Ontology+AI (policy)', 'Threshold'});
ylim(ax1, [0 1]);
ylabel(ax1, 'score');
title(ax1, 'Overall Metrics Comparison', 'FontSize', FONT_TITLE);
annotateTotalScenario(ax1, nTotalScenario, FONT_AX);
legend(ax1, {'Accuracy', 'F1', 'Unsafe landing rate'}, ...
    'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FONT_LEGEND);
set(ax1, 'FontSize', FONT_AX);
grid(ax1, 'on');

ax2 = nexttile(tl, 2);
confVals = [mProposed.tp mProposed.fp mProposed.fn mProposed.tn; ...
            mBaseline.tp mBaseline.fp mBaseline.fn mBaseline.tn];
b2 = bar(ax2, confVals, 'stacked', 'BarWidth', 0.8);
b2(1).FaceColor = [0.20 0.65 0.25];
b2(2).FaceColor = [0.85 0.30 0.20];
b2(3).FaceColor = [0.95 0.65 0.15];
b2(4).FaceColor = [0.25 0.50 0.90];
xticks(ax2, 1:2);
xticklabels(ax2, {'Ontology+AI (policy)', 'Threshold'});
ylabel(ax2, 'scenario count');
title(ax2, 'Decision Outcome Composition', 'FontSize', FONT_TITLE);
annotateTotalScenario(ax2, nTotalScenario, FONT_AX);
legend(ax2, {'TP','FP','FN','TN'}, ...
    'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FONT_LEGEND);
set(ax2, 'FontSize', FONT_AX);
grid(ax2, 'on');

exportgraphics(fig1, fullfile(outputDir, 'paper_fig1_method_comparison.png'), 'Resolution', 220);

n = height(datasetTbl);
sid = getScenarioId(datasetTbl);
if isempty(sid) || numel(sid) ~= n || any(~isfinite(sid))
    sid = (1:n)';
end
trendP = cumulativeTrend(gtSafe, predProposed);
trendB = cumulativeTrend(gtSafe, predBaseline);

fig2 = figure('Name', 'CumulativeTrends', 'Color', 'w', 'Position', [120 120 1180 520]);
tl2 = tiledlayout(fig2, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

ax21 = nexttile(tl2, 1);
plot(ax21, sid, smoothAdaptive(trendP.accuracy), '-', 'LineWidth', 2.0, 'Color', [0.10 0.45 0.78]);
hold(ax21, 'on');
plot(ax21, sid, smoothAdaptive(trendB.accuracy), '-', 'LineWidth', 1.8, 'Color', [0.30 0.30 0.30]);
ylim(ax21, [0 1]);
ylabel(ax21, 'accuracy');
title(ax21, 'Cumulative Accuracy Trend', 'FontSize', FONT_TITLE);
annotateTotalScenario(ax21, nTotalScenario, FONT_AX);
legend(ax21, {'Ontology+AI (policy)','Threshold'}, ...
    'Location', 'southoutside', 'Orientation', 'horizontal', 'FontSize', FONT_LEGEND);
set(ax21, 'FontSize', FONT_AX);
grid(ax21, 'on');

ax22 = nexttile(tl2, 2);
plot(ax22, sid, smoothAdaptive(trendP.unsafeLandingRate), '-', 'LineWidth', 2.0, 'Color', [0.78 0.22 0.22]);
hold(ax22, 'on');
plot(ax22, sid, smoothAdaptive(trendB.unsafeLandingRate), '-', 'LineWidth', 1.8, 'Color', [0.35 0.35 0.35]);
ylim(ax22, [0 1]);
xlabel(ax22, 'scenario');
ylabel(ax22, 'unsafe landing rate');
title(ax22, 'Cumulative Unsafe Landing Rate', 'FontSize', FONT_TITLE);
annotateTotalScenario(ax22, nTotalScenario, FONT_AX);
legend(ax22, {'Ontology+AI (policy)','Threshold'}, ...
    'Location', 'southoutside', 'Orientation', 'horizontal', 'FontSize', FONT_LEGEND);
set(ax22, 'FontSize', FONT_AX);
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
title(ax3, 'Scenario Risk Map (Ontology+AI policy decision)', 'FontSize', FONT_TITLE);
annotateTotalScenario(ax3, nTotalScenario, FONT_AX);
grid(ax3, 'on');
legend(ax3, 'Location', 'eastoutside', 'FontSize', FONT_LEGEND);
set(ax3, 'FontSize', FONT_AX);

if isfield(baseline.thresholds, 'wind_threshold')
    xline(ax3, baseline.thresholds.wind_threshold, '--', 'Threshold wind', 'Color', [0.35 0.35 0.35], 'LineWidth', 1.1);
end
if isfield(baseline.thresholds, 'tag_error_threshold')
    yline(ax3, baseline.thresholds.tag_error_threshold, '--', 'Threshold visual', 'Color', [0.35 0.35 0.35], 'LineWidth', 1.1);
end

exportgraphics(fig3, fullfile(outputDir, 'paper_fig3_risk_map.png'), 'Resolution', 220);

fig4 = figure('Name', 'ConfusionMatrices', 'Color', 'w', 'Position', [160 160 980 440]);
tl4 = tiledlayout(fig4, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
ax41 = nexttile(tl4, 1);
plotConfusion(ax41, mProposed, 'Ontology+AI (policy)');
annotateTotalScenario(ax41, nTotalScenario, FONT_AX);
ax42 = nexttile(tl4, 2);
plotConfusion(ax42, mBaseline, 'Threshold baseline');
annotateTotalScenario(ax42, nTotalScenario, FONT_AX);
exportgraphics(fig4, fullfile(outputDir, 'paper_fig4_confusion_matrices.png'), 'Resolution', 220);

if ~isempty(traceTbl) && ismember('pred_stable_prob', traceTbl.Properties.VariableNames)
    fig5 = figure('Name', 'ConfidenceHistogram', 'Color', 'w', 'Position', [180 180 980 420]);
    c = traceTbl.pred_stable_prob;
    c = c(isfinite(c));
    histogram(c, 25, 'FaceColor', [0.10 0.45 0.78], 'EdgeColor', [0.1 0.1 0.1]);
    xlabel('pred\_stable\_prob', 'FontSize', FONT_LABEL);
    ylabel('count', 'FontSize', FONT_LABEL);
    title('Prediction Confidence Distribution (trace-level)', 'FontSize', FONT_TITLE);
    ax5 = gca;
    set(ax5, 'FontSize', FONT_AX);
    annotateTotalScenario(ax5, nTotalScenario, FONT_AX);
    grid(ax5, 'on');
    exportgraphics(fig5, fullfile(outputDir, 'paper_fig5_confidence_hist.png'), 'Resolution', 220);
end

decisionScoreP = double(predProposed(:));
decisionScoreB = double(predBaseline(:));
[windRiskTotal, windMean, windGust] = buildWindRiskSeries(datasetTbl, baseline.thresholds);

fig6 = figure('Name', 'ScenarioDecisionAndWindRisk', 'Color', 'w', 'Position', [190 190 1200 420]);
ax6 = axes(fig6);
sidv = sid(:);
hold(ax6, 'on');

% Background shading: AttemptLanding=light blue, HoldLanding=light red, per scenario band.
landMask = decisionScoreP > 0.5;
riskYMax = max(1.05, 1.05 * max(windRiskTotal, [], 'omitnan'));

colLand  = [0.82 0.90 0.98];
colAbort = [0.98 0.86 0.84];

for kk = 1:numel(sidv)
    xL = sidv(kk) - 0.5;
    xR = sidv(kk) + 0.5;
    if landMask(kk)
        fill(ax6, [xL xR xR xL], [0 0 riskYMax riskYMax], colLand, ...
            'EdgeColor', 'none', 'FaceAlpha', 1.0, 'HandleVisibility', 'off');
    else
        fill(ax6, [xL xR xR xL], [0 0 riskYMax riskYMax], colAbort, ...
            'EdgeColor', 'none', 'FaceAlpha', 1.0, 'HandleVisibility', 'off');
    end
end

% Wind risk curve on top (single line, bold enough to read clearly).
hWind = plot(ax6, sidv, smoothAdaptive(windRiskTotal), '-', ...
    'LineWidth', 1.5, 'Color', [0.15 0.15 0.15], 'DisplayName', 'Wind risk');
yline(ax6, 1.0, ':', 'Color', [0.50 0.50 0.50], 'LineWidth', 0.8, 'HandleVisibility', 'off');

ylim(ax6, [0 riskYMax]);
xMin = min(sidv, [], 'omitnan');
xMax = max(sidv, [], 'omitnan');
if ~isfinite(xMin) || ~isfinite(xMax)
    xMin = 1;
    xMax = max(1, numel(sidv));
end
if xMax <= xMin
    xMax = xMin + 1;
end
xlim(ax6, [xMin - 0.5, xMax + 0.5]);
xlabel(ax6, 'Scenario', 'FontSize', FONT_LABEL);
ylabel(ax6, 'Wind risk (normalized)', 'FontSize', FONT_LABEL);
title(ax6, 'Ontology+AI Decision  |  {\color[rgb]{0.27,0.52,0.79}■ AttemptLanding}  {\color[rgb]{0.85,0.28,0.22}■ HoldLanding}  —  Wind risk', ...
    'FontSize', FONT_TITLE);
annotateTotalScenario(ax6, nTotalScenario, FONT_AX);
set(ax6, 'FontSize', FONT_AX, 'Box', 'on');
grid(ax6, 'on');

lg6 = legend(ax6, hWind, 'Location', 'northeast', 'FontSize', FONT_LEGEND, 'Box', 'off');

exportgraphics(fig6, fullfile(outputDir, 'paper_fig6_decision_wind_risk.png'), 'Resolution', 220);

% === fig7: Relative correctness and error bias per wind band ===
windVec7  = pickNumeric(datasetTbl, {'mean_wind_speed','wind_speed_cmd','max_wind_speed'}, nan(n,1));
srcVec7   = repmat("", n, 1);
if ismember('action_source', datasetTbl.Properties.VariableNames)
    srcVec7 = string(datasetTbl.action_source);
end
oc7 = classifyOutcome(gtSafe, predProposed);
oc7b = classifyOutcome(gtSafe, predBaseline);

bEdges  = [0, 1.5, 2.0, 2.5, 3.0, Inf];
bLabels = {'0–1.5', '1.5–2.0', '2.0–2.5', '2.5–3.0', '≥3.0'};
nB7 = numel(bLabels);

% Show all non-empty bands while keeping extreme ratio spikes bounded.
ratioCap = 20.0;

% cols: [TP_active | FP_active | FN_passive(timeout/forced) | FN_active | TN]
bCounts7    = zeros(nB7, 5);
bCounts7b   = zeros(nB7, 5);
bTotal7     = zeros(nB7, 1);

for b = 1:nB7
    mask = windVec7 >= bEdges(b) & windVec7 < bEdges(b+1);
    if ~any(mask); continue; end
    oc_b  = oc7(mask);
    src_b = srcVec7(mask);
    isPassive = contains(src_b, 'timeout', 'IgnoreCase', true) | ...
                contains(src_b, 'forced',  'IgnoreCase', true);
    bCounts7(b,1) = sum(oc_b == "TP");
    bCounts7(b,2) = sum(oc_b == "FP");
    bCounts7(b,3) = sum(oc_b == "FN" &  isPassive);
    bCounts7(b,4) = sum(oc_b == "FN" & ~isPassive);
    bCounts7(b,5) = sum(oc_b == "TN");
    oc_bb = oc7b(mask);
    bCounts7b(b,1) = sum(oc_bb == "TP");
    bCounts7b(b,2) = sum(oc_bb == "FP");
    bCounts7b(b,3) = sum(oc_bb == "FN" &  isPassive);
    bCounts7b(b,4) = sum(oc_bb == "FN" & ~isPassive);
    bCounts7b(b,5) = sum(oc_bb == "TN");
    bTotal7(b) = sum(mask);
end

fig7 = figure('Name', 'WindBandDecisionBreakdown', 'Color', 'w', 'Position', [200 200 980 480]);
ax7L = axes(fig7);
hold(ax7L, 'on');

tpOverTn7 = nan(nB7, 1);
fnOverFp7 = nan(nB7, 1);
tpOverTn7b = nan(nB7, 1);
fnOverFp7b = nan(nB7, 1);
for b = 1:nB7
    if bTotal7(b) > 0
        fnTotal = bCounts7(b,3) + bCounts7(b,4);
        fnTotalB = bCounts7b(b,3) + bCounts7b(b,4);

        tpOverTn7(b) = safeDivForPlot(bCounts7(b,1), bCounts7(b,5), ratioCap);
        fnOverFp7(b) = safeDivForPlot(fnTotal, bCounts7(b,2), ratioCap);

        tpOverTn7b(b) = safeDivForPlot(bCounts7b(b,1), bCounts7b(b,5), ratioCap);
        fnOverFp7b(b) = safeDivForPlot(fnTotalB, bCounts7b(b,2), ratioCap);
    end
end

tpOverTn7Pct = ratioToPairPercent(tpOverTn7);
fnOverFp7Pct = ratioToPairPercent(fnOverFp7);
tpOverTn7bPct = ratioToPairPercent(tpOverTn7b);
fnOverFp7bPct = ratioToPairPercent(fnOverFp7b);

pTP = plot(ax7L, 1:nB7, tpOverTn7Pct, '-o', ...
    'LineWidth', 2.2, ...
    'Color', [0.15 0.62 0.22], ...
    'MarkerFaceColor', [0.15 0.62 0.22], ...
    'MarkerSize', 7, ...
    'DisplayName', 'TP/TN');

pFN = plot(ax7L, 1:nB7, fnOverFp7Pct, '-s', ...
    'LineWidth', 2.2, ...
    'Color', [0.90 0.46 0.10], ...
    'MarkerFaceColor', [0.90 0.46 0.10], ...
    'MarkerSize', 7, ...
    'DisplayName', 'FN/FP');

pTPb = plot(ax7L, 1:nB7, tpOverTn7bPct, '--o', ...
    'LineWidth', 1.8, ...
    'Color', [0.10 0.35 0.72], ...
    'MarkerFaceColor', [0.10 0.35 0.72], ...
    'MarkerSize', 6, ...
    'DisplayName', 'TP/TN (threshold)');

pFNb = plot(ax7L, 1:nB7, fnOverFp7bPct, '--s', ...
    'LineWidth', 1.8, ...
    'Color', [0.55 0.30 0.08], ...
    'MarkerFaceColor', [0.55 0.30 0.08], ...
    'MarkerSize', 6, ...
    'DisplayName', 'FN/FP (threshold)');

hRef = yline(ax7L, 50.0, ':', '50% balance', ...
    'Color', [0.45 0.45 0.45], ...
    'LineWidth', 1.1, ...
    'LabelHorizontalAlignment', 'left', ...
    'LabelVerticalAlignment', 'bottom');

set(ax7L, 'XTick', 1:nB7, 'XTickLabel', bLabels, 'FontSize', FONT_AX);
xlabel(ax7L, 'Wind speed band (m/s)', 'FontSize', FONT_LABEL);
ylabel(ax7L, 'Relative ratio (%)', 'FontSize', FONT_LABEL);
title(ax7L, 'TP/TN and FN/FP per Wind Band', 'FontSize', FONT_TITLE);
annotateTotalScenario(ax7L, nTotalScenario, FONT_AX);

yCandidates = [tpOverTn7Pct; fnOverFp7Pct; tpOverTn7bPct; fnOverFp7bPct];
yCandidates = yCandidates(isfinite(yCandidates));
if isempty(yCandidates)
    ylim(ax7L, [0 100]);
else
    ylim(ax7L, [0 100]);
end
grid(ax7L, 'on');

for b = 1:nB7
    yMark = ax7L.YLim(1) + 0.08 * (ax7L.YLim(2) - ax7L.YLim(1));
    text(ax7L, b, yMark, sprintf('n=%d', round(bTotal7(b))), ...
        'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'bottom', ...
        'FontSize', max(12, FONT_AX - 8), ...
        'Color', [0.30 0.30 0.30]);
end

legend(ax7L, [pTP pFN pTPb pFNb], ...
    {'TP/TN (Ontology+AI)', 'FN/FP (Ontology+AI)', 'TP/TN (Threshold)', 'FN/FP (Threshold)'}, ...
    'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FONT_LEGEND, 'Box', 'off');

exportgraphics(fig7, fullfile(outputDir, 'paper_fig7_wind_band_breakdown.png'), 'Resolution', 220);

save(fullfile(outputDir, 'paper_metrics_struct.mat'), ...
    'mProposed', 'mBaseline', 'baseline', 'datasetPath', 'tracePath', 'perfPath', 'dmetPath');

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
    fprintf(fid, '\n[Threshold baseline]\n');
    dumpMetric(fid, mBaseline);
    fclose(fid);
end

paperPlotResult = struct();
paperPlotResult.runDir = runDir;
paperPlotResult.outputDir = outputDir;
paperPlotResult.datasetPath = datasetPath;
paperPlotResult.tracePath = tracePath;
paperPlotResult.performancePath = perfPath;
paperPlotResult.decisionMetricsPath = dmetPath;
paperPlotResult.methodTablePath = fullfile(outputDir, 'paper_table_method_comparison.csv');
paperPlotResult.thresholdTablePath = fullfile(outputDir, 'paper_table_thresholds.csv');

assignin('base', 'paperPlotResult', paperPlotResult);
fprintf('[AutoSimPaperPlots] done. outputDir=%s\n', outputDir);


function runDir = findLatestRunDir(dataRoots)
    files = autosimPaperCollectFiles(dataRoots, {'autosim_dataset_latest.csv', 'autosim_dataset_*_completed.csv', 'autosim_dataset_*_interrupted.csv'});
    if isempty(files)
        error('AutoSimPaperPlots:NoRunDir', 'No dataset CSV found under discovered data roots.');
    end
    [~, idx] = max([files.datenum]);
    runDir = files(idx).folder;
end


function roots = autosimPaperDiscoverDataRoots(rootDir, dataRoot)
    roots = string(dataRoot);
    parallelOutputData = autosimPaperFindLatestParallelOutput(rootDir);
    if strlength(parallelOutputData) > 0
        roots(end+1, 1) = parallelOutputData; %#ok<AGROW>
    end
    roots = unique(roots, 'stable');
end


function [T, recentN] = autosimPaperApplyRecentWindow(T)
recentN = autosimPaperResolveRecentDatasetN();
if ~(isfinite(recentN) && recentN > 0)
    return;
end
n = height(T);
if n <= 0
    return;
end
k = min(n, round(recentN));
T = T(n - k + 1:n, :);
end


function recentN = autosimPaperResolveRecentDatasetN()
recentN = inf;
if exist('recentDatasetN', 'var')
    vLocal = double(recentDatasetN);
    if isfinite(vLocal) && vLocal > 0
        recentN = round(vLocal);
        return;
    end
end
raw = string(getenv('AUTOSIM_RECENT_DATASET_N'));
if strlength(raw) == 0
    return;
end
v = str2double(raw);
if isfinite(v) && v > 0
    recentN = round(v);
end
end


function out = autosimPaperFindLatestParallelOutput(rootDir)
    out = "";
    parallelRoot = fullfile(rootDir, 'parallel_runs');
    if ~isfolder(parallelRoot)
        return;
    end

    runDirs = dir(parallelRoot);
    runDirs = runDirs([runDirs.isdir]);
    runDirs = runDirs(~ismember({runDirs.name}, {'.', '..'}));
    if isempty(runDirs)
        return;
    end

    [~, ord] = sort([runDirs.datenum], 'descend');
    for i = 1:numel(ord)
        candidate = fullfile(runDirs(ord(i)).folder, runDirs(ord(i)).name, 'output', 'data');
        if isfolder(candidate)
            out = string(candidate);
            return;
        end
    end
end


function files = autosimPaperCollectFiles(roots, patterns)
    files = struct('folder', {}, 'name', {}, 'datenum', {});
    for i = 1:numel(roots)
        root = char(roots(i));
        if ~isfolder(root)
            continue;
        end
        for j = 1:numel(patterns)
            d = dir(fullfile(root, '**', patterns{j}));
            if isempty(d)
                continue;
            end
            if isempty(files)
                files = d;
            else
                files = [files; d]; %#ok<AGROW>
            end
        end
    end

    if isempty(files)
        return;
    end

    fullPaths = strings(numel(files), 1);
    for k = 1:numel(files)
        fullPaths(k) = string(fullfile(files(k).folder, files(k).name));
    end
    [~, ia] = unique(fullPaths, 'stable');
    files = files(ia);
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
        gt = normalizeActionLabel(tbl.gt_safe_to_land);
        gtSafe = (gt == "AttemptLanding");
        return;
    end
    if ismember('label', tbl.Properties.VariableNames)
        lb = normalizeActionLabel(tbl.label);
        gtSafe = (lb == "AttemptLanding");
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
        p = normalizeActionLabel(tbl.(decisionField));
        predLand = (p == "AttemptLanding");
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

    [hoverWindLimit, landingWindLimit, windLimitMeta] = estimateWindPhysicsLimit(tbl);

    safeMask = gtSafe & isfinite(wind) & isfinite(tagErr);
    if sum(safeMask) >= 20
        windThrData = prctile(wind(safeMask), 90);
        tagThr = prctile(tagErr(safeMask), 90);
    else
        windThrData = 1.8;
        tagThr = 0.20;
    end

    windThr = windThrData;
    if isfinite(landingWindLimit) && landingWindLimit > 0
        windThr = min(windThr, landingWindLimit);
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
        'wind_threshold_data', windThrData, ...
        'wind_threshold', windThr, ...
        'hover_wind_limit', hoverWindLimit, ...
        'landing_wind_limit', landingWindLimit, ...
        'tag_error_threshold', tagThr, ...
        'roll_threshold_deg', rollThr, ...
        'pitch_threshold_deg', pitchThr, ...
        'mass_kg', windLimitMeta.mass_kg, ...
        'max_total_thrust_n', windLimitMeta.max_total_thrust_n, ...
        'thrust_margin_n', windLimitMeta.thrust_margin_n, ...
        'air_density_kgpm3', windLimitMeta.air_density_kgpm3, ...
        'drag_coefficient', windLimitMeta.drag_coefficient, ...
        'frontal_area_m2', windLimitMeta.frontal_area_m2, ...
        'landing_limit_factor', windLimitMeta.landing_limit_factor);
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


function [hoverLimit, landingLimit, meta] = estimateWindPhysicsLimit(tbl)
    defaults = readDronePhysicsDefaults();

    massKg = pickScalarNumeric(tbl, {'mass_kg','drone_mass_kg','drone_mass','mass'}, defaults.mass_kg);
    g = pickScalarNumeric(tbl, {'gravity_mps2','gravity'}, 9.81);
    maxThrust = pickScalarNumeric(tbl, {'max_total_thrust_n','t_max','max_thrust_n','total_thrust_n'}, defaults.max_total_thrust_n);
    rho = pickScalarNumeric(tbl, {'air_density_kgpm3','rho_air','rho'}, defaults.air_density_kgpm3);
    cd = pickScalarNumeric(tbl, {'drag_coefficient','c_d','cd'}, defaults.drag_coefficient);
    area = pickScalarNumeric(tbl, {'frontal_area_m2','drag_area_m2','reference_area_m2','area_m2'}, defaults.frontal_area_m2);
    landingFactor = pickScalarNumeric(tbl, {'landing_limit_factor'}, defaults.landing_limit_factor);
    minMargin = pickScalarNumeric(tbl, {'min_thrust_margin_n'}, 0.5);

    margin = max(maxThrust - massKg * g, minMargin);
    denom = rho * cd * area;

    if isfinite(margin) && isfinite(denom) && denom > 0
        hoverLimit = sqrt(max(0.0, 2.0 * margin / denom));
        landingLimit = max(0.0, landingFactor) * hoverLimit;
    else
        hoverLimit = nan;
        landingLimit = nan;
    end

    meta = struct( ...
        'mass_kg', massKg, ...
        'max_total_thrust_n', maxThrust, ...
        'thrust_margin_n', margin, ...
        'air_density_kgpm3', rho, ...
        'drag_coefficient', cd, ...
        'frontal_area_m2', area, ...
        'landing_limit_factor', landingFactor);
end


function d = readDronePhysicsDefaults()
    d = struct( ...
        'mass_kg', 1.4, ...
        'max_total_thrust_n', 24.0, ...
        'air_density_kgpm3', 1.225, ...
        'drag_coefficient', 1.10, ...
        'frontal_area_m2', 0.075, ...
        'landing_limit_factor', 0.5);

    thisDir = fileparts(mfilename('fullpath'));
    repoRoot = fileparts(thisDir);

    yamlPath = fullfile(repoRoot, 'sjtu_drone_bringup', 'config', 'drone.yaml');
    urdfPath = fullfile(repoRoot, 'sjtu_drone_description', 'urdf', 'sjtu_drone.urdf');
    sdfPath = fullfile(repoRoot, 'sjtu_drone_description', 'models', 'sjtu_drone', 'sjtu_drone.sdf');

    if isfile(yamlPath)
        d.max_total_thrust_n = firstFinite([readYamlScalar(yamlPath, 'maxForce'), d.max_total_thrust_n]);
        d.mass_kg = firstFinite([readYamlScalar(yamlPath, 'mass_kg'), readYamlScalar(yamlPath, 'mass'), d.mass_kg]);
        d.frontal_area_m2 = firstFinite([readYamlScalar(yamlPath, 'frontal_area_m2'), readYamlScalar(yamlPath, 'drag_area_m2'), d.frontal_area_m2]);
        d.drag_coefficient = firstFinite([readYamlScalar(yamlPath, 'drag_coefficient'), readYamlScalar(yamlPath, 'cd'), d.drag_coefficient]);
        d.air_density_kgpm3 = firstFinite([readYamlScalar(yamlPath, 'air_density_kgpm3'), d.air_density_kgpm3]);
        d.landing_limit_factor = firstFinite([readYamlScalar(yamlPath, 'landing_limit_factor'), d.landing_limit_factor]);
    end

    if isfile(urdfPath)
        d.mass_kg = firstFinite([readXmlAttributeScalar(urdfPath, '<mass', 'value'), d.mass_kg]);
    end

    if isfile(sdfPath)
        d.mass_kg = firstFinite([readXmlTagScalar(sdfPath, 'mass'), d.mass_kg]);
        d.max_total_thrust_n = firstFinite([readXmlTagScalar(sdfPath, 'maxForce'), d.max_total_thrust_n]);
    end
end


function v = pickScalarNumeric(tbl, candidates, fallback)
    v = fallback;
    for i = 1:numel(candidates)
        fn = candidates{i};
        if ~ismember(fn, tbl.Properties.VariableNames)
            continue;
        end
        x = tbl.(fn);
        if ~isnumeric(x)
            continue;
        end
        x = double(x(:));
        x = x(isfinite(x));
        if ~isempty(x)
            v = median(x);
            return;
        end
    end
end


function val = readYamlScalar(filePath, key)
    val = nan;
    try
        txt = fileread(filePath);
    catch
        return;
    end
    pat = ['(?m)^\s*' regexptranslate('escape', key) '\s*:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*$'];
    tok = regexp(txt, pat, 'tokens', 'once');
    if ~isempty(tok)
        num = str2double(tok{1});
        if isfinite(num)
            val = num;
        end
    end
end


function val = readXmlTagScalar(filePath, tagName)
    val = nan;
    try
        txt = fileread(filePath);
    catch
        return;
    end
    pat = ['<' regexptranslate('escape', tagName) '>\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*</' regexptranslate('escape', tagName) '>'];
    tok = regexp(txt, pat, 'tokens', 'once');
    if ~isempty(tok)
        num = str2double(tok{1});
        if isfinite(num)
            val = num;
        end
    end
end


function val = readXmlAttributeScalar(filePath, tagHead, attrName)
    val = nan;
    try
        txt = fileread(filePath);
    catch
        return;
    end
    pat = [regexptranslate('escape', tagHead) '[^>]*' regexptranslate('escape', attrName) '\s*=\s*"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)"'];
    tok = regexp(txt, pat, 'tokens', 'once');
    if ~isempty(tok)
        num = str2double(tok{1});
        if isfinite(num)
            val = num;
        end
    end
end


function v = firstFinite(values)
    v = nan;
    values = double(values(:));
    for i = 1:numel(values)
        if isfinite(values(i))
            v = values(i);
            return;
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
    vals = [m.recall, m.specificity];
    vals = vals(isfinite(vals));
    if isempty(vals)
        m.balancedAccuracy = nan;
    else
        m.balancedAccuracy = mean(vals);
    end
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


function annotateTotalScenario(ax, nTotal, baseFont)
    if ~isgraphics(ax)
        return;
    end
    if ~isfinite(nTotal) || nTotal <= 0
        return;
    end
    fs = max(10, min(16, round(baseFont * 0.55)));
    text(ax, 0.99, 0.98, sprintf('N=%d', round(nTotal)), ...
        'Units', 'normalized', ...
        'HorizontalAlignment', 'right', ...
        'VerticalAlignment', 'top', ...
        'FontSize', fs, ...
        'FontWeight', 'bold', ...
        'Color', [0.20 0.20 0.20], ...
        'BackgroundColor', [1 1 1], ...
        'Margin', 2, ...
        'Clipping', 'on');
end


function plotConfusion(ax, m, ttl)
    cm = [m.tp m.fn; m.fp m.tn];
    imagesc(ax, cm);
    cmap = parula(256);
    colormap(ax, cmap);
    colorbar(ax);
    axis(ax, 'equal');
    axis(ax, 'tight');
    xticks(ax, 1:2);
    yticks(ax, 1:2);
    xticklabels(ax, {'Pred AttemptLanding', 'Pred HoldLanding'});
    yticklabels(ax, {'GT AttemptLanding', 'GT HoldLanding'});
    title(ax, ttl, 'FontSize', 13);
    set(ax, 'FontSize', 11);

    cMin = min(cm(:));
    cMax = max(cm(:));
    cSpan = max(cMax - cMin, eps);

    for r = 1:2
        for c = 1:2
            % Choose text color by rendered cell brightness:
            % bright (yellow) -> black text, dark -> white text.
            idx = 1 + round((size(cmap, 1) - 1) * (cm(r,c) - cMin) / cSpan);
            idx = min(max(idx, 1), size(cmap, 1));
            rgb = cmap(idx, :);
            luminance = 0.2126 * rgb(1) + 0.7152 * rgb(2) + 0.0722 * rgb(3);
            if luminance >= 0.55
                txtColor = [0.10 0.10 0.10];
            else
                txtColor = [1 1 1];
            end
            text(ax, c, r, num2str(cm(r,c)), 'HorizontalAlignment', 'center', ...
                'VerticalAlignment', 'middle', 'Color', txtColor, 'FontWeight', 'bold', 'FontSize', 12);
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


function v = safeDivForPlot(a, b, ratioCap)
    if a <= 0 && b <= 0
        v = 0;
        return;
    end

    if b <= 0
        v = ratioCap;
        return;
    end

    v = a / b;
    if ~isfinite(v)
        v = nan;
        return;
    end

    if nargin >= 3 && isfinite(ratioCap) && ratioCap > 0
        if v > ratioCap
            v = ratioCap;
        end
    end
end


function p = ratioToPairPercent(r)
p = nan(size(r));
mask = isfinite(r) & (r >= 0);
if ~any(mask)
    return;
end
p(mask) = 100 .* (r(mask) ./ (1 + r(mask)));
end


function dumpMetric(fid, m)
    fprintf(fid, 'n_valid: %d\n', m.nValid);
    fprintf(fid, 'accuracy: %.4f\n', m.accuracy);
    fprintf(fid, 'precision: %.4f\n', m.precision);
    fprintf(fid, 'safe_recall: %.4f\n', m.recall);
    fprintf(fid, 'unsafe_reject: %.4f\n', m.specificity);
    fprintf(fid, 'balanced_accuracy: %.4f\n', m.balancedAccuracy);
    fprintf(fid, 'f1: %.4f\n', m.f1);
    fprintf(fid, 'unsafe_landing_rate: %.4f\n', m.unsafeLandingRate);
    fprintf(fid, 'TP=%d FP=%d FN=%d TN=%d\n', m.tp, m.fp, m.fn, m.tn);
end


function label = normalizeActionLabel(x)
    s = lower(strtrim(string(x)));
    label = repmat("Unknown", size(s));

    attemptMask = (s == "attemptlanding") | (s == "attempt_landing") | (s == "land") | ...
        (s == "landing") | (s == "safe") | (s == "stable") | (s == "safetoland") | ...
        (s == "attempt_landing_recommended") | (s == "clear_to_land") | ...
        (s == "proceed") | (s == "1") | (s == "true");

    holdMask = (s == "holdlanding") | (s == "hold_landing") | (s == "hold") | ...
        (s == "abort") | (s == "abortlanding") | (s == "delaylanding") | ...
        (s == "continuehover") | (s == "reapproach") | (s == "descend") | ...
        (s == "cancellanding") | (s == "goaround") | (s == "unsafe") | ...
        (s == "unstable") | (s == "unsafetoland") | (s == "stop") | ...
        (s == "hold_landing_recommended") | (s == "abort_recommended") | ...
        (s == "monitor_and_reassess") | (s == "0") | (s == "false");

    label(attemptMask) = "AttemptLanding";
    label(holdMask) = "HoldLanding";
end


function m = medianOmitNan(x)
    x = x(isfinite(x));
    if isempty(x)
        m = 0;
    else
        m = median(x);
    end
end


function [riskTotal, windNorm, gustNorm] = buildWindRiskSeries(tbl, thresholds)
    n = height(tbl);

    riskRawNew = pickNumeric(tbl, {'mean_wind_risk_raw','wind_risk_raw'}, nan(n,1));
    bodyRiskNew = pickNumeric(tbl, {'mean_wind_body_risk','wind_body_risk'}, nan(n,1));
    gustRiskNew = pickNumeric(tbl, {'mean_wind_gust_risk','wind_gust_risk'}, nan(n,1));

    hasNewRisk = any(isfinite(riskRawNew)) || any(isfinite(bodyRiskNew)) || any(isfinite(gustRiskNew));
    if hasNewRisk
        risk01 = fillSeriesNan(riskRawNew);
        body01 = fillSeriesNan(bodyRiskNew);
        gust01 = fillSeriesNan(gustRiskNew);

        if ~any(isfinite(bodyRiskNew))
            body01 = risk01;
        end
        if ~any(isfinite(gustRiskNew))
            gust01 = risk01;
        end

        riskTotal = min(2.0, max(0.0, 2.0 * risk01));
        windNorm = min(2.0, max(0.0, 2.0 * body01));
        gustNorm = min(2.0, max(0.0, 2.0 * gust01));
        return;
    end

    windRaw = pickNumeric(tbl, ...
        {'mean_wind_speed','wind_speed_cmd','max_wind_speed','wind_speed','wind_mps'}, ...
        nan(n,1));
    gustRaw = pickNumeric(tbl, ...
        {'gust_speed','max_gust_speed','wind_gust_speed','gust_mps','max_wind_speed'}, ...
        nan(n,1));

    hasExplicitGust = any(isfinite(gustRaw));

    windRaw = fillSeriesNan(windRaw);
    gustRaw = fillSeriesNan(gustRaw);

    % If explicit gust data is absent, approximate gustiness from wind deviation.
    if ~hasExplicitGust || all(abs(gustRaw - windRaw) < 1e-9)
        gustRaw = max(0, windRaw - movmean(windRaw, max(5, 2 * floor(n / 30) + 1), 'omitnan'));
    end

    windThr = 1.0;
    if isfield(thresholds, 'wind_threshold') && isfinite(thresholds.wind_threshold) && thresholds.wind_threshold > 0
        windThr = thresholds.wind_threshold;
    elseif isfield(thresholds, 'wind_threshold_data') && isfinite(thresholds.wind_threshold_data) && thresholds.wind_threshold_data > 0
        windThr = thresholds.wind_threshold_data;
    end

    gustScale = max(0.5, 0.6 * windThr);
    windNorm = min(2.0, max(0.0, windRaw ./ windThr));
    gustNorm = min(2.0, max(0.0, gustRaw ./ gustScale));
    riskTotal = min(2.0, 0.65 * windNorm + 0.35 * gustNorm);
end


function x = fillSeriesNan(x)
    x = double(x(:));
    if all(~isfinite(x))
        x = zeros(size(x));
        return;
    end
    mid = medianOmitNan(x);
    x(~isfinite(x)) = mid;
end


function s = buildOntologyDecisionScore(tbl, predLand)
    n = height(tbl);
    s = pickNumeric(tbl, ...
        {'landing_feasibility','pred_stable_prob','model_stable_prob','stable_prob','pred_prob_stable'}, ...
        nan(n,1));

    if any(isfinite(s))
        s = fillSeriesNan(s);
        s = min(1.0, max(0.0, s));
    else
        % Fallback: binary decision only if no confidence field exists.
        s = double(predLand(:));
    end
end


function s = buildThresholdDecisionScore(tbl, thresholds, predLand)
    n = height(tbl);
    wind = fillSeriesNan(pickNumeric(tbl, {'mean_wind_speed','wind_speed_cmd','max_wind_speed'}, nan(n,1)));
    tagErr = fillSeriesNan(pickNumeric(tbl, {'max_tag_error','mean_tag_error','final_tag_error'}, nan(n,1)));
    rollAbs = fillSeriesNan(pickNumeric(tbl, {'mean_abs_roll_deg','final_abs_roll_deg'}, nan(n,1)));
    pitchAbs = fillSeriesNan(pickNumeric(tbl, {'mean_abs_pitch_deg','final_abs_pitch_deg'}, nan(n,1)));

    windThr = pickThreshold(thresholds, {'wind_threshold','wind_threshold_data'}, 1.8);
    tagThr = pickThreshold(thresholds, {'tag_error_threshold'}, 0.20);
    rollThr = pickThreshold(thresholds, {'roll_threshold_deg'}, 8.0);
    pitchThr = pickThreshold(thresholds, {'pitch_threshold_deg'}, 8.0);

    windScore = normalizedSafeScore(wind, windThr);
    tagScore = normalizedSafeScore(tagErr, tagThr);
    rollScore = normalizedSafeScore(rollAbs, rollThr);
    pitchScore = normalizedSafeScore(pitchAbs, pitchThr);

    s = 0.40 * windScore + 0.25 * tagScore + 0.175 * rollScore + 0.175 * pitchScore;
    if ~any(isfinite(s))
        s = double(predLand(:));
    end
    s = min(1.0, max(0.0, s));
end


function thr = pickThreshold(thresholds, names, fallback)
    thr = fallback;
    for i = 1:numel(names)
        nm = names{i};
        if isfield(thresholds, nm)
            v = thresholds.(nm);
            if isfinite(v) && v > 0
                thr = v;
                return;
            end
        end
    end
end


function score = normalizedSafeScore(value, threshold)
    denom = max(1e-6, threshold);
    score = 1.0 - min(1.0, max(0.0, value ./ denom));
end